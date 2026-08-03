/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

/* LIS3DH via Zephyr sensor API */
#include <zephyr/drivers/sensor.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000
#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define ACCEL_SAMPLE_RATE_HZ 100
#define ACCEL_INTERVAL_MS (1000 / ACCEL_SAMPLE_RATE_HZ) 

#define SAMPLES_PER_PACKET 20  // 5 header + 20×6 = 125 bytes — safe under 244
#define LIS3DH_ADDR 0x19  // change to 0x18 if SA0 pin is low
#define LIS3DH_CTRL_REG2 0x21

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} __attribute__((packed)) accel_sample_t;

/* compile-time check — will error if packet exceeds BLE payload */
BUILD_ASSERT((5 + SAMPLES_PER_PACKET * sizeof(accel_sample_t)) <= 244,
             "Packet too large for BLE MTU");

// Static buffer holding pending samples
static accel_sample_t sample_buf[SAMPLES_PER_PACKET];
static uint8_t sample_count = 0;
static uint8_t pkt_seq = 0; 

K_SEM_DEFINE(accel_sem, 0, 1);  // initial=0, max=1
K_SEM_DEFINE(ble_init_ok, 0, 1); 

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;
static struct k_timer accel_timer;
//static struct k_sem accel_sem;

/* LIS3DH device handle */
const struct device *const accel_dev = DEVICE_DT_GET_ANY(st_lis2dh);

#if DT_HAS_CHOSEN(nordic_nus_uart)
#define NUS_UART_NODE DT_CHOSEN(nordic_nus_uart)
#elif DT_HAS_CHOSEN(zephyr_shell_uart)
#define NUS_UART_NODE DT_CHOSEN(zephyr_shell_uart)
#else
#error "No NUS UART: add chosen 'nordic,nus-uart' to devicetree."
#endif

static const struct device *uart = DEVICE_DT_GET(NUS_UART_NODE);
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

static void accel_timer_expiry(struct k_timer *timer) 
{
	k_sem_give(&accel_sem); // Signal the accelerometer thread to read and send data (Unlocks accel thread which is waiting on this semaphore)
}


/* ────────────────────────────────────────────
 * LIS3DH accelerometer read + BLE transmit
 * SDA = P27, SCL = P26
 * ──────────────────────────────────────────── */

/* ── REPLACED: now buffers samples and sends in binary bursts ── */
static inline int16_t sv_to_i16(struct sensor_value *v)
{
    /* Converts sensor_value to int16 in units of 0.01 m/s²
     * val1 = whole m/s², val2 = millionths of m/s² */
    return (int16_t)(v->val1 * 100 + v->val2 / 10000);
}

static void send_accel_over_ble(void)
{

	static uint32_t fetch_fail_count = 0; 
	
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("Accel device NOT ready");
        return;
    }

    if (!current_conn) {
        return;
    }

    struct sensor_value accel[3];

    if (sensor_sample_fetch(accel_dev) ||
        sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel)) {
        fetch_fail_count++;
        LOG_WRN("Accel read failed (%u times so far)", fetch_fail_count);
        return;
    }

    /* Append sample to buffer */
    sample_buf[sample_count].x = sv_to_i16(&accel[0]);
    sample_buf[sample_count].y = sv_to_i16(&accel[1]);
    sample_buf[sample_count].z = sv_to_i16(&accel[2]);
    sample_count++;

    /* Only transmit when buffer is full */
    if (sample_count >= SAMPLES_PER_PACKET) {

        /* Packet layout: [0xAC][count][interval_lo][interval_hi][...samples...] */
        uint8_t pkt[5 + SAMPLES_PER_PACKET * sizeof(accel_sample_t)];
        pkt[0] = 0xAC;
        pkt[1] = pkt_seq++;
		pkt[2] = SAMPLES_PER_PACKET;
        pkt[3] = ACCEL_INTERVAL_MS & 0xFF;
        pkt[4] = (ACCEL_INTERVAL_MS >> 8) & 0xFF;
        memcpy(&pkt[5], sample_buf, sizeof(sample_buf));

		LOG_INF("TX seq=%d: %d samples @ %dms", pkt[1], SAMPLES_PER_PACKET, ACCEL_INTERVAL_MS);
		for (int i = 0; i< SAMPLES_PER_PACKET; i++) {
			//int whole = sample_buf[i].z / 100;
			//int frac = abs(sample_buf[i].z % 100);
			//LOG_INF("  [%3d] %s%d.%02d", i, (sample_buf[i].z < 0 && whole == 0) ? "-" : "", whole, frac);
		}

        int err = bt_nus_send(NULL, pkt, sizeof(pkt));
        if (err) {
            LOG_WRN("BLE send failed: %d", err);
        } else {
            //LOG_INF("Sent %d samples (%d bytes)", SAMPLES_PER_PACKET, (int)sizeof(pkt));
        }

        sample_count = 0;   /* reset buffer */
    }
}
/* ─────────────────────────────────────────────────── */

static void lis3dh_check_odr(void)
{
    const struct device *i2c_dev =
        DEVICE_DT_GET(DT_BUS(DT_COMPAT_GET_ANY_STATUS_OKAY(st_lis2dh)));

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C bus not ready for ODR check");
        return;
    }

    uint8_t reg = 0x20; // CTRL_REG1
    uint8_t val = 0;
    int err = i2c_write_read(i2c_dev, LIS3DH_ADDR, &reg, 1, &val, 1);
    if (err) {
        LOG_ERR("Failed to read CTRL_REG1: %d", err);
        return;
    }

    uint8_t odr_bits = (val >> 4) & 0x0F;
    static const int odr_table[] = {
        0, 1, 10, 25, 50, 100, 200, 400, 1620, 1344 /* or 5376 in LP mode */
    };
    int odr_hz = (odr_bits < ARRAY_SIZE(odr_table)) ? odr_table[odr_bits] : -1;

    LOG_INF("CTRL_REG1=0x%02X, ODR bits=0x%X -> %d Hz", val, odr_bits, odr_hz);
}

/* ── NEW: enable LIS3DH hardware high-pass filter (gravity cancellation) ── */
static int lis3dh_enable_hpf(void)
{
    const struct device *i2c_dev =
        DEVICE_DT_GET(DT_BUS(DT_COMPAT_GET_ANY_STATUS_OKAY(st_lis2dh)));

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C bus not ready for HPF config");
        return -ENODEV;
    }

    /* CTRL_REG2 = 0x08
     * FDS=1  (bit3) — route HPF output to data registers
     * HPM=00        — normal mode
     * HPCF=00       — highest cutoff (~2Hz at 100Hz ODR), removes DC gravity */
    uint8_t buf[2] = { LIS3DH_CTRL_REG2, 0x08 };
    int err = i2c_write(i2c_dev, buf, sizeof(buf), LIS3DH_ADDR);
    if (err) {
        LOG_ERR("Failed to write CTRL_REG2: %d", err);
        return err;
    }

    LOG_INF("LIS3DH HPF enabled — gravity cancelled");
    return 0;
}
/* ──────────────────────────────────────────────────────────────────────── */

/* Accelerometer thread */
static void accel_thread_fn(void)
{
	k_sem_take(&ble_init_ok, K_FOREVER);
	k_sem_give(&ble_init_ok);
	k_timer_init(&accel_timer, accel_timer_expiry, NULL);
	//k_timer_start(&accel_timer, K_MSEC(ACCEL_INTERVAL_MS), K_MSEC(ACCEL_INTERVAL_MS));

	while (1) {
		k_sem_take(&accel_sem, K_FOREVER);
		send_accel_over_ble();
	}
}

#define ACCEL_THREAD_STACK_SIZE 1024
#define ACCEL_THREAD_PRIORITY   8

K_THREAD_DEFINE(accel_thread_id,
		ACCEL_THREAD_STACK_SIZE,
		accel_thread_fn,
		NULL, NULL, NULL,
		ACCEL_THREAD_PRIORITY, 0, 0);

/* ────────────────────────────────────────────
 * UART callbacks — unchanged
 * ──────────────────────────────────────────── */

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) || (!evt->data.tx.buf)) {
			return;
		}
		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data[0]);
		}
		k_free(buf);
		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}
		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}
		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;
		if (disable_req) {
			return;
		}
		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}
		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}
		uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}
		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]);
		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}
		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}
		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t, data);
		uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS);
		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;
	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}
	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
		(const struct uart_driver_api *)dev->api;
	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;
			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) break;
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) LOG_WRN("Failed to set DCD,xret code %d", err);
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) LOG_WRN("Failed to set DSR, ret code %d", err);
	}

	tx = k_malloc(sizeof(*tx));
	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service sample\r\n");
		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}
		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		k_free(rx);
	}

	return err;
}

/* ────────────────────────────────────────────
 * BLE callbacks 
 * ──────────────────────────────────────────── */

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
	LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    // 1. Check error FIRST
    if (err) {
        LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);
    current_conn = bt_conn_ref(conn);
    dk_set_led_on(CON_STATUS_LED);
	
	struct bt_conn_info info;
	bt_conn_get_info(conn, &info);
	LOG_INF("Initial interval: %d units = %d ms",
        info.le.interval,
        info.le.interval * 125 / 100);

    // 3. Request tighter interval AFTER confirming connection is valid
    // struct bt_le_conn_param param = {
    //     .interval_min = 6,    // 7.5ms
    //     .interval_max = 6,
    //     .latency      = 0,
    //     .timeout      = 4000,  // 4 second supervision timeout
    // };
	// 3. connection interval AFTER confirming connection is valid
	struct bt_le_conn_param param = {
    .interval_min = ACCEL_INTERVAL_MS * SAMPLES_PER_PACKET/2.5,  
    .interval_max = ACCEL_INTERVAL_MS * SAMPLES_PER_PACKET/2.5,
    .latency      = 0,
    .timeout      = 400,
};

    int ret = bt_conn_le_param_update(conn, &param);
    if (ret) {
        LOG_WRN("Failed to request conn param update: %d", ret);
    } else {
        LOG_INF("Conn param update requested (%d interval)", ACCEL_INTERVAL_MS * SAMPLES_PER_PACKET/2);
    }
	k_timer_start(&accel_timer, K_MSEC(500), K_MSEC(ACCEL_INTERVAL_MS)); 
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	k_timer_stop(&accel_timer);

	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));
	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
	}
}
#endif

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                              uint16_t latency, uint16_t timeout)
{
    LOG_INF("Conn params updated — interval: %d units (%d ms), latency: %d, timeout: %d ms",
            interval,
            interval * 125 / 100,
            latency,
            timeout * 10);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.recycled     = recycled_cb,
	.le_param_updated = le_param_updated,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	auth_conn = bt_conn_ref(conn);
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Passkey for %s: %06u", addr, passkey);
	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54H) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54L)) {
		LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
	} else {
		LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
	}
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel          = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed   = pairing_failed,
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));
		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}
		size_t tx_data_size = sizeof(tx->data) - 1;
		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}
		memcpy(tx->data, &data[pos], tx->len);
		pos += tx->len;
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}
		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("ATT MTU updated: TX=%d RX=%d bytes", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = att_mtu_updated,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);
	while (true) {
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}
	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;
	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) num_comp_reply(true);
		if (buttons & KEY_PASSKEY_REJECT) num_comp_reply(false);
	}
}
#endif

static void configure_gpio(void)
{
	int err;
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) LOG_ERR("Cannot init buttons (err: %d)", err);
#endif
	err = dk_leds_init();
	if (err) LOG_ERR("Cannot init LEDs (err: %d)", err);
}

/* ────────────────────────────────────────────
 * Main
 * ──────────────────────────────────────────── */

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio(); // configures LEDs and buttons

	/* Init LIS3DH */
	if (!device_is_ready(accel_dev)) {
    	LOG_ERR("LIS3DH not ready — check wiring/overlay");
		printk("System booted\n");
    	//accel_dev = NULL;
	} else {
    	LOG_INF("LIS3DH ready");
		/* Set ODR to target Hz */
    	struct sensor_value odr = { .val1 = ACCEL_SAMPLE_RATE_HZ, .val2 = 0 };
    	int err2 = sensor_attr_set(accel_dev, SENSOR_CHAN_ACCEL_XYZ,
                               SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    	if (err2) {
        	LOG_ERR("Failed to set ODR: %d", err2);
    	} else {
        	LOG_INF("ODR set to %d Hz", ACCEL_SAMPLE_RATE_HZ);
    	}
		//lis3dh_enable_hpf();
		lis3dh_check_odr(); 
	}

	err = uart_init();
	if (err) {
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization callbacks (err: %d)", err);
			return 0;
		}
		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			LOG_ERR("Failed to register authorization info callbacks (err: %d)", err);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");
	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	bt_gatt_cb_register(&gatt_callbacks);
	advertising_start();

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

/* ────────────────────────────────────────────
 * BLE write thread — unchanged
 * ──────────────────────────────────────────── */

void ble_write_thread(void)
{
	k_sem_take(&ble_init_ok, K_FOREVER);
	k_sem_give(&ble_init_ok);

	struct uart_data_t nus_data = { .len = 0 };

	for (;;) {
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);

		int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
		int loc = 0;

		while (plen > 0) {
			memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
			nus_data.len += plen;
			loc += plen;

			if (nus_data.len >= sizeof(nus_data.data) ||
			    (nus_data.data[nus_data.len - 1] == '\n') ||
			    (nus_data.data[nus_data.len - 1] == '\r')) {
				if (bt_nus_send(NULL, nus_data.data, nus_data.len)) {
					LOG_WRN("Failed to send data over BLE connection");
				}
				nus_data.len = 0;
			}
			plen = MIN(sizeof(nus_data.data), buf->len - loc);
		}
		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);