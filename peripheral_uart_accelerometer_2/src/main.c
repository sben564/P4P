/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <uart_async_adapter.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
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

/* Accelerometer transmit interval — 100ms = 10Hz */
#define ACCEL_INTERVAL_MS 100

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

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

/* ────────────────────────────────────────────
 * LIS3DH accelerometer read + BLE transmit
 * SDA = P27, SCL = P26
 * ──────────────────────────────────────────── */

static void send_accel_over_ble(void)
{

	if (!device_is_ready(accel_dev)) {
    LOG_ERR("Accel device NOT ready at boot");
	return;
	}	

	if (!current_conn) {
		return;
	}

	if (!accel_dev || !device_is_ready(accel_dev)) {
		LOG_ERR("LIS3DH not ready");
		return;
	}

	struct sensor_value accel[3];
	int err;

	err = sensor_sample_fetch(accel_dev);
	if (err) {
		LOG_ERR("Failed to fetch accel sample: %d", err);
		return;
	}

	err = sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (err) {
		LOG_ERR("Failed to get accel data: %d", err);
		return;
	}

	/* Convert sensor_value to integer + decimal parts
	 * avoiding float printf which can cause issues on ARM */
	int x_whole   = accel[0].val1;
	int x_dec     = abs(accel[0].val2) / 10000;
	int y_whole   = accel[1].val1;
	int y_dec     = abs(accel[1].val2) / 10000;
	int z_whole   = accel[2].val1;
	int z_dec     = abs(accel[2].val2) / 10000;

	/* Handle negative decimals correctly */
	char x_sign = (accel[0].val1 == 0 && accel[0].val2 < 0) ? '-' : ' ';
	char y_sign = (accel[1].val1 == 0 && accel[1].val2 < 0) ? '-' : ' ';
	char z_sign = (accel[2].val1 == 0 && accel[2].val2 < 0) ? '-' : ' ';

	/* Format: "ACCEL:X:1.23,Y:-0.45,Z:9.81\r\n" */
	char buf[64];
	int len = snprintf(buf, sizeof(buf),
			   "ACCEL:X:%c%d.%02d,Y:%c%d.%02d,Z:%c%d.%02d\r\n",
			   x_sign, x_whole, x_dec,
			   y_sign, y_whole, y_dec,
			   z_sign, z_whole, z_dec);

	if (len <= 0 || len >= sizeof(buf)) {
		LOG_ERR("Accel format error");
		return;
	}

	err = bt_nus_send(NULL, (uint8_t *)buf, len);
	if (err) {
		LOG_WRN("BLE send failed (err: %d)", err);
	} else {
		LOG_INF("Sent: %s", buf);
	}
}

/* Accelerometer thread */
static void accel_thread_fn(void)
{
	k_sem_take(&ble_init_ok, K_FOREVER);
	k_sem_give(&ble_init_ok);

	while (1) {
		send_accel_over_ble();
		k_sleep(K_MSEC(ACCEL_INTERVAL_MS));
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
		if (err) LOG_WRN("Failed to set DCD, ret code %d", err);
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
 * BLE callbacks — unchanged
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
	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);
	current_conn = bt_conn_ref(conn);
	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
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

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.recycled     = recycled_cb,
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

	configure_gpio();

	/* Init LIS3DH */
	//accel_dev = DEVICE_DT_GET(DT_NODELABEL(lis3dh));
	if (!device_is_ready(accel_dev)) {
    	LOG_ERR("LIS3DH not ready — check wiring/overlay");
		printk("System booted\n");
    	//accel_dev = NULL;
	} else {
    	LOG_INF("LIS3DH ready");
		printk("LIS3DH ready\n");
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