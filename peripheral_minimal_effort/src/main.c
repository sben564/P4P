#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(accel_sleep_demo);

#define ACCEL_SAMPLE_RATE_HZ 100
#define ACCEL_INTERVAL_MS (1000 / ACCEL_SAMPLE_RATE_HZ)  // 10ms

const struct device *const accel_dev = DEVICE_DT_GET_ANY(st_lis2dh);

K_SEM_DEFINE(wake_sem, 0, 1);
static struct k_timer sample_timer;

/* Timer ISR — just signals the thread, does no work itself.
 * Keeping ISR work minimal means the CPU can go back to sleep
 * faster after handling the wake event. */
static void sample_timer_expiry(struct k_timer *timer)
{
    k_sem_give(&wake_sem);
}

static inline int16_t sv_to_i16(struct sensor_value *v)
{
    return (int16_t)(v->val1 * 100 + v->val2 / 10000);
}

static void accel_sample_thread(void)
{
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("LIS3DH not ready");
        return;
    }

    /* Set ODR once at startup — sensor free-runs internally at its
     * own low current regardless of what the CPU is doing */
    struct sensor_value odr = { .val1 = ACCEL_SAMPLE_RATE_HZ, .val2 = 0 };
    sensor_attr_set(accel_dev, SENSOR_CHAN_ACCEL_XYZ,
                     SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

    k_timer_init(&sample_timer, sample_timer_expiry, NULL);
    k_timer_start(&sample_timer, K_MSEC(ACCEL_INTERVAL_MS), K_MSEC(ACCEL_INTERVAL_MS));

    struct sensor_value accel[3];

    while (1) {
        /* CPU sleeps here — k_sem_take with K_FOREVER blocks this
         * thread, Zephyr's idle thread takes over and calls
         * k_cpu_idle(), entering System ON sleep until the timer
         * ISR above fires and gives the semaphore */
        k_sem_take(&wake_sem, K_FOREVER);

        /* Woken up — do the actual I2C read now */
        if (sensor_sample_fetch(accel_dev) == 0 &&
            sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel) == 0) {

            int16_t z = sv_to_i16(&accel[2]);
            //LOG_INF("Z: %d", z);

            /* ... pack into your BLE packet / buffer here ... */
        }

        /* Falls back to k_sem_take → CPU sleeps again until next tick */
    }
}

#define ACCEL_THREAD_STACK_SIZE 1024
#define ACCEL_THREAD_PRIORITY   8

K_THREAD_DEFINE(accel_sample_thread_id,
                ACCEL_THREAD_STACK_SIZE,
                accel_sample_thread,
                NULL, NULL, NULL,
                ACCEL_THREAD_PRIORITY, 0, 0);