#include <stdio.h>

#include "dshot600.h"
#include "dshot_config.h"
#include "esc_telemetry.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "esc_test";

#define TEST_ESC_INDEX      0
/* DShot throttle span is 48–2047; remove props for bench tests. */
#define TEST_MIN_THROTTLE   100
#define TEST_MAX_THROTTLE   300
#define ARM_DURATION_MS     1000
#define RAMP_UP_MS          500
#define RAMP_HOLD_MS        1000
#define RAMP_DOWN_MS        500
#define TELEM_LOG_INTERVAL  pdMS_TO_TICKS(200)
/* Below max priority so IDLE/WDT stay healthy; core 1 keeps UART work on core 0. */
#define DSHOT_TX_TASK_PRIO      10
#define DSHOT_TX_TASK_CORE      1

static dshot600_handle_t s_esc[DSHOT_ESC_COUNT];
static volatile uint16_t s_throttle[DSHOT_ESC_COUNT];
static volatile int s_telem_rr;
static volatile int s_last_telem_esc = -1;

static void dshot_tx_task(void *arg)
{
    (void)arg;

    while (true) {
        const int telem_index = s_telem_rr;
        s_last_telem_esc = telem_index;

        for (int i = 0; i < DSHOT_ESC_COUNT; i++) {
            const bool telem_req = (i == telem_index);
            (void)dshot600_send(s_esc[i], s_throttle[i], telem_req);
            taskYIELD();
        }

        s_telem_rr = (telem_index + 1) % DSHOT_ESC_COUNT;
        vTaskDelay(1);
    }
}

static void set_all_throttles(uint16_t value)
{
    for (int i = 0; i < DSHOT_ESC_COUNT; i++) {
        s_throttle[i] = value;
    }
}

static void log_telemetry_if_available(void)
{
    esc_telem_t telem = {0};
    if (!esc_telemetry_read(&telem, pdMS_TO_TICKS(5))) {
        return;
    }

    const float voltage_v = telem.voltage_cv / 100.0f;
    const float current_a = telem.current_ca / 100.0f;
    const uint32_t erpm = (uint32_t)telem.erpm_100 * 100U;

    ESP_LOGI(TAG,
             "Telem ESC%u: temp=%u C  V=%.2f V  I=%.2f A  mAh=%u  eRPM=%lu",
             (unsigned)(s_last_telem_esc + 1U),
             telem.temperature_c,
             voltage_v,
             current_a,
             telem.consumption_mah,
             (unsigned long)erpm);
}

static void wait_with_telemetry_logging(TickType_t duration)
{
    const TickType_t end = xTaskGetTickCount() + duration;
    TickType_t last_log = xTaskGetTickCount();

    while ((int32_t)(end - xTaskGetTickCount()) > 0) {
        if ((xTaskGetTickCount() - last_log) >= TELEM_LOG_INTERVAL) {
            log_telemetry_if_available();
            last_log = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void run_throttle_ramp(void)
{
    ESP_LOGI(TAG, "Minimum throttle hold on ESC%u (DShot %u)", TEST_ESC_INDEX + 1U, TEST_MAX_THROTTLE);

    const uint16_t start = TEST_MIN_THROTTLE;
    const uint16_t end = TEST_MAX_THROTTLE;
    const int up_steps = RAMP_UP_MS;
    const int down_steps = RAMP_DOWN_MS;

    for (int step = 0; step <= up_steps; step++) {
        s_throttle[TEST_ESC_INDEX] = start + ((end - start) * step) / up_steps;
        wait_with_telemetry_logging(pdMS_TO_TICKS(1));
    }

    wait_with_telemetry_logging(pdMS_TO_TICKS(RAMP_HOLD_MS));

    for (int step = down_steps; step >= 0; step--) {
        s_throttle[TEST_ESC_INDEX] = start + ((end - start) * step) / down_steps;
        wait_with_telemetry_logging(pdMS_TO_TICKS(1));
    }

    s_throttle[TEST_ESC_INDEX] = DSHOT_CMD_MOTOR_STOP;
    ESP_LOGI(TAG, "Ramp complete, ESC%u idle", TEST_ESC_INDEX + 1U);
}

void app_main(void)
{
    const int esc_gpios[DSHOT_ESC_COUNT] = {
        DSHOT_ESC1_GPIO,
        DSHOT_ESC2_GPIO,
        DSHOT_ESC3_GPIO,
        DSHOT_ESC4_GPIO,
    };

    ESP_LOGI(TAG, "HobbyWing XRotor 45A FPV 4in1 DShot600 bench test");
    ESP_LOGI(TAG, "ESC pins: %d,%d,%d,%d  telem: %d",
             esc_gpios[0], esc_gpios[1], esc_gpios[2], esc_gpios[3], DSHOT_TELEM_GPIO);

    for (int i = 0; i < DSHOT_ESC_COUNT; i++) {
        ESP_ERROR_CHECK(dshot600_new(esc_gpios[i], &s_esc[i]));
    }
    ESP_ERROR_CHECK(esc_telemetry_init(DSHOT_TELEM_GPIO));

    set_all_throttles(DSHOT_CMD_MOTOR_STOP);
    s_telem_rr = 0;

    xTaskCreatePinnedToCore(dshot_tx_task, "dshot_tx", 4096, NULL, DSHOT_TX_TASK_PRIO, NULL,
                            DSHOT_TX_TASK_CORE);

    ESP_LOGI(TAG, "Arming all ESC outputs for %u ms", ARM_DURATION_MS);
    wait_with_telemetry_logging(pdMS_TO_TICKS(ARM_DURATION_MS));

    while (true) {
        run_throttle_ramp();
        wait_with_telemetry_logging(pdMS_TO_TICKS(1000));
    }
}
