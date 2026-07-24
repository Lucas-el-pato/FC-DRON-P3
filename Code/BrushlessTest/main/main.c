/*
 * Hobbywing XRotor 45A ESC motor test.
 *
 * Set ESC_TEST_USE_DSHOT to 1 for the DShot300 test (the default), or 0 to
 * use the known-working servo-style PWM test.
 *
 * Wiring:
 *   ESC S1 signal -> ESP32-S3 GPIO4
 *   ESC GND       -> ESP32-S3 GND
 *
 * Remove the propeller before applying power.
 */

#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define ESC_TEST_USE_DSHOT 1

#if ESC_TEST_USE_DSHOT
#include "dshot_esc.h"
#else
#include "pwm_esc.h"
#endif

#define ESC_SIGNAL_GPIO       4
#define ESC_ARM_TIME_MS       5000
#define RAMP_STEP_TIME_MS     2000
#define MAX_TEST_THROTTLE     50

#if ESC_TEST_USE_DSHOT
#define DSHOT_RESOLUTION_HZ   40000000
#define DSHOT_BAUD_RATE       300000
#define DSHOT_POST_DELAY_US   50
#define DSHOT_MAX_SAFE_VALUE  1024
#else
#define ESC_PWM_FREQUENCY_HZ  50
#define ESC_MIN_PULSE_US      1000
#define ESC_MAX_PULSE_US      2000
#endif

static const char *TAG = "brushless_test";

#if ESC_TEST_USE_DSHOT

static uint16_t throttle_to_dshot(uint8_t percent)
{
    if (percent == 0) {
        return 0;
    }
    if (percent > MAX_TEST_THROTTLE) {
        percent = MAX_TEST_THROTTLE;
    }

    /*
     * DShot throttle values 1-47 are commands. Start normal throttle at 48
     * and cap the test at approximately 50% of the full DShot range.
     */
    const uint32_t throttle_span = 2047 - 48;
    uint16_t value = (uint16_t)(48 + (throttle_span * percent) / 100U);
    if (value > DSHOT_MAX_SAFE_VALUE) {
        value = DSHOT_MAX_SAFE_VALUE;
    }
    return value;
}

static void run_dshot_step(uint8_t percent)
{
    uint16_t value = throttle_to_dshot(percent);
    ESP_LOGI(TAG, "DShot throttle %u%% -> value %u", percent, value);
    ESP_ERROR_CHECK(dshot_esc_set_throttle(value));
    vTaskDelay(pdMS_TO_TICKS(RAMP_STEP_TIME_MS));
}

static void run_dshot_test(void)
{
    const dshot_esc_config_t config = {
        .gpio_num = ESC_SIGNAL_GPIO,
        .resolution_hz = DSHOT_RESOLUTION_HZ,
        .baud_rate = DSHOT_BAUD_RATE,
        .post_delay_us = DSHOT_POST_DELAY_US,
    };

    ESP_ERROR_CHECK(dshot_esc_init(&config));

    ESP_LOGI(TAG, "DShot300 armed at value 0 for %d ms", ESC_ARM_TIME_MS);
    ESP_ERROR_CHECK(dshot_esc_stop());
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME_MS));

    /*
     * Start low, then increase. Keep telemetry disabled: this is an infinite
     * loop of frames and telemetry requests on every frame are not valid for
     * this simple test.
     */
    static const uint8_t ramp_up[] = {5, 10, 15, 20, 25, 30, 40, 50};
    static const uint8_t ramp_down[] = {40, 30, 25, 20, 15, 10, 5};

    ESP_LOGI(TAG, "DShot ramp UP");
    for (size_t i = 0; i < sizeof(ramp_up) / sizeof(ramp_up[0]); i++) {
        run_dshot_step(ramp_up[i]);
    }

    ESP_LOGI(TAG, "DShot ramp DOWN");
    for (size_t i = 0; i < sizeof(ramp_down) / sizeof(ramp_down[0]); i++) {
        run_dshot_step(ramp_down[i]);
    }

    ESP_LOGI(TAG, "DShot test complete; returning to value 0");
    ESP_ERROR_CHECK(dshot_esc_stop());
}

#else

static uint16_t throttle_to_pulse_us(uint8_t percent)
{
    return ESC_MIN_PULSE_US +
           (uint16_t)(((ESC_MAX_PULSE_US - ESC_MIN_PULSE_US) * percent) / 100U);
}

static void run_pwm_step(uint8_t percent)
{
    if (percent > MAX_TEST_THROTTLE) {
        percent = MAX_TEST_THROTTLE;
    }

    ESP_LOGI(TAG, "PWM throttle %u%% -> %u us", percent,
             throttle_to_pulse_us(percent));
    ESP_ERROR_CHECK(pwm_esc_set_throttle_percent(percent));
    vTaskDelay(pdMS_TO_TICKS(RAMP_STEP_TIME_MS));
}

static void run_pwm_test(void)
{
    ESP_ERROR_CHECK(pwm_esc_init(ESC_SIGNAL_GPIO, ESC_PWM_FREQUENCY_HZ,
                                 ESC_MIN_PULSE_US, ESC_MAX_PULSE_US));

    ESP_LOGI(TAG, "PWM armed at %d us for %d ms",
             ESC_MIN_PULSE_US, ESC_ARM_TIME_MS);
    ESP_ERROR_CHECK(pwm_esc_stop());
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_TIME_MS));

    static const uint8_t ramp_up[] = {5, 10, 15, 20, 25, 30, 40, 50};
    static const uint8_t ramp_down[] = {40, 30, 25, 20, 15, 10, 5};

    ESP_LOGI(TAG, "PWM ramp UP");
    for (size_t i = 0; i < sizeof(ramp_up) / sizeof(ramp_up[0]); i++) {
        run_pwm_step(ramp_up[i]);
    }

    ESP_LOGI(TAG, "PWM ramp DOWN");
    for (size_t i = 0; i < sizeof(ramp_down) / sizeof(ramp_down[0]); i++) {
        run_pwm_step(ramp_down[i]);
    }

    ESP_LOGI(TAG, "PWM test complete; returning to minimum throttle");
    ESP_ERROR_CHECK(pwm_esc_stop());
}

#endif

void app_main(void)
{
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, " REMOVE THE PROPELLER BEFORE TESTING");
#if ESC_TEST_USE_DSHOT
    ESP_LOGW(TAG, " Hobbywing XRotor 45A DShot300 test");
    ESP_LOGW(TAG, " Signal GPIO%d, 40 MHz RMT, 300 kbaud",
             ESC_SIGNAL_GPIO);
#else
    ESP_LOGW(TAG, " Hobbywing XRotor 45A PWM test");
    ESP_LOGW(TAG, " Signal GPIO%d, %d Hz, %d-%d us",
             ESC_SIGNAL_GPIO, ESC_PWM_FREQUENCY_HZ,
             ESC_MIN_PULSE_US, ESC_MAX_PULSE_US);
#endif
    ESP_LOGW(TAG, " Test throttle capped at %d%%", MAX_TEST_THROTTLE);
    ESP_LOGW(TAG, "========================================");

#if ESC_TEST_USE_DSHOT
    run_dshot_test();
#else
    run_pwm_test();
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
