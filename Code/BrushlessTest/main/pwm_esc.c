#include "pwm_esc.h"

#include <stdbool.h>
#include "driver/ledc.h"

#define PWM_ESC_SPEED_MODE       LEDC_LOW_SPEED_MODE
#define PWM_ESC_TIMER            LEDC_TIMER_0
#define PWM_ESC_CHANNEL          LEDC_CHANNEL_0
#define PWM_ESC_DUTY_RESOLUTION  LEDC_TIMER_14_BIT
#define PWM_ESC_MAX_DUTY         ((1U << 14) - 1U)

static bool s_initialized;
static uint32_t s_frequency_hz;
static uint16_t s_minimum_pulse_us;
static uint16_t s_maximum_pulse_us;

static uint32_t pulse_us_to_duty(uint16_t pulse_us)
{
    uint64_t duty = (uint64_t)pulse_us * s_frequency_hz * PWM_ESC_MAX_DUTY;
    return (uint32_t)(duty / 1000000ULL);
}

esp_err_t pwm_esc_init(int gpio_num, uint32_t frequency_hz,
                       uint16_t minimum_pulse_us, uint16_t maximum_pulse_us)
{
    if (gpio_num < 0 || frequency_hz == 0 ||
        minimum_pulse_us == 0 || maximum_pulse_us <= minimum_pulse_us ||
        ((uint32_t)maximum_pulse_us * frequency_hz) >= 1000000U) {
        return ESP_ERR_INVALID_ARG;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = PWM_ESC_SPEED_MODE,
        .duty_resolution = PWM_ESC_DUTY_RESOLUTION,
        .timer_num = PWM_ESC_TIMER,
        .freq_hz = frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t error = ledc_timer_config(&timer_config);
    if (error != ESP_OK) {
        return error;
    }

    s_frequency_hz = frequency_hz;
    s_minimum_pulse_us = minimum_pulse_us;
    s_maximum_pulse_us = maximum_pulse_us;

    const ledc_channel_config_t channel_config = {
        .gpio_num = gpio_num,
        .speed_mode = PWM_ESC_SPEED_MODE,
        .channel = PWM_ESC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_ESC_TIMER,
        .duty = pulse_us_to_duty(minimum_pulse_us),
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    error = ledc_channel_config(&channel_config);
    if (error != ESP_OK) {
        return error;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t pwm_esc_set_pulse_us(uint16_t pulse_us)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (pulse_us < s_minimum_pulse_us) {
        pulse_us = s_minimum_pulse_us;
    } else if (pulse_us > s_maximum_pulse_us) {
        pulse_us = s_maximum_pulse_us;
    }

    esp_err_t error = ledc_set_duty(PWM_ESC_SPEED_MODE, PWM_ESC_CHANNEL,
                                    pulse_us_to_duty(pulse_us));
    if (error != ESP_OK) {
        return error;
    }
    return ledc_update_duty(PWM_ESC_SPEED_MODE, PWM_ESC_CHANNEL);
}

esp_err_t pwm_esc_set_throttle_percent(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }

    uint32_t range_us = s_maximum_pulse_us - s_minimum_pulse_us;
    uint16_t pulse_us = s_minimum_pulse_us + (uint16_t)(range_us * percent / 100U);
    return pwm_esc_set_pulse_us(pulse_us);
}

esp_err_t pwm_esc_stop(void)
{
    return pwm_esc_set_pulse_us(s_minimum_pulse_us);
}
