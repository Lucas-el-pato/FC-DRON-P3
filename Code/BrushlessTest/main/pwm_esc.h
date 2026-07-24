#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure servo-style PWM output for an ESC.
 *
 * @param gpio_num Output GPIO connected to the ESC signal input.
 * @param frequency_hz PWM refresh rate, normally 50 Hz.
 * @param minimum_pulse_us Pulse width representing stopped throttle.
 * @param maximum_pulse_us Pulse width representing full throttle.
 */
esp_err_t pwm_esc_init(int gpio_num, uint32_t frequency_hz,
                       uint16_t minimum_pulse_us, uint16_t maximum_pulse_us);

/**
 * @brief Set an exact ESC pulse width, clamped to the configured range.
 */
esp_err_t pwm_esc_set_pulse_us(uint16_t pulse_us);

/**
 * @brief Set throttle from 0 to 100 percent.
 */
esp_err_t pwm_esc_set_throttle_percent(uint8_t percent);

/**
 * @brief Set the output to the configured minimum pulse.
 */
esp_err_t pwm_esc_stop(void);

#ifdef __cplusplus
}
#endif
