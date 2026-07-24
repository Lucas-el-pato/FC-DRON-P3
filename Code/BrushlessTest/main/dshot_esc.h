#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DShot ESC output configuration.
 */
typedef struct {
    gpio_num_t gpio_num;
    uint32_t resolution_hz;
    uint32_t baud_rate;
    uint32_t post_delay_us;
} dshot_esc_config_t;

/**
 * @brief Initialize the RMT DShot transmitter.
 */
esp_err_t dshot_esc_init(const dshot_esc_config_t *config);

/**
 * @brief Continuously transmit a throttle value without telemetry requests.
 *
 * DShot values 0-47 are reserved commands. Use 0 for stop and 48-2047
 * for throttle.
 */
esp_err_t dshot_esc_set_throttle(uint16_t throttle);

/**
 * @brief Continuously transmit the stop value.
 */
esp_err_t dshot_esc_stop(void);

#ifdef __cplusplus
}
#endif
