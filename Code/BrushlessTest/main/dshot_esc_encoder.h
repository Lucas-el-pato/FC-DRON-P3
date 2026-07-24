/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Based on ESP-IDF examples/peripherals/rmt/dshot_esc
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Throttle representation in DShot protocol
 */
typedef struct {
    uint16_t throttle;   /*!< Throttle value (0 = stop, 48-2047 = throttle) */
    bool telemetry_req;  /*!< Telemetry request bit */
} dshot_esc_throttle_t;

/**
 * @brief Type of Dshot ESC encoder configuration
 */
typedef struct {
    uint32_t resolution;    /*!< Encoder resolution, in Hz */
    uint32_t baud_rate;     /*!< e.g. DSHOT300 = 300000 */
    uint32_t post_delay_us; /*!< Delay after one DShot frame, in microseconds */
} dshot_esc_encoder_config_t;

/**
 * @brief Create RMT encoder for encoding DShot ESC frames into RMT symbols
 */
esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
