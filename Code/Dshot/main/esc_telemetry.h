#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    uint8_t temperature_c;
    uint16_t voltage_cv;
    uint16_t current_ca;
    uint16_t consumption_mah;
    uint16_t erpm_100;
} esc_telem_t;

esp_err_t esc_telemetry_init(int rx_gpio);
bool esc_telemetry_read(esc_telem_t *out, TickType_t timeout);
uint8_t esc_telem_crc8(const uint8_t *buf, size_t len);
