#include "esc_telemetry.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"

#define ESC_TELEM_UART_NUM      UART_NUM_1
#define ESC_TELEM_BAUD_RATE     115200
#define ESC_TELEM_FRAME_LEN     10
#define ESC_TELEM_UART_BUF_SIZE 256

static bool s_uart_ready = false;

uint8_t esc_telem_crc8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

esp_err_t esc_telemetry_init(int rx_gpio)
{
    const uart_config_t uart_config = {
        .baud_rate = ESC_TELEM_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(ESC_TELEM_UART_NUM, ESC_TELEM_UART_BUF_SIZE, 0, 0, NULL, 0),
                        "esc_telemetry", "uart install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(ESC_TELEM_UART_NUM, &uart_config),
                        "esc_telemetry", "uart config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(ESC_TELEM_UART_NUM, UART_PIN_NO_CHANGE, rx_gpio,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        "esc_telemetry", "uart pin failed");

    uart_flush_input(ESC_TELEM_UART_NUM);
    s_uart_ready = true;
    return ESP_OK;
}

static bool esc_telemetry_parse_frame(const uint8_t *frame, esc_telem_t *out)
{
    if (esc_telem_crc8(frame, ESC_TELEM_FRAME_LEN - 1) != frame[ESC_TELEM_FRAME_LEN - 1]) {
        return false;
    }

    out->temperature_c = frame[0];
    out->voltage_cv = ((uint16_t)frame[1] << 8) | frame[2];
    out->current_ca = ((uint16_t)frame[3] << 8) | frame[4];
    out->consumption_mah = ((uint16_t)frame[5] << 8) | frame[6];
    out->erpm_100 = ((uint16_t)frame[7] << 8) | frame[8];
    return true;
}

bool esc_telemetry_read(esc_telem_t *out, TickType_t timeout)
{
    if (!s_uart_ready || out == NULL) {
        return false;
    }

    uint8_t frame[ESC_TELEM_FRAME_LEN];
    size_t index = 0;
    const TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout) {
        uint8_t byte = 0;
        const int read = uart_read_bytes(ESC_TELEM_UART_NUM, &byte, 1, pdMS_TO_TICKS(10));
        if (read <= 0) {
            continue;
        }

        if (index == 0 && byte == 0xFFU) {
            continue;
        }

        frame[index++] = byte;
        if (index < ESC_TELEM_FRAME_LEN) {
            continue;
        }

        if (esc_telemetry_parse_frame(frame, out)) {
            return true;
        }

        memmove(frame, frame + 1, ESC_TELEM_FRAME_LEN - 1);
        index = ESC_TELEM_FRAME_LEN - 1;
    }

    return false;
}
