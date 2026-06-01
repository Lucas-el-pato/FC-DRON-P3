#include "dshot600.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#define DSHOT600_RESOLUTION_HZ 40000000
#define DSHOT600_FRAME_BITS    16
/* One RMT mem block per ESC (48 words); 64 would reserve 2 blocks and only 2 ESCs fit on S3. */
#define DSHOT600_MEM_BLOCK_SYMBOLS SOC_RMT_MEM_WORDS_PER_CHANNEL

#define DSHOT600_BIT1_HIGH_TICKS 50
#define DSHOT600_BIT1_LOW_TICKS  17
#define DSHOT600_BIT0_HIGH_TICKS 25
#define DSHOT600_BIT0_LOW_TICKS  42

typedef struct dshot600_ctx {
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    rmt_symbol_word_t bit0;
    rmt_symbol_word_t bit1;
    rmt_symbol_word_t frame[DSHOT600_FRAME_BITS];
} dshot600_ctx_t;

static uint16_t dshot600_build_packet(uint16_t value, bool telemetry_request)
{
    uint16_t packet = (value << 1) | (telemetry_request ? 1U : 0U);
    uint16_t csum = (packet >> 4) ^ (packet >> 8) ^ (packet >> 12);
    csum &= 0x0FU;
    return (packet << 4) | csum;
}

static void dshot600_build_frame(dshot600_ctx_t *ctx, uint16_t value, bool telemetry_request)
{
    const uint16_t packet = dshot600_build_packet(value, telemetry_request);

    for (int i = 0; i < DSHOT600_FRAME_BITS; i++) {
        const bool bit = (packet >> (DSHOT600_FRAME_BITS - 1 - i)) & 0x01U;
        ctx->frame[i] = bit ? ctx->bit1 : ctx->bit0;
    }
}

esp_err_t dshot600_new(int gpio, dshot600_handle_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, "dshot600", "out is NULL");

    dshot600_ctx_t *ctx = calloc(1, sizeof(dshot600_ctx_t));
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_NO_MEM, "dshot600", "no memory");

    ctx->bit1 = (rmt_symbol_word_t) {
        .level0 = 1,
        .duration0 = DSHOT600_BIT1_HIGH_TICKS,
        .level1 = 0,
        .duration1 = DSHOT600_BIT1_LOW_TICKS,
    };
    ctx->bit0 = (rmt_symbol_word_t) {
        .level0 = 1,
        .duration0 = DSHOT600_BIT0_HIGH_TICKS,
        .level1 = 0,
        .duration1 = DSHOT600_BIT0_LOW_TICKS,
    };

    const rmt_tx_channel_config_t tx_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DSHOT600_RESOLUTION_HZ,
        .mem_block_symbols = DSHOT600_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {
            .invert_out = false,
            .with_dma = false,
            .allow_pd = false,
            .init_level = 0,
        },
    };

    esp_err_t err = rmt_new_tx_channel(&tx_config, &ctx->channel);
    if (err != ESP_OK) {
        free(ctx);
        return err;
    }

    const rmt_copy_encoder_config_t copy_config = {};
    err = rmt_new_copy_encoder(&copy_config, &ctx->encoder);
    if (err != ESP_OK) {
        rmt_del_channel(ctx->channel);
        free(ctx);
        return err;
    }

    err = rmt_enable(ctx->channel);
    if (err != ESP_OK) {
        rmt_del_encoder(ctx->encoder);
        rmt_del_channel(ctx->channel);
        free(ctx);
        return err;
    }

    *out = ctx;
    return ESP_OK;
}

static esp_err_t dshot600_transmit(dshot600_ctx_t *ctx, uint16_t value, bool telemetry_request)
{
    dshot600_build_frame(ctx, value, telemetry_request);

    const rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = true,
        },
    };

    esp_err_t err = rmt_transmit(ctx->channel, ctx->encoder, ctx->frame,
                                 sizeof(ctx->frame), &tx_cfg);
    if (err != ESP_OK) {
        (void)rmt_tx_wait_all_done(ctx->channel, 0);
        err = rmt_transmit(ctx->channel, ctx->encoder, ctx->frame,
                           sizeof(ctx->frame), &tx_cfg);
    }

    return err;
}

esp_err_t dshot600_send(dshot600_handle_t handle, uint16_t throttle, bool telemetry_request)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, "dshot600", "invalid handle");
    ESP_RETURN_ON_FALSE(throttle == DSHOT_CMD_MOTOR_STOP ||
                        (throttle >= DSHOT_MIN_THROTTLE && throttle <= DSHOT_MAX_THROTTLE),
                        ESP_ERR_INVALID_ARG, "dshot600",
                        "invalid throttle (use dshot600_send_command for values 1-47)");

    return dshot600_transmit(handle, throttle, telemetry_request);
}

esp_err_t dshot600_send_command(dshot600_handle_t handle, uint16_t command, bool telemetry_request)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, "dshot600", "invalid handle");
    ESP_RETURN_ON_FALSE(command <= 47, ESP_ERR_INVALID_ARG, "dshot600", "invalid command");

    return dshot600_transmit(handle, command, telemetry_request);
}

esp_err_t dshot600_arm(dshot600_handle_t handle, uint32_t duration_ms)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, "dshot600", "invalid handle");

    const TickType_t end_tick = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
    while ((int32_t)(end_tick - xTaskGetTickCount()) > 0) {
        ESP_RETURN_ON_ERROR(dshot600_send(handle, DSHOT_CMD_MOTOR_STOP, false), "dshot600", "arm frame failed");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

void dshot600_delete(dshot600_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    dshot600_ctx_t *ctx = handle;
    (void)rmt_disable(ctx->channel);
    (void)rmt_del_encoder(ctx->encoder);
    (void)rmt_del_channel(ctx->channel);
    free(ctx);
}
