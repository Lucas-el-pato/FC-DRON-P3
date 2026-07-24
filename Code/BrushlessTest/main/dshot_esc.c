#include "dshot_esc.h"

#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

static rmt_channel_handle_t s_channel;
static rmt_encoder_handle_t s_encoder;
static dshot_esc_throttle_t s_frame = {
    .throttle = 0,
    .telemetry_req = false,
};
static bool s_initialized;
static bool s_loop_running;

esp_err_t dshot_esc_init(const dshot_esc_config_t *config)
{
    if (config == NULL || config->gpio_num < 0 ||
        config->resolution_hz == 0 || config->baud_rate == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const rmt_tx_channel_config_t channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = config->resolution_hz,
        /* Match Espressif's example; one infinite-loop transaction is being
         * replaced by the next update. */
        .trans_queue_depth = 10,
    };
    esp_err_t error = rmt_new_tx_channel(&channel_config, &s_channel);
    if (error != ESP_OK) {
        return error;
    }

    const dshot_esc_encoder_config_t encoder_config = {
        .resolution = config->resolution_hz,
        .baud_rate = config->baud_rate,
        .post_delay_us = config->post_delay_us,
    };
    error = rmt_new_dshot_esc_encoder(&encoder_config, &s_encoder);
    if (error != ESP_OK) {
        rmt_del_channel(s_channel);
        s_channel = NULL;
        return error;
    }

    error = rmt_enable(s_channel);
    if (error != ESP_OK) {
        rmt_del_encoder(s_encoder);
        rmt_del_channel(s_channel);
        s_encoder = NULL;
        s_channel = NULL;
        return error;
    }

    s_initialized = true;
    s_loop_running = false;
    return ESP_OK;
}

esp_err_t dshot_esc_set_throttle(uint16_t throttle)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (throttle > 2047) {
        throttle = 2047;
    }

    /*
     * Keep telemetry disabled for this motor test. The RMT transaction loops
     * forever, so setting telemetry_req would request telemetry on every
     * DShot packet.
     */
    s_frame.throttle = throttle;
    s_frame.telemetry_req = false;

    const rmt_transmit_config_t transmit_config = {
        .loop_count = -1,
    };

    /*
     * Matches Espressif's dshot_esc example. On the first call the channel
     * is idle, so rmt_transmit() starts the infinite loop directly and it
     * must be left running: rmt_disable() would kill the loop we just
     * started and rmt_enable() would find nothing pending, leaving the pin
     * silent. On subsequent calls a loop is already running, so the new
     * frame only gets queued; disable/enable stops the old loop and starts
     * the queued one.
     */
    esp_err_t error = rmt_transmit(s_channel, s_encoder, &s_frame,
                                   sizeof(s_frame), &transmit_config);
    if (error != ESP_OK) {
        return error;
    }

    if (!s_loop_running) {
        s_loop_running = true;
        return ESP_OK;
    }

    error = rmt_disable(s_channel);
    if (error != ESP_OK) {
        return error;
    }
    return rmt_enable(s_channel);
}

esp_err_t dshot_esc_stop(void)
{
    return dshot_esc_set_throttle(0);
}
