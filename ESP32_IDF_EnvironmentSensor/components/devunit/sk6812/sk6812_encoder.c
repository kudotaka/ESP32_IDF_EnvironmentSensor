#include "esp_check.h"
#include "sk6812_encoder.h"

static const char *TAG = "sk6812_encoder";

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_sk6812_encoder_t;

static size_t rmt_encode_sk6812(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_sk6812_encoder_t *sk6812_encoder = __containerof(encoder, rmt_sk6812_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = sk6812_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = sk6812_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    switch (sk6812_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            sk6812_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &sk6812_encoder->reset_code,
                                                sizeof(sk6812_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            sk6812_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_sk6812_encoder(rmt_encoder_t *encoder)
{
    rmt_sk6812_encoder_t *sk6812_encoder = __containerof(encoder, rmt_sk6812_encoder_t, base);
    rmt_del_encoder(sk6812_encoder->bytes_encoder);
    rmt_del_encoder(sk6812_encoder->copy_encoder);
    free(sk6812_encoder);
    return ESP_OK;
}

static esp_err_t rmt_sk6812_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_sk6812_encoder_t *sk6812_encoder = __containerof(encoder, rmt_sk6812_encoder_t, base);
    rmt_encoder_reset(sk6812_encoder->bytes_encoder);
    rmt_encoder_reset(sk6812_encoder->copy_encoder);
    sk6812_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_sk6812_encoder(const sk6812_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_sk6812_encoder_t *sk6812_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    sk6812_encoder = calloc(1, sizeof(rmt_sk6812_encoder_t));
    ESP_GOTO_ON_FALSE(sk6812_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for sk6812 encoder");
    sk6812_encoder->base.encode = rmt_encode_sk6812;
    sk6812_encoder->base.del = rmt_del_sk6812_encoder;
    sk6812_encoder->base.reset = rmt_sk6812_encoder_reset;
    // different led strip might have its own timing requirements, following parameter is for WS2812
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 0.3 * config->resolution / 1000000, // T0H=0.3us
            .level1 = 0,
            .duration1 = 0.9 * config->resolution / 1000000, // T0L=0.9us
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 0.9 * config->resolution / 1000000, // T1H=0.9us
            .level1 = 0,
            .duration1 = 0.3 * config->resolution / 1000000, // T1L=0.3us
        },
        .flags.msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &sk6812_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &sk6812_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    uint32_t reset_ticks = config->resolution / 1000000 * 50 / 2; // reset code duration defaults to 50us
    sk6812_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    *ret_encoder = &sk6812_encoder->base;
    return ESP_OK;
err:
    if (sk6812_encoder) {
        if (sk6812_encoder->bytes_encoder) {
            rmt_del_encoder(sk6812_encoder->bytes_encoder);
        }
        if (sk6812_encoder->copy_encoder) {
            rmt_del_encoder(sk6812_encoder->copy_encoder);
        }
        free(sk6812_encoder);
    }
    return ret;
}
