#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "sk6812_encoder.h"
#include "sk6812.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000

/* ===================================================================================================*/
/* --------------------------------------------- SK6812 ----------------------------------------------*/

esp_err_t Sk6812_Init(gpio_num_t pin, rmt_channel_handle_t *channel, rmt_encoder_handle_t *encoder) {
	esp_err_t res = ESP_OK;
    rmt_tx_channel_config_t tx_chan_config = {                                                
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = pin,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background                                           
    };
	res = rmt_new_tx_channel(&tx_chan_config, channel);
	if (res != ESP_OK) {
		return res;
	}

	sk6812_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
	res = rmt_new_sk6812_encoder(&encoder_config, encoder);
	if (res != ESP_OK) {
		return res;
	}

	return res;
}

esp_err_t Sk6812_Init_Ex(pixel_settings_t *px, uint8_t pixelCount, gpio_num_t pin, rmt_channel_handle_t *channel, rmt_encoder_handle_t *encoder) {
	px->pixel_count = pixelCount;
	px->brightness = 10;
	sprintf(px->color_order, "GRBW");
	px->nbits = 24;
	px->pixels = (uint8_t *)malloc((px->nbits / 8) * px->pixel_count);
	return Sk6812_Init(pin, channel, encoder);
}

esp_err_t Sk6812_Enable(rmt_channel_handle_t channel)
{
	esp_err_t res = ESP_OK;
	res = rmt_enable(channel);

	return res;
}
esp_err_t Sk6812_Enable_Ex(pixel_settings_t *px, rmt_channel_handle_t channel)
{
	return Sk6812_Enable(channel);
}

void Sk6812_Deinit(rmt_channel_handle_t channel)
{
	rmt_del_channel(channel);
}

void Sk6812_Deinit_Ex(pixel_settings_t *px, rmt_channel_handle_t channel)
{
	rmt_del_channel(channel);
}

void Sk6812_SetColor_Ex(pixel_settings_t *px, uint16_t pos, uint32_t color) {
    np_set_pixel_color(px, pos, color << 8);
}

void Sk6812_SetAllColor_Ex(pixel_settings_t *px, uint32_t color) {
    for (uint8_t i = 0; i < px->pixel_count; i++) {
        np_set_pixel_color(px, i, color << 8);
    }
}

void Sk6812_SetBrightness_Ex(pixel_settings_t *px, uint8_t brightness) {
    px->brightness = brightness;
}

esp_err_t Sk6812_Show(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder, const void *payload, size_t payload_bytes) {
	esp_err_t res = ESP_OK;
	rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
	res = rmt_transmit(channel, encoder, payload, payload_bytes, &tx_config);
	if (res != ESP_OK) {
		return res;
	}
	res = rmt_tx_wait_all_done(channel, portMAX_DELAY);
	if (res != ESP_OK) {
		return res;
	}

	return res;
}

esp_err_t Sk6812_Clear(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder, const void *payload, size_t payload_bytes)
{
	esp_err_t res = ESP_OK;
	rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
	res = rmt_transmit(channel, encoder, payload, payload_bytes, &tx_config);
	if (res != ESP_OK) {
		return res;
	}
	res = rmt_tx_wait_all_done(channel, portMAX_DELAY);
	if (res != ESP_OK) {
		return res;
	}

	return res;
}

esp_err_t Sk6812_Show_Ex(pixel_settings_t *px, rmt_channel_handle_t channel, rmt_encoder_handle_t encoder) {
	return Sk6812_Show(channel, encoder, px->pixels, (px->nbits / 8) * px->pixel_count);
}

esp_err_t Sk6812_Clear_Ex(pixel_settings_t *px, rmt_channel_handle_t channel, rmt_encoder_handle_t encoder) {
	memset(px->pixels, 0, (px->nbits / 8) * px->pixel_count);
	return Sk6812_Clear(channel, encoder, px->pixels, (px->nbits / 8) * px->pixel_count);
}

/* ----------------------------------------------- End -----------------------------------------------*/
/* ===================================================================================================*/

// Get color value of RGB component
//---------------------------------------------------
static uint8_t offset_color_withBrightness(char o, uint32_t color, uint8_t brightness) {
	float _brightness = (float)((float)brightness / 255.0);
	uint8_t clr = 0;
	switch(o) {
		case 'R':
			clr = (uint8_t)(color >> 24);//(uint8_t)(red * 255.0) << 16)
			break;
		case 'G':
			clr = (uint8_t)(color >> 16);
			break;
		case 'B':
			clr = (uint8_t)(color >> 8);
			break;
		case 'W':
			clr = (uint8_t)(color & 0xFF);
			break;
		default:
			clr = 0;
	}
	return (uint8_t)(clr * _brightness);
}

// Set pixel color at buffer position from RGB color value
//=========================================================================
void np_set_pixel_color(pixel_settings_t *px, uint16_t idx, uint32_t color) {
	uint16_t ofs = idx * (px->nbits / 8);
	px->pixels[ofs] = offset_color_withBrightness(px->color_order[0], color, px->brightness);
	px->pixels[ofs+1] = offset_color_withBrightness(px->color_order[1], color, px->brightness);
	px->pixels[ofs+2] = offset_color_withBrightness(px->color_order[2], color, px->brightness);
	if (px->nbits == 32) px->pixels[ofs+3] = offset_color_withBrightness(px->color_order[3], color, px->brightness);
}
