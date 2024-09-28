#pragma once

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "sk6812_encoder.h"

#define SK6812_COLOR_OFF     0x000000
#define SK6812_COLOR_BLACK   0x000000
#define SK6812_COLOR_BLUE    0x0000FF
#define SK6812_COLOR_LIME    0x00FF00
#define SK6812_COLOR_AQUA    0x00FFFF
#define SK6812_COLOR_RED     0xFF0000
#define SK6812_COLOR_MAGENTA 0xFF00FF
#define SK6812_COLOR_YELLOW  0xFFFF00
#define SK6812_COLOR_WHITE   0xFFFFFF

typedef struct pixel_settings {
	uint8_t *pixels;		// buffer containing pixel values, 3 (RGB) or 4 (RGBW) bytes per pixel
	uint8_t pixel_count;	// number of used pixels
	uint8_t brightness;		// brightness factor applied to pixel color
	char color_order[5];
	uint8_t nbits;			// number of bits used (24 for RGB devices, 32 for RGBW devices)
} pixel_settings_t;


esp_err_t Sk6812_Init(gpio_num_t pin, rmt_channel_handle_t *channel, rmt_encoder_handle_t *encoder);
esp_err_t Sk6812_Enable(rmt_channel_handle_t channel);
void Sk6812_Deinit(rmt_channel_handle_t channel);
esp_err_t Sk6812_Show(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder, const void *payload, size_t payload_bytes);
esp_err_t Sk6812_Clear(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder, const void *payload, size_t payload_bytes);

esp_err_t Sk6812_Init_Ex(pixel_settings_t *px, uint8_t pixelCount, gpio_num_t pin, rmt_channel_handle_t *channel, rmt_encoder_handle_t *encoder);
esp_err_t Sk6812_Enable_Ex(pixel_settings_t *px, rmt_channel_handle_t channel);
void Sk6812_Deinit_Ex(pixel_settings_t *px, rmt_channel_handle_t channel);
esp_err_t Sk6812_Show_Ex(pixel_settings_t *px, rmt_channel_handle_t channel, rmt_encoder_handle_t encoder);
esp_err_t Sk6812_Clear_Ex(pixel_settings_t *px, rmt_channel_handle_t channel, rmt_encoder_handle_t encoder);
void Sk6812_SetColor_Ex(pixel_settings_t *px, uint16_t pos, uint32_t color);
void Sk6812_SetAllColor_Ex(pixel_settings_t *px, uint32_t color);
void Sk6812_SetBrightness_Ex(pixel_settings_t *px, uint8_t brightness);

void np_set_pixel_color(pixel_settings_t *px, uint16_t idx, uint32_t color);
