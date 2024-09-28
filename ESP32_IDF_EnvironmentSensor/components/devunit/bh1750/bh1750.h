#pragma once
// https://github.com/m5stack/M5-DLight
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// CMD
#define BH1750_POWER_DOWN                      (0B00000000)
#define BH1750_POWER_ON                        (0B00000001)
#define BH1750_RESET                           (0B00000111)
#define BH1750_CONTINUOUSLY_H_RESOLUTION_MODE  (0B00010000)
#define BH1750_CONTINUOUSLY_H_RESOLUTION_MODE2 (0B00010001)
#define BH1750_CONTINUOUSLY_L_RESOLUTION_MODE  (0B00010011)
#define BH1750_ONE_TIME_H_RESOLUTION_MODE      (0B00100000)
#define BH1750_ONE_TIME_H_RESOLUTION_MODE2     (0B00100001)
#define BH1750_ONE_TIME_L_RESOLUTION_MODE      (0B00100011)

esp_err_t BH1750_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t BH1750_DeInit();
esp_err_t BH1750_PowerOn();
esp_err_t BH1750_PowerOff();
esp_err_t BH1750_SetMode(uint8_t mode);
uint16_t BH1750_GetLUX();
