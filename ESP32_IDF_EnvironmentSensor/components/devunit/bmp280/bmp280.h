// https://github.com/Seeed-Studio/Grove_BMP280/blob/master/Seeed_BMP280.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

esp_err_t Bmp280_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t Bmp280_DeInit();
float Bmp280_getTemperature(void);
float Bmp280_getPressure(void);
float Bmp280_calcAltitude1(float p0);
float Bmp280_calcAltitude2(float p0, float p1, float t);

#ifdef __cplusplus
}
#endif
