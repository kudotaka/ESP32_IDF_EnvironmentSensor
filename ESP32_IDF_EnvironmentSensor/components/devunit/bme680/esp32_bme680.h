// https://github.com/Seeed-Studio/Seeed_Arduino_BME68x
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "bme680.h"

esp_err_t Esp32_Bme680_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t Esp32_Bme680_DeInit(void);
int8_t Esp32_Bme680_read_sensor_data(void);
float Esp32_Bme680_get_temperature(void);
float Esp32_Bme680_get_pressure(void);
float Esp32_Bme680_get_humidity(void);
float Esp32_Bme680_get_gas(void);

#ifdef __cplusplus
}
#endif
