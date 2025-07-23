// https://github.com/sensidev/sensor-dps310
#pragma once

#include "dps310_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

esp_err_t Dps310_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t Dps310_DeInit();
esp_err_t Dps310_Read();
float Dps310_GetPressure();
float Dps310_GetTemperature();

#ifdef __cplusplus
}
#endif
