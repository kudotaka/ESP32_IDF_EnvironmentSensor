// https://github.com/m5stack/M5Unit-ENV/blob/master/src/QMP6988.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

esp_err_t Qmp6988_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t Qmp6988_DeInit();
esp_err_t Qmp6988_GetChipID();
float Qmp6988_CalcPressure();
float Qmp6988_calcTemperature();

#ifdef __cplusplus
}
#endif
