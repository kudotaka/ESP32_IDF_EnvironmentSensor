#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

esp_err_t Adt7410_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t Adt7410_DeInit(void);
float Adt7410_getTemperature_13bit(void);

#ifdef __cplusplus
}
#endif
