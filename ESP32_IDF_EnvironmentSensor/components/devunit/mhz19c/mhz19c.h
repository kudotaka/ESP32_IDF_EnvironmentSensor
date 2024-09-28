#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

esp_err_t Mhz19c_Init(uart_port_t uart_num, gpio_num_t tx, gpio_num_t rx, gpio_num_t rts, gpio_num_t cts, uint32_t baud);
uint32_t Mhz19c_GetCO2Concentration(uart_port_t uart_num);
void Mhz19c_SetAutoCalibration(uart_port_t uart_num, bool mode);

#ifdef __cplusplus
}
#endif
