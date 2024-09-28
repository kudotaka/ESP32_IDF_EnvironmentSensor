#include "bh1750.h"
#include <string.h>

#define BH1750_ADDR (0x23)
#define BH1750_STANDARD_BAUD (400000)
#define BH1750_TIMEOUT_VALUE_MS (100)

static const char *TAG = "MY-BH1750";

static i2c_master_dev_handle_t dev_bh1750_device_handle;

uint16_t BH1750_GetLUX() {
    esp_err_t ret = ESP_OK;
    uint8_t tempdata[2];
    ret = i2c_master_receive(dev_bh1750_device_handle, tempdata, 2, BH1750_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_receive is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_receive error");
    }

    uint16_t lux;
    lux = tempdata[0] << 8 | tempdata[1];
    return lux / 1.2;
}

esp_err_t BH1750_PowerOn() {
    uint8_t cmd[1] = { BH1750_POWER_ON };
    return i2c_master_transmit(dev_bh1750_device_handle, cmd, 1, BH1750_TIMEOUT_VALUE_MS);
}

esp_err_t BH1750_PowerOff() {
    uint8_t cmd[1] = { BH1750_POWER_DOWN };
    return i2c_master_transmit(dev_bh1750_device_handle, cmd, 1, BH1750_TIMEOUT_VALUE_MS);
}

esp_err_t BH1750_SetMode(uint8_t mode) {
    uint8_t cmd[1] = { mode };
    return i2c_master_transmit(dev_bh1750_device_handle, cmd, 1, BH1750_TIMEOUT_VALUE_MS);
}

esp_err_t BH1750_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_bh1750_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_rm_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_rm_device error");
    }   
    return ret;
}

esp_err_t BH1750_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_bh1750_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BH1750_ADDR,
        .scl_speed_hz = BH1750_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_bh1750_cfg, &dev_bh1750_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    ret = BH1750_PowerOn();
//    BH1750_SetMode(BH1750_CONTINUOUSLY_H_RESOLUTION_MODE);
    return ret;
}
