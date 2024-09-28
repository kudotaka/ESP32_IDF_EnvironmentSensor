#include "freertos/FreeRTOS.h"
#include "adt7410.h"

#define ADT7410_ADDR (0x48)

#define ADT7410_REG_TEMPDATA        (0x00)
#define ADT7410_REG_CONFIG          (0x03)
#define ADT7410_REG_ID              (0x08)
#define ADT7410_CONF_READ_13BIT        (0x00)
#define ADT7410_CONF_READ_16BIT        (0x00)

#define ADT7410_STANDARD_BAUD (400000)
#define ADT7410_TIMEOUT_VALUE_MS (100)

static const char *TAG = "MY-ADT7410";

static i2c_master_dev_handle_t dev_adt7410_device_handle;

float Adt7410_getTemperature_13bit(void) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1];
    cmd[0] = ADT7410_CONF_READ_13BIT;
    ret = i2c_master_transmit(dev_adt7410_device_handle, cmd, 1, ADT7410_TIMEOUT_VALUE_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Adt7410_getTemperature_13bit write error!");
        return 0;
    }
    vTaskDelay( pdMS_TO_TICKS(100) );

    uint8_t tempdata[2] = {0};
    uint16_t tempvalue = 0;
    int32_t inttempvalue = 0;
    cmd[0] = ADT7410_REG_TEMPDATA;
    ret = i2c_master_transmit_receive(dev_adt7410_device_handle, cmd, 1, tempdata, 2, ADT7410_TIMEOUT_VALUE_MS);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Adt7410_getTemperature_13bit read error!");
        return 0;
    }

    tempvalue = (uint16_t)( tempdata[0] << 8 ) | ( tempdata[1] );
    tempvalue = tempvalue >> 3;

    if (tempvalue & 0x1000) {
        inttempvalue = tempvalue - 0x2000;
    } else {
        inttempvalue = tempvalue;
    }

    return inttempvalue / 16.0;
}

esp_err_t Adt7410_getID() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1] = {0};
    uint8_t tempdata[1] = {0};
    cmd[0] = ADT7410_REG_ID;
    ret = i2c_master_transmit_receive(dev_adt7410_device_handle, cmd, 1, tempdata, 1, ADT7410_TIMEOUT_VALUE_MS);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Adt7410_getID read error!");
        return ret;
    }

    return ret;
}

esp_err_t Adt7410_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_adt7410_device_handle);
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

esp_err_t Adt7410_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    ESP_LOGI(TAG, "Adt7410_Init Init()");
    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_adt7410_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADT7410_ADDR,
        .scl_speed_hz = ADT7410_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_adt7410_cfg, &dev_adt7410_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    ret = Adt7410_getID();
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Adt7410_getID is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C Adt7410_getID error");
    }

    return ret;
}
