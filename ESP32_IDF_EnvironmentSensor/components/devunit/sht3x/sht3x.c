// https://github.com/m5stack/M5Unit-ENV/blob/master/src/SHT3X.cpp
#include "freertos/FreeRTOS.h"
#include "sht3x.h"

#define SHT3X_ADDR (0x44)

#define SHT3X_STANDARD_BAUD (400000)
#define SHT3X_TIMEOUT_VALUE_MS (100)
#define Sht3x_COMMAND_MEASURE {0x2C, 0x06}
static i2c_master_dev_handle_t dev_sht3x_device_handle;

static const char *TAG = "MY-SHT3X";

static float temperature = 0.0;
static float humidity = 0.0;

static esp_err_t I2CWrite(uint8_t* cmd, uint8_t len) {
    esp_err_t ret = ESP_OK;

    ret = i2c_master_transmit(dev_sht3x_device_handle, cmd, len, SHT3X_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "I2CWrite i2c_master_transmit(1) is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2CWrite I2C i2c_master_transmit(1) error");
    }
    return ret;
}

static esp_err_t I2CRead(uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;

    ret = i2c_master_receive(dev_sht3x_device_handle, buf, len, SHT3X_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "I2CRead i2c_master_transmit_receive is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2CRead I2C i2c_master_transmit_receive error");
    }
    return ret;
}

/*
static esp_err_t I2CRead(uint8_t* cmd, uint8_t len1, uint8_t* buf, uint8_t len2) {
    esp_err_t ret = ESP_OK;

    ret = i2c_master_transmit_receive(dev_sht3x_device_handle, cmd, len1, buf, len2, SHT3X_TIMEOUT_VALUE_MS);
//    ret = i2c_master_receive(dev_sht3x_device_handle, buf, len, SHT3X_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "I2CRead i2c_master_transmit_receive is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2CRead I2C i2c_master_transmit_receive error");
    }
    return ret;
}
*/

float Sht3x_GetTemperature() {
    return temperature;
}

int32_t Sht3x_GetIntTemperature() {
    return (int32_t)temperature;
}

float Sht3x_GetHumidity() {
    return humidity;
}

int32_t Sht3x_GetIntHumidity() {
    return (int32_t)humidity;
}

esp_err_t Sht3x_Read() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {0x2C, 0x06};
    uint8_t tempdata[6] = {0};

    ret = I2CWrite(cmd, 2);
    vTaskDelay( pdMS_TO_TICKS(100) );

    if (ret == ESP_OK) {
        ret = I2CRead(tempdata, 6);
        vTaskDelay(10);

        humidity = ((((tempdata[3] * 256.0) + tempdata[4]) * 100) / 65535.0);
        temperature = ((((tempdata[0] * 256.0) + tempdata[1]) * 175) / 65535.0) - 45;
    }

/*
        ret = I2CRead(cmd, 2, tempdata, 6);
        humidity = ((((tempdata[3] * 256.0) + tempdata[4]) * 100) / 65535.0);
        temperature = ((((tempdata[0] * 256.0) + tempdata[1]) * 175) / 65535.0) - 45;
*/

    return ret;
}

esp_err_t Sht3x_SoftReset() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {0x30, 0xA2};

    ret = I2CWrite(cmd, 2);
    vTaskDelay( pdMS_TO_TICKS(100) );

    return ret;
}

esp_err_t Sht3x_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_sht3x_device_handle);
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

esp_err_t Sht3x_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    ESP_LOGI(TAG, "Sht3x_Init Init()");
    esp_err_t ret = ESP_OK;

    i2c_device_config_t dev_sht3x_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT3X_ADDR,
        .scl_speed_hz = SHT3X_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_sht3x_cfg, &dev_sht3x_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    // device check
    ret = Sht3x_SoftReset();
    return ret;
}