// https://github.com/Seeed-Studio/Seeed_SCD30/blob/master/SCD30.cpp
#include "freertos/FreeRTOS.h"
#include "scd30.h"
#include "string.h"

#define SCD30_I2C_ADDRESS (0x61)
#define SCD30_STANDARD_BAUD (400000)
#define SCD30_TIMEOUT_VALUE_MS (5000)

#define SCD30_CONTINUOUS_MEASUREMENT            (0x0010)
#define SCD30_SET_MEASUREMENT_INTERVAL          (0x4600)
#define SCD30_GET_DATA_READY                    (0x0202)
#define SCD30_READ_MEASUREMENT                  (0x0300)
#define SCD30_STOP_MEASUREMENT                  (0x0104)
#define SCD30_AUTOMATIC_SELF_CALIBRATION        (0x5306)
#define SCD30_SET_FORCED_RECALIBRATION_FACTOR   (0x5204)
#define SCD30_SET_TEMPERATURE_OFFSET            (0x5403)
#define SCD30_SET_ALTITUDE_COMPENSATION         (0x5102)
#define SCD30_READ_SERIALNBR                    (0xD033)
#define SCD30_READ_FW_VER                       (0xD100)

#define SCD30_SET_TEMP_OFFSET                   (0x5403)

#define SCD30_POLYNOMIAL                        (0x31)

static i2c_master_dev_handle_t dev_scd30_device_handle;

static const char *TAG = "MY-SCD30";

static float temperature = 0.0;
static float humidity = 0.0;
static float co2 = 0.0;

uint8_t Scd30_CalculateCrc(uint8_t data[], uint8_t len) {
    uint8_t bit, byteCtr, crc = 0xff;

    // calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < len; byteCtr ++) {
        crc ^= (data[byteCtr]);

        for (bit = 8; bit > 0; -- bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ SCD30_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

esp_err_t Scd30_ReadBuffer(uint8_t* data, uint8_t len) {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_receive(dev_scd30_device_handle, data, len, SCD30_TIMEOUT_VALUE_MS);
    vTaskDelay( pdMS_TO_TICKS(100) );
    return ret;
}

esp_err_t Scd30_WriteBuffer(uint8_t* data, uint8_t len) {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_transmit(dev_scd30_device_handle, data, len, SCD30_TIMEOUT_VALUE_MS);
    vTaskDelay( pdMS_TO_TICKS(100) );
    return ret;
}

esp_err_t Scd30_WriteCommand(uint16_t command) {
    uint8_t data[2] = { 0 };
    data[0] = command >> 8;
    data[1] = command & 0xff;
    return Scd30_WriteBuffer(data, 2);
}

esp_err_t Scd30_WriteCommandWithArguments(uint16_t command, uint16_t arguments) {
    uint8_t checkSum, buf[5] = { 0 };

    buf[0] = command >> 8;
    buf[1] = command & 0xff;
    buf[2] = arguments >> 8;
    buf[3] = arguments & 0xff;
    checkSum = Scd30_CalculateCrc(&buf[2], 2);
    buf[4] = checkSum;

    return Scd30_WriteBuffer(buf, 5);
}

uint16_t Scd30_ReadRegister(uint16_t address) {
    esp_err_t ret = ESP_OK;
    uint8_t buf[2] = { 0 };

    ret = Scd30_WriteCommand(address);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Scd30_ReadRegister Scd30_WriteCommand()");
        return 0;
    }
    vTaskDelay( pdMS_TO_TICKS(100) );
    ret = Scd30_ReadBuffer(buf, 2);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Scd30_ReadRegister Scd30_ReadBuffer:%u", (((uint16_t)buf[0]) << 8) | buf[1]);
        return 0;
    }

    return ((((uint16_t)buf[0]) << 8) | buf[1]);
}

esp_err_t Scd30_SetTemperatureOffset(uint16_t offset) {
    return Scd30_WriteCommandWithArguments(SCD30_SET_TEMP_OFFSET, offset);
}

bool Scd30_IsAvailable(void) {
    if (Scd30_ReadRegister(SCD30_GET_DATA_READY) == 1)
    {
        return true;
    }
    return false;
}

uint16_t Scd30_GetFwVersion() {
    return Scd30_ReadRegister(SCD30_READ_FW_VER);
}

esp_err_t Scd30_SetAutoSelfCalibration(bool enable) {
    if (enable) {
        return Scd30_WriteCommandWithArguments(SCD30_AUTOMATIC_SELF_CALIBRATION, 1);    //Activate continuous ASC
    } else {
        return Scd30_WriteCommandWithArguments(SCD30_AUTOMATIC_SELF_CALIBRATION, 0);    //Deactivate continuous ASC
    }
}

esp_err_t Scd30_SetMeasurementInterval(uint16_t interval) {
    return Scd30_WriteCommandWithArguments(SCD30_SET_MEASUREMENT_INTERVAL, interval);
}

esp_err_t Scd30_StartPeriodicMeasurement(void) {
    return Scd30_WriteCommandWithArguments(SCD30_CONTINUOUS_MEASUREMENT, 0x0000);
}

esp_err_t Scd30_StopMeasurement(void) {
    return Scd30_WriteCommand(SCD30_STOP_MEASUREMENT);
}

esp_err_t Scd30_Read() {
    esp_err_t ret = ESP_OK;
    uint8_t buf[18] = { 0 };
    uint32_t co2U32 = 0;
    uint32_t tempU32 = 0;
    uint32_t humU32 = 0;


    ret = Scd30_WriteCommand(SCD30_READ_MEASUREMENT);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    vTaskDelay( pdMS_TO_TICKS(100) );
    ret = Scd30_ReadBuffer(buf, 18);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    co2U32 = (uint32_t)((((uint32_t)buf[0]) << 24) | (((uint32_t)buf[1]) << 16) |
                        (((uint32_t)buf[3]) << 8) | ((uint32_t)buf[4]));

    tempU32 = (uint32_t)((((uint32_t)buf[6]) << 24) | (((uint32_t)buf[7]) << 16) |
                         (((uint32_t)buf[9]) << 8) | ((uint32_t)buf[10]));

    humU32 = (uint32_t)((((uint32_t)buf[12]) << 24) | (((uint32_t)buf[13]) << 16) |
                        (((uint32_t)buf[15]) << 8) | ((uint32_t)buf[16]));

    memcpy(&co2, &co2U32, sizeof(co2));
    memcpy(&temperature, &tempU32, sizeof(temperature));
    memcpy(&humidity, &humU32, sizeof(humidity));

    return ret;
}

float Scd30_GetTemperature() {
    return temperature;
}

int32_t Scd30_GetIntTemperature() {
    return (int32_t)temperature;
}

float Scd30_GetHumidity() {
    return humidity;
}

int32_t SScd30_GetIntHumidity() {
    return (int32_t)humidity;
}

int32_t Scd30_GetIntCo2() {
    return (int32_t)co2;
}

esp_err_t Scd30_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_scd30_device_handle);
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

esp_err_t Scd30_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    ESP_LOGI(TAG, "Scd30_Init Init()");
    esp_err_t ret = ESP_OK;

    i2c_device_config_t dev_scd30_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD30_I2C_ADDRESS,
        .scl_speed_hz = SCD30_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_scd30_cfg, &dev_scd30_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "i2c_master_bus_add_device error");
        return ESP_ERR_NOT_FOUND;
    }

    Scd30_SetMeasurementInterval(3); // 3 seconds between measurements
    Scd30_StartPeriodicMeasurement(); // start periodic measuments

//    Scd30_SetAutoSelfCalibration(false); // auto-self-calibration

    // device check
    if (Scd30_GetFwVersion() > 0)
    {
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}