// https://github.com/Seeed-Studio/Grove_BMP280/blob/master/Seeed_BMP280.cpp
#include "freertos/FreeRTOS.h"
#include "math.h"
#include "bmp280.h"

#define BMP280_ADDR (0x76)

#define BMP280_REG_DIG_T1    (0x88)
#define BMP280_REG_DIG_T2    (0x8A)
#define BMP280_REG_DIG_T3    (0x8C)

#define BMP280_REG_DIG_P1    (0x8E)
#define BMP280_REG_DIG_P2    (0x90)
#define BMP280_REG_DIG_P3    (0x92)
#define BMP280_REG_DIG_P4    (0x94)
#define BMP280_REG_DIG_P5    (0x96)
#define BMP280_REG_DIG_P6    (0x98)
#define BMP280_REG_DIG_P7    (0x9A)
#define BMP280_REG_DIG_P8    (0x9C)
#define BMP280_REG_DIG_P9    (0x9E)

#define BMP280_REG_CHIPID          (0xD0)
#define BMP280_REG_VERSION         (0xD1)
#define BMP280_REG_SOFTRESET       (0xE0)

#define BMP280_REG_CONTROL         (0xF4)
#define BMP280_REG_CONFIG          (0xF5)
#define BMP280_REG_PRESSUREDATA    (0xF7)
#define BMP280_REG_TEMPDATA        (0xFA)

#define BMP280_CHIPID (0x58)

#define BMP280_STANDARD_BAUD (400000)
#define BMP280_TIMEOUT_VALUE_MS (100)

typedef struct _bmp280_cali_data {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine;
} bmp280_cali_data_t;

typedef struct _bmp280_data {
    uint8_t chipid;
    float temperature;
    float pressure;
    float altitude;
    bmp280_cali_data_t bmp280_cali;
} bmp280_data_t;

static const char *TAG = "MY-BMP280";

static i2c_master_dev_handle_t dev_bmp280_device_handle;
static bmp280_data_t bmp280;


static esp_err_t I2CWrite(uint8_t addr, uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1 + len];
    cmd[0] = addr;
    for (uint8_t i = 0; i < len; i++)
    {
        cmd[i + 1] = buf[i];
    }

    ret = i2c_master_transmit(dev_bmp280_device_handle, cmd, 1 + len, BMP280_TIMEOUT_VALUE_MS);
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

static esp_err_t I2CRead(uint8_t addr, uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[] = { 0 };
    cmd[0] = addr;

    ret = i2c_master_transmit_receive(dev_bmp280_device_handle, cmd, 1, buf, len, BMP280_TIMEOUT_VALUE_MS);
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

uint8_t Bmp280_bmp280Read8(uint8_t reg) {
    esp_err_t ret = ESP_OK;
    uint8_t data = 0;

    ret = I2CRead(reg, &(data), 1);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP280_bmp280Read8 read error!");
        return 0;
    }
    return data;
}

uint16_t Bmp280_bmp280Read16(uint8_t reg) {
    esp_err_t ret = ESP_OK;
    uint8_t data[2] = {0};

    ret = I2CRead(reg, data, 2);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP280_bmp280Read16 read error!");
        return 0;
    }
    return (uint16_t) data[0] << 8 | data[1];
}

uint16_t Bmp280_bmp280Read16LE(uint8_t reg) {
    esp_err_t ret = ESP_OK;
    uint8_t data[2] = {0};

    ret = I2CRead(reg, data, 2);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP280_bmp280Read16 read error!");
        return 0;
    }
    return (uint16_t) data[1] << 8 | data[0];
}

uint32_t Bmp280_bmp280Read24(uint8_t reg) {
    esp_err_t ret = ESP_OK;
    uint8_t data[3] = {0};

    ret = I2CRead(reg, data, 3);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP280_bmp280Read24 read error!");
        return 0;
    }
    return (uint32_t) data[0] << 16 | data[1] << 8 | data[2];
}

esp_err_t Bmp280_writeRegister(uint8_t reg, uint8_t val) {
    esp_err_t ret = ESP_OK;

    uint8_t data = val;
    ret = I2CWrite(reg, &(data), 1);
    ESP_LOGI(TAG, "BMP280_writeRegister 0x%x", data);
    vTaskDelay( pdMS_TO_TICKS(100) );
    return ret;
}

float Bmp280_getTemperature(void) {
  int32_t var1, var2;
  int32_t adc_T = Bmp280_bmp280Read24(BMP280_REG_TEMPDATA);

  adc_T >>= 4;
  var1 = (((adc_T >> 3) - ((int32_t)(bmp280.bmp280_cali.dig_T1 << 1))) *
          ((int32_t)bmp280.bmp280_cali.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)bmp280.bmp280_cali.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)bmp280.bmp280_cali.dig_T1))) >> 12) *
          ((int32_t)bmp280.bmp280_cali.dig_T3)) >> 14;
  bmp280.bmp280_cali.t_fine = var1 + var2;
  float T = (bmp280.bmp280_cali.t_fine * 5 + 128) >> 8;
  return T / 100;
}

float Bmp280_getPressure(void) {
  int64_t var1, var2, p;
  // Call getTemperature to get t_fine
  Bmp280_getTemperature();

  int32_t adc_P = Bmp280_bmp280Read24(BMP280_REG_PRESSUREDATA);
  adc_P >>= 4;
  var1 = ((int64_t)bmp280.bmp280_cali.t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)bmp280.bmp280_cali.dig_P6;
  var2 = var2 + ((var1 * (int64_t)bmp280.bmp280_cali.dig_P5) << 17);
  var2 = var2 + (((int64_t)bmp280.bmp280_cali.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)bmp280.bmp280_cali.dig_P3) >> 8) + ((var1 * (int64_t)bmp280.bmp280_cali.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280.bmp280_cali.dig_P1) >> 33;
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)bmp280.bmp280_cali.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)bmp280.bmp280_cali.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280.bmp280_cali.dig_P7) << 4);
  return (float)p / 256;
}

float Bmp280_calcAltitude2(float p0, float p1, float t) {
  float C;
  C = (p0 / p1);
  C = pow(C, (1 / 5.25588)) - 1.0;
  C = (C * (t + 273.15)) / 0.0065;
  return C;
}

float Bmp280_calcAltitude1(float p0) {
  float t = Bmp280_getTemperature();
  float p1 = Bmp280_getPressure();
  return Bmp280_calcAltitude2(p0, p1, t);
}

void Bmp280_GetCalibrationData() {
    bmp280.bmp280_cali.dig_T1 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_T1);
    bmp280.bmp280_cali.dig_T2 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_T2);
    bmp280.bmp280_cali.dig_T3 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_T3);
    bmp280.bmp280_cali.dig_P1 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P1);
    bmp280.bmp280_cali.dig_P2 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P2);
    bmp280.bmp280_cali.dig_P3 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P3);
    bmp280.bmp280_cali.dig_P4 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P4);
    bmp280.bmp280_cali.dig_P5 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P5);
    bmp280.bmp280_cali.dig_P6 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P6);
    bmp280.bmp280_cali.dig_P7 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P7);
    bmp280.bmp280_cali.dig_P8 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P8);
    bmp280.bmp280_cali.dig_P9 = Bmp280_bmp280Read16LE(BMP280_REG_DIG_P9);
    Bmp280_writeRegister(BMP280_REG_CONTROL, 0x3F);

}

esp_err_t Bmp280_GetChipID() {
    esp_err_t ret = ESP_OK;

    ret = I2CRead((uint8_t)BMP280_REG_CHIPID, &(bmp280.chipid), 1);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK) {
        if (bmp280.chipid != BMP280_CHIPID) {
            ESP_LOGE(TAG, "BMP280_GetChipID not error.");
            return ESP_ERR_INVALID_VERSION;
        }
        ESP_LOGI(TAG, "BMP280_GetChipID %02X", bmp280.chipid);
        return ret;
    }
    return ret;
}

esp_err_t Bmp280_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_bmp280_device_handle);
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

esp_err_t Bmp280_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    ESP_LOGI(TAG, "BMP280_Init Init()");
    esp_err_t ret = ESP_OK;

    i2c_device_config_t dev_bmp280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_ADDR,
        .scl_speed_hz = BMP280_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_bmp280_cfg, &dev_bmp280_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    ret = Bmp280_GetChipID();
    if (ret == ESP_OK)
    {
        Bmp280_GetCalibrationData();
    }

    return ret;
}
