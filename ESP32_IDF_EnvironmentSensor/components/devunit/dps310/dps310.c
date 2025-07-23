// https://github.com/sensidev/sensor-dps310
#include "freertos/FreeRTOS.h"
#include "dps310.h"

#define DPS310_ADDR (0x77)

#define DPS310_STANDARD_BAUD (400000)
#define DPS310_TIMEOUT_VALUE_MS (100)
i2c_master_dev_handle_t dev_dps310_device_handle;

static const char *TAG = "MY-DPS310";

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} calibration_coefs_t;

float pressure = 0.0;
float temperature = 0.0;

static calibration_coefs_t g_coefs;
static uint8_t g_pressure_rate = DPS310_CFG_RATE_1_MEAS;
static uint8_t g_temperature_rate = DPS310_CFG_RATE_1_MEAS;
static float g_last_temp_raw_sc;

static esp_err_t I2CWrite(uint8_t* cmd, uint8_t len) {
    esp_err_t ret = ESP_OK;

    ret = i2c_master_transmit(dev_dps310_device_handle, cmd, len, DPS310_TIMEOUT_VALUE_MS);
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

    ret = i2c_master_receive(dev_dps310_device_handle, buf, len, DPS310_TIMEOUT_VALUE_MS);
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

static esp_err_t I2CWriteAndRead(uint8_t* cmd, uint8_t len1, uint8_t* buf, uint8_t len2) {
    esp_err_t ret = ESP_OK;

    ret = i2c_master_transmit_receive(dev_dps310_device_handle, cmd, len1, buf, len2, DPS310_TIMEOUT_VALUE_MS);
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

float Dps310_GetPressure() {
    return pressure;
}
float Dps310_GetTemperature() {
    return temperature;
}

esp_err_t Dps310_Meas(uint8_t data) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {0};

    cmd[0] = DPS310_MEAS_CFG_REG;
    cmd[1] = data;
    ret = I2CWrite(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t Dps310_GetScaleFactorFor(uint8_t rate, uint32_t *p_factor) {
    esp_err_t ret = ESP_OK;

    switch (rate) {
        case 0x00: //SINGLE
            *p_factor = 524288;
            break;
        case 0x01: //2_TIMES
            *p_factor = 1572864;
            break;
        case 0x02: //4_TIMES
            *p_factor = 3670016;
            break;
        case 0x03: //8_TIMES
            *p_factor = 7864320;
            break;
        case 0x04: //16_TIMES
            *p_factor = 253952;
            break;
        case 0x05: //32_TIMES
            *p_factor = 516096;
            break;
        case 0x06: //64_TIMES
            *p_factor = 1040384;
            break;
        case 0x07: //128_TIMES
            *p_factor = 2088960;
            break;
        default:
            ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

int32_t Dps310_GetTwoComplementOf(uint32_t value, uint8_t length) {
    int32_t ret = value;
    bool b_is_negative = value & ((uint32_t)1 << (length - 1));

    if (b_is_negative) {
        ret -= ((uint32_t) 1 << length);
    }

    return ret;
}

esp_err_t Dps310_ReadPressure() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1] = {0};
    uint8_t buff[3] = {0};

    cmd[0] = DPS310_PSR_B2_REG;
    ret = I2CWriteAndRead(cmd, 1, buff, 3);
    if (ret != ESP_OK)
    {
        return ret;
    }

    int32_t pressure_raw = Dps310_GetTwoComplementOf(
            ((uint32_t) buff[0] << 16) | ((uint32_t) buff[1] << 8) | (uint32_t) buff[2],
            24);

    uint32_t factor;
    ret = Dps310_GetScaleFactorFor(g_pressure_rate, &factor);
    if (ret != ESP_OK){
        return ret;
    }

    float pressure_raw_sc = (float) pressure_raw / factor;

    pressure = g_coefs.c00 +
                  pressure_raw_sc * (g_coefs.c10 + pressure_raw_sc * (g_coefs.c20 + pressure_raw_sc * g_coefs.c30)) +
                  g_last_temp_raw_sc * (g_coefs.c01 + pressure_raw_sc * (g_coefs.c11 + pressure_raw_sc * g_coefs.c21));

    ESP_LOGI(TAG, "Dps310_ReadPressure:%f", pressure);
    return ESP_OK;
}

esp_err_t Dps310_ReadTemperature() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1] = {0};
    uint8_t buff[3] = {0};

    cmd[0] = DPS310_TMP_B2_REG;

    ret = I2CWriteAndRead(cmd, 1, buff, 3);
    if (ret != ESP_OK)
    {
        return ret;
    }

    int32_t temp_raw = Dps310_GetTwoComplementOf(
            ((uint32_t) buff[0] << 16) | ((uint32_t) buff[1] << 8) | (uint32_t) buff[2],
            24);

    uint32_t factor;
    ret = Dps310_GetScaleFactorFor(g_temperature_rate, &factor);
    if (ret != ESP_OK)
    {
        return ret;
    }

    g_last_temp_raw_sc = (float) temp_raw / factor;
    temperature = (float) g_coefs.c0 * 0.5 + (float) g_coefs.c1 * g_last_temp_raw_sc;

    ESP_LOGI(TAG, "Dps310_ReadTemperature:%f", temperature);
    return ESP_OK;
}

esp_err_t Dps310_Read() {
    esp_err_t ret = ESP_OK;
    ret = Dps310_Meas(DPS310_MEAS_CFG_MEAS_CTRL_TMP);
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_ReadTemperature();
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = Dps310_Meas(DPS310_MEAS_CFG_MEAS_CTRL_PRS);
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_ReadPressure();
    if (ret != ESP_OK)
    {
        return ret;
    }
    return ESP_OK;
}

esp_err_t Dps310_ConfigureTemperature(uint8_t data) {
    esp_err_t ret = ESP_OK;
    g_temperature_rate = DPS310_TMP_CFG_TMP_RATE_MASK & data;
    uint8_t cmd[2] = {0};
    cmd[0] = DPS310_TMP_CFG_REG;
    cmd[1] = data;
    ret = I2CWrite(cmd, 2);
    if (ret != ESP_OK)
    {
        return ret;
    }
    return ESP_OK;
}

esp_err_t Dps310_ConfigurePressure(uint8_t data) {
    esp_err_t ret = ESP_OK;
    g_pressure_rate = DPS310_PRS_CFG_PM_RATE_MASK & data;
    uint8_t cmd[2] = {0};
    cmd[0] = DPS310_PRS_CFG_REG;
    cmd[1] = data;
    ret = I2CWrite(cmd, 2);
    if (ret != ESP_OK)
    {
        return ret;
    }
    return ESP_OK;
}

esp_err_t Dps310_ReadCoefs() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1] = {0};
    uint8_t buff[18] = {0};

    cmd[0] = DPS310_COEF_REG;
    ret = I2CWriteAndRead(cmd, 1, buff, 18);
    if (ret != ESP_OK) {
        return ret;
    }

    g_coefs.c0 = Dps310_GetTwoComplementOf(
            ((uint16_t) buff[0] << 4) | (((uint16_t) buff[1] >> 4) & 0x0F),
            12);

    g_coefs.c1 = Dps310_GetTwoComplementOf(
            ((((uint16_t) buff[1] & 0x0F) << 8) | (uint16_t) buff[2]),
            12);

    g_coefs.c00 = Dps310_GetTwoComplementOf(
            ((uint32_t) buff[3] << 12) | ((uint32_t) buff[4] << 4) | (((uint32_t) buff[5] >> 4) & 0x0F),
            20);

    g_coefs.c10 = Dps310_GetTwoComplementOf(
            (((uint32_t) buff[5] & 0x0F) << 16) | ((uint32_t) buff[6] << 8) | (uint32_t) buff[7],
            20);

    g_coefs.c01 = Dps310_GetTwoComplementOf(
            ((uint16_t) buff[8] << 8) | (uint16_t) buff[9],
            16);

    g_coefs.c11 = Dps310_GetTwoComplementOf(
            ((uint16_t) buff[10] << 8) | (uint16_t) buff[11],
            16);

    g_coefs.c20 = Dps310_GetTwoComplementOf(
            ((uint16_t) buff[12] << 8) | (uint16_t) buff[13],
            16);

    g_coefs.c21 = Dps310_GetTwoComplementOf(
            ((uint16_t) buff[14] << 8) | (uint16_t) buff[15],
            16);

    g_coefs.c30 = Dps310_GetTwoComplementOf(
            ((uint16_t) buff[16] << 8) | (uint16_t) buff[17],
            16);

    return ESP_OK;
}

esp_err_t Dps310_EnShift() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {0};

    cmd[0] = DPS310_CFG_REG_REG;
    cmd[1] = DPS310_CFG_RET_PRS_SHIFT_EN;

    ret = I2CWrite(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t Dps310_Standby() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {0};

    cmd[0] = DPS310_MEAS_CFG_REG;
    cmd[1] = DPS310_MEAS_CFG_MEAS_CTRL_IDLE;
    ret = I2CWrite(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t Dps310_IdCheck() {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1] = {0};
    uint8_t buff[1] = {0};

    cmd[0] = DPS310_PRODUCT_ID_REG;
    ret = I2CWriteAndRead(cmd, 1, buff, 1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    bool b_is_product_id_valid = buff[0] == DPS310_PRODUCT_ID_VALUE;
    if (!b_is_product_id_valid)
    {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t Dps310_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_dps310_device_handle);
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

esp_err_t Dps310_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    ESP_LOGI(TAG, "Dps310_Init Init()");
    esp_err_t ret = ESP_OK;

    i2c_device_config_t dev_dps310_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DPS310_ADDR,
        .scl_speed_hz = DPS310_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_dps310_cfg, &dev_dps310_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }

    // id check
    ret = Dps310_IdCheck();
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }

    ret = Dps310_Standby();
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_ReadCoefs();
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_ConfigureTemperature(DPS310_CFG_RATE_4_MEAS | DPS310_TMP_CFG_REG_TMP_EXT_EXTERNAL | DPS310_TMP_CFG_TMP_PRC_SINGLE);
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_ConfigurePressure(DPS310_CFG_RATE_4_MEAS | DPS310_PRS_CFG_PM_PRC_64_TIMES);
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_EnShift();
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_ReadTemperature();
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Dps310_Standby();
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }

    return ret;
}