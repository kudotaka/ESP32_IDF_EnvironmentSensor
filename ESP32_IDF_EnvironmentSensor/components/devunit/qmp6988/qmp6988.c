// https://github.com/m5stack/M5Unit-ENV/blob/master/src/QMP6988.cpp
#include "freertos/FreeRTOS.h"
#include "qmp6988.h"

#define QMP6988_ADDR (0x70)
#define QMP6988_CHIP_ID_REG (0xD1)
#define QMP6988_RESET_REG   (0xE0)
#define QMP6988_DEVICE_STAT_REG (0xF3)
#define QMP6988_CTRLMEAS_REG    (0xF4)
#define QMP6988_PRESSURE_MSB_REG    (0xF7)
#define QMP6988_TEMPERATURE_MSB_REG    (0xFA)

#define QMP6988_U16_t unsigned short
#define QMP6988_S16_t short
#define QMP6988_U32_t unsigned int
#define QMP6988_S32_t int
#define QMP6988_U64_t unsigned long long
#define QMP6988_S64_t long long

#define QMP6988_CHIP_ID (0x5C)
#define QMP6988_CALIBRATION_DATA_START  (0xA0)
#define QMP6988_CALIBRATION_DATA_LENGTH (25)

#define SHIFT_RIGHT_4_POSITION (4)
#define SHIFT_LEFT_2_POSITION  (2)
#define SHIFT_LEFT_4_POSITION  (4)
#define SHIFT_LEFT_5_POSITION  (5)
#define SHIFT_LEFT_8_POSITION  (8)
#define SHIFT_LEFT_12_POSITION (12)
#define SHIFT_LEFT_16_POSITION (16)

#define QMP6988_SLEEP_MODE  (0x00)
#define QMP6988_FORCED_MODE (0x01)
#define QMP6988_NORMAL_MODE (0x03)

#define QMP6988_OVERSAMPLING_SKIPPED (0x00)
#define QMP6988_OVERSAMPLING_1X      (0x01)
#define QMP6988_OVERSAMPLING_2X      (0x02)
#define QMP6988_OVERSAMPLING_4X      (0x03)
#define QMP6988_OVERSAMPLING_8X      (0x04)
#define QMP6988_OVERSAMPLING_16X     (0x05)
#define QMP6988_OVERSAMPLING_32X     (0x06)
#define QMP6988_OVERSAMPLING_64X     (0x07)

#define QMP6988_FILTERCOEFF_OFF (0x00)
#define QMP6988_FILTERCOEFF_2   (0x01)
#define QMP6988_FILTERCOEFF_4   (0x02)
#define QMP6988_FILTERCOEFF_8   (0x03)
#define QMP6988_FILTERCOEFF_16  (0x04)
#define QMP6988_FILTERCOEFF_32  (0x05)

#define QMP6988_CONFIG_REG      (0xF1)

#define SUBTRACTOR (8388608)

typedef struct _qmp6988_cali_data {
    QMP6988_S32_t COE_a0;
    QMP6988_S16_t COE_a1;
    QMP6988_S16_t COE_a2;
    QMP6988_S32_t COE_b00;
    QMP6988_S16_t COE_bt1;
    QMP6988_S16_t COE_bt2;
    QMP6988_S16_t COE_bp1;
    QMP6988_S16_t COE_b11;
    QMP6988_S16_t COE_bp2;
    QMP6988_S16_t COE_b12;
    QMP6988_S16_t COE_b21;
    QMP6988_S16_t COE_bp3;
} qmp6988_cali_data_t;

typedef struct _qmp6988_fk_data {
    float a0, b00;
    float a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_fk_data_t;

typedef struct _qmp6988_ik_data {
    QMP6988_S32_t a0, b00;
    QMP6988_S32_t a1, a2;
    QMP6988_S64_t bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_ik_data_t;

typedef struct _qmp6988_data {
    uint8_t slave;
    uint8_t chip_id;
    uint8_t power_mode;
    float temperature;
    float pressure;
    float altitude;
    qmp6988_cali_data_t qmp6988_cali;
    qmp6988_ik_data_t ik;
} qmp6988_data_t;


static const char *TAG = "MY-QMP6988";

#define QMP6988_STANDARD_BAUD (400000)
#define QMP6988_TIMEOUT_VALUE_MS (100)
static i2c_master_dev_handle_t dev_qmp6988_device_handle;
static qmp6988_data_t qmp6988;

void Qmp6988_SetOversamplingT(unsigned char oversampling_t);
void Qmp6988_SetOversamplingP(unsigned char oversampling_p);
void Qmp6988_SetFilter(unsigned char filter);
void Qmp6988_SetPowermode(int power_mode);
esp_err_t Qmp6988_GetCalibrationData();
esp_err_t Qmp6988_SoftwareReset();
QMP6988_S16_t Qmp6988_ConvTx02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dt);
QMP6988_S32_t Qmp6988_GetPressure02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dp, QMP6988_S16_t tx);

static esp_err_t I2CWrite(uint8_t addr, uint8_t* buf, uint8_t len) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[1 + len];
    cmd[0] = addr;
    for (uint8_t i = 0; i < len; i++)
    {
        cmd[i + 1] = buf[i];
    }

    ret = i2c_master_transmit(dev_qmp6988_device_handle, cmd, 1 + len, QMP6988_TIMEOUT_VALUE_MS);
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

    ret = i2c_master_transmit_receive(dev_qmp6988_device_handle, cmd, 1, buf, len, QMP6988_TIMEOUT_VALUE_MS);
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

void Qmp6988_SetOversamplingT(unsigned char oversampling_t) {
    uint8_t data = 0;

    I2CRead(QMP6988_CTRLMEAS_REG, &(data), 1);
    data &= 0x1f;
    data |= (oversampling_t << 5);
    I2CWrite(QMP6988_CTRLMEAS_REG, &(data), 1);

    vTaskDelay( pdMS_TO_TICKS(100) );
}

void Qmp6988_SetOversamplingP(unsigned char oversampling_p) {
    uint8_t data = 0;

    I2CRead(QMP6988_CTRLMEAS_REG, &(data), 1);
    data &= 0xe3;
    data |= (oversampling_p << 2);
    I2CWrite(QMP6988_CTRLMEAS_REG, &(data), 1);

    vTaskDelay( pdMS_TO_TICKS(100) );
}

void Qmp6988_SetFilter(unsigned char filter) {
    uint8_t data = 0;

    data = (filter & 0x03);
    I2CWrite(QMP6988_CONFIG_REG, &(data), 1);

    vTaskDelay( pdMS_TO_TICKS(100) );
}

void Qmp6988_SetPowermode(int power_mode) {
    uint8_t data = 0;

    ESP_LOGI(TAG, "qmp_set_powermode %d", power_mode);

    qmp6988.power_mode = power_mode;
    I2CRead(QMP6988_CTRLMEAS_REG, &(data), 1);
    vTaskDelay( pdMS_TO_TICKS(100) );
    data = data & 0xfc;
    if (power_mode == QMP6988_SLEEP_MODE) {
        data |= 0x00;
    } else if (power_mode == QMP6988_FORCED_MODE) {
        data |= 0x01;
    } else if (power_mode == QMP6988_NORMAL_MODE) {
        data |= 0x03;
    }
    I2CWrite(QMP6988_CTRLMEAS_REG, &(data), 1);
    ESP_LOGI(TAG, "qmp_set_powermode 0xf4=0x%x", data);

    vTaskDelay( pdMS_TO_TICKS(100) );
}

esp_err_t Qmp6988_GetCalibrationData() {
    esp_err_t ret = ESP_OK;
    uint8_t a_data_uint8_tr[QMP6988_CALIBRATION_DATA_LENGTH] = {0};
    int len = 0;
    for (len = 0; len < QMP6988_CALIBRATION_DATA_LENGTH; len++) {
        ret = I2CRead((QMP6988_CALIBRATION_DATA_START + len), &(a_data_uint8_tr[len]), 1);
        vTaskDelay( pdMS_TO_TICKS(100) );
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "qmp6988 read 0xA0+ error!");
            return ret;
        }
    }

    qmp6988.qmp6988_cali.COE_a0 = 
        (QMP6988_S32_t)(((a_data_uint8_tr[18] << SHIFT_LEFT_12_POSITION) |
                         (a_data_uint8_tr[19] << SHIFT_LEFT_4_POSITION) |
                         (a_data_uint8_tr[24] & 0x0f))
                        << 12);
    qmp6988.qmp6988_cali.COE_a0 = qmp6988.qmp6988_cali.COE_a0 >> 12;

    qmp6988.qmp6988_cali.COE_a1 =
        (QMP6988_S16_t)(((a_data_uint8_tr[20]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[21]);
    qmp6988.qmp6988_cali.COE_a2 =
        (QMP6988_S16_t)(((a_data_uint8_tr[22]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[23]);

    qmp6988.qmp6988_cali.COE_b00 =
        (QMP6988_S32_t)(((a_data_uint8_tr[0] << SHIFT_LEFT_12_POSITION) |
                         (a_data_uint8_tr[1] << SHIFT_LEFT_4_POSITION) |
                         ((a_data_uint8_tr[24] & 0xf0) >>
                          SHIFT_RIGHT_4_POSITION))
                        << 12);
    qmp6988.qmp6988_cali.COE_b00 = qmp6988.qmp6988_cali.COE_b00 >> 12;

    qmp6988.qmp6988_cali.COE_bt1 =
        (QMP6988_S16_t)(((a_data_uint8_tr[2]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[3]);
    qmp6988.qmp6988_cali.COE_bt2 =
        (QMP6988_S16_t)(((a_data_uint8_tr[4]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[5]);
    qmp6988.qmp6988_cali.COE_bp1 =
        (QMP6988_S16_t)(((a_data_uint8_tr[6]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[7]);
    qmp6988.qmp6988_cali.COE_b11 =
        (QMP6988_S16_t)(((a_data_uint8_tr[8]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[9]);
    qmp6988.qmp6988_cali.COE_bp2 =
        (QMP6988_S16_t)(((a_data_uint8_tr[10]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[11]);
    qmp6988.qmp6988_cali.COE_b12 =
        (QMP6988_S16_t)(((a_data_uint8_tr[12]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[13]);
    qmp6988.qmp6988_cali.COE_b21 =
        (QMP6988_S16_t)(((a_data_uint8_tr[14]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[15]);
    qmp6988.qmp6988_cali.COE_bp3 =
        (QMP6988_S16_t)(((a_data_uint8_tr[16]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[17]);

    ESP_LOGI(TAG, "<-----------calibration data-------------->");
    ESP_LOGI(TAG, "COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]",
                qmp6988.qmp6988_cali.COE_a0, qmp6988.qmp6988_cali.COE_a1,
                qmp6988.qmp6988_cali.COE_a2, qmp6988.qmp6988_cali.COE_b00);
    ESP_LOGI(TAG, "COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]",
                qmp6988.qmp6988_cali.COE_bt1, qmp6988.qmp6988_cali.COE_bt2,
                qmp6988.qmp6988_cali.COE_bp1, qmp6988.qmp6988_cali.COE_b11);
    ESP_LOGI(TAG, "COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]",
                qmp6988.qmp6988_cali.COE_bp2, qmp6988.qmp6988_cali.COE_b12,
                qmp6988.qmp6988_cali.COE_b21, qmp6988.qmp6988_cali.COE_bp3);
    ESP_LOGI(TAG, "<-----------calibration data-------------->");

    qmp6988.ik.a0  = qmp6988.qmp6988_cali.COE_a0;   // 20Q4
    qmp6988.ik.b00 = qmp6988.qmp6988_cali.COE_b00;  // 20Q4

    qmp6988.ik.a1 = 3608L * (QMP6988_S32_t)qmp6988.qmp6988_cali.COE_a1 -
                    1731677965L;  // 31Q23
    qmp6988.ik.a2 = 16889L * (QMP6988_S32_t)qmp6988.qmp6988_cali.COE_a2 -
                    87619360L;  // 30Q47

    qmp6988.ik.bt1 = 2982L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bt1 +
                     107370906L;  // 28Q15
    qmp6988.ik.bt2 = 329854L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bt2 +
                     108083093L;  // 34Q38
    qmp6988.ik.bp1 = 19923L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp1 +
                     1133836764L;  // 31Q20
    qmp6988.ik.b11 = 2406L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b11 +
                     118215883L;  // 28Q34
    qmp6988.ik.bp2 = 3079L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp2 -
                     181579595L;  // 29Q43
    qmp6988.ik.b12 = 6846L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b12 +
                     85590281L;  // 29Q53
    qmp6988.ik.b21 = 13836L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b21 +
                     79333336L;  // 29Q60
    qmp6988.ik.bp3 = 2915L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp3 +
                     157155561L;  // 28Q65
    ESP_LOGI(TAG, "<----------- int calibration data -------------->");
    ESP_LOGI(TAG, "a0[%d]	a1[%d] a2[%d] b00[%d]", qmp6988.ik.a0,
                qmp6988.ik.a1, qmp6988.ik.a2, qmp6988.ik.b00);
    ESP_LOGI(TAG, "bt1[%lld]	bt2[%lld]	bp1[%lld]	b11[%lld]",
                qmp6988.ik.bt1, qmp6988.ik.bt2, qmp6988.ik.bp1, qmp6988.ik.b11);
    ESP_LOGI(TAG, "bp2[%lld]	b12[%lld]	b21[%lld]	bp3[%lld]",
                qmp6988.ik.bp2, qmp6988.ik.b12, qmp6988.ik.b21, qmp6988.ik.bp3);
    ESP_LOGI(TAG, "<----------- int calibration data -------------->");

    return ret;
}

esp_err_t Qmp6988_SoftwareReset() {
    uint8_t data = 0;
    esp_err_t ret = ESP_OK;
    data = 0xe6;
    ret = I2CWrite(QMP6988_RESET_REG, &(data), 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "softwareReset(1) fail!!!");
    }
    vTaskDelay( pdMS_TO_TICKS(100) );
    data = 0x00;
    ret = I2CWrite(QMP6988_RESET_REG, &(data), 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "softwareReset(2) fail!!!");
    }
    return ret;
}

QMP6988_S16_t Qmp6988_ConvTx02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dt) {
    QMP6988_S16_t ret;
    QMP6988_S64_t wk1, wk2;

    // wk1: 60Q4 // bit size
    wk1 = ((QMP6988_S64_t)ik->a1 * (QMP6988_S64_t)dt);  // 31Q23+24-1=54 (54Q23)
    wk2 = ((QMP6988_S64_t)ik->a2 * (QMP6988_S64_t)dt) >> 14;// 30Q47+24-1=53 (39Q33)
    wk2 = (wk2 * (QMP6988_S64_t)dt) >> 10;       // 39Q33+24-1=62 (52Q23)
    wk2 = ((wk1 + wk2) / 32767) >> 19;           // 54,52->55Q23 (20Q04)
    ret = (QMP6988_S16_t)((ik->a0 + wk2) >> 4);  // 21Q4 -> 17Q0
    return ret;
}

QMP6988_S32_t Qmp6988_GetPressure02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dp, QMP6988_S16_t tx) {
    QMP6988_S32_t ret;
    QMP6988_S64_t wk1, wk2, wk3;

    // wk1 = 48Q16 // bit size
    wk1 =
        ((QMP6988_S64_t)ik->bt1 * (QMP6988_S64_t)tx);  // 28Q15+16-1=43 (43Q15)
    wk2 = ((QMP6988_S64_t)ik->bp1 * (QMP6988_S64_t)dp) >> 5;     // 31Q20+24-1=54 (49Q15)
    wk1 += wk2;  // 43,49->50Q15
    wk2 = ((QMP6988_S64_t)ik->bt2 * (QMP6988_S64_t)tx) >> 1;     // 34Q38+16-1=49 (48Q37)
    wk2 = (wk2 * (QMP6988_S64_t)tx) >> 8;  // 48Q37+16-1=63 (55Q29)
    wk3 = wk2;                             // 55Q29
    wk2 = ((QMP6988_S64_t)ik->b11 * (QMP6988_S64_t)tx) >> 4;     // 28Q34+16-1=43 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;  // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                            // 55,61->62Q29
    wk2 = ((QMP6988_S64_t)ik->bp2 * (QMP6988_S64_t)dp) >> 13;    // 29Q43+24-1=52 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;  // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                            // 62,61->63Q29
    wk1 += wk3 >> 14;                      // Q29 >> 14 -> Q15
    wk2 =
        ((QMP6988_S64_t)ik->b12 * (QMP6988_S64_t)tx);  // 29Q53+16-1=45 (45Q53)
    wk2 = (wk2 * (QMP6988_S64_t)tx) >> 22;             // 45Q53+16-1=61 (39Q31)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;              // 39Q31+24-1=62 (61Q30)
    wk3 = wk2;                                         // 61Q30
    wk2 = ((QMP6988_S64_t)ik->b21 * (QMP6988_S64_t)tx) >> 6;   // 29Q60+16-1=45 (39Q54)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23;  // 39Q54+24-1=62 (39Q31)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;   // 39Q31+24-1=62 (61Q20)
    wk3 += wk2;                             // 61,61->62Q30
    wk2 = ((QMP6988_S64_t)ik->bp3 * (QMP6988_S64_t)dp) >> 12;  // 28Q65+24-1=51 (39Q53)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23;  // 39Q53+24-1=62 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp);        // 39Q30+24-1=62 (62Q30)
    wk3 += wk2;                             // 62,62->63Q30
    wk1 += wk3 >> 15;                       // Q30 >> 15 = Q15
    wk1 /= 32767L;
    wk1 >>= 11;      // Q15 >> 7 = Q4
    wk1 += ik->b00;  // Q4 + 20Q4
    // wk1 >>= 4; // 28Q4 -> 24Q0
    ret = (QMP6988_S32_t)wk1;
    return ret;
}

esp_err_t Qmp6988_GetChipID() {
    esp_err_t ret = ESP_OK;
    ret = I2CRead(QMP6988_CHIP_ID_REG, &(qmp6988.chip_id), 1);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK) {
        if (qmp6988.chip_id != QMP6988_CHIP_ID) {
            return ESP_ERR_INVALID_VERSION;
        }
    }

    return ret;
}

float Qmp6988_CalcPressure() {
    esp_err_t ret = ESP_OK;

    QMP6988_U32_t P_read, T_read;
    QMP6988_S32_t P_raw, T_raw;
    uint8_t a_data_uint8_tr[6] = {0};
    QMP6988_S32_t T_int, P_int;

    ret = I2CRead(QMP6988_PRESSURE_MSB_REG, a_data_uint8_tr, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "qmp6988 read press raw error!");
        return 0.0;
    }

    P_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[0]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[1]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[2]));
    P_raw  = (QMP6988_S32_t)(P_read - SUBTRACTOR);

    T_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[3]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[4]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[5]));
    T_raw  = (QMP6988_S32_t)(T_read - SUBTRACTOR);

    T_int               = Qmp6988_ConvTx02e(&(qmp6988.ik), T_raw);
    P_int               = Qmp6988_GetPressure02e(&(qmp6988.ik), P_raw, T_int);
    qmp6988.temperature = (float)T_int / 256.0f;
    qmp6988.pressure    = (float)P_int / 16.0f;

    return qmp6988.pressure;
}

float Qmp6988_calcTemperature() {
    esp_err_t ret = ESP_OK;

    QMP6988_U32_t P_read, T_read;
    QMP6988_S32_t P_raw, T_raw;
    uint8_t a_data_uint8_tr[6] = {0};
    QMP6988_S32_t T_int, P_int;

    // press
    ret = I2CRead(QMP6988_PRESSURE_MSB_REG, a_data_uint8_tr, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "qmp6988 read press raw error!");
        return 0.0;
    }

    P_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[0]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[1]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[2]));
    P_raw  = (QMP6988_S32_t)(P_read - SUBTRACTOR);

    // temp
    T_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[3]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[4]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[5]));
    T_raw  = (QMP6988_S32_t)(T_read - SUBTRACTOR);

    T_int               = Qmp6988_ConvTx02e(&(qmp6988.ik), T_raw);
    P_int               = Qmp6988_GetPressure02e(&(qmp6988.ik), P_raw, T_int);
    qmp6988.temperature = (float)T_int / 256.0f;
    qmp6988.pressure    = (float)P_int / 16.0f;

    return qmp6988.temperature;
}

esp_err_t Qmp6988_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_qmp6988_device_handle);
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

esp_err_t Qmp6988_Init(i2c_master_bus_handle_t i2c_master_bus_handle) {
    ESP_LOGI(TAG, "Qmp6988_Init Init()");
    esp_err_t ret = ESP_OK;

    i2c_device_config_t dev_qmp6988_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMP6988_ADDR,
        .scl_speed_hz = QMP6988_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_qmp6988_cfg, &dev_qmp6988_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
    }
    ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");

    ret = Qmp6988_GetChipID();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Qmp6988_GetChipID error");
    }
    else
    {
//        Qmp6988_SoftwareReset();
        Qmp6988_GetCalibrationData();
        Qmp6988_SetPowermode(QMP6988_NORMAL_MODE);
        Qmp6988_SetFilter(QMP6988_FILTERCOEFF_4);
        Qmp6988_SetOversamplingP(QMP6988_OVERSAMPLING_8X);
        Qmp6988_SetOversamplingT(QMP6988_OVERSAMPLING_1X);
    }

    return ret;
}
