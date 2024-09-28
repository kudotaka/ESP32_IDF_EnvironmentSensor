#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
#if CONFIG_SOFTWARE_ESP_MQTT_SUPPORT
#include "mqtt_client.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#endif //CONFIG_SOFTWARE_ESP_MQTT_SUPPORT
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
#include "driver/gpio.h"
#include "esp_sntp.h"
#include "esp_intr_alloc.h"
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT

#if CONFIG_SOFTWARE_SERIAL_TXRX_SUPPORT
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#endif //CONFIG_SOFTWARE_SERIAL_TXRX_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
#include "driver/i2c_master.h"
#endif //CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT

#if CONFIG_DEEP_SLEEP_SUPPORT
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"
#include "nvs.h"
#endif //CONFIG_DEEP_SLEEP_SUPPORT

#if CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#endif //CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT

#if (  CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT \
    || CONFIG_SOFTWARE_INTERNAL_BUTTON_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT \
    || CONFIG_SOFTWARE_SENSOR_SHT3X \
    || CONFIG_SOFTWARE_SENSOR_SHT4X \
    || CONFIG_SOFTWARE_SENSOR_BMP280 \
    || CONFIG_SOFTWARE_SENSOR_QMP6988 \
    || CONFIG_SOFTWARE_SENSOR_BME680 \
    || CONFIG_SOFTWARE_SENSOR_ADT7410 \
    || CONFIG_SOFTWARE_SENSOR_SCD30 \
    || CONFIG_SOFTWARE_SENSOR_SCD40 \
    || CONFIG_SOFTWARE_SENSOR_MHZ19C \
    || CONFIG_SOFTWARE_SENSOR_BH1750 \
    || CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_BUTTON_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_LED_SUPPORT )
#include "devunit.h"
#endif

static const char *TAG = "MY-MAIN";


#if ( CONFIG_SOFTWARE_ESP_MQTT_SUPPORT || CONFIG_SOFTWARE_SENSOR_USE_SENSOR )
int8_t g_sensor_mode = 0;
float g_temperature = 0.0;
float g_humidity = 0.0;
float g_pressure = 0.0;
int g_co2 = 0;
uint16_t g_lux = 0;
uint8_t g_loopCount = 0;
#endif //CONFIG_SOFTWARE_SENSOR_USE_SENSOR

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
void RtcInterruptInit(void);
void RtcStartClockInit(void);
#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
static char g_lastSyncDatetime[72] = {0};
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
#if CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
static bool g_clockout_status = false;
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT

#if CONFIG_SOFTWARE_CLOCK_SOUND_SUPPORT
uint8_t g_clockCount = 0;
uint8_t g_clockCurrent = 0;
uint8_t g_clockSoundEnabled = 1;
#endif //CONFIG_SOFTWARE_CLOCK_SOUND_SUPPORT

#if CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT
static int g_adc_raw;
static int g_voltage;
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
#endif //CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT


#if CONFIG_SOFTWARE_INTERNAL_BUTTON_SUPPORT
TaskHandle_t xInternalButton;
Button_Internal_t* button_int1;
static void vInternal_button_task(void* pvParameters) {
    ESP_LOGD(TAG, "start INTERNAL Button");

    Button_Internal_Init();
    if (Button_Internal_Enable(BUTTON_INT1_GPIO_PIN, BUTTON_INTERNAL_ACTIVE_LOW) == ESP_OK) {
        button_int1 = Button_Internal_Attach(BUTTON_INT1_GPIO_PIN, BUTTON_INTERNAL_ACTIVE_LOW);
    }
    while(1){

        if (Button_Internal_WasPressed(button_int1)) {
            ESP_LOGI(TAG, "BUTTON INT1  PRESSED!");
#if CONFIG_SOFTWARE_UI_SUPPORT
            ui_button_label_update(true);
#endif //CONFIG_SOFTWARE_UI_SUPPORT
        }
        if (Button_Internal_WasReleased(button_int1)) {
            ESP_LOGI(TAG, "BUTTON INT1 RELEASED!");
#if CONFIG_SOFTWARE_UI_SUPPORT
            ui_button_label_update(false);
#endif //CONFIG_SOFTWARE_UI_SUPPORT
        }
        if (Button_Internal_WasLongPress(button_int1, pdMS_TO_TICKS(1000))) { // 1Sec
            ESP_LOGI(TAG, "BUTTON INT1 LONGPRESS!");
#if CONFIG_SOFTWARE_UI_SUPPORT
            ui_button_label_update(false);
#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
            ui_status_set(g_lastSyncDatetime);
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
#endif //CONFIG_SOFTWARE_UI_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
#if CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
            if (g_clockout_status)
            {
                g_clockout_status = 0;
            }
            else
            {
                g_clockout_status = 1;
            }
            PCF8563_ClockOutForTrimmer(g_clockout_status);
            ESP_LOGI(TAG, "PCF8563_ClockOutForTrimmer: %d", g_clockout_status);
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT

        }

        vTaskDelay(pdMS_TO_TICKS(80));
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif //CONFIG_SOFTWARE_INTERNAL_BUTTON_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_BUTTON_SUPPORT
// SELECT GPIO_NUM_XX
TaskHandle_t xExternalButton;
Button_External_t* button_ext1;
static void vExternal_button_task(void* pvParameters) {
    ESP_LOGD(TAG, "start EXTERNAL Button");

    Button_External_Init();
    if (Button_External_Enable(BUTTON_EXT1_GPIO_PIN, BUTTON_EXTERNAL_ACTIVE_LOW) == ESP_OK) {
        button_ext1 = Button_External_Attach(BUTTON_EXT1_GPIO_PIN, BUTTON_EXTERNAL_ACTIVE_LOW);
    }
    while(1){
        if (Button_External_WasPressed(button_ext1)) {
            ESP_LOGI(TAG, "BUTTON EXT1 PRESSED!");
#if CONFIG_SOFTWARE_UI_SUPPORT
            ui_button_label_update(true);
#endif
        }
        if (Button_External_WasReleased(button_ext1)) {
            ESP_LOGI(TAG, "BUTTON EXT1 RELEASED!");
#if CONFIG_SOFTWARE_UI_SUPPORT
            ui_button_label_update(false);
#endif
        }
        if (Button_External_WasLongPress(button_ext1, pdMS_TO_TICKS(1000))) { // 1Sec
            ESP_LOGI(TAG, "BUTTON EXT1 LONGPRESS!");
#if CONFIG_SOFTWARE_UI_SUPPORT
            ui_button_label_update(false);
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(80));
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif //CONFIG_SOFTWARE_EXTERNAL_BUTTON_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
TaskHandle_t xExternalI2c;
i2c_master_bus_handle_t i2c0_master_bus_handle;
void vExternal_i2c_task(void *pvParametes)
{
    ESP_LOGD(TAG, "start EXTERNAL I2C");
    esp_err_t ret = ESP_OK;
    i2c_master_bus_config_t i2c_mst_config_0 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C0_MASTER_PORT,
        .scl_io_num = I2C0_MASTER_SCL_PIN,
        .sda_io_num = I2C0_MASTER_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&i2c_mst_config_0, &i2c0_master_bus_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGD(TAG, "i2c_new_master_bus is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_new_master_bus error");
    }

#if CONFIG_SOFTWARE_SENSOR_ADT7410
    bool _isSensorAdt7410 = false;
    ret = Adt7410_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Adt7410_Init() is OK!");
        _isSensorAdt7410 = true;
    }
    else
    {
        ESP_LOGE(TAG, "Adt7410_Init Error");
        g_sensor_mode -= 1;
    }
#endif

#if CONFIG_SOFTWARE_SENSOR_BMP280
    bool _isSensorBmp280 = false;
    ret = Bmp280_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Bmp280_Init() is OK!");
        _isSensorBmp280 = true;
    }
    else
    {
        ESP_LOGE(TAG, "Bmp280_Init Error");
        g_sensor_mode -= 4;
    }
#endif
#if CONFIG_SOFTWARE_SENSOR_QMP6988
    bool _isSensorQmp6988 = false;
    ret = Qmp6988_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Qmp6988_Init() is OK!");
        _isSensorQmp6988 = true;
    }
    else
    {
        ESP_LOGE(TAG, "Qmp6988_Init Error");
        g_sensor_mode -= 4;
    }
#endif

#if CONFIG_SOFTWARE_SENSOR_BME680
    bool _isSensorBme680 = false;
    ret = Esp32_Bme680_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Esp32_Bme680_Init() is OK!");
        _isSensorBme680 = true;
    }
    else
    {
        ESP_LOGE(TAG, "Esp32_Bme680_Init Error");
        g_sensor_mode -= 7;
    }
#endif

#if CONFIG_SOFTWARE_SENSOR_SHT3X
    bool _isSensorSht3x = false;
    ret = Sht3x_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Sht3x_Init() is OK!");
        _isSensorSht3x = true;
    }
    else
    {
        ESP_LOGE(TAG, "Sht3x_Init Error");
        g_sensor_mode -= 3;
    }
#endif
#if CONFIG_SOFTWARE_SENSOR_SHT4X
    bool _isSensorSht4x = false;
    ret = Sht4x_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Sht4x_Init() is OK!");
        _isSensorSht4x = true;
    }
    else
    {
        ESP_LOGE(TAG, "Sht4x_Init Error");
        g_sensor_mode -= 3;
    }
#endif

#if CONFIG_SOFTWARE_SENSOR_BH1750
    bool _isSensorBh1750 = false;
    ret = BH1750_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "BH1750_Init() is OK!");
        _isSensorBh1750 = true;

        ret = BH1750_SetMode(BH1750_CONTINUOUSLY_H_RESOLUTION_MODE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BH1750_SetMode Error");
        }
    }
    else
    {
        ESP_LOGE(TAG, "BH1750_Init Error");
        g_sensor_mode -= 16;
    }

#endif

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
    ret = PCF8563_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "PCF8563_Init() is OK!");

        RtcInterruptInit();
        RtcStartClockInit();
    }
    else
    {
        ESP_LOGE(TAG, "PCF8563_Init Error");
    }
#endif

    while (1) {
#if CONFIG_SOFTWARE_SENSOR_ADT7410
        if (_isSensorAdt7410) {
            g_temperature = Adt7410_getTemperature_13bit();
        }
#endif

#if CONFIG_SOFTWARE_SENSOR_BMP280
        if (_isSensorBmp280) {
            g_pressure = Bmp280_getPressure() / 100.0;
        }
#endif
#if CONFIG_SOFTWARE_SENSOR_QMP6988
        if (_isSensorQmp6988) {
            g_pressure = Qmp6988_CalcPressure() / 100.0;
        }
#endif

#if CONFIG_SOFTWARE_SENSOR_BME680
        if (_isSensorBme680) {
            if (Esp32_Bme680_read_sensor_data() == 0) {
                g_temperature = Esp32_Bme680_get_temperature();
                g_humidity = Esp32_Bme680_get_humidity();
                g_pressure = Esp32_Bme680_get_pressure() / 100.0;
//                g_gas = Esp32_Bme680_get_gas() / 1000.0;
            }
        }
#endif

#if CONFIG_SOFTWARE_SENSOR_SHT3X
        if (_isSensorSht3x) {
            ret = Sht3x_Read();
            if (ret == ESP_OK) {
                vTaskDelay( pdMS_TO_TICKS(100) );
                g_temperature = Sht3x_GetTemperature();
                g_humidity = Sht3x_GetHumidity();
            }
        }
#endif
#if CONFIG_SOFTWARE_SENSOR_SHT4X
        if (_isSensorSht4x) {
            ret = Sht4x_Read();
            if (ret == ESP_OK) {
                vTaskDelay( pdMS_TO_TICKS(100) );
                g_temperature = Sht4x_GetTemperature();
                g_humidity = Sht4x_GetHumidity();
            }
        }
#endif

#if CONFIG_SOFTWARE_SENSOR_BH1750
        if (_isSensorBh1750) {
            uint16_t tmp_lux = 0;
            tmp_lux = 0;
            tmp_lux = BH1750_GetLUX();
            if (tmp_lux != 0)
            {
                g_lux = tmp_lux;
            } else {
                ESP_LOGE(TAG, "BH1750_GetLUX is return lux = 0");
                vTaskDelay( pdMS_TO_TICKS(1000) );
            }
        }
#endif
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }
}
#endif //CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_UART_SUPPORT
#if CONFIG_SOFTWARE_SENSOR_MHZ19C
bool g_mhz19cInitialized = false;
TaskHandle_t xExternalUart;
void vExternal_uart_task(void *pvParametes)
{
    ESP_LOGD(TAG, "start EXTERNAL UART");
    esp_err_t ret = ESP_OK;
    ret = Mhz19c_Init(UART1_PORT, UART1_TXD_PIN, UART1_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART1_RATE_BAUD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mhz19c_Init Error");
        g_sensor_mode -= 8;
        vTaskDelay( pdMS_TO_TICKS(500) );
        vTaskDelete(NULL);
    }
    while (!g_mhz19cInitialized)
    {
        Mhz19c_SetAutoCalibration(UART1_PORT, false);
        vTaskDelay( pdMS_TO_TICKS(5000) );

        g_co2 = Mhz19c_GetCO2Concentration(UART1_PORT);
//        if (g_co2 != 0)
        if (g_co2 >= 400)
        {
            g_mhz19cInitialized = true;
        }
        ESP_LOGE(TAG, "Mhz19c_SetAutoCalibration retry");
        Mhz19c_SetAutoCalibration(UART1_PORT, true);
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }
    ESP_LOGD(TAG, "Mhz19c_Init() is OK!");

    int tmp_co2 = 0;
    while (1) {
        tmp_co2 = 0;
        tmp_co2 = Mhz19c_GetCO2Concentration(UART1_PORT);
        if (tmp_co2 != 0)
        {
            g_co2 = tmp_co2;
        } else {
            ESP_LOGE(TAG, "Mhz19c_GetCO2Concentration is return co2 = 0");
            vTaskDelay( pdMS_TO_TICKS(1000) );
        }

       vTaskDelay( pdMS_TO_TICKS(5000) );
    }
}
#endif //CONFIG_SOFTWARE_SENSOR_MHZ19C
#endif //CONFIG_SOFTWARE_EXTERNAL_UART_SUPPORT


#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
TaskHandle_t xClock;
static QueueHandle_t gpio_evt_clock_queue = NULL;
static QueueHandle_t gpio_evt_int_queue = NULL;
static void IRAM_ATTR gpio_clock_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_clock_queue, &gpio_num, NULL);
}

static void IRAM_ATTR gpio_int_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_int_queue, &gpio_num, NULL);
}

static void gpio_clock_task(void* arg)
{
#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
    bool bTm1637Init = false;
    DigitDisplay_t* digitdisplay_1;
    uint8_t listDisp_1[XDIGIT_DISPLAY_DIGIT_COUNT];
    Tm1637_Init();
    if (Tm1637_Enable(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN, BRIGHT_DARKEST, XDIGIT_DISPLAY_DIGIT_COUNT);
        bTm1637Init = true;
        Tm1637_ClearDisplay(digitdisplay_1);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
    }
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT

    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_clock_queue, &io_num, portMAX_DELAY)) {
//            ESP_LOGI(TAG, "CLOCK GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));
            rtc_date_t rtcdate;
            PCF8563_GetTime(&rtcdate);
#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
            if (bTm1637Init == true)
            {
                uint8_t hour10 = rtcdate.hour / 10;
                uint8_t hour01 = ( rtcdate.hour % 10 );
                uint8_t minute10 = rtcdate.minute / 10;
                uint8_t minute01 = ( rtcdate.minute % 10 );
                uint8_t second10 = rtcdate.second / 10;
                uint8_t second01 = ( rtcdate.second % 10 );
                listDisp_1[0] = hour10;
                listDisp_1[1] = hour01;
                listDisp_1[2] = minute10;
                listDisp_1[3] = minute01;
                listDisp_1[4] = second10;
                listDisp_1[5] = second01;
                Tm1637_DisplayAll(digitdisplay_1, listDisp_1);
            }
            else
            {
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
                ESP_LOGI(TAG, "%04d/%02d/%02d %02d:%02d:%02d", rtcdate.year, rtcdate.month, rtcdate.day, rtcdate.hour, rtcdate.minute, rtcdate.second);
#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
            }
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
        }
    }
}

static void gpio_int_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_int_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "INT GPIO[%"PRIu32"] intr, val: %d", io_num, gpio_get_level(io_num));
        }
    }
}

void RtcInterruptInit()
{
    ESP_LOGI(TAG, "start RtcInterruptInit()");

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((1ULL<<PCF8563__CLOCK_PIN) | (1ULL<<PCF8563__INT_PIN));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_evt_clock_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_evt_int_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_clock_task, "gpio_clock_task", 4096 * 1, NULL, 10, NULL);
    xTaskCreate(gpio_int_task, "gpio_int_task", 4096 * 1, NULL, 10, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PCF8563__CLOCK_PIN, gpio_clock_isr_handler, (void*) PCF8563__CLOCK_PIN));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PCF8563__INT_PIN, gpio_int_isr_handler, (void*) PCF8563__INT_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(PCF8563__CLOCK_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(PCF8563__INT_PIN));
    ESP_ERROR_CHECK(gpio_set_intr_type(PCF8563__CLOCK_PIN, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(PCF8563__INT_PIN, GPIO_INTR_NEGEDGE));

}

void RtcStartClockInit()
{
    ESP_LOGI(TAG, "start RtcStartClockInit()");
#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
#if CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT

    while (!PCF8563_isInitialized()) {
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }
    g_clockout_status = 1;
    PCF8563_ClockOutForTrimmer(g_clockout_status);
    ESP_LOGI(TAG, "PCF8563_ClockOutForTrimmer: %d", g_clockout_status);

#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_CLOCKOUT_1KHZ_SUPPORT
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
}

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
TaskHandle_t xExternalRtc;
static bool g_timeInitialized = false;

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGD(TAG, "Notification of a time synchronization event");
    g_timeInitialized = true;
}

void vExternal_rtc_task(void *pvParametes)
{
    //PCF8563
    ESP_LOGD(TAG, "start EXTERNAL Rtc");

    // Set timezone to Japan Standard Time
    setenv("TZ", "JST-9", 1);
    tzset();

    ESP_LOGD(TAG, "NTP ServerName:%s", CONFIG_NTP_SERVER_NAME);
    esp_sntp_setservername(0, CONFIG_NTP_SERVER_NAME);

    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    while (1) {
        if (wifi_isConnected() == ESP_OK) {
            esp_sntp_init();
        } else {
            vTaskDelay( pdMS_TO_TICKS(60000) );
            continue;
        }

        ESP_LOGD(TAG, "Waiting for time synchronization with SNTP server");
        while (!g_timeInitialized)
        {
            vTaskDelay( pdMS_TO_TICKS(5000) );
        }

        time_t now = 0;
        struct tm timeinfo = {0};
        time(&now);
        localtime_r(&now, &timeinfo);
        sprintf(g_lastSyncDatetime,"NTP Update : %04d/%02d/%02d %02d:%02d", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min);
        ESP_LOGI(TAG, "%s", g_lastSyncDatetime);
#if CONFIG_SOFTWARE_UI_SUPPORT
        ui_status_set(g_lastSyncDatetime);
#endif //CONFIG_SOFTWARE_UI_SUPPORT

        if (PCF8563_isInitialized() == true)
        {
            rtc_date_t rtcdate;
            rtcdate.year = timeinfo.tm_year+1900;
            rtcdate.month = timeinfo.tm_mon+1;
            rtcdate.day = timeinfo.tm_mday;
            rtcdate.hour = timeinfo.tm_hour;
            rtcdate.minute = timeinfo.tm_min;
            rtcdate.second = timeinfo.tm_sec;
            PCF8563_SetTime(&rtcdate);
        }

        g_timeInitialized = false;
        esp_sntp_stop();
        vTaskDelay( pdMS_TO_TICKS(CONFIG_NTP_UPDATE_INTERVAL_TIME_MS) );
    }
}
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT

void vClock_task(void *pvParametes)
{
    //PCF8563
    ESP_LOGD(TAG, "start Clock");
 
    // Set timezone to Japan Standard Time
    setenv("TZ", "JST-9", 1);
    tzset();

    while (!PCF8563_isInitialized()) {
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }

    while (1) {
        rtc_date_t rtcdate;
        PCF8563_GetTime(&rtcdate);
#if CONFIG_SOFTWARE_CLOCK_SOUND_SUPPORT
        if (g_clockCurrent != rtcdate.hour) {
            if (rtcdate.hour == 0) { // 0:00
                g_clockCount = 12;
            } else if (rtcdate.hour > 12) { // 13:00-23:00
                g_clockCount = rtcdate.hour - 12;
            } else { // 1:00-12:00
                g_clockCount = rtcdate.hour;
            }
            g_clockCurrent = rtcdate.hour;
            vTaskResume(xSpeaker);
        }
#endif //CONFIG_SOFTWARE_CLOCK_SOUND_SUPPORT
        char str1[30] = {0};
        sprintf(str1,"%04d/%02d/%02d %02d:%02d:%02d", rtcdate.year, rtcdate.month, rtcdate.day, rtcdate.hour, rtcdate.minute, rtcdate.second);
#if CONFIG_SOFTWARE_UI_SUPPORT
        ui_datetime_set(str1);
#else
#if CONFIG_NTP_CLOCK_LOG_ENABLE
    ESP_LOGI(TAG, "%s", str1);
#endif //CONFIG_NTP_CLOCK_LOG_ENABLE
#endif //CONFIG_SOFTWARE_UI_SUPPORT

        vTaskDelay( pdMS_TO_TICKS(990) );
    }
}
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT


#if CONFIG_SOFTWARE_SENSOR_USE_SENSOR
#if (CONFIG_SOFTWARE_ESP_MQTT_SUPPORT != 1)
TaskHandle_t xSensorViewer;
void vSensor_Viewer_task(void *pvParametes)
{
    ESP_LOGD(TAG, "start Sensor Viewer");
    vTaskDelay( pdMS_TO_TICKS(10000) );

    #if CONFIG_SOFTWARE_SENSOR_TYPE_TEMPERATURE
    g_sensor_mode += 1;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_HUMIDITY
    g_sensor_mode += 2;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_PRESSURE
    g_sensor_mode += 4;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_CO2
    g_sensor_mode += 8;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_LIGHT
    g_sensor_mode += 16;
    #endif

    while (1) {
        ESP_LOGD(TAG, "SENSOR mode %d", g_sensor_mode);
        switch (g_sensor_mode)
        {
        case 1:
            ESP_LOGI(TAG, "SENSOR temperature:%f", g_temperature);
            break;
        case 2:
            ESP_LOGI(TAG, "SENSOR humidity:%f", g_humidity);
            break;
        case 3:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f", g_temperature, g_humidity);
            break;
        case 4:
            ESP_LOGI(TAG, "SENSOR pressure:%f", g_pressure);
            break;
        case 5:
            ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f", g_temperature, g_pressure);
            break;
        case 6:
            ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f", g_humidity, g_pressure);
            break;
        case 7:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f", g_temperature, g_humidity, g_pressure);
            break;
        case 8:
            ESP_LOGI(TAG, "SENSOR CO2:%d", g_co2);
            break;
        case 9:
            ESP_LOGI(TAG, "SENSOR temperature:%f CO2:%d", g_temperature, g_co2);
            break;
        case 10:
            ESP_LOGI(TAG, "SENSOR humidity:%f CO2:%d", g_humidity, g_co2);
            break;
        case 11:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f CO2:%d", g_temperature, g_humidity, g_co2);
            break;
        case 12:
            ESP_LOGI(TAG, "SENSOR pressure:%f CO2:%d", g_pressure, g_co2);
            break;
        case 13:
            ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f CO2:%d", g_temperature, g_pressure, g_co2);
            break;
        case 14:
            ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f CO2:%d", g_humidity, g_pressure, g_co2);
            break;
        case 15:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f CO2:%d", g_temperature, g_humidity, g_pressure, g_co2);
            break;
        case 16:
            ESP_LOGI(TAG, "SENSOR lux:%u", g_lux);
            break;
        case 17:
            ESP_LOGI(TAG, "SENSOR temperature:%f lux:%u", g_temperature, g_lux);
            break;
        case 18:
            ESP_LOGI(TAG, "SENSOR humidity:%f lux:%u", g_humidity, g_lux);
            break;
        case 19:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f lux:%u", g_temperature, g_humidity, g_lux);
            break;
        case 20:
            ESP_LOGI(TAG, "SENSOR pressure:%f lux:%u", g_pressure, g_lux);
            break;
        case 21:
            ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f lux:%u", g_temperature, g_pressure, g_lux);
            break;
        case 22:
            ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f lux:%u", g_humidity, g_pressure, g_lux);
            break;
        case 23:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f lux:%u", g_temperature, g_humidity, g_pressure, g_lux);
            break;
        case 24:
            ESP_LOGI(TAG, "SENSOR CO2:%d lux:%u", g_co2, g_lux);
            break;
        case 25:
            ESP_LOGI(TAG, "SENSOR temperature:%f CO2:%d lux:%u", g_temperature, g_co2, g_lux);
            break;
        case 26:
            ESP_LOGI(TAG, "SENSOR humidity:%f CO2:%d lux:%u", g_humidity, g_co2, g_lux);
            break;
        case 27:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f CO2:%d lux:%u", g_temperature, g_humidity, g_co2, g_lux);
            break;
        case 28:
            ESP_LOGI(TAG, "SENSOR pressure:%f CO2:%d lux:%u", g_pressure, g_co2, g_lux);
            break;
        case 29:
            ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f CO2:%d lux:%u", g_temperature, g_pressure, g_co2, g_lux);
            break;
        case 30:
            ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f CO2:%d lux:%u", g_humidity, g_pressure, g_co2, g_lux);
            break;
        case 31:
            ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f CO2:%d lux:%u", g_temperature, g_humidity, g_pressure, g_co2, g_lux);
            break;
        default:
            break;
        }

        if (g_loopCount == UINT8_MAX)
        {
            g_loopCount = 0;
        }
        else
        {
            g_loopCount++;
        }
        ESP_LOGD(TAG, "g_loopCount = %d. MAX = %d", g_loopCount, UINT8_MAX);

        vTaskDelay( pdMS_TO_TICKS(10000) );
    }
}
#endif //(CONFIG_SOFTWARE_ESP_MQTT_SUPPORT != 1)
#endif //CONFIG_SOFTWARE_SENSOR_USE_SENSOR


#if CONFIG_SOFTWARE_EXTERNAL_LED_SUPPORT
TaskHandle_t xExternalLED;
Led_t* led_ext1;
static void vExternal_led_task(void* pvParameters) {
    ESP_LOGD(TAG, "start EXTERNAL LED");
    Led_Init();

    if (Led_Enable(LED_EXT1_GPIO_PIN) == ESP_OK) {
        led_ext1 = Led_Attach(LED_EXT1_GPIO_PIN);
    }
    while(1){
        Led_OnOff(led_ext1, true);
        vTaskDelay(pdMS_TO_TICKS(700));

        Led_OnOff(led_ext1, false);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    vTaskDelete(NULL); // Should never get to here...
}
#endif //CONFIG_SOFTWARE_EXTERNAL_LED_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT
TaskHandle_t xExternalRGBLedBlink;
rmt_channel_handle_t rgb_ext1_channel = NULL;
rmt_encoder_handle_t rgb_ext1_encoder = NULL;
pixel_settings_t px_ext1;

void vExternal_RGBLedBlink_task(void *pvParametes)
{
    ESP_LOGD(TAG, "start EXTERNAL RGBLedBlink");

    esp_err_t res = ESP_OK;
    uint8_t rgbled_count = 8;
    uint8_t blink_count = 5;
    uint32_t colors[] = {SK6812_COLOR_BLUE, SK6812_COLOR_LIME, SK6812_COLOR_AQUA
                    , SK6812_COLOR_RED, SK6812_COLOR_MAGENTA, SK6812_COLOR_YELLOW
                    , SK6812_COLOR_WHITE};

    res = Sk6812_Init_Ex(&px_ext1, rgbled_count, RGBLED_EXT1_GPIO_PIN, &rgb_ext1_channel, &rgb_ext1_encoder);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "vExternal_RGBLedBlink_task() Sk6812_Init_Ex");
	}
    res = Sk6812_Enable_Ex(&px_ext1, rgb_ext1_channel);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "vExternal_RGBLedBlink_task() Sk6812_Enable_Ex");
	}
    if (rgb_ext1_channel == NULL || rgb_ext1_encoder == NULL) {
        ESP_LOGE(TAG, "vExternal_RGBLedBlink_task() NULL!");
    }
    res = Sk6812_Clear_Ex(&px_ext1, rgb_ext1_channel, rgb_ext1_encoder);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "vExternal_RGBLedBlink_task() Sk6812_Clear_Ex");
	}

    while (1) {
        for (uint8_t c = 0; c < sizeof(colors)/sizeof(uint32_t); c++) {
            for (uint8_t i = 0; i < blink_count; i++) {
                Sk6812_SetAllColor_Ex(&px_ext1, colors[c]);
                Sk6812_Show_Ex(&px_ext1, rgb_ext1_channel, rgb_ext1_encoder);
                vTaskDelay(pdMS_TO_TICKS(1000));

                Sk6812_SetAllColor_Ex(&px_ext1, SK6812_COLOR_OFF);
                Sk6812_Show_Ex(&px_ext1, rgb_ext1_channel, rgb_ext1_encoder);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}
#endif //CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT

#if CONFIG_HTTPS_OTA_SUPPORT
TaskHandle_t xHttpsOta;
/*
static void event_handler(void* arg, esp_event_base_t event_base,
                        int32_t event_id, void* event_data)
{
    if (event_base == ESP_HTTPS_OTA_EVENT) {
        switch (event_id) {
            case ESP_HTTPS_OTA_START:
                ESP_LOGI(TAG, "OTA started");
                break;
            case ESP_HTTPS_OTA_CONNECTED:
                ESP_LOGI(TAG, "Connected to server");
                break;
            case ESP_HTTPS_OTA_GET_IMG_DESC:
                ESP_LOGI(TAG, "Reading Image Description");
                break;
            case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
                ESP_LOGI(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
                break;
            case ESP_HTTPS_OTA_DECRYPT_CB:
                ESP_LOGI(TAG, "Callback to decrypt function");
                break;
            case ESP_HTTPS_OTA_WRITE_FLASH:
                ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data);
                break;
            case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
                ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
                break;
            case ESP_HTTPS_OTA_FINISH:
                ESP_LOGI(TAG, "OTA finish");
                break;
            case ESP_HTTPS_OTA_ABORT:
                ESP_LOGI(TAG, "OTA abort");
                break;
        }
    }
}
*/

esp_err_t do_firmware_upgrade()
{
    esp_http_client_config_t config = {
        .url = CONFIG_HTTPS_OTA_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
//        .cert_pem = (char *)server_cert_pem_start,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void vHttpsOta_task(void* pvParameters) {
    ESP_LOGI(TAG, "start HTTPS OTA");

    while(1){
        ESP_LOGI(TAG, "do_firmware_upgrade()");
        do_firmware_upgrade();
        vTaskDelay(pdMS_TO_TICKS(300000));
    }

    vTaskDelete(NULL); // Should never get to here...
}
#endif //CONFIG_HTTPS_OTA_SUPPORT

#if CONFIG_DEEP_SLEEP_SUPPORT
#if SOC_RTC_FAST_MEM_SUPPORTED
static RTC_DATA_ATTR struct timeval sleep_enter_time;
#else
static struct timeval sleep_enter_time;
#endif

static void deep_sleep_task(void *args)
{
    /**
     * Prefer to use RTC mem instead of NVS to save the deep sleep enter time, unless the chip
     * does not support RTC mem(such as esp32c2). Because the time overhead of NVS will cause
     * the recorded deep sleep enter time to be not very accurate.
     */
#if !SOC_RTC_FAST_MEM_SUPPORTED
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "Open NVS done");
    }

    // Get deep sleep enter time
    nvs_get_i32(nvs_handle, "slp_enter_sec", (int32_t *)&sleep_enter_time.tv_sec);
    nvs_get_i32(nvs_handle, "slp_enter_usec", (int32_t *)&sleep_enter_time.tv_usec);
#endif

    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            ESP_LOGD(TAG, "Wake up from timer. Time spent in deep sleep: %dms", sleep_time_ms);
            break;
        }

#if CONFIG_DEEP_SLEEP_GPIO_WAKEUP
        case ESP_SLEEP_WAKEUP_GPIO: {
            uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                ESP_LOGD(TAG, "Wake up from GPIO %d", pin);
            } else {
                ESP_LOGD(TAG, "Wake up from GPIO");
            }
            break;
        }
#endif //CONFIG_EXAMPLE_GPIO_WAKEUP
/*
#if CONFIG_EXAMPLE_EXT0_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT0: {
            printf("Wake up from ext0\n");
            break;
        }
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP

#ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
#endif // CONFIG_EXAMPLE_EXT1_WAKEUP

#ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
        case ESP_SLEEP_WAKEUP_TOUCHPAD: {
            printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
            break;
        }
#endif // CONFIG_EXAMPLE_TOUCH_WAKEUP
*/
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGD(TAG, "Not a deep sleep reset");
    }

//    vTaskDelay(1000 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(1000));
/*
#if CONFIG_IDF_TARGET_ESP32
    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif
*/

#if CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
    while (g_loopCount < 2)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGD(TAG, "check g_loopCount");
#endif //CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
    ESP_LOGD(TAG, "call esp_wifi_disconnect()");
    esp_wifi_disconnect();
    ESP_LOGD(TAG, "call esp_wifi_stop()");
    esp_wifi_stop();
    ESP_LOGD(TAG, "call esp_wifi_deinit()");
    esp_wifi_deinit();
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
    vTaskDelay(pdMS_TO_TICKS(500));


    ESP_LOGI(TAG, "Entering deep sleep");

    // get deep sleep enter time
    gettimeofday(&sleep_enter_time, NULL);

#if !SOC_RTC_FAST_MEM_SUPPORTED
    // record deep sleep enter time via nvs
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_sec", sleep_enter_time.tv_sec));
    ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "slp_enter_usec", sleep_enter_time.tv_usec));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
#endif

    // enter deep sleep
    esp_deep_sleep_start();
}

#if CONFIG_DEEP_SLEEP_GPIO_WAKEUP
#define DEFAULT_WAKEUP_PIN      CONFIG_DEEP_SLEEP_GPIO_WAKEUP_PIN
#ifdef CONFIG_DEEP_SLEEP_GPIO_WAKEUP_HIGH_LEVEL
#define DEFAULT_WAKEUP_LEVEL    ESP_GPIO_WAKEUP_GPIO_HIGH
#else
#define DEFAULT_WAKEUP_LEVEL    ESP_GPIO_WAKEUP_GPIO_LOW
#endif

void deep_sleep_register_gpio_wakeup(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = BIT(DEFAULT_WAKEUP_PIN),
        .mode = GPIO_MODE_INPUT,
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT(DEFAULT_WAKEUP_PIN), DEFAULT_WAKEUP_LEVEL));

    ESP_LOGI(TAG, "Enabling GPIO wakeup on pins GPIO%d", DEFAULT_WAKEUP_PIN);
}
#endif

#define DEFAULT_WAKEUP_TIME_SEC      CONFIG_DEEP_SLEEP_WAKEUP_TIME_SEC
static void deep_sleep_register_rtc_timer_wakeup(void)
{
    ESP_LOGI(TAG, "Enabling timer wakeup, %ds", DEFAULT_WAKEUP_TIME_SEC);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(DEFAULT_WAKEUP_TIME_SEC * 1000000));
}
#endif

#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
TaskHandle_t xDigitDisplay;
DigitDisplay_t* digitdisplay_1;
void vDigitDisplayTask(void *pvParametes)
{
    ESP_LOGI(TAG, "start Digit Display");

// Sample Rowdata
//  --0x01--
// |        |
//0x20     0x02
// |        |
//  --0x40- -
// |        |
//0x10     0x04
// |        |
//  --0x08--   0x80

// CountDown
    uint8_t listDisp_1[XDIGIT_DISPLAY_DIGIT_COUNT];
    uint8_t count = UINT8_MAX;
    Tm1637_Init();
    if (Tm1637_Enable(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN, BRIGHT_TYPICAL, XDIGIT_DISPLAY_DIGIT_COUNT);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
    }
    Tm1637_ClearDisplay(digitdisplay_1);
    while(1){
        if (count == 0) {
            count = UINT8_MAX;
        }
        uint8_t digit3 = count / 100;
        uint8_t digit2 = ( count % 100 ) / 10;
        uint8_t digit1 = ( count % 10 );
        listDisp_1[0] = 0x7f; //DISPLAY_OFF
        listDisp_1[1] = 0x7f; //DISPLAY_OFF
        listDisp_1[2] = 0x7f; //DISPLAY_OFF
        listDisp_1[3] = digit3;
        listDisp_1[4] = digit2;
        listDisp_1[5] = digit1;
        Tm1637_DisplayAll(digitdisplay_1, listDisp_1);
        count--;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    vTaskDelete(NULL); // Should never get to here...

/*
    uint8_t anime[] = {0x00, 0x30, 0x38, 0x78, 0x79, 0x7f};
    uint8_t animeCurrent = 0;
    uint8_t animeMax = sizeof(anime)/sizeof(uint8_t);
    uint8_t animeDigitPosition = 1;
    uint8_t animeDigitMax = XDIGIT_DISPLAY_DIGIT_COUNT;
    uint8_t data = 0x00;

    Tm1637_Init();
    if (Tm1637_Enable(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN, BRIGHT_TYPICAL, XDIGIT_DISPLAY_DIGIT_COUNT);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
        return;
    }
    while(1) {
        Tm1637_ClearDisplay(digitdisplay_1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        for (animeDigitPosition = 1; animeDigitPosition < animeDigitMax+1; animeDigitPosition++) {
            for (animeCurrent = 0; animeCurrent < animeMax; animeCurrent++) {
                for (uint8_t position = DIGIT_COUNT; position > animeDigitPosition-1; position--) {
                    for (uint8_t digit = DIGIT_COUNT; digit > 0; digit--) {
                        data = 0x00;
                        if (digit == position) {
                            data += 0x80;
                        } else {
                            //data = 0x00;
                        }

                        if (digit == animeDigitPosition) {
                            data += anime[animeCurrent];
                        } else if (digit < animeDigitPosition) {
                            data = 0xff;
                        }

                        Tm1637_DisplayBitRowdata(digitdisplay_1, digit-1, data);
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }
    }
    vTaskDelete(NULL); // Should never get to here...
*/
/*
// Sample number
    uint8_t listDisp_1[XDIGIT_DISPLAY_DIGIT_COUNT];
    uint8_t count = 0;
    Tm1637_Init();
    if (Tm1637_Enable(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN, BRIGHT_TYPICAL, XDIGIT_DISPLAY_DIGIT_COUNT);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
    }
    Tm1637_ClearDisplay(digitdisplay_1);
    while(1){
        if (count == UINT8_MAX) {
            count = 0;
        }
        if (count%2) {
            for (uint8_t i = 0; i < DIGIT_COUNT; i++) {
                listDisp_1[i] = i;
            }
        } else {
            for (uint8_t i = 0; i < DIGIT_COUNT; i++) {
                listDisp_1[i] = i+4;
            }
        }
        Tm1637_DisplayAll(digitdisplay_1, listDisp_1);
        count++;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    vTaskDelete(NULL); // Should never get to here...
*/
/*
// Sample message
    Tm1637_Init();
    if (Tm1637_Enable(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(XDIGIT_DISPLAY_CLK_EXT1_GPIO_PIN, XDIGIT_DISPLAY_DATA_EXT1_GPIO_PIN, BRIGHT_TYPICAL, XDIGIT_DISPLAY_DIGIT_COUNT);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
    }
    while (1) {
        Tm1637_ClearDisplay(digitdisplay_1);
        vTaskDelay(pdMS_TO_TICKS(500));
        Tm1637_DisplayStr(digitdisplay_1, "HELL0-1234567890", 500);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL); // Should never get to here...
*/
}
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT

#if CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT
    // ADC
//    xTaskCreate(&vMeasureOperationVoltageTask, "vMeasureOperationVoltageTask", 4096 * 1, NULL, 2, &vMeasureOperationVoltage);
TaskHandle_t xMeasureOperationVoltage;

// ADC Calibration
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif //ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif //ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static int orignalVoltage(int vol)
{
//    return (int)(vol / 100.0 * (100.0+270.0));
    return (int)(vol / MEASURE_OPERATION_VOLTAGE_RESISTOR2 * ( MEASURE_OPERATION_VOLTAGE_RESISTOR2 + MEASURE_OPERATION_VOLTAGE_RESISTOR1 ));
}

void vMeasureOperationVoltageTask(void *pvParametes)
{
    ESP_LOGI(TAG, "start Measure Operation Voltage");
    adc_oneshot_unit_handle_t adc_handle1;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = MEASURE_OPERATION_VOLTAGE_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle1));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = MEASURE_OPERATION_VOLTAGE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle1, MEASURE_OPERATION_VOLTAGE_CHANNEL, &config));

    adc_cali_handle_t adc_cali_handle1 = NULL;
    bool do_calibration1 = adc_calibration_init(MEASURE_OPERATION_VOLTAGE_UNIT, MEASURE_OPERATION_VOLTAGE_CHANNEL, MEASURE_OPERATION_VOLTAGE_ADC_ATTEN, &adc_cali_handle1);

    while(1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle1, MEASURE_OPERATION_VOLTAGE_CHANNEL, &g_adc_raw));
//        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", MEASURE_OPERATION_VOLTAGE_UNIT + 1, MEASURE_OPERATION_VOLTAGE_CHANNEL, g_adc_raw);
        if (do_calibration1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle1, g_adc_raw, &g_voltage));
            ESP_LOGI(TAG, "VOLTAGE ADC%d_CH%d Cali Voltage: %dmV, Operation Voltage: %dmV", MEASURE_OPERATION_VOLTAGE_UNIT + 1, MEASURE_OPERATION_VOLTAGE_CHANNEL, g_voltage, orignalVoltage(g_voltage));
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle1));
    if (do_calibration1) {
        adc_calibration_deinit(adc_cali_handle1);
    }

    vTaskDelete(NULL); // Should never get to here...
}
#endif //CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void app_main(void)
{
    ESP_LOGI(TAG, "app_main() start.");
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("MY-MAIN", ESP_LOG_DEBUG);
    esp_log_level_set("MY-WIFI", ESP_LOG_INFO);


#if CONFIG_SOFTWARE_UI_SUPPORT
    ui_init();
#endif //CONFIG_SOFTWARE_UI_SUPPORT

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
    wifi_initialise();
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
    // I2C
    xTaskCreatePinnedToCore(&vExternal_i2c_task, "external_i2c_task", 4096 * 1, NULL, 2, &xExternalI2c, TASK_CORE);
#endif //CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_UART_SUPPORT
    // UART
    xTaskCreatePinnedToCore(&vExternal_uart_task, "external_uart_task", 4096 * 1, NULL, 2, &xExternalUart, TASK_CORE);
#endif //CONFIG_SOFTWARE_EXTERNAL_UART_SUPPORT

#if CONFIG_SOFTWARE_INTERNAL_BUTTON_SUPPORT
    // BUTTON
    xTaskCreatePinnedToCore(&vInternal_button_task, "internal_button_task", 4096 * 1, NULL, 2, &xInternalButton, TASK_CORE);
#endif //CONFIG_SOFTWARE_INTERNAL_BUTTON_SUPPORT

#if CONFIG_SOFTWARE_SENSOR_USE_SENSOR
#if (CONFIG_SOFTWARE_ESP_MQTT_SUPPORT != 1)
    // SENSOR VIEWER
    xTaskCreatePinnedToCore(&vSensor_Viewer_task, "vSensor_Viewer_task", 4096 * 1, NULL, 2, &xSensorViewer, TASK_CORE);
#endif //(CONFIG_SOFTWARE_ESP_MQTT_SUPPORT != 1)
#endif //CONFIG_SOFTWARE_SENSOR_USE_SENSOR

#if CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT
#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
    // rtc
    xTaskCreatePinnedToCore(&vExternal_rtc_task, "vExternal_rtc_task", 4096 * 1, NULL, 2, &xExternalRtc, TASK_CORE);
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
    // clock
    xTaskCreatePinnedToCore(&vClock_task, "clock_task", 4096 * 1, NULL, 2, &xClock, TASK_CORE);
#endif //CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_LED_SUPPORT
    // EXTERNAL LED
    xTaskCreatePinnedToCore(&vExternal_led_task, "external_led_task", 4096 * 1, NULL, 2, &xExternalLED, TASK_CORE);
#endif //CONFIG_SOFTWARE_EXTERNAL_LED_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_BUTTON_SUPPORT
    // EXTERNAL BUTTON
    xTaskCreatePinnedToCore(&vExternal_button_task, "external_button_task", 4096 * 1, NULL, 2, &xExternalButton, TASK_CORE);
#endif //CONFIG_SOFTWARE_EXTERNAL_BUTTON_SUPPORT

#if CONFIG_SOFTWARE_ESP_MQTT_SUPPORT
    // ESP_MQTT
    xTaskCreatePinnedToCore(&vEspMqttClient_task, "vEspMqttClient_task", 4096 * 1, NULL, 2, &xEspMqttClient, TASK_CORE);
#endif //CONFIG_SOFTWARE_ESP_MQTT_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT
    // EXTERNAL RGB LED BLINK
    xTaskCreatePinnedToCore(&vExternal_RGBLedBlink_task, "External_RGBLedBlink_task", 4096 * 1, NULL, 2, &xExternalRGBLedBlink, TASK_CORE);
#endif //CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT

#if CONFIG_HTTPS_OTA_SUPPORT
    // HTTPS OTA
    xTaskCreatePinnedToCore(&vHttpsOta_task, "HttpsOta_task", 4096 * 1, NULL, 2, &xHttpsOta, TASK_CORE);
#endif //CONFIG_HTTPS_OTA_SUPPORT

#if CONFIG_DEEP_SLEEP_SUPPORT
    // DEEP SLEEP
    /* Enable wakeup from deep sleep by rtc timer */
    deep_sleep_register_rtc_timer_wakeup();
#if CONFIG_DEEP_SLEEP_GPIO_WAKEUP
    deep_sleep_register_gpio_wakeup();
#endif //CONFIG_DEEP_SLEEP_GPIO_WAKEUP
    xTaskCreate(deep_sleep_task, "deep_sleep_task", 4096, NULL, 6, NULL);
#endif //CONFIG_DEEP_SLEEP_SUPPORT

#if CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT
    // DIGIT DISPLAY
//    xTaskCreatePinnedToCore(&vDigitDisplayTask, "vDigitDisplayTask", 4096 * 1, NULL, 2, &xDigitDisplay, TASK_CORE);
#endif //CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT

#if CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT
    // ADC
    xTaskCreate(&vMeasureOperationVoltageTask, "vMeasureOperationVoltageTask", 4096 * 1, NULL, 2, &xMeasureOperationVoltage);
#endif //CONFIG_MEASURE_OPERATION_VOLTAGE_SUPPORT
}