#include "freertos/FreeRTOS.h"
#include "string.h"
#include "mhz19c.h"

static const char *TAG = "MY-MHZ19C";

static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;
QueueHandle_t uart_queue;
const uint8_t READ_CO2[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
const uint8_t CALIBRATION_ON[9] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
const uint8_t CALIBRATION_OFF[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
uint8_t ReturnValue[9];

int Mhz19c_SendData(uart_port_t uart_num, const uint8_t* data, uint32_t datalen)
{
    const int txBytes = uart_write_bytes(uart_num, data, datalen);
//    ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
//    ESP_LOG_BUFFER_HEXDUMP(TAG, data, txBytes, ESP_LOG_DEBUG);
    return txBytes;
}

int Mhz19c_ReadData(uart_port_t uart_num, uint8_t* data, uint32_t datalen)
{
    const int rxBytes = uart_read_bytes(uart_num, data, datalen, 100);
//    if (rxBytes > 0) {
//        ESP_LOGI(TAG, "Read %d bytes", rxBytes);
//        ESP_LOG_BUFFER_HEXDUMP(TAG, data, rxBytes, ESP_LOG_DEBUG);
//    }
    return rxBytes;
}

bool Mhz19c_Checksum_check(uint8_t ReturnValue[])
{
    uint8_t sum = 0;
    uint8_t checksum = 0;
    for (int i = 1; i < 8; i++)
    {
        sum += ReturnValue[i] & 0xFF;
    }
    checksum = 0xff - sum + 1;
    if (checksum == ReturnValue[8])
    {
        ESP_LOGI(TAG, "checksum OK");
        return true;
    }
    ESP_LOGI(TAG, "checksum NG. %d vs %d", checksum, ReturnValue[8]);
    return false;
}

uint32_t Mhz19c_GetCO2Concentration(uart_port_t uart_num)
{
    Mhz19c_SendData(uart_num, READ_CO2, sizeof(READ_CO2));

    vTaskDelay( pdMS_TO_TICKS(50) );
    memset(ReturnValue, 0x00, sizeof(ReturnValue));

    Mhz19c_ReadData(uart_num, ReturnValue, sizeof(ReturnValue));
/*
    if (ReturnValue[0] == 0xFF && ReturnValue[1] == 0x86 && Mhz19c_Checksum_check(ReturnValue) == true)
    {
        return ReturnValue[2]*256 + ReturnValue[3];
    }
*/
    if (ReturnValue[0] == 0xFF && ReturnValue[1] == 0x86)
    {
        if (Mhz19c_Checksum_check(ReturnValue) == true)
        {
            return ReturnValue[2]*256 + ReturnValue[3];
        }
        else
        {
            ESP_LOGI(TAG, "GetCO2Concentration [0]0xFF vs %2X ,[1]0x86 vs %2X", ReturnValue[0], ReturnValue[1]);
            ESP_LOGD(TAG, "GetCO2Concentration retuen 1");
            return 1;
        }
    }

    ESP_LOGD(TAG, "GetCO2Concentration retuen 0");
    return 0;
}

void Mhz19c_SetAutoCalibration(uart_port_t uart_num, bool mode)
{
    if (mode == true)
    {
        // turned on
        // 0xA0
        Mhz19c_SendData(uart_num, CALIBRATION_ON, sizeof(CALIBRATION_ON));
    }
    else
    {
        // turned off
        // 0x00
        Mhz19c_SendData(uart_num, CALIBRATION_OFF, sizeof(CALIBRATION_OFF));
    }
}

esp_err_t Mhz19c_CheckDeviceConnect(uart_port_t uart_num)
{
    Mhz19c_SendData(uart_num, READ_CO2, sizeof(READ_CO2));

    vTaskDelay( pdMS_TO_TICKS(50) );
    memset(ReturnValue, 0x00, sizeof(ReturnValue));

    int rxBytes = Mhz19c_ReadData(uart_num, ReturnValue, sizeof(ReturnValue));
    if (rxBytes == 0)
    {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t Mhz19c_Init(uart_port_t uart_num, gpio_num_t tx, gpio_num_t rx, gpio_num_t rts, gpio_num_t cts, uint32_t baud)
{
    const uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32
        .source_clk = UART_SCLK_APB
#elif CONFIG_IDF_TARGET_ESP32C6
        .rx_flow_ctrl_thresh = 122,
#endif
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE, TX_BUF_SIZE, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx, rx, rts, cts));

    return Mhz19c_CheckDeviceConnect(uart_num);
}
