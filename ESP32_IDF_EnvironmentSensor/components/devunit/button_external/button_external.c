#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "button_external.h"

#define TAG "EXTERNAL_BUTTON"

Button_External_t* button_ahead = NULL;
static SemaphoreHandle_t button_lock = NULL;
static void Button_External_UpdateTask(void *arg);

void Button_External_Init() {
    if (button_lock == NULL) {
        button_lock = xSemaphoreCreateMutex();
        xTaskCreatePinnedToCore(Button_External_UpdateTask, "Button_External", 2 * 1024, NULL, 1, NULL, 0);
    }
}

esp_err_t Button_External_Enable(gpio_num_t pin, Button_External_ButtonActiveType type) {
    esp_err_t ret = ESP_OK;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << pin);

    io_conf.mode = GPIO_MODE_INPUT;
    if (type == BUTTON_EXTERNAL_ACTIVE_LOW) {
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    } else {
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    }
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error configuring GPIO %d. Error code: 0x%x.", pin, ret);
    }
    return ret;
}

Button_External_t* Button_External_Attach(gpio_num_t pin, Button_External_ButtonActiveType type) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    Button_External_t *button = (Button_External_t *)malloc(sizeof(Button_External_t) * 1);
    button->pin = pin;
    button->type = type;
    button->last_value = 0;
    button->last_press_time = 0;
    button->long_press_time = 0;
    button->next = NULL;
    button->state = 0;
    if (button_ahead == NULL) {
        button_ahead = button;
    } else {
        Button_External_t* button_last = button_ahead;
        while (button_last->next != NULL) {
            button_last = button_last->next;
        }
        button_last->next = button;
    }
    xSemaphoreGive(button_lock);
    return button;
}

bool Button_External_Read(gpio_num_t pin) {
    //- 0 the GPIO input level is 0
    //- 1 the GPIO input level is 1
    return gpio_get_level(pin);
}

uint8_t Button_External_WasPressed(Button_External_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->state & BUTTON_EXTERNAL_PRESS) > 0;
    button->state &= ~BUTTON_EXTERNAL_PRESS;
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_External_WasReleased(Button_External_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->state & BUTTON_EXTERNAL_RELEASE) > 0;
    button->state &= ~BUTTON_EXTERNAL_RELEASE;
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_External_WasLongPress(Button_External_t* button, uint32_t long_press_time) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    button->long_press_time = long_press_time;
    uint8_t result = (button->state & BUTTON_EXTERNAL_LONGPRESS) > 0;
    button->state &= ~BUTTON_EXTERNAL_LONGPRESS;
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_External_IsPress(Button_External_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->value == 1);
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_External_IsRelease(Button_External_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->value == 0);
    xSemaphoreGive(button_lock);
    return result;
}

void Button_External_Update(Button_External_t* button, uint8_t press) {
    uint8_t value = press;
    uint32_t now_ticks = xTaskGetTickCount();
    if (value != button->last_value) {
        if (value == 1) {
            button->state |= BUTTON_EXTERNAL_PRESS;
            button->last_press_time = now_ticks;
        } else {
            if (button->long_press_time && (now_ticks - button->last_press_time > button->long_press_time)) {
                button->state |= BUTTON_EXTERNAL_LONGPRESS;
            } else {
                button->state |= BUTTON_EXTERNAL_RELEASE;
            }
        }
        button->last_value = value;
    }
    button->last_value = value;
    button->value = value;
}

static void Button_External_UpdateTask(void *arg) {
    Button_External_t* button;

    for (;;) {
        xSemaphoreTake(button_lock, portMAX_DELAY);
        button = button_ahead;
        while (button != NULL) {
            if (button->type == BUTTON_EXTERNAL_ACTIVE_LOW) {
                Button_External_Update(button, !Button_External_Read(button->pin));
            } else {
                Button_External_Update(button, Button_External_Read(button->pin));
            }
            button = button->next;
        }
        xSemaphoreGive(button_lock);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}