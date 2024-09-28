#pragma once

#include "stdio.h"
#include "driver/gpio.h"
#include "esp_log.h"

typedef enum {
    BUTTON_EXTERNAL_PRESS = (1 << 0),       //button was pressed.
    BUTTON_EXTERNAL_RELEASE = (1 << 1),     //button was released.
    BUTTON_EXTERNAL_LONGPRESS = (1 << 2),   //button was long pressed.
} Button_External_PressEvent;

typedef enum _Button_External_ButtonActiveType{
    BUTTON_EXTERNAL_ACTIVE_LOW = 0,
    BUTTON_EXTERNAL_ACTIVE_HIGH = 1,
} Button_External_ButtonActiveType;

typedef struct _Button_External_t  {
    gpio_num_t pin;             //GPIO
    Button_External_ButtonActiveType type;
    uint8_t value;              //Current button touched state
    uint8_t last_value;         //Previous button touched state
    uint32_t last_press_time;   //FreeRTOS ticks when button was last touched
    uint32_t long_press_time;   //Number of FreeRTOS ticks to elapse to consider holding the button a long press
    uint8_t state;              //The button press event
    struct _Button_External_t* next;     //Pointer to the next button
} Button_External_t;

void Button_External_Init();
esp_err_t Button_External_Enable(gpio_num_t pin, Button_External_ButtonActiveType type);
Button_External_t* Button_External_Attach(gpio_num_t pin, Button_External_ButtonActiveType type);
uint8_t Button_External_WasPressed(Button_External_t* button);
uint8_t Button_External_WasReleased(Button_External_t* button);
uint8_t Button_External_IsPress(Button_External_t* button);
uint8_t Button_External_IsRelease(Button_External_t* button);
uint8_t Button_External_WasLongPress(Button_External_t* button, uint32_t long_press_time);
