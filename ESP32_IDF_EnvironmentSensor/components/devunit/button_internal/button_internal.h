#pragma once

#include "stdio.h"
#include "driver/gpio.h"
#include "esp_log.h"

typedef enum {
    BUTTON_INTERNAL_PRESS = (1 << 0),       //button was pressed.
    BUTTON_INTERNAL_RELEASE = (1 << 1),     //button was released.
    BUTTON_INTERNAL_LONGPRESS = (1 << 2),   //button was long pressed.
} Button_Internal_PressEvent;

typedef enum _Button_Internal_ButtonActiveType{
    BUTTON_INTERNAL_ACTIVE_LOW = 0,
    BUTTON_INTERNAL_ACTIVE_HIGH = 1,
} Button_Internal_ButtonActiveType;

typedef struct _Button_Internal_t  {
    gpio_num_t pin;             //GPIO
    Button_Internal_ButtonActiveType type;
    uint8_t value;              //Current button touched state
    uint8_t last_value;         //Previous button touched state
    uint32_t last_press_time;   //FreeRTOS ticks when button was last touched
    uint32_t long_press_time;   //Number of FreeRTOS ticks to elapse to consider holding the button a long press
    uint8_t state;              //The button press event
    struct _Button_Internal_t* next;     //Pointer to the next button
} Button_Internal_t;

void Button_Internal_Init();
esp_err_t Button_Internal_Enable(gpio_num_t pin, Button_Internal_ButtonActiveType type);
Button_Internal_t* Button_Internal_Attach(gpio_num_t pin, Button_Internal_ButtonActiveType type);
uint8_t Button_Internal_WasPressed(Button_Internal_t* button);
uint8_t Button_Internal_WasReleased(Button_Internal_t* button);
uint8_t Button_Internal_IsPress(Button_Internal_t* button);
uint8_t Button_Internal_IsRelease(Button_Internal_t* button);
uint8_t Button_Internal_WasLongPress(Button_Internal_t* button, uint32_t long_press_time);
