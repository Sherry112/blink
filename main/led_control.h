#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#define ADC_MAX_VALUE 4095 
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Define GPIO and ADC settings
#define LED_PIN 2

extern QueueHandle_t adc_queue;
// Initialize GPIO for LED
void init_led(void);

// LED control task
void led_control_task(void *pvParameter);
#endif // LED_CONTROL_H
