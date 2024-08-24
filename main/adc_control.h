#ifndef ADC_CONTROL_H
#define ADC_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Define ADC settings
#define ADC1_CHANNEL ADC1_CHANNEL_1 // Corresponds to GPIO 36
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_0
#define MAX_FREQUENCY 10  // Maximum blinking frequency in Hz
#define ADC_MAX_VALUE 4095 
extern QueueHandle_t adc_queue;

// Initialize ADC and GPIO for LED
void init_adc(void);

// ADC reading task
void adc_read_task(void *pvParameter);

#endif // ADC_CONTROL_H
