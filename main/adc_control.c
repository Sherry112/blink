#include "adc_control.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Initialize ADC
void init_adc(void)
{
    // Initialize ADC
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN);
}

// Read ADC value and send frequency to queue
void adc_read_task(void *pvParameter)
{
    int frequency;
    int adc_reading;

    while (1) {
        // Read the potentiometer value
        adc_reading = adc1_get_raw(ADC1_CHANNEL);
        // Map the ADC reading to a frequency range (0-10 Hz)
        frequency = (adc_reading * MAX_FREQUENCY) / ADC_MAX_VALUE;
        ESP_LOGI("TAG", "Blinking frequency of LED is: %d", frequency);
        // Send the frequency to the LED control task
        xQueueSend(adc_queue, &frequency, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));  // Adjust sampling period if needed
    }
}