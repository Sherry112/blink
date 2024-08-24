// // /* Blink Example

// //    This example code is in the Public Domain (or CC0 licensed, at your option.)

// //    Unless required by applicable law or agreed to in writing, this
// //    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// //    CONDITIONS OF ANY KIND, either express or implied.
// // */
// // #include <stdio.h>
// // #include "freertos/FreeRTOS.h"
// // #include "freertos/task.h"
// // #include "driver/gpio.h"
// // #include "driver/adc.h"
// // #include "esp_log.h"
// // #include "led_strip.h"
// // #include "sdkconfig.h"
// // #include "driver/ledc.h"

// // #define LEDC_CHANNEL LEDC_CHANNEL_0
// // #define LEDC_TIMER LEDC_TIMER_0
// // #define LEDC_MODE LEDC_LOW_SPEED_MODE
// // #define LEDC_DUTY_RES LEDC_TIMER_8_BIT
// // #define LEDC_FREQUENCY 5000  // 5 kHz frequency

// // static const char *TAG = "example";

// // /* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
// //    or you can edit the following line and set a number here.
// // */
// // #define BLINK_GPIO 2
// // #define POT_PIN 36  // GPIO number for the potentiometer
// // static uint8_t s_led_state = 0;

// // #ifdef CONFIG_BLINK_LED_STRIP

// // static led_strip_handle_t led_strip;

// // static void blink_led(void)
// // {
// //     /* If the addressable LED is enabled */
// //     if (s_led_state) {
// //         /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
// //         led_strip_set_pixel(led_strip, 0, 16, 16, 16);
// //         /* Refresh the strip to send data */
// //         led_strip_refresh(led_strip);
// //     } else {
// //         /* Set all LED off to clear all pixels */
// //         led_strip_clear(led_strip);
// //     }
// // }

// // static void configure_led(void)
// // {
// //     ESP_LOGI(TAG, "Example configured to blink addressable LED!");
// //     /* LED strip initialization with the GPIO and pixels number*/
// //     led_strip_config_t strip_config = {
// //         .strip_gpio_num = BLINK_GPIO,
// //         .max_leds = 1, // at least one LED on board
// //     };
// // #if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
// //     led_strip_rmt_config_t rmt_config = {
// //         .resolution_hz = 10 * 1000 * 1000, // 10MHz
// //         .flags.with_dma = false,
// //     };
// //     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
// // #elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
// //     led_strip_spi_config_t spi_config = {
// //         .spi_bus = SPI2_HOST,
// //         .flags.with_dma = true,
// //     };
// //     ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
// // #else
// // #error "unsupported LED strip backend"
// // #endif
// //     /* Set all LED off to clear all pixels */
// //     led_strip_clear(led_strip);
// // }

// // #elif CONFIG_BLINK_LED_GPIO

// // static void blink_led(void)
// // {
// //     /* Set the GPIO level according to the state (LOW or HIGH)*/
// //     gpio_set_level(BLINK_GPIO, s_led_state);
// // }

// // static void configure_led(void)
// // {
// //         // Initialize LEDC for PWM
// //     ledc_timer_config_t ledc_timer = {
// //         .duty_resolution = LEDC_DUTY_RES,
// //         .freq_hz = LEDC_FREQUENCY,
// //         .speed_mode = LEDC_MODE,
// //         .timer_num = LEDC_TIMER
// //     };
// //     ledc_timer_config(&ledc_timer);

// //     ledc_channel_config_t ledc_channel = {
// //         .channel = LEDC_CHANNEL,
// //         .duty = 0,
// //         .gpio_num = BLINK_GPIO,
// //         .speed_mode = LEDC_MODE,
// //         .timer_sel = LEDC_TIMER
// //     };
// //     ledc_channel_config(&ledc_channel);
// //     ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
// //     // gpio_reset_pin(BLINK_GPIO);
// //     // /* Set the GPIO as a push/pull output */
// //     // gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
// // }

// // #else
// // #error "unsupported LED type"
// // #endif

// // void app_main(void)
// // {

// //     /* Configure the peripheral according to the LED type */
// //     configure_led();
// //     adc1_config_width(ADC_WIDTH_BIT_12);
// //     adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
// //     while (1) {
// //         ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
// //             // Initialize ADC
// //         int adc_reading = adc1_get_raw(ADC1_CHANNEL_0);
// //         int duty = (adc_reading * 255) / 4095;
// //         ESP_LOGI(TAG, "ADC Reading: %d, LED Duty: %d", adc_reading, duty);
// //         ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
// //         ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
// //         //blink_led();
// //         /* Toggle the LED state */
// //        // s_led_state = !s_led_state;
// //         vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
// //     }
// // }
// #include <stdio.h>
// #include "driver/adc.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"

// #define LED_PIN 2          // GPIO number for the LED
// #define POT_PIN 36         // GPIO number for the potentiometer

// #define ADC1_CHANNEL ADC1_CHANNEL_0  // Corresponds to GPIO 36
// #define ADC_WIDTH ADC_WIDTH_BIT_12
// #define ADC_ATTEN ADC_ATTEN_DB_0

// #define MAX_FREQUENCY 10  // Maximum blinking frequency in Hz

// static const char *TAG = "LED Control";
// static QueueHandle_t frequency_queue;
// static QueueHandle_t adc_queue;

// void adc_read_task(void *pvParameter)
// {
//     while (1) {
//         // Read the potentiometer value
//         int adc_reading = adc1_get_raw(ADC1_CHANNEL);
//         ESP_LOGI(TAG, "reading from adc %d", adc_reading);
//         // Map the ADC reading (0-4095) to a frequency range (0-10 Hz)
//         float normalized_adc_value = (float)adc_reading / 4095;
//         float frequency = normalized_adc_value * MAX_FREQUENCY;

//         // Send the frequency to the LED control task
//         xQueueSend(frequency_queue, &frequency, portMAX_DELAY);

//         vTaskDelay(pdMS_TO_TICKS(1000));  // Adjust sampling period if needed
//     }
// }

// void led_control_task(void *pvParameter)
// {
//     float frequency;
//     uint32_t delay_ms;

//     while (1) {
//         // Receive the frequency value from the queue
//         if (xQueueReceive(frequency_queue, &frequency, portMAX_DELAY)) {
//             // Calculate the delay based on the frequency
//             if (frequency == 0) {
//                 delay_ms = UINT32_MAX; // Effectively "infinite" delay
//             } else {
//                 delay_ms = 1000 / (2 * frequency); // Calculate delay in milliseconds
//             }

//             // Blink the LED
//             gpio_set_level(LED_PIN, 1);  // Turn the LED on
//             vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Wait for half of the period

//             gpio_set_level(LED_PIN, 0);  // Turn the LED off
//             vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Wait for the other half of the period
//         }
//     }
// }

// void logging_task(void *pvParameter)
// {
//     int adc_reading;
//     //float frequency;

//     while (1) {
//         ESP_LOGI(TAG, "i AM ALIVE");
//         //Log ADC reading
//         if (xQueueReceive(adc_queue, &adc_reading, portMAX_DELAY)) {
//             ESP_LOGI(TAG, "ADC Reading: %d", adc_reading);
//         }
//         else {
//             ESP_LOGW(TAG, "Failed to receive frequency from queue");
//         }
//         vTaskDelay(500 / portTICK_PERIOD_MS);  // Adjust logging interval if needed
//     }
// }

// void app_main(void)
// {
//     // Initialize LED GPIO
//     gpio_reset_pin(LED_PIN);
//     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

//     // Initialize ADC
//     adc1_config_width(ADC_WIDTH);
//     adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN);

//     // Create queues to pass data between tasks
//     frequency_queue = xQueueCreate(10, sizeof(float));
//     adc_queue = xQueueCreate(10, sizeof(int));

//     if (frequency_queue == NULL || adc_queue == NULL) {
//         ESP_LOGE(TAG, "Failed to create queue");
//         return;
//     }

//     // Create tasks
//     xTaskCreate(adc_read_task, "adc_read_task", 2048, NULL, 5, NULL);
//     xTaskCreate(led_control_task, "led_control_task", 2048, NULL, 5, NULL);
//     xTaskCreate(logging_task, "logging_task", 2048, NULL, 4, NULL);
// }
#include <stdio.h>
#include "adc_control.h"
#include "led_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Define queue handles
QueueHandle_t adc_queue;

void app_main(void)
{

    
    // Initialize queues
    adc_queue = xQueueCreate(10, sizeof(int));
    if (adc_queue == NULL) {
        ESP_LOGE("Main", "Failed to create queues");
        return;
    }

    // Initialize ADC and LED
    init_led();
    init_adc();

    // Create tasks
    xTaskCreate(adc_read_task, "adc_read_task", 2048, NULL, 5, NULL);
    xTaskCreate(led_control_task, "led_control_task", 4096, NULL, 5, NULL);
}

