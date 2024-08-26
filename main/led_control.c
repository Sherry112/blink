#include "led_control.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "led_strip.h"
// Initialize ADC and GPIO for LED

static led_strip_handle_t led;

void init_led(void)
{
    // Initialize LED GPIO
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

static void set_led_color(float frequency)
{
    uint8_t red, green, blue;
    //shah ESP_LOGI("LED", "FREQUENCY RECIEVED IS %f", frequency);
    // Map frequency to color (example)
    if (frequency < 3.3) {
        red = 0;
        green = 0;
        blue = (uint8_t)((3.3 - frequency) * 77); // Blue increases with frequency
    } else if (frequency < 6.6) {
        red = 0;
        green = (uint8_t)((frequency - 3.3) * 77); // Green increases with frequency
        blue = (uint8_t)((6.6 - frequency) * 10); // Blue decreases with frequency
    } else {
        red = (uint8_t)((frequency - 6.6) * 77); // Red increases with frequency
        green = (uint8_t)((10.0 - frequency) * 10); // Green decreases with frequency
        blue = 0;
    }

    // Set color to the strip
    led_strip_set_pixel(led, 0, red, green, blue);
    led_strip_refresh(led);
}

static void initialize_led(void)
{
    ESP_LOGI("TAG", "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = 17,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led);
}

void led_control_task(void *pvParameter)
{
    int frequency;
    int delay_ms;
    initialize_led();
    while (1) {
        //Receive the frequency value from the queue
        if (xQueueReceive(adc_queue, &frequency, pdMS_TO_TICKS(100))) {
            // Calculate the delay based on the frequency
            if (frequency == 0) {
                set_led_color(frequency);
                gpio_set_level(LED_PIN, 1);
            } else {
                //shah ESP_LOGI("TAG", "frequency receieved is %d", frequency);
                delay_ms = 1000 / (2 * frequency); // Calculate delay in milliseconds
            
            ESP_LOGI("TAG", "LED DELAY %d", delay_ms);
            // Blink the LED
            set_led_color(frequency);
            gpio_set_level(LED_PIN, 1);  // Turn the LED on
            vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Wait for half of the period
            
            led_strip_clear(led);

            
            gpio_set_level(LED_PIN, 0);  // Turn the LED off
            vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Wait for the other half of the period
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    }


