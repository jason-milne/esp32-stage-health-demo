#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define BLINK_GPIO 2  // Change this to a valid pin on your ESP32 devkit

static const char *TAG = "HEALTH_DEMO";

void app_main(void)
{
    // Configure BLINK_GPIO as an output
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    int count = 0;

    while (1)
    {
        // Toggle LED pin (or just a GPIO you can probe later)
        gpio_set_level(BLINK_GPIO, (count % 2));

        // Print a simple message once per second
        ESP_LOGI(TAG, "Hello from ESP32! Count = %d", count);

        count++;

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second
    }
}
