#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// Disabled Kconfig bool options are omitted from sdkconfig.h.
#ifndef CONFIG_BLINK_ACTIVE_LOW
#define CONFIG_BLINK_ACTIVE_LOW 0
#endif

static const char *TAG = "blink";

static int led_level(bool led_on)
{
    return CONFIG_BLINK_ACTIVE_LOW ? !led_on : led_on;
}

void app_main(void)
{
    const gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << CONFIG_BLINK_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&led_config));
    ESP_LOGI(TAG, "Blink started: GPIO%d, active-%s, interval=%d ms",
             CONFIG_BLINK_GPIO,
             CONFIG_BLINK_ACTIVE_LOW ? "low" : "high",
             CONFIG_BLINK_INTERVAL_MS);

    bool led_on = false;
    while (true) {
        led_on = !led_on;
        ESP_ERROR_CHECK(gpio_set_level(CONFIG_BLINK_GPIO, led_level(led_on)));
        ESP_LOGI(TAG, "LED %s", led_on ? "ON" : "OFF");
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BLINK_INTERVAL_MS));
    }
}
