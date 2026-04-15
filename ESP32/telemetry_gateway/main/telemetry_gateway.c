#include <stdio.h>
#include "wifi_connect.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_2

void blink_task(void *arg)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    while (1)
    {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    xTaskCreate(blink_task, "blink", 2048, NULL, 5, NULL);
    my_wifi_connect();
    
}
