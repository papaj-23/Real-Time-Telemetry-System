#include "rf_link.h"
#include "rf_init.h"
#include "rf_peripherals.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define MPU_6050_PAYLOAD_LEN    14U

extern nrf905_handle_t rf_handle;
TaskHandle_t nrf905_task_handle;
QueueHandle_t mpu_queue_handle;
static uint8_t rx_data[MPU_6050_PAYLOAD_LEN] = {0};
static const char *TAG = "RF TRANSCIEVER";

static void nrf905_task_handler(void *pvParameters)
{
    if(nrf905_device_init(NRF905_MODE_RX) != 0) {
        ESP_LOGE(TAG, "Faied to init nrf905 transciever");
    }

    if(rf_gpio_DR_init() != ESP_OK) {
        ESP_LOGE(TAG, "Faied to init DR pin");
    }

    for(;;) {
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 1) {
            ESP_LOGI(TAG, "Data irq overlapped");
        }

        if(nrf905_get_rx_payload(&rf_handle, rx_data, MPU_6050_PAYLOAD_LEN) != 0) {
            ESP_LOGE(TAG, "Faied to get rx payload");
        }

        if(xQueueSendToBack(mpu_queue_handle, rx_data, 10/portTICK_PERIOD_MS) != pdPASS) {
            ESP_LOGI(TAG, "Mpu6050 queue is full");
        }
    }
}

void start_nrf905_thread(void)
{
    BaseType_t pd = xTaskCreate(nrf905_task_handler, "nrf905_task",
                                1024, NULL, 6, &nrf905_task_handle);
    if(pd != pdPASS) {
        ESP_LOGE(TAG, "Failed to create nrf905 task");
    }

    mpu_queue_handle = xQueueCreate(8, MPU_6050_PAYLOAD_LEN);
}

