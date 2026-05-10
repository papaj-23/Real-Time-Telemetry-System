#include "rf_link.h"
#include "rf_init.h"
#include "rf_peripherals.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PAYLOAD_LEN    14U
#define CD_EVENT       1UL << 0
#define AM_EVENT       1UL << 1
#define DR_EVENT       1UL << 2

extern nrf905_handle_t rf_handle;
TaskHandle_t nrf905_task_handle;
QueueHandle_t mpu_queue_handle;
static uint8_t rx_data[PAYLOAD_LEN] = {0};
static const char *TAG = "NRF905";
static uint32_t cnt_dr = 0;
static uint32_t cnt_am = 0;
static uint32_t cnt_cd = 0;

static void rf_dr_isr(void *arg);
static void rf_am_isr(void *arg);
static void rf_cd_isr(void *arg);

static uint8_t check_conf[10] = {0};

static void nrf905_task_handler(void *pvParameters)
{
    uint32_t event = 0U;

    if(irq_enable() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable GPIO interrupts");
    }

    if(rf_gpio_int_input_init(RF_DR_GPIO, rf_dr_isr) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init DR pin");
    }

    if(rf_gpio_int_input_init(RF_AM_GPIO, rf_am_isr) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init AM pin");
    }

    if(rf_gpio_int_input_init(RF_CD_GPIO, rf_cd_isr) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init CD pin");
    }   

    if(nrf905_device_init(NRF905_MODE_RX) != 0) {
        ESP_LOGE(TAG, "Failed to init nrf905 transciever");
    }

    nrf905_read_conf(&rf_handle, check_conf);
    for(int i = 0; i<10; i++) {
        ESP_LOGE(TAG, "nrf905: conf check: %02X.\n", check_conf[i]);
    }
    
    for(;;) {
        //ESP_LOGI(TAG, "Counter DR: %lu", (unsigned long)cnt_dr);
        //ESP_LOGI(TAG, "Counter AM: %lu", (unsigned long)cnt_am);
        //ESP_LOGI(TAG, "Counter CD: %lu", (unsigned long)cnt_cd);

        if(xTaskNotifyWait(0, DR_EVENT | AM_EVENT | CD_EVENT, &event, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGI(TAG, "Notification timeout");
        }
        
        if(event & CD_EVENT) {
            //ESP_LOGI(TAG, "Carrier detect event");
        }

        if(event & AM_EVENT) {
            //ESP_LOGI(TAG, "Address match event");
        }

        if(event & DR_EVENT) {
            ESP_LOGI(TAG, "Data ready event");
            if(nrf905_get_rx_payload(&rf_handle, rx_data, PAYLOAD_LEN) != 0) {
                ESP_LOGE(TAG, "Failed to get rx payload");
            }
            for(int i = 0; i < PAYLOAD_LEN; i++) {
                ESP_LOGI(TAG, "Byte %d: %d", i, rx_data[i]);
            }
            ESP_LOGI(TAG, "First byte: %d", rx_data[0]);
            ESP_LOGI(TAG, "Last byte: %d", rx_data[13]);

            if(xQueueSendToBack(mpu_queue_handle, rx_data, pdMS_TO_TICKS(1000)) != pdPASS) {
                ESP_LOGI(TAG, "Mpu6050 queue is full");
            }
        }  
    }
}

void start_nrf905_thread(void)
{
    BaseType_t pd = xTaskCreate(nrf905_task_handler, "nrf905_task",
                                2048, NULL, 7, &nrf905_task_handle);
    if(pd != pdPASS) {
        ESP_LOGE(TAG, "Failed to create nrf905 task");
    }

    mpu_queue_handle = xQueueCreate(8, PAYLOAD_LEN);
}

static void rf_dr_isr(void *arg)
{
    cnt_dr++;
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(nrf905_task_handle, DR_EVENT, eSetBits, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void rf_am_isr(void *arg)
{
    cnt_am++;
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(nrf905_task_handle, AM_EVENT, eSetBits, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void rf_cd_isr(void *arg)
{
    cnt_cd++;
    BaseType_t hpw = pdFALSE;
    xTaskNotifyFromISR(nrf905_task_handle, CD_EVENT, eSetBits, &hpw);
    portYIELD_FROM_ISR(hpw);
}

