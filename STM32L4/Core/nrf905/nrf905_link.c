#include "nrf905_link.h"
#include "nrf905_init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf905_event_index.h"
#include "debug_print.h"
#include "stm32l4xx_hal.h"

#define PAYLOAD_LEN    14U

static uint8_t send_data(uint8_t *addr, uint8_t *buf, uint8_t len);

static uint8_t tx_address[4] = {0xF3, 0x12, 0xB2, 0x9F};
static uint8_t tx_address_check[4] = {0};
static uint8_t buf_check[PAYLOAD_LEN] = {0};
static uint8_t conf_check[10] = {0};

extern nrf905_handle_t rf_handle;
extern QueueHandle_t mpu_queue_handle;
extern SPI_HandleTypeDef hspi1;

/* NRF905 task */
#define NRF905_STACK_SIZE 512
static StackType_t nrf905_stack[NRF905_STACK_SIZE];
static StaticTask_t nrf905_tcb;
TaskHandle_t nrf905_task_handle = NULL;
static UBaseType_t nrf905_prio = 2;

static void nrf905_handler(void* pvParameters) {
    uint32_t events = 0U;
    uint8_t buf[PAYLOAD_LEN] = {10U, 20U, 30U, 40U, 50U, 60U, 70U, 80U, 90U, 100U, 110U, 120U, 130U, 140U};

    nrf905_device_init(NRF905_MODE_TX);
<<<<<<< HEAD
    nrf905_read_conf(&rf_handle, conf_check);
    for(int i = 0; i<10; i++) {
        debug_print("nrf905: conf check: %02X.\n", conf_check[i]);
    }
    nrf905_set_tx_address(&rf_handle, tx_address, 4);
    nrf905_get_tx_address(&rf_handle, tx_address_check, 4);
    debug_print("nrf905: tx address check: %02X %02X %02X %02X.\n", tx_address_check[0], tx_address_check[1], tx_address_check[2], tx_address_check[3]);
=======

>>>>>>> 097ae79 (finally fixed SPI - it was a nucleo board solder brige problem)
    for(;;) {
        //xQueueReceive(mpu_queue_handle, buf, portMAX_DELAY);
        if(send_data((uint8_t *)tx_address, buf, PAYLOAD_LEN) != 0) {
            debug_print("nrf905: send data failed.\n");
        }
        
        if(xTaskNotifyWaitIndexed(NRF905_NOTIFY_DR_INDEX, 0, DR_INT, &events, portMAX_DELAY) == pdTRUE) {
            if(events & DR_INT) {
                //nrf905_get_tx_payload(&rf_handle, buf_check, PAYLOAD_LEN);
                debug_print("nrf905: data sent.\n");
                //debug_print("nrf905: sent data check: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X.\n", buf_check[0], buf_check[1], buf_check[2], buf_check[3], buf_check[4], buf_check[5], buf_check[6], buf_check[7], buf_check[8], buf_check[9], buf_check[10], buf_check[11], buf_check[12], buf_check[13]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));   
    }

}

static uint8_t send_data(uint8_t *addr, uint8_t *buf, uint8_t len)
{
    uint8_t res;
    
    /* set tx address */
    res = nrf905_set_tx_address(&rf_handle, addr, 4);
    if (res != 0)
    {
        return 1;
    }
    
    /* send data */
    res = nrf905_send(&rf_handle, buf, len);
    if (res != 0)
    {
        return 1;
    }
    
    return 0;
}

uint8_t nrf905_rtos_init() {
    nrf905_task_handle = xTaskCreateStatic(nrf905_handler, "nrf905", NRF905_STACK_SIZE, NULL,
                                    nrf905_prio, nrf905_stack, &nrf905_tcb);
    if(nrf905_task_handle == NULL) {
        return 1;
    }

    return 0;
}
