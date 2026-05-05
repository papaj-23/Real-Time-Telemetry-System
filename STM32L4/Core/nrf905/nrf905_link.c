#include "nrf905_link.h"
#include "nrf905_init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf905_event_index.h"

#define PAYLOAD_LEN    14U

static uint8_t send_data(uint8_t *addr, uint8_t *buf, uint8_t len);

static const uint8_t tx_address[4] = {0xE7, 0xE7, 0xE7, 0xE7};

extern nrf905_handle_t rf_handle;
extern QueueHandle_t mpu_queue_handle;

/* NRF905 task */
#define NRF905_STACK_SIZE 256
static StackType_t nrf905_stack[NRF905_STACK_SIZE];
static StaticTask_t nrf905_tcb;
TaskHandle_t nrf905_task_handle = NULL;
static UBaseType_t nrf905_prio = 2;

static void nrf905_handler(void* pvParameters) {
    uint32_t events = 0U;
    uint8_t buf[PAYLOAD_LEN] = {0};

    nrf905_device_init(NRF905_MODE_TX);

    for(;;) {
        xQueueReceive(mpu_queue_handle, buf, portMAX_DELAY);
        if(send_data((uint8_t *)tx_address, buf, PAYLOAD_LEN) != 0) {
            
        }
        /*
        if(xTaskNotifyWait(0, DR_INT, &events, portMAX_DELAY) == pdTRUE) {
            if(events & DR_INT) {
            nrf905_interrupt_irq_handler();
            }
        }*/
        
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
    res = nrf905_send(&rf_handle, (uint8_t *)buf, len);
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