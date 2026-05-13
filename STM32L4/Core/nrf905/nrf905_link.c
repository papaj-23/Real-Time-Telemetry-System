#include "nrf905_link.h"
#include "nrf905_init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf905_event_index.h"
#include "debug_print.h"
#include "stm32l4xx_hal.h"

#define PAYLOAD_LEN    18U
#define TIMESTAMP_IDX  14U

static uint8_t send_data(uint8_t *buf, uint8_t len);

static uint8_t tx_address[4] = {0xF3, 0x12, 0xB2, 0x9F};

extern nrf905_handle_t rf_handle;
extern QueueHandle_t mpu_queue_handle;
extern SPI_HandleTypeDef hspi1;

/* NRF905 task */
#define NRF905_STACK_SIZE 512
static StackType_t nrf905_stack[NRF905_STACK_SIZE];
static StaticTask_t nrf905_tcb;
TaskHandle_t nrf905_task_handle = NULL;
static UBaseType_t nrf905_prio = 2;

static void add_timestamp(uint8_t *buf)
{
    uint32_t timestamp = HAL_GetTick();
    *(buf + TIMESTAMP_IDX) = (timestamp >> 24) & 0xFFU;
    *(buf + TIMESTAMP_IDX + 1) = (timestamp >> 16) & 0xFFU;
    *(buf + TIMESTAMP_IDX + 2) = (timestamp >> 8) & 0xFFU;
    *(buf + TIMESTAMP_IDX + 3) = timestamp & 0xFFU;
}

static void nrf905_handler(void* pvParameters)
{
    uint32_t events = 0U;
    uint8_t buf[PAYLOAD_LEN] = {0};

    nrf905_device_init(NRF905_MODE_TX);
    nrf905_set_tx_address(&rf_handle, tx_address, 4);

    for(;;)
    {
        xQueueReceive(mpu_queue_handle, buf, portMAX_DELAY);

        add_timestamp(buf);
        
        if(send_data(buf, PAYLOAD_LEN) != 0)
        {
            debug_print("nrf905: send data failed.\n");
        }
        
        if(xTaskNotifyWaitIndexed(NRF905_NOTIFY_DR_INDEX, 0, NRF905_DR_INT, &events, portMAX_DELAY) == pdTRUE)
        {
            if(events & NRF905_DR_INT)
            {
                debug_print("nrf905: data sent.\n");
                events &= ~NRF905_DR_INT;
            }
        }  
    }
}

static uint8_t send_data(uint8_t *buf, uint8_t len)
{
    uint8_t res;
    
    /* send data */
    res = nrf905_send(&rf_handle, buf, len);
    if (res != 0)
    {
        return 1;
    }
    
    return 0;
}

uint8_t nrf905_rtos_init()
{
    nrf905_task_handle = xTaskCreateStatic(nrf905_handler, "nrf905", NRF905_STACK_SIZE, NULL,
                                        nrf905_prio, nrf905_stack, &nrf905_tcb);
    if(nrf905_task_handle == NULL)
    {
        return 1;
    }

    return 0;
}
