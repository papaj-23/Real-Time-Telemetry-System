#include "imu_data_input.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mpu-6050.h"
#include "init.h"
#include "debug_print.h"
#include "imu_event_index.h"

#define MPU_PAYLOAD_SIZE 14

/* static function declarations */
static void mpu6050_handler(void* pvParameters);
static void delay_ms_wrapper(uint32_t ms);

/* MPU6050 task */
#define MPU6050_STACK_SIZE 256
static StackType_t mpu6050_stack[MPU6050_STACK_SIZE];
static StaticTask_t mpu6050_tcb;
TaskHandle_t mpu6050_task_handle = NULL;
static UBaseType_t mpu6050_prio = 2;

QueueHandle_t mpu_queue_handle;

/* static buffers */
static uint8_t dma_i2c_rx_buf[14] = {0};
static MPU_6050_rawdata_t inter_buffer = {0};

/* mpu6050 library handle */
MPU_6050_t mpu_handle;

static void mpu6050_handler(void* pvParameters)
{
    uint32_t events = 0U;
    mpu_handle = (MPU_6050_t)
    {
        .hi2c = &hi2c1,
        .rx_buffer = dma_i2c_rx_buf,
        .inter_buffer = &inter_buffer,
        .delay_ms_wrapper = delay_ms_wrapper
    };
    
    MPU_6050_init(&mpu_handle);
    MPU_6050_set_mode(&mpu_handle, MPU_SINGLE_MODE);

    for(;;) 
    {
        if(xTaskNotifyWait(0, IMU_DR_INT, &events, portMAX_DELAY) != pdTRUE)
        {
            debug_print("mpu6050: wait for notify timeout.\n");
        }

        if(events & IMU_DR_INT)
        {
            MPU_6050_int_isr(&mpu_handle);
        }

        if(xTaskNotifyWait(0, IMU_I2C_RX_CPLT, &events, portMAX_DELAY) != pdTRUE)
        {
            debug_print("mpu6050: wait for notify timeout.\n");
        }

        if(mpu_handle.int_status & 1U)
        {
            MPU_6050_single_read(&mpu_handle);
        }

        if(xTaskNotifyWait(0, IMU_I2C_RX_CPLT, &events, portMAX_DELAY) != pdTRUE)
        {
            debug_print("mpu6050: wait for notify timeout.\n");
        }

        if(xQueueSendToBack(mpu_queue_handle, dma_i2c_rx_buf, portMAX_DELAY) != pdPASS)
        {
            debug_print("mpu6050: queue full.\n");
        }
    }
}

static void delay_ms_wrapper(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Init RTOS resources for MPU6050.
 *
 * Creates data receive task and queue (8 elements, MPU_PAYLOAD_SIZE each).
 *
 * @retval 0 OK
 * @retval 1 Error
 */
uint8_t imu_rtos_init()
{
    mpu6050_task_handle = xTaskCreateStatic(mpu6050_handler, "mpu6050", MPU6050_STACK_SIZE, NULL,
                                    mpu6050_prio, mpu6050_stack, &mpu6050_tcb);
    if(mpu6050_task_handle == NULL) 
    {
        return 1;
    }

    mpu_queue_handle = xQueueCreate(8, MPU_PAYLOAD_SIZE);
    if(mpu_queue_handle == NULL) 
    {
        return 1;
    }
    
    return 0;
}