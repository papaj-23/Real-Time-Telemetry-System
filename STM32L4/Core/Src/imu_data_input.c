#include "FreeRTOS.h"
#include "task.h"
#include "mpu-6050.h"
#include "rtos_init.h"
#include "init.h"
#include "IMU_data_input.h"

#define BURST_COUNT 600

static void MPU6050_data_receive_handler(void* pvParameters);

/* MPU6050 data transfer task */
#define MPU6050_DATA_RECEIVE_STACK_SIZE 512
static StackType_t mpu6050_data_receive_stack[MPU6050_DATA_RECEIVE_STACK_SIZE];
static StaticTask_t mpu6050_data_receive_tcb;
TaskHandle_t mpu6050_data_receive_handle = NULL;
static const task_init_t mpu6050_data_receive_init = {
    .entry = MPU6050_data_receive_handler,
    .name = "MPU6050_data_receive",
    .stack_size = MPU6050_DATA_RECEIVE_STACK_SIZE,
    .args = NULL,
    .priority = 3,
    .stack = mpu6050_data_receive_stack,
    .tcb = &mpu6050_data_receive_tcb
};

uint8_t dma_i2c_rx_buf[14];

int16_t intermediate_rx_buf[7];
MPU_6050_t mpu_handle;
MPU_6050_selftest_t selftest_results;
MPU_6050_data_t current_data;


static void MPU6050_data_receive_handler(void* pvParameters) {

    HAL_TIM_Base_Start_IT(&htim6);

    mpu_handle = (MPU_6050_t) {
        .hi2c = &hi2c1,
        .rx_buffer = dma_i2c_rx_buf,
        .delay_ms_wrapper = delay_wrapper_ms,
        .burst_count = BURST_COUNT
    };
    uint32_t events = 0U;

    MPU_6050_Init(&mpu_handle);
    MPU_6050_Self_Test(&mpu_handle, &selftest_results);

    MPU_6050_Set_Mode(&mpu_handle, MPU_SINGLE_MODE);
    

    for(;;) {
        check_registers(&mpu_handle);
        check_registers(&mpu_handle);
        xTaskNotifyWait(0U, 0xFFFFFFFFU, &events, portMAX_DELAY);

        /* external interrupt from MPU6050 */
        if(events & 0x01U) {
            MPU_6050_Interrupt_Handler(&mpu_handle);
        }

        /* i2c dma transfer complete */
        if(events & 0x02U) {
            MPU_6050_parse_payload(dma_i2c_rx_buf, intermediate_rx_buf);
            current_data = MPU_6050_payload_to_readable(&mpu_handle, intermediate_rx_buf);
            //MPU_6050_Process_Burst_Cnt(&mpu_handle);
            if(mpu_handle.fifo_counter >= BURST_COUNT) {
                //MPU_6050_Burst_Read(&mpu_handle);
                mpu_handle.fifo_counter -= BURST_COUNT;
            }
        }

        /* trigger fifo_count read */
        if(events & 0x04) {
            //MPU_6050_Read_FIFO_Cnt(&mpu_handle);
        }


    }
}

void imu_rtos_init() {
    mpu6050_data_receive_handle = create_static_task(&mpu6050_data_receive_init);
}