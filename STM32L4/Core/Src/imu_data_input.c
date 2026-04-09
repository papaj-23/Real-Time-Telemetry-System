#include "FreeRTOS.h"
#include "task.h"
#include "mpu-6050.h"
#include "rtos_init.h"
#include "init.h"
#include "IMU_data_input.h"

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

uint8_t dma_i2c_rx_buf[14] = {0};
MPU_6050_rawdata_t inter_buffer;

MPU_6050_t mpu_handle;
MPU_6050_selftest_t selftest_results;
MPU_6050_data_t current_data;


static void MPU6050_data_receive_handler(void* pvParameters) {

    HAL_TIM_Base_Start_IT(&htim6);

    mpu_handle = (MPU_6050_t) {
        .hi2c = &hi2c1,
        .rx_buffer = dma_i2c_rx_buf,
        .inter_buffer = &inter_buffer,
        .delay_ms_wrapper = delay_wrapper_ms,
    };
    uint32_t events = 0U;
    MPU_6050_init(&mpu_handle);
    MPU_6050_self_test(&mpu_handle, &selftest_results);
    MPU_6050_set_lp_wakeup_freq(&mpu_handle, F_40HZ);
    MPU_6050_set_mode(&mpu_handle, MPU_SINGLE_MODE);

    for(;;) {
        //MPU_6050_check_registers(&mpu_handle);
        xTaskNotifyWait(0U, 0xFFFFFFFFU, &events, portMAX_DELAY);

        /* external interrupt from MPU6050 */
        if(events & 0x01U) {
            
        }

        /* i2c dma transfer complete */
        if(events & 0x02U) {
            MPU_6050_parse_payload(&mpu_handle);
            current_data = MPU_6050_payload_to_readable(&mpu_handle);
            if(mpu_handle.fifo_oflow_flag == 1U) {
                MPU_6050_fifo_reset(&mpu_handle);
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