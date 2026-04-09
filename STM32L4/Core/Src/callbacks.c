#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "init.h"
#include "mpu-6050.h"

extern TaskHandle_t mpu6050_data_receive_handle;
extern MPU_6050_t mpu_handle;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM6) {
        if((mpu6050_data_receive_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            //MPU_6050_read_fifo_cnt(&mpu_handle);
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_data_receive_handle, 0x04U, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }

    if(htim->Instance == TIM7) {
        HAL_IncTick();
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c == &hi2c1) {
        if((mpu6050_data_receive_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            MPU_6050_i2c_rxcplt_isr(&mpu_handle);
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_data_receive_handle, 0x02U, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_12) {
        if((mpu6050_data_receive_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            MPU_6050_int_isr(&mpu_handle);
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_data_receive_handle, 0x01U, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

/*
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  if(hi2c == &hi2c1)

}
  */