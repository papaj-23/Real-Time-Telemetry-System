#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "init.h"
#include "mpu-6050.h"
#include "nrf905_event_index.h"
#include "nrf905_init.h"

extern TaskHandle_t mpu6050_task_handle;
extern TaskHandle_t nrf905_task_handle;
extern MPU_6050_t mpu_handle;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM7) {
        HAL_IncTick();
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi == &hspi1) {
        if((nrf905_task_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(nrf905_task_handle, SPI_TX_CPLT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi == &hspi1) {
        if((nrf905_task_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(nrf905_task_handle, SPI_TXRX_CPLT | SPI_RX_CPLT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &hi2c1) {
        if((mpu6050_task_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            MPU_6050_i2c_rxcplt_isr(&mpu_handle);
            BaseType_t hpw = pdFALSE;
            vTaskNotifyGiveFromISR(mpu6050_task_handle, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_12) {
        if((mpu6050_task_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            MPU_6050_int_isr(&mpu_handle);
        }
    }

    if(GPIO_Pin == nrf_DR_Pin) {
        if((nrf905_task_handle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            //nrf905_interrupt_irq_handler();
            /*BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(nrf905_task_handle, DR_INT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);*/
        }
    }
}