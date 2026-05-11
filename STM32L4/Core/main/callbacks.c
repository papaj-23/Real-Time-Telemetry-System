#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "init.h"
#include "mpu-6050.h"
#include "nrf905_init.h"
#include "nrf905_event_index.h"
#include "imu_event_index.h"


extern TaskHandle_t mpu6050_task_handle;
extern TaskHandle_t nrf905_task_handle;
extern MPU_6050_t mpu_handle;
extern nrf905_handle_t rf_handle;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM7)
    {
        HAL_IncTick();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi == &hspi1)
    {
        if((nrf905_task_handle != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
        {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyIndexedFromISR(nrf905_task_handle, NRF905_NOTIFY_SPI_INDEX,
                                    NRF905_SPI_TXRX_CPLT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi == &hspi1)
    {
        if((nrf905_task_handle != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
        {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyIndexedFromISR(nrf905_task_handle, NRF905_NOTIFY_SPI_INDEX,
                                    NRF905_SPI_ERROR, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi == &hspi1)
    {
        if((nrf905_task_handle != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
        {
            BaseType_t hpw = pdFALSE;
            xTaskNotifyIndexedFromISR(nrf905_task_handle, NRF905_NOTIFY_SPI_INDEX,
                                    NRF905_SPI_ABORT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &hi2c1)
    {
        if((mpu6050_task_handle != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
        {
            MPU_6050_i2c_rxcplt_isr(&mpu_handle);
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_task_handle, IMU_I2C_RX_CPLT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_12)
    {
        if((mpu6050_task_handle != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
        {
            //MPU_6050_int_isr(&mpu_handle);
            BaseType_t hpw = pdFALSE;
            xTaskNotifyFromISR(mpu6050_task_handle, IMU_DR_INT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }

    if(GPIO_Pin == nrf_DR_Pin)
    {
        if((nrf905_task_handle != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
        {
            rf_handle.finished = 1;
            BaseType_t hpw = pdFALSE;
            xTaskNotifyIndexedFromISR(nrf905_task_handle, NRF905_NOTIFY_DR_INDEX,
                                    NRF905_DR_INT, eSetBits, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}
