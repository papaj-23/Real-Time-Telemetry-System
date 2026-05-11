#ifndef INIT_H
#define INIT_H

#include "stm32l4xx_hal.h"

#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define nrf_DR_Pin GPIO_PIN_3
#define nrf_DR_GPIO_Port GPIOA
#define nrf_DR_EXTI_IRQn EXTI3_IRQn
#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOB
#define nrf_TXEN_Pin GPIO_PIN_1
#define nrf_TXEN_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define nrf_PWR_Pin GPIO_PIN_6
#define nrf_PWR_GPIO_Port GPIOB
#define nrf_CE_Pin GPIO_PIN_7
#define nrf_CE_GPIO_Port GPIOB

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_SPI1_Init(void);
void Peripherals_Init(void);
void Error_Handler(void);

#endif