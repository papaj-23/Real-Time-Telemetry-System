/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_nrf905_interface_template.c
 * @brief     driver nrf905 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-03-31
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/03/31  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_nrf905_interface.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "init.h"
#include "nrf905_event_index.h"
#include <stdarg.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
uint8_t tx[64];
uint8_t rx[64];

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t nrf905_interface_spi_init(void)
{
    MX_SPI1_Init();
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t nrf905_interface_spi_deinit(void)
{
    if(HAL_SPI_DeInit(&hspi1) != HAL_OK)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t nrf905_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    uint32_t event = 0U;

    if ((len + 1) > sizeof(tx)) {
        return 1;
    }

    tx[0] = reg;
    memset(&tx[1], 0xFF, len); // dummy bytes

    xTaskNotifyWait(SPI_RX_CPLT, 0U, &event, 0U);
    event = 0U;

    if(HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, len + 1) != HAL_OK)
    {
        return 1;
    }

    if(xTaskNotifyWait(0, SPI_RX_CPLT, &event, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        HAL_SPI_Abort(&hspi1);
        return 1;
    }

    if(event & SPI_RX_CPLT)
    {
        if(buf && len) {
            memcpy(buf, &rx[1], len);
        }
        return 0;
    }

    return 1;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t nrf905_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    memset(tx, 0, sizeof(tx));
    uint32_t event = 0U;

    if ((len + 1) > sizeof(tx)) {
        return 1;
    }

    tx[0] = reg;
    if (buf && len) {
        memcpy(&tx[1], buf, len);
    }

    xTaskNotifyWait(SPI_TX_CPLT, 0U, &event, 0U);
    event = 0U;

    if(HAL_SPI_Transmit_DMA(&hspi1, tx, len + 1) != HAL_OK)
    {
        return 1;
    }

    if(xTaskNotifyWait(0, SPI_TX_CPLT, &event, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        HAL_SPI_Abort(&hspi1);
        return 1;
    }

    if(event & SPI_TX_CPLT)
    {
        return 0;
    }

    return 1;
}

/**
 * @brief      interface spi bus transmit
 * @param[in]  *tx pointer to a tx data buffer
 * @param[out] *rx pointer to a rx data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 transmit failed
 * @note       none
 */
uint8_t nrf905_interface_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint32_t event = 0U;

    xTaskNotifyWait(SPI_TXRX_CPLT, 0U, &event, 0U);
    event = 0U;

    if(HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, len) != HAL_OK)
    {
        return 1;
    }

    if(xTaskNotifyWait(0, SPI_TXRX_CPLT, &event, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        HAL_SPI_Abort(&hspi1);
        return 1;
    }

    if(event & SPI_TXRX_CPLT) {
        return 0;
    }

    return 1;
}

/**
 * @brief  interface ce gpio init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t nrf905_interface_ce_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOB, nrf_CE_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = nrf_CE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    return 0;
}

/**
 * @brief  interface ce gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t nrf905_interface_ce_gpio_deinit(void)
{
    HAL_GPIO_DeInit(GPIOB, nrf_CE_Pin);
    return 0;
}

/**
 * @brief     interface ce gpio write
 * @param[in] data written data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t nrf905_interface_ce_gpio_write(uint8_t data)
{
    GPIO_PinState state = data ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOB, nrf_CE_Pin, state);
    return 0;
}

/**
 * @brief  interface tx en gpio init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t nrf905_interface_tx_en_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOB, nrf_TXEN_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = nrf_TXEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    return 0;
}

/**
 * @brief  interface tx en gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t nrf905_interface_tx_en_gpio_deinit(void)
{
    HAL_GPIO_DeInit(GPIOB, nrf_TXEN_Pin);
    return 0;
}

/**
 * @brief     interface tx en gpio write
 * @param[in] data written data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t nrf905_interface_tx_en_gpio_write(uint8_t data)
{
    GPIO_PinState state = data ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOB, nrf_TXEN_Pin, state);
    return 0;
}

/**
 * @brief  interface pwr up gpio init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t nrf905_interface_pwr_up_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOB, nrf_PWR_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = nrf_PWR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    return 0;
}

/**
 * @brief  interface pwr up gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t nrf905_interface_pwr_up_gpio_deinit(void)
{
    HAL_GPIO_DeInit(GPIOB, nrf_PWR_Pin);
    return 0;
}

/**
 * @brief     interface pwr up gpio write
 * @param[in] data written data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t nrf905_interface_pwr_up_gpio_write(uint8_t data)
{
    GPIO_PinState state = data ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOB, nrf_PWR_Pin, state);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void nrf905_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void nrf905_interface_debug_print(const char *const fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        if (len > (int)sizeof(buf)) {
            len = sizeof(buf);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

/**
 * @brief     interface receive callback
 * @param[in] type receive callback type
 * @param[in] *buf pointer to a data buffer
 * @param[in] len buffer length
 * @note      none
 */
void nrf905_interface_receive_callback(uint8_t type, uint8_t *buf, uint8_t len)
{
    switch (type)
    {
        case NRF905_STATUS_AM :
        {
            //nrf905_interface_debug_print("nrf905: address match.\n");
            
            break;
        }
        case NRF905_STATUS_TX_DONE :
        {
            //nrf905_interface_debug_print("nrf905: tx done.\n");
            
            break;
        }
        case NRF905_STATUS_RX_DONE :
        {
            //nrf905_interface_debug_print("nrf905: rx done.\n");
            
            break;
        }
        default :
        {
            //nrf905_interface_debug_print("nrf905: unknown code.\n");
            
            break;
        }
    }
}
