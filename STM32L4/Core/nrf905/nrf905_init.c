#include "nrf905_init.h"

nrf905_handle_t rf_handle;

/**
 * @brief  nrf905 irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t nrf905_interrupt_irq_handler(void)
{
    if (nrf905_irq_handler(&rf_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief     basic example init
 * @param[in] mode chip working mode
 * @param[in] *callback pointer to a callback function
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t nrf905_device_init(nrf905_mode_t mode)
{
    uint8_t res;
    uint16_t reg;
    uint8_t addr[4] = NRF905_BASIC_DEFAULT_RX_ADDR;
    
    /* link function */
    DRIVER_NRF905_LINK_INIT(&rf_handle, nrf905_handle_t);
    DRIVER_NRF905_LINK_SPI_INIT(&rf_handle, nrf905_interface_spi_init);
    DRIVER_NRF905_LINK_SPI_DEINIT(&rf_handle, nrf905_interface_spi_deinit);
    DRIVER_NRF905_LINK_SPI_READ(&rf_handle, nrf905_interface_spi_read);
    DRIVER_NRF905_LINK_SPI_WRITE(&rf_handle, nrf905_interface_spi_write);
    DRIVER_NRF905_LINK_SPI_TRANSMIT(&rf_handle, nrf905_interface_spi_transmit);
    DRIVER_NRF905_LINK_CE_GPIO_INIT(&rf_handle, nrf905_interface_ce_gpio_init);
    DRIVER_NRF905_LINK_CE_GPIO_DEINIT(&rf_handle, nrf905_interface_ce_gpio_deinit);
    DRIVER_NRF905_LINK_CE_GPIO_WRITE(&rf_handle, nrf905_interface_ce_gpio_write);
    DRIVER_NRF905_LINK_TX_EN_GPIO_INIT(&rf_handle, nrf905_interface_tx_en_gpio_init);
    DRIVER_NRF905_LINK_TX_EN_GPIO_DEINIT(&rf_handle, nrf905_interface_tx_en_gpio_deinit);
    DRIVER_NRF905_LINK_TX_EN_GPIO_WRITE(&rf_handle, nrf905_interface_tx_en_gpio_write);
    DRIVER_NRF905_LINK_PWR_UP_GPIO_INIT(&rf_handle, nrf905_interface_pwr_up_gpio_init);
    DRIVER_NRF905_LINK_PWR_UP_GPIO_DEINIT(&rf_handle, nrf905_interface_pwr_up_gpio_deinit);
    DRIVER_NRF905_LINK_PWR_UP_GPIO_WRITE(&rf_handle, nrf905_interface_pwr_up_gpio_write);
    DRIVER_NRF905_LINK_DELAY_MS(&rf_handle, nrf905_interface_delay_ms);
    DRIVER_NRF905_LINK_DEBUG_PEINT(&rf_handle, nrf905_interface_debug_print);
    DRIVER_NRF905_LINK_RECEIVE_CALLBACK(&rf_handle, nrf905_interface_receive_callback);
    
    /* init */
    res = nrf905_init(&rf_handle);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: init failed.\n");
        
        return 1;
    }
    
    /* power up the chip */
    res = nrf905_set_power_up(&rf_handle, NRF905_BOOL_TRUE);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: power up failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }

    res = nrf905_set_enable(&rf_handle, NRF905_BOOL_TRUE);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set enable failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    /* disable the chip */
    res = nrf905_set_enable(&rf_handle, NRF905_BOOL_FALSE);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set enable failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the pll mode */
    res = nrf905_set_pll_mode(&rf_handle, NRF905_BASIC_DEFAULT_PLL_MODE);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set pll mode failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the output power */
    res = nrf905_set_output_power(&rf_handle, NRF905_BASIC_DEFAULT_OUTPUT_POWER);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set output power failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the rx mode */
    res = nrf905_set_rx_mode(&rf_handle, NRF905_BASIC_DEFAULT_RX_MODE);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set rx mode failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the auto retransmit */
    res = nrf905_set_auto_retransmit(&rf_handle, NRF905_BASIC_DEFAULT_AUTO_RETRANSMIT);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set auto retransmit failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the rx address width */
    res = nrf905_set_rx_address_width(&rf_handle, NRF905_BASIC_DEFAULT_RX_ADDRESS_WIDTH);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set rx address width failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the tx address width */
    res = nrf905_set_tx_address_width(&rf_handle, NRF905_BASIC_DEFAULT_TX_ADDRESS_WIDTH);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set tx address width failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the rx payload width */
    res = nrf905_set_rx_payload_width(&rf_handle, NRF905_BASIC_DEFAULT_RX_PAYLOAD_WIDTH);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set rx payload width failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the tx payload width  */
    res = nrf905_set_tx_payload_width(&rf_handle, NRF905_BASIC_DEFAULT_TX_PAYLOAD_WIDTH);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set tx payload width failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the rx address */
    res = nrf905_set_rx_address(&rf_handle, (uint8_t *)addr);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set rx address failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set output clock frequency */
    res = nrf905_set_output_clock_frequency(&rf_handle, NRF905_BASIC_DEFAULT_OUTPUT_CLOCK_FREQUENCY);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set output clock frequency failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the output clock */
    res = nrf905_set_output_clock(&rf_handle, NRF905_BASIC_DEFAULT_OUTPUT_CLOCK);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set output clock failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the crystal oscillator frequency */
    res = nrf905_set_crystal_oscillator_frequency(&rf_handle, NRF905_BASIC_DEFAULT_CRYSTAL_OSCILLATOR_FREQUENCY);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set crystal oscillator frequency failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the crc */
    res = nrf905_set_crc(&rf_handle, NRF905_BASIC_DEFAULT_CRC);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set crc failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the crc mode */
    res = nrf905_set_crc_mode(&rf_handle, NRF905_BASIC_DEFAULT_CRC_MODE);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set crc mode failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* 433 MHz */
    res = nrf905_frequency_convert_to_register(&rf_handle, NRF905_BASIC_DEFAULT_FREQUENCY, &reg);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: frequency convert to register failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* set the frequency */
    res = nrf905_set_frequency(&rf_handle, reg);  
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: set frequency failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }
    
    /* write the conf */
    res = nrf905_write_conf(&rf_handle);
    if (res != 0)
    {
        nrf905_interface_debug_print("nrf905: write conf failed.\n");
        (void)nrf905_deinit(&rf_handle);
        
        return 1;
    }

    if (mode == NRF905_MODE_TX)
    {
        return 0;
    }
    else
    {
        /* set rx mode */
        res = nrf905_set_mode(&rf_handle, NRF905_MODE_RX);
        if (res != 0)
        {
            nrf905_interface_debug_print("nrf905: set mode failed.\n");
            (void)nrf905_deinit(&rf_handle);
            
            return 1;
        }
        
        /* enable the chip */
        res = nrf905_set_enable(&rf_handle, NRF905_BOOL_TRUE);
        if (res != 0)
        {
            nrf905_interface_debug_print("nrf905: set enable failed.\n");
            (void)nrf905_deinit(&rf_handle);
            
            return 1;
        }
        
        return 0;
    }
}

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t nrf905_device_deinit(void)
{
    if (nrf905_deinit(&rf_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
