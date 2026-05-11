#include "rf_peripherals.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

#define RF_SPI_HOST         SPI2_HOST
#define RF_SPI_MOSI_GPIO    23
#define RF_SPI_MISO_GPIO    19
#define RF_SPI_SCLK_GPIO    18
#define RF_SPI_CS_GPIO      5

#define RF_SPI_CLOCK_HZ     (1000 * 1000)
#define RF_SPI_QUEUE_SIZE   4

spi_device_handle_t s_rf_spi;

/* SPI config */

esp_err_t rf_spi_init(void)
{
    spi_bus_config_t buscfg = 
    {
        .mosi_io_num = RF_SPI_MOSI_GPIO,
        .miso_io_num = RF_SPI_MISO_GPIO,
        .sclk_io_num = RF_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = 
    {
        .clock_speed_hz = RF_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = RF_SPI_CS_GPIO,
        .queue_size = RF_SPI_QUEUE_SIZE,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
    };

    esp_err_t err;

    err = spi_bus_initialize(RF_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
    {
        return err;
    }

    err = spi_bus_add_device(RF_SPI_HOST, &devcfg, &s_rf_spi);
    if (err != ESP_OK)
    {
        spi_bus_free(RF_SPI_HOST);
        return err;
    }

    return ESP_OK;
}

esp_err_t rf_spi_deinit(void)
{
    esp_err_t err;
    if(s_rf_spi != NULL)
    {
        err = spi_bus_remove_device(s_rf_spi);
        if (err != ESP_OK) {
            return err;
        }
    }

    return spi_bus_free(RF_SPI_HOST);
}

esp_err_t rf_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (len == 0)
    {
        return ESP_OK;
    }

    spi_transaction_t t =
    {
        .flags = 0,
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    return spi_device_transmit(s_rf_spi, &t);
}

esp_err_t rf_spi_write_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[64] = {0};

    if ((len + 1) > sizeof(tx))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    tx[0] = reg;
    if (data && len)
    {
        memcpy(&tx[1], data, len);
    }

    spi_transaction_t t =
    {
        .flags = 0,
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };

    return spi_device_transmit(s_rf_spi, &t);
}

esp_err_t rf_spi_read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[64] = {0};
    uint8_t rx[64] = {0};

    if ((len + 1) > sizeof(tx))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    tx[0] = reg;
    memset(&tx[1], 0xFF, len); // dummy bytes

    spi_transaction_t t = 
    {
        .flags = 0,
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_transmit(s_rf_spi, &t);
    if (err != ESP_OK)
    {
        return err;
    }

    if (data && len)
    {
        memcpy(data, &rx[1], len);
    }

    return ESP_OK;
}

/* GPIO config */

esp_err_t rf_gpio_output_init(uint32_t gpio)
{
    gpio_config_t cfg =
    {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    return gpio_config(&cfg);
}

esp_err_t rf_gpio_output_deinit(uint32_t gpio)
{
    return gpio_reset_pin(gpio);
}

esp_err_t rf_gpio_output_write(uint32_t gpio, uint32_t level)
{
    return gpio_set_level(gpio, level ? 1 : 0);
}

esp_err_t gpio_irq_enable(void)
{
    esp_err_t err;

    err = gpio_install_isr_service(0);
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}

esp_err_t rf_gpio_int_input_init(uint32_t gpio, void (*isr_handler)(void *))
{
    esp_err_t err;

    gpio_config_t cfg =
    {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };

    err = gpio_config(&cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    err = gpio_isr_handler_add(gpio, isr_handler, NULL);
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}

esp_err_t rf_gpio_int_input_deinit(uint32_t gpio)
{
    return gpio_reset_pin(gpio);
}