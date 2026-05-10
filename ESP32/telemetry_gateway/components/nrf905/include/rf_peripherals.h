#ifndef RF_PERIPHERALS_H
#define RF_PERIPHERALS_H

#include "esp_err.h"
#include <stdint.h>

#define RF_CE_GPIO          33
#define RF_TX_EN_GPIO       32
#define RF_PWR_UP_GPIO      25
#define RF_DR_GPIO          26
#define RF_AM_GPIO          22
#define RF_CD_GPIO          21

esp_err_t rf_spi_init(void);
esp_err_t rf_spi_deinit(void);

esp_err_t rf_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len);
esp_err_t rf_spi_write_reg(uint8_t reg, uint8_t *data, uint16_t len);
esp_err_t rf_spi_read_reg(uint8_t reg, uint8_t *data, uint16_t len);

esp_err_t rf_gpio_output_init(uint32_t gpio);
esp_err_t rf_gpio_output_deinit(uint32_t gpio);
esp_err_t rf_gpio_output_write(uint32_t gpio, uint32_t level);

esp_err_t irq_enable(void);
esp_err_t rf_gpio_int_input_init(uint32_t gpio, void (*isr_handler)(void *));
esp_err_t rf_gpio_int_input_deinit(uint32_t gpio);

#endif
