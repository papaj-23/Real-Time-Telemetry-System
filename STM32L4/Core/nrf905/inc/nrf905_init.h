#ifndef RF_INIT_H
#define RF_INIT_H

#include "driver_nrf905_interface.h"

#define NRF905_BASIC_DEFAULT_PLL_MODE                            NRF905_PLL_MODE_433_MHZ
#define NRF905_BASIC_DEFAULT_OUTPUT_POWER                        NRF905_OUTPUT_POWER_10_DBM
#define NRF905_BASIC_DEFAULT_RX_MODE                             NRF905_RX_MODE_NORMAL
#define NRF905_BASIC_DEFAULT_AUTO_RETRANSMIT                     NRF905_BOOL_FALSE
#define NRF905_BASIC_DEFAULT_RX_ADDRESS_WIDTH                    NRF905_ADDRESS_WIDTH_4_BYTE
#define NRF905_BASIC_DEFAULT_TX_ADDRESS_WIDTH                    NRF905_ADDRESS_WIDTH_4_BYTE
#define NRF905_BASIC_DEFAULT_RX_PAYLOAD_WIDTH                    32
#define NRF905_BASIC_DEFAULT_TX_PAYLOAD_WIDTH                    18
#define NRF905_BASIC_DEFAULT_RX_ADDR                             {0xF7, 0xE7, 0xE7, 0xE2}
#define NRF905_BASIC_DEFAULT_OUTPUT_CLOCK_FREQUENCY              NRF905_OUTPUT_CLOCK_FREQUENCY_500KHZ
#define NRF905_BASIC_DEFAULT_OUTPUT_CLOCK                        NRF905_BOOL_FALSE
#define NRF905_BASIC_DEFAULT_CRYSTAL_OSCILLATOR_FREQUENCY        NRF905_CRYSTAL_OSCILLATOR_FREQUENCY_16MHZ
#define NRF905_BASIC_DEFAULT_CRC                                 NRF905_BOOL_TRUE
#define NRF905_BASIC_DEFAULT_CRC_MODE                            NRF905_CRC_MODE_8
#define NRF905_BASIC_DEFAULT_FREQUENCY                           433.2f

uint8_t nrf905_interrupt_irq_handler(void);
uint8_t nrf905_device_init(nrf905_mode_t mode);
uint8_t nrf905_device_deinit(void);

#endif