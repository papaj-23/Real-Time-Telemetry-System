#ifndef RF_LINK_H
#define RF_LINK_H

#include <stdint.h>

/**
 * @brief Init RTOS resources for NRF905.
 *
 * @retval 0 OK
 * @retval 1 Error
 */
uint8_t nrf905_rtos_init(void);

#endif