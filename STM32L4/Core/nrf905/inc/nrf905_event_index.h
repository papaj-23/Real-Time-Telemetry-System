#ifndef NRF905_EVENT_INDEX_H
#define NRF905_EVENT_INDEX_H

#define NRF905_NOTIFY_DR_INDEX    0U
#define NRF905_NOTIFY_SPI_INDEX   1U

#define NRF905_DR_INT         (1U << 0U)
#define NRF905_SPI_ERROR      (1U << 0U)
#define NRF905_SPI_ABORT      (1U << 1U)
#define NRF905_SPI_TXRX_CPLT  (1U << 2U)

#endif
