#ifndef NRF905_EVENT_INDEX_H
#define NRF905_EVENT_INDEX_H

#define NRF905_NOTIFY_DR_INDEX    0U
#define NRF905_NOTIFY_SPI_INDEX   1U

#define DR_INT         (1U << 0)
#define SPI_ERROR      (1U << 1)
#define SPI_ABORT      (1U << 2)
#define SPI_TXRX_CPLT  (1U << 3)

#endif
