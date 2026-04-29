#include "rf_link.h"
#include "rf_init.h"
#include "rf_peripherals.h"

extern nrf905_handle_t gs_handle;

static void nrf905_task_handler(void *pvParameters)
{

    for(;;) {

    }
}

void start_nrf905_device(void)
{
    xTaskCreate(nrf905_task_handler, "nrf905_task", 1024, NULL, 6, NULL);
}

