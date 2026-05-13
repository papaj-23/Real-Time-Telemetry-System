#include <stdio.h>
#include "wifi_connect.h"
#include "udp_client.h"
#include "rf_link.h"

void app_main(void)
{
    my_wifi_connect();
    start_nrf905_thread();
    start_udp_client();
}
