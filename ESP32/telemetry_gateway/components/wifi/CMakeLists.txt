idf_component_register(
    SRCS
        "udp_client.c"
        "wifi_connect.c"

    INCLUDE_DIRS
        "include"

    PRIV_REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
)
