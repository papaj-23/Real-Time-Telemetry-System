idf_component_register(
    SRCS
        "telemetry_gateway.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        wifi
        nrf905
        esp_driver_gpio
)
