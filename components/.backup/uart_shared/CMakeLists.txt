idf_component_register(
    SRCS "src/uart_shared.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_uart 
)
