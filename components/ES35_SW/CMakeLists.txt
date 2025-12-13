idf_component_register(
    SRCS "src/ES35_SW.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_timer uart_shared
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)