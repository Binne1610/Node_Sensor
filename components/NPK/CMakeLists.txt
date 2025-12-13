idf_component_register(
    SRCS "src/NPK.cpp"
    INCLUDE_DIRS "include"
    REQUIRES uart_shared
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_timer
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)