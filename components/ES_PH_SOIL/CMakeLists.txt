idf_component_register(
    SRCS "src/ES_PH_SOIL.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_driver_gpio esp_timer modbus_base
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)