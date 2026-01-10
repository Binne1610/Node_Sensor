idf_component_register(
    SRCS "src/LoRa_SX1278.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_spi freertos
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)