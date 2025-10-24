idf_component_register(
    SRCS "src/LoRa_E32_UART.cpp"
    INCLUDE_DIRS "include"
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)