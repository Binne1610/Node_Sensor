idf_component_register(SRCS "Node_Sensor.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES ES_PH_SOIL ES_SM_THEC NPK ES35_SW uart_shared LoRa_SX1278)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)