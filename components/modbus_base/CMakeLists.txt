idf_component_register(SRCS "src/ModbusSensorBase.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_timer)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
