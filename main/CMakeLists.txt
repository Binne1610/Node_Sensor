idf_component_register(SRCS "Node_Sensor.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES system_manager)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)