idf_component_register(SRCS "src/sensor_tasks.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos sensor_data ES_PH_SOIL ES_SM_THEC NPK ES35_SW uart_shared)
