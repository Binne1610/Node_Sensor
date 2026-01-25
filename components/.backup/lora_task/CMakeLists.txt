idf_component_register(SRCS "src/lora_task.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos sensor_data LoRa_SX1278)
