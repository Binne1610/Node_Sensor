idf_component_register(
    SRCS "src/system_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        modbus_base
        ES_PH_SOIL
        ES_SM_THEC
        NPK
        ES35_SW
        LoRa_SX1278
        DS3231_RTC
        WiFiHelper
        esp_system
        esp_hw_support
        esp_timer
)
