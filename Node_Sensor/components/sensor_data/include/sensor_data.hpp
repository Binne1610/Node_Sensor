#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct SensorData {
    float pH = 0.0f;
    float RH_THEC = 0.0f;
    float Temp_THEC = 0.0f;
    float EC_THEC = 0.0f;
    float N_value = 0.0f;
    float P_value = 0.0f;
    float K_value = 0.0f;
    float RH_SW = 0.0f;
    float Temp_SW = 0.0f;
};

// Global data và mutex
extern SensorData gSensorData;
extern SemaphoreHandle_t gDataMutex;
extern SemaphoreHandle_t gUartMutex;

// Khởi tạo mutex
void sensor_data_init();
