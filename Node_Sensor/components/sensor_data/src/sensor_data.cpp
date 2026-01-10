#include "sensor_data.hpp"
#include <esp_log.h>

static const char* TAG = "SENSOR_DATA";

// Định nghĩa biến toàn cục
SensorData gSensorData;
SemaphoreHandle_t gDataMutex = nullptr;
SemaphoreHandle_t gUartMutex = nullptr;

void sensor_data_init() {
    gUartMutex = xSemaphoreCreateMutex();
    if (gUartMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create UART mutex");
        return;
    }

    gDataMutex = xSemaphoreCreateMutex();
    if (gDataMutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create Data mutex");
        return;
    }

    ESP_LOGI(TAG, "Sensor data mutexes initialized");
}
