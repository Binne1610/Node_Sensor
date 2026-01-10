#include "lora_task.hpp"
#include "sensor_data.hpp"
#include "LoRa_SX1278.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char* TAG = "LORA_TASK";

// Task truyền dữ liệu qua LoRa SX1278
void lora_transmission_task(void *parameter) {
    LoRa_SX1278 *lora = static_cast<LoRa_SX1278*>(parameter);

    ESP_LOGI(TAG, "LoRa task started");
    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay 10s để các sensor đọc dữ liệu trước

    // Khởi tạo LoRa
    if (lora->init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        vTaskDelete(nullptr);
        return;
    }

    char message[256]; // Buffer đủ lớn cho tất cả dữ liệu sensor
    
    for (;;) {
        // Đọc dữ liệu từ biến toàn cục (có mutex bảo vệ)
        if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Format message với tất cả giá trị cảm biến
            snprintf(message, sizeof(message), 
                "pH:%.2f;RH_THEC:%.2f;Temp_THEC:%.2f;EC_THEC:%.2f;N:%.2f;P:%.2f;K:%.2f;RH_SW:%.1f;Temp_SW:%.1f;",
                gSensorData.pH,
                gSensorData.RH_THEC,
                gSensorData.Temp_THEC,
                gSensorData.EC_THEC,
                gSensorData.N_value,
                gSensorData.P_value,
                gSensorData.K_value,
                gSensorData.RH_SW,
                gSensorData.Temp_SW
            );
            xSemaphoreGive(gDataMutex);
            
            // Gửi qua LoRa
            esp_err_t err = lora->send(reinterpret_cast<const uint8_t*>(message), strlen(message));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Sensor data sent: %s", message);
            } else {
                ESP_LOGE(TAG, "Failed to send message");
            }
        } else {
            ESP_LOGW(TAG, "Cannot access sensor data - mutex timeout");
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Gửi mỗi 10 giây
    }
}
