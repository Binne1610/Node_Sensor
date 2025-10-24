#include "ES_PH_SOIL.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

extern "C" void app_main() {
    ES_PH_SOIL ph_sensor(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
    if (ph_sensor.init() != ESP_OK) {
        ESP_LOGE("PH_SENSOR", "Init failed");
        return;
    }

    while (1) {
        float ph_value = 0.0f;
        if (ph_sensor.readPH(ph_value) == ESP_OK) {
            ESP_LOGI("PH_SENSOR", "pH Value: %.2f", ph_value);
        } else {
            ESP_LOGE("PH_SENSOR", "Failed to read pH value");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}