#include <ES_PH_SOIL.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>  
#include <esp_log.h>

extern "C" void app_main() {
    ES_PH_SOIL ph_sensor(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16);
    ph_sensor.init();

    while(1) {
        float ph_value = ph_sensor.readPH();
        if (ph_value != -1.0f) {
            ESP_LOGI("PH_SENSOR", "pH Value: %.2f\n", ph_value);
        } else {
            ESP_LOGE("PH_SENSOR", "Failed to read pH value\n");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}