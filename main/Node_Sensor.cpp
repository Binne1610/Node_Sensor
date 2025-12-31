#include "sensor_data.hpp"
#include "sensor_tasks.hpp"
#include "lora_task.hpp"
#include "ES_PH_SOIL.hpp"
#include "ES_SM_THEC.hpp"
#include "NPK.hpp"
#include "ES35_SW.hpp"
#include "LoRa_SX1278.hpp"
#include <esp_log.h>

static const char* TAG = "APP_MAIN";

extern "C" void app_main() {
    // Khởi tạo các cảm biến trên cùng UART với địa chỉ khác nhau
    static ES_PH_SOIL pH_sensor   (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x03, 9600);
    static ES_SM_THEC thec_sensor (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x02, 9600);
    static NPK        npk_sensor  (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x01, 9600);
    static ES35_SW    sw_sensor   (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x04, 9600);
    static LoRa_SX1278 lora;      // Khởi tạo LoRa instance

    // Init các sensor
    if (pH_sensor.init()    != ESP_OK){ ESP_LOGE(TAG, "Init pH failed");       return; }
    if (thec_sensor.init()  != ESP_OK){ ESP_LOGE(TAG, "Init THEC failed");     return; }
    if (npk_sensor.init()   != ESP_OK){ ESP_LOGE(TAG, "Init NPK failed");      return; }
    if (sw_sensor.init()    != ESP_OK){ ESP_LOGE(TAG, "Init ES35_SW failed");  return; }

    // Khởi tạo mutexes
    sensor_data_init();

    ESP_LOGI(TAG, "Sensors initialized. Starting address scan...");
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Scan địa chỉ trước khi start tasks (optional)
    scan_modbus_addresses();
    
    ESP_LOGI(TAG, "Starting sensor reading tasks...");
    // Priority: Sensor tasks (5) > LoRa task (2) - Ưu tiên đọc sensor trước
    xTaskCreate(read_pH_task,   "read_pH",   4096, &pH_sensor,   5, nullptr);
    xTaskCreate(read_THEC_task, "read_THEC", 4096, &thec_sensor, 5, nullptr);
    xTaskCreate(read_NPK_task,  "read_NPK",  4096, &npk_sensor,  5, nullptr);
    xTaskCreate(read_SW_task,   "read_SW",   4096, &sw_sensor,   5, nullptr);
    xTaskCreate(lora_transmission_task, "lora_task", 4096, &lora, 2, nullptr);
    
    ESP_LOGI(TAG, "All tasks created successfully (including LoRa)");
}
