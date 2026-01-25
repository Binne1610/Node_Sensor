#include "system_manager.hpp"
#include <esp_log.h>

static const char* TAG = "NODE_SENSOR";

extern "C" void app_main() {
    ESP_LOGI(TAG, "=== Node Sensor Starting (Level 2 Architecture) ===");
    
    // Khởi tạo System Manager
    SystemManager* system_manager = new SystemManager();
    
    // Khởi tạo system
    system_manager->init();
    
    // Chạy state machine (sẽ loop đến khi vào deep sleep)
    system_manager->run();
    
    // Code sau này không bao giờ chạy (deep sleep restart)
}
