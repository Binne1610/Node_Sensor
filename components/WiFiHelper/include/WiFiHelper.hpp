#pragma once

#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_event.h>

class WiFiHelper {
public:
    WiFiHelper();
    ~WiFiHelper();
    
    // Khởi tạo WiFi và kết nối
    esp_err_t connect(const char* ssid, const char* password, uint32_t timeout_ms = 10000);
    
    // Ngắt kết nối và deinit WiFi
    esp_err_t disconnect();
    
    // Kiểm tra đã kết nối chưa
    bool is_connected();

private:
    bool initialized_;
    bool connected_;
    
    static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                                   int32_t event_id, void* event_data);
};
