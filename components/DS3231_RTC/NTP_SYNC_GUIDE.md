# DS3231 RTC với NTP Sync

## Tính năng mới

### 1. Set datetime thủ công
```cpp
struct tm timeinfo;
timeinfo.tm_year = 2026 - 1900;  // 2026
timeinfo.tm_mon = 0;              // January (0-11)
timeinfo.tm_mday = 16;            // 16th
timeinfo.tm_hour = 14;            // 14:00
timeinfo.tm_min = 30;
timeinfo.tm_sec = 0;

rtc->set_datetime(&timeinfo);
```

### 2. Đọc datetime từ DS3231
```cpp
struct tm timeinfo;
rtc->get_datetime(&timeinfo);
ESP_LOGI(TAG, "Current: %04d-%02d-%02d %02d:%02d:%02d",
         timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
```

### 3. Đồng bộ từ NTP (Cần WiFi)
```cpp
// Sử dụng NTP server mặc định (pool.ntp.org) và timezone Việt Nam (UTC+7)
rtc->sync_from_ntp("pool.ntp.org", "UTC-7");

// Hoặc dùng NTP server Việt Nam
rtc->sync_from_ntp("time.google.com", "UTC-7");
rtc->sync_from_ntp("1.asia.pool.ntp.org", "UTC-7");
```

## Timezone cho Việt Nam
- **UTC-7**: `"UTC-7"` hoặc `"ICT-7"` (Indochina Time)
- Với daylight saving: `"<+07>-7"`

## Cách tích hợp vào SystemManager

### Option 1: Sync NTP mỗi lần boot (nếu có WiFi)
```cpp
void SystemManager::init() {
    // ... existing code ...
    
    // Khởi tạo WiFi (nếu cần NTP)
    wifi_init();
    wifi_connect("SSID", "PASSWORD");
    
    // Sync NTP
    if (rtc_->sync_from_ntp("pool.ntp.org", "UTC-7") == ESP_OK) {
        ESP_LOGI(TAG, "RTC synced with NTP");
    }
    
    // Disconnect WiFi để tiết kiệm pin
    wifi_disconnect();
}
```

### Option 2: Chỉ sync NTP khi cần (thêm vào state machine)
```cpp
enum class SystemState {
    INIT,
    SYNC_TIME,     // State mới
    READ_SENSORS,
    SEND_DATA,
    SLEEP,
    ERROR_RECOVERY
};
```

## Lưu ý
- NTP cần WiFi → Tốn thêm ~3-5 giây boot time
- Chỉ cần sync 1 lần sau đó DS3231 tự giữ thời gian
- DS3231 có pin backup (CR2032) → Giữ thời gian khi ESP32 mất nguồn
- Độ chính xác: ±2ppm (±1 phút/năm)
