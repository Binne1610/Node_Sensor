#pragma once
#include "system_states.hpp"
#include "ES_PH_SOIL.hpp"
#include "ES_SM_THEC.hpp"
#include "NPK.hpp"
#include "ES35_SW.hpp"
#include "LoRa_SX1278.hpp"
#include "DS3231_RTC.hpp"
#include "WiFiHelper.hpp"
#include <esp_err.h>

class SystemManager {
public:
    SystemManager();
    ~SystemManager();
    
    // Main control methods
    void init();
    void run();
    void transition_to(SystemState new_state);
    
    // State handlers
    esp_err_t handle_init_state();
    esp_err_t handle_read_sensors_state();
    esp_err_t handle_send_data_state();
    esp_err_t handle_sleep_state();
    esp_err_t handle_error_recovery_state();
    
    // Utility
    void setup_watchdog();
    void reset_watchdog();
    void log_system_status();
    
private:
    SystemContext context_;
    
    // Sensors
    ES_PH_SOIL* pH_sensor_;
    ES_SM_THEC* thec_sensor_;
    NPK* npk_sensor_;
    ES35_SW* sw_sensor_;
    LoRa_SX1278* lora_;
    DS3231_RTC* rtc_;  // RTC module for wakeup
    
    // Sensor data buffer
    float pH_value_;
    float temp_value_;
    float rh_value_;
    float ec_value_;
    float n_value_;
    float p_value_;
    float k_value_;
    float sw_temp_value_;
    float sw_rh_value_;
    
    // Config
    static constexpr uint32_t WATCHDOG_TIMEOUT_SEC = 60;
    static constexpr uint32_t MAX_ERROR_COUNT = 5;
    static constexpr uint32_t DEFAULT_SLEEP_SEC = 10;  // 1 tiếng (3600 giây)
    static constexpr uint32_t ERROR_RETRY_DELAY_MS = 5000;
    
    // BACKUP - Multi-Stage Sensor Reading (không dùng, giữ lại để sau có thể enable)
    // static constexpr uint32_t FAST_SLEEP_SEC = 60;
    // static constexpr uint32_t FULL_READ_INTERVAL = 6;
    
    // DS3231 RTC pins (đổi sang GPIO an toàn)
    static constexpr gpio_num_t RTC_SDA_PIN = GPIO_NUM_3;  // Standard I2C SDA
    static constexpr gpio_num_t RTC_SCL_PIN = GPIO_NUM_2;  // Standard I2C SCL
    static constexpr gpio_num_t RTC_INT_PIN = GPIO_NUM_35;  // SQW/INT pin wake ESP32
    
    // WiFi credentials cho NTP sync
    static constexpr const char* WIFI_SSID = "NhaCuaChuoi";
    static constexpr const char* WIFI_PASSWORD = "12356789";
    static constexpr const char* NTP_SERVER = "pool.ntp.org";
    static constexpr const char* TIMEZONE = "UTC-7";  // Vietnam UTC+7
    
    // Helper methods
    esp_err_t read_all_sensors();
    esp_err_t build_and_send_lora_packet();
    void enter_deep_sleep();
    void handle_error(ErrorType error);
    void setup_rtc_wakeup(uint32_t seconds);
    esp_err_t sync_ntp_if_needed();  // Sync NTP on first boot
};
