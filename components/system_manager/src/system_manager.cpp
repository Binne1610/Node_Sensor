#include "system_manager.hpp"
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

static const char* TAG = "SYS_MANAGER";

// RTC memory để lưu trạng thái qua deep sleep
RTC_DATA_ATTR static uint32_t rtc_boot_count = 0;
RTC_DATA_ATTR static uint32_t rtc_error_count = 0;
RTC_DATA_ATTR static SystemState rtc_last_state = SystemState::INIT;
// BACKUP - Multi-Stage (không dùng, bỏ để sau này có thể enable lại)
// RTC_DATA_ATTR static uint32_t rtc_read_cycle_count = 0;

SystemManager::SystemManager() 
    : pH_sensor_(nullptr)
    , thec_sensor_(nullptr)
    , npk_sensor_(nullptr)
    , sw_sensor_(nullptr)
    , lora_(nullptr)
    , rtc_(nullptr)
    , pH_value_(0.0f)
    , temp_value_(0.0f)
    , rh_value_(0.0f)
    , ec_value_(0.0f)
    , n_value_(0.0f)
    , p_value_(0.0f)
    , k_value_(0.0f)
    , sw_temp_value_(0.0f)
    , sw_rh_value_(0.0f)
{
    // Kiểm tra nếu first boot → reset RTC memory
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    // DEBUG: Log chi tiết wakeup reason
    ESP_LOGI(TAG, "=== WAKEUP REASON DEBUG ===");
    ESP_LOGI(TAG, "esp_sleep_get_wakeup_cause() = %d", wakeup_reason);
    ESP_LOGI(TAG, "RTC memory RAW: boot=%lu, error=%lu", 
             rtc_boot_count, rtc_error_count);
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            ESP_LOGI(TAG, "Wakeup: UNDEFINED (0) = first boot or reset");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(TAG, "Wakeup: EXT0 (2) = RTC GPIO interrupt");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Wakeup: TIMER (4) = deep sleep timer expired");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            ESP_LOGI(TAG, "Wakeup: EXT1 (3) = multiple GPIO");
            break;
        default:
            ESP_LOGI(TAG, "Wakeup: OTHER (%d) = unknown", wakeup_reason);
            break;
    }
    
    if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER && wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
        // First boot - reset RTC memory
        ESP_LOGI(TAG, "[ACTION] First boot/reset detected - RESETTING RTC memory to 0");
        rtc_boot_count = 0;
        rtc_error_count = 0;
    } else {
        ESP_LOGI(TAG, "[ACTION] Deep sleep wakeup - PRESERVING RTC memory");
    }
    
    // Khôi phục context từ RTC memory
    context_.boot_count = ++rtc_boot_count;
    context_.error_count = rtc_error_count;
    context_.current_state = rtc_last_state;
    context_.next_state = SystemState::INIT;
    context_.last_error = ErrorType::NONE;
    context_.sleep_interval_sec = DEFAULT_SLEEP_SEC;
    
    ESP_LOGI(TAG, "=== SYSTEM MANAGER CREATED ===");
    ESP_LOGI(TAG, "Boot: %lu (wakeup_reason=%d)", 
             context_.boot_count, wakeup_reason);
    ESP_LOGI(TAG, "Mode: Always read ALL sensors (no multi-stage)");
}

SystemManager::~SystemManager() {
    delete pH_sensor_;
    delete thec_sensor_;
    delete npk_sensor_;
    delete sw_sensor_;
    delete lora_;
    delete rtc_;
}

void SystemManager::init() {
    ESP_LOGI(TAG, "=== System Manager Init ===");
    
    // Kiểm tra nguồn đánh thức
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(TAG, "Wakeup from DS3231 RTC alarm");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Wakeup from timer (fallback)");
            break;
        default:
            ESP_LOGI(TAG, "First boot or hardware reset");
            break;
    }
    
    setup_watchdog();
    
    // Khởi tạo DS3231 RTC
    rtc_ = new DS3231_RTC(RTC_SDA_PIN, RTC_SCL_PIN, RTC_INT_PIN);
    if (rtc_->init() != ESP_OK) {
        ESP_LOGE(TAG, "RTC init failed - using timer fallback");
    } else {
        // Xóa alarm flag sau khi wake
        rtc_->clear_alarm_flag();
        
        // Sync NTP chỉ ở lần boot đầu tiên
        sync_ntp_if_needed();
        
        // Đọc nhiệt độ từ RTC (bonus)
        float rtc_temp;
        if (rtc_->read_temperature(rtc_temp) == ESP_OK) {
            ESP_LOGI(TAG, "RTC temperature: %.2f°C", rtc_temp);
        }
    }
    
    // Khởi tạo UART1 driver MỘT LẦN (shared cho tất cả sensors)
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    esp_err_t ret = uart_param_config(UART_NUM_1, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
    }
    
    ret = uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
    }
    
    // Install driver với buffer size 1024 (RX + TX)
    ret = uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "UART1 driver installed successfully");
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "UART driver already installed - OK");
    } else {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
    }
    
    // Khởi tạo sensors (không gọi init() vì UART đã setup)
    pH_sensor_ = new ES_PH_SOIL(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x03, 9600);
    thec_sensor_ = new ES_SM_THEC(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x02, 9600);
    npk_sensor_ = new NPK(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x01, 9600);
    sw_sensor_ = new ES35_SW(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x04, 9600);
    lora_ = new LoRa_SX1278();
    
    // Chuyển sang state đọc sensor
    transition_to(SystemState::READ_SENSORS);
}

void SystemManager::run() {
    ESP_LOGI(TAG, "Starting state machine...");
    
    while (context_.current_state != SystemState::SLEEP) {
        reset_watchdog();
        log_system_status();
        
        esp_err_t result = ESP_OK;
        
        switch (context_.current_state) {
            case SystemState::INIT:
                result = handle_init_state();
                break;
                
            case SystemState::READ_SENSORS:
                result = handle_read_sensors_state();
                break;
                
            case SystemState::SEND_DATA:
                result = handle_send_data_state();
                break;
                
            case SystemState::ERROR_RECOVERY:
                result = handle_error_recovery_state();
                break;
                
            default:
                ESP_LOGE(TAG, "Unknown state!");
                transition_to(SystemState::ERROR_RECOVERY);
                break;
        }
        
        if (result != ESP_OK) {
            handle_error(ErrorType::SENSOR_TIMEOUT);
        }
        
        // Chuyển sang state tiếp theo
        context_.current_state = context_.next_state;
    }
    
    // Cuối cùng vào sleep
    handle_sleep_state();
}

void SystemManager::transition_to(SystemState new_state) {
    ESP_LOGI(TAG, "State transition: %d -> %d", 
             static_cast<int>(context_.current_state), 
             static_cast<int>(new_state));
    context_.next_state = new_state;
}

esp_err_t SystemManager::handle_init_state() {
    ESP_LOGI(TAG, "[INIT] Initializing sensors...");
    
    esp_err_t ret = ESP_OK;
    
    // KHÔNG gọi sensor->init() vì UART đã được setup trong SystemManager::init()
    // Chỉ cần init LoRa (dùng SPI riêng)
    
    if (lora_->init() != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed");
        ret = ESP_FAIL;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "All modules initialized successfully");
        transition_to(SystemState::READ_SENSORS);
    } else {
        transition_to(SystemState::ERROR_RECOVERY);
    }
    
    return ret;
}

esp_err_t SystemManager::handle_read_sensors_state() {
    ESP_LOGI(TAG, "[READ_SENSORS] Reading all sensors...");
    
    esp_err_t ret = read_all_sensors();
    
    if (ret == ESP_OK) {
        transition_to(SystemState::SEND_DATA);
    } else {
        transition_to(SystemState::ERROR_RECOVERY);
    }
    
    return ret;
}

esp_err_t SystemManager::handle_send_data_state() {
    ESP_LOGI(TAG, "[SEND_DATA] Sending LoRa packet...");
    
    esp_err_t ret = build_and_send_lora_packet();
    
    if (ret == ESP_OK) {
        // Reset error count khi thành công
        context_.error_count = 0;
        rtc_error_count = 0;
        transition_to(SystemState::SLEEP);
    } else {
        handle_error(ErrorType::LORA_FAILED);
        transition_to(SystemState::ERROR_RECOVERY);
    }
    
    return ret;
}

esp_err_t SystemManager::handle_sleep_state() {
    ESP_LOGI(TAG, "[SLEEP] Entering deep sleep for %lu seconds", context_.sleep_interval_sec);
    
    // Lưu state vào RTC memory
    rtc_last_state = SystemState::READ_SENSORS; // Sau khi wake sẽ đọc sensor
    rtc_error_count = context_.error_count;
    
    enter_deep_sleep();
    
    // Code sau dòng này không chạy
    return ESP_OK;
}

esp_err_t SystemManager::handle_error_recovery_state() {
    ESP_LOGE(TAG, "[ERROR_RECOVERY] Attempting recovery. Error count: %lu", context_.error_count);
    
    context_.error_count++;
    rtc_error_count = context_.error_count;
    
    if (context_.error_count >= MAX_ERROR_COUNT) {
        ESP_LOGE(TAG, "Too many errors! Resetting system...");
        // Factory reset hoặc restart
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    
    // Retry sau 5 giây
    vTaskDelay(pdMS_TO_TICKS(ERROR_RETRY_DELAY_MS));
    
    // Retry từ đầu
    transition_to(SystemState::INIT);
    
    return ESP_OK;
}

esp_err_t SystemManager::read_all_sensors() {
    ESP_LOGI(TAG, "[READING] Reading ALL sensors...");
    
    // Đọc pH
    if (pH_sensor_->read_PH(pH_value_) == ESP_OK) {
        ESP_LOGI(TAG, "pH: %.2f", pH_value_);
    } else {
        ESP_LOGW(TAG, "pH read failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100ms giữa các lần đọc
    
    // Đọc THEC
    if (thec_sensor_->read_THEC(temp_value_, rh_value_, ec_value_) == ESP_OK) {
        ESP_LOGI(TAG, "THEC: T=%.1f°C, RH=%.1f%%, EC=%.1f", temp_value_, rh_value_, ec_value_);
    } else {
        ESP_LOGW(TAG, "THEC read failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100ms giữa các lần đọc
    
    // Đọc NPK
    if (npk_sensor_->read_NPK(n_value_, p_value_, k_value_) == ESP_OK) {
        ESP_LOGI(TAG, "NPK: N=%.0f, P=%.0f, K=%.0f", n_value_, p_value_, k_value_);
    } else {
        ESP_LOGW(TAG, "NPK read failed");
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100ms giữa các lần đọc
    
    // Đọc SW
    if (sw_sensor_->read_ES35_SW(sw_temp_value_, sw_rh_value_) == ESP_OK) {
        ESP_LOGI(TAG, "SW: T=%.1f°C, RH=%.1f%%", sw_temp_value_, sw_rh_value_);
    } else {
        ESP_LOGW(TAG, "SW read failed");
    }
    
    return ESP_OK;
}

esp_err_t SystemManager::build_and_send_lora_packet() {
    char data[256];
    char timestamp[64] = "unknown";  // Tăng buffer size để tránh truncation warning
    
    // Lấy timestamp từ RTC
    if (rtc_ != nullptr) {
        struct tm timeinfo;
        if (rtc_->get_datetime(&timeinfo) == ESP_OK) {
            snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        }
    }
    
    // Format LoRa packet (luôn gửi tất cả sensors)
    snprintf(data, sizeof(data), 
             "pH:%.2f;T:%.1f;RH:%.1f;EC:%.1f;N:%.0f;P:%.0f;K:%.0f;SW_T:%.1f;SW_RH:%.1f;time:%s;boot:%lu",
             pH_value_, temp_value_, rh_value_, ec_value_,
             n_value_, p_value_, k_value_,
             sw_temp_value_, sw_rh_value_,
             timestamp, context_.boot_count);
    
    ESP_LOGI(TAG, "LoRa data: %s", data);
    
    esp_err_t ret = lora_->send((uint8_t*)data, strlen(data));
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LoRa packet sent successfully");
    } else {
        ESP_LOGE(TAG, "LoRa send failed");
    }
    
    return ret;
}

void SystemManager::enter_deep_sleep() {
    ESP_LOGI(TAG, "[SLEEP] Preparing for deep sleep...");
    
    // Cleanup watchdog - SAFER METHOD
    esp_err_t wdt_err = esp_task_wdt_delete(NULL);
    ESP_LOGI(TAG, "Watchdog delete result: %d", wdt_err);
    vTaskDelay(pdMS_TO_TICKS(50));  // Tăng delay
    
    wdt_err = esp_task_wdt_deinit();
    ESP_LOGI(TAG, "Watchdog deinit result: %d", wdt_err);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // TẠM THỜI TẮT RTC WAKEUP - CHỈ DÙNG TIMER
    /*
    // Cấu hình DS3231 RTC alarm
    setup_rtc_wakeup(context_.sleep_interval_sec);
    
    // Cấu hình ESP32 wake on RTC INT pin (EXT0)
    esp_sleep_enable_ext0_wakeup(RTC_INT_PIN, 0);  // Wake on LOW
    */
    
    // Timer wakeup (chính)
    uint64_t sleep_time_us = context_.sleep_interval_sec * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_time_us);
    
    ESP_LOGI(TAG, "=== DEEP SLEEP DEBUG ===");
    ESP_LOGI(TAG, "Sleep time: %lu seconds (%llu us)", context_.sleep_interval_sec, sleep_time_us);
    ESP_LOGI(TAG, "RTC memory - Boot: %lu", rtc_boot_count);
    ESP_LOGI(TAG, "Goodbye! Entering deep sleep NOW...");
    
    vTaskDelay(pdMS_TO_TICKS(200)); // Tăng delay để log flush hoàn toàn
    
    // Vào deep sleep
    esp_deep_sleep_start();
    
    // Code này không bao giờ chạy được
    ESP_LOGE(TAG, "ERROR: esp_deep_sleep_start() failed!");
}

void SystemManager::handle_error(ErrorType error) {
    context_.last_error = error;
    
    switch (error) {
        case ErrorType::SENSOR_TIMEOUT:
            ESP_LOGE(TAG, "Error: Sensor timeout");
            break;
        case ErrorType::LORA_FAILED:
            ESP_LOGE(TAG, "Error: LoRa transmission failed");
            break;
        case ErrorType::WATCHDOG_TIMEOUT:
            ESP_LOGE(TAG, "Error: Watchdog timeout");
            break;
        default:
            break;
    }
}

void SystemManager::setup_watchdog() {
    ESP_LOGI(TAG, "Setting up watchdog (%lu sec timeout)", WATCHDOG_TIMEOUT_SEC);
    
    // Deinit watchdog cũ nếu đã tồn tại (tránh lỗi already initialized)
    esp_task_wdt_deinit();
    
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
}

void SystemManager::reset_watchdog() {
    esp_task_wdt_reset();
}

void SystemManager::log_system_status() {
    ESP_LOGI(TAG, "--- System Status ---");
    ESP_LOGI(TAG, "Boot: %lu, Errors: %lu, State: %d", 
             context_.boot_count, context_.error_count, static_cast<int>(context_.current_state));
}

void SystemManager::setup_rtc_wakeup(uint32_t seconds) {
    if (rtc_ == nullptr) {
        ESP_LOGW(TAG, "RTC not initialized, using timer wakeup only");
        return;
    }
    
    esp_err_t ret = rtc_->set_alarm_interval(seconds);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DS3231 alarm set for %lu seconds", seconds);
    } else {
        ESP_LOGE(TAG, "Failed to set RTC alarm, will use timer fallback");
    }
}

esp_err_t SystemManager::sync_ntp_if_needed() {
    // Chỉ sync NTP ở lần boot đầu tiên (first power-on, không phải wake từ sleep)
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    if (context_.boot_count == 1 && wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG, "First boot detected - syncing time from NTP...");
        
        // Kết nối WiFi
        WiFiHelper wifi;
        esp_err_t ret = wifi.connect(WIFI_SSID, WIFI_PASSWORD, 15000);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi connection failed - skipping NTP sync");
            return ret;
        }
        
        // Sync NTP
        ret = rtc_->sync_from_ntp(NTP_SERVER, TIMEZONE);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "RTC synced with NTP successfully!");
            
            // Hiển thị thời gian đã sync
            struct tm timeinfo;
            if (rtc_->get_datetime(&timeinfo) == ESP_OK) {
                ESP_LOGI(TAG, "Current time: %04d-%02d-%02d %02d:%02d:%02d",
                         timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            }
        } else {
            ESP_LOGE(TAG, "NTP sync failed");
        }
        
        // Ngắt WiFi để tiết kiệm pin
        wifi.disconnect();
        
        return ret;
    } else {
        ESP_LOGI(TAG, "Not first boot (boot_count=%lu) - skipping NTP sync", context_.boot_count);
        return ESP_OK;
    }
}

