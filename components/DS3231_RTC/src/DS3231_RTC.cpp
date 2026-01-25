#include "DS3231_RTC.hpp"
#include <esp_log.h>
#include <esp_sntp.h>
#include <string.h>

static const char* TAG = "DS3231";

DS3231_RTC::DS3231_RTC(gpio_num_t sda_pin, gpio_num_t scl_pin, gpio_num_t int_pin)
    : bus_handle_(nullptr)
    , dev_handle_(nullptr)
    , sda_pin_(sda_pin)
    , scl_pin_(scl_pin)
    , int_pin_(int_pin)
{
}

DS3231_RTC::~DS3231_RTC() {
    if (dev_handle_) {
        i2c_master_bus_rm_device(dev_handle_);
    }
    if (bus_handle_) {
        i2c_del_master_bus(bus_handle_);
    }
}

esp_err_t DS3231_RTC::init() {
    ESP_LOGI(TAG, "Initializing DS3231 RTC with new I2C master driver...");
    
    // Cấu hình I2C master bus (new driver)
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = sda_pin_;
    bus_config.scl_io_num = scl_pin_;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Thêm DS3231 device vào bus
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = DS3231_I2C_ADDR;
    dev_config.scl_speed_hz = 100000;  // 100kHz
    
    ret = i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DS3231 device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Kiểm tra kết nối với DS3231
    uint8_t status;
    ret = read_register(DS3231_REG_STATUS, status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot communicate with DS3231");
        return ret;
    }
    
    // Cấu hình Control Register: Enable INTCN (Interrupt Control)
    // INTCN = 1: SQW/INT pin outputs interrupt signal when alarm matches
    uint8_t ctrl = DS3231_CTRL_INTCN;
    ret = write_register(DS3231_REG_CONTROL, ctrl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure control register");
        return ret;
    }
    
    // Xóa các alarm flags
    ret = clear_alarm_flag();
    
    ESP_LOGI(TAG, "DS3231 initialized successfully");
    return ret;
}

esp_err_t DS3231_RTC::set_alarm_interval(uint32_t seconds) {
    ESP_LOGI(TAG, "Setting alarm for %lu seconds", seconds);
    
    // DS3231 có 2 alarms:
    // - Alarm 1: Có thể match giây/phút/giờ/ngày
    // - Alarm 2: Chỉ match phút/giờ/ngày (không có giây)
    
    // Đọc thời gian hiện tại
    uint8_t time_data[7];
    esp_err_t ret = read_registers(DS3231_REG_SECONDS, time_data, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current time");
        return ret;
    }
    
    // Convert BCD to decimal
    uint8_t curr_sec = bcd_to_dec(time_data[0] & 0x7F);
    uint8_t curr_min = bcd_to_dec(time_data[1] & 0x7F);
    uint8_t curr_hour = bcd_to_dec(time_data[2] & 0x3F);
    
    // Tính toán thời gian alarm
    uint32_t total_sec = curr_sec + (curr_min * 60) + (curr_hour * 3600) + seconds;
    
    uint8_t alarm_sec = (total_sec % 60);
    uint8_t alarm_min = ((total_sec / 60) % 60);
    uint8_t alarm_hour = ((total_sec / 3600) % 24);
    
    ESP_LOGI(TAG, "Current time: %02d:%02d:%02d", curr_hour, curr_min, curr_sec);
    ESP_LOGI(TAG, "Alarm set to: %02d:%02d:%02d", alarm_hour, alarm_min, alarm_sec);
    
    // Cấu hình Alarm 1 (seconds/minutes/hours match)
    // A1M1-A1M4 = 0: Alarm when seconds, minutes, hours match
    uint8_t alarm1_data[4];
    alarm1_data[0] = dec_to_bcd(alarm_sec);       // Seconds (A1M1=0)
    alarm1_data[1] = dec_to_bcd(alarm_min);       // Minutes (A1M2=0)
    alarm1_data[2] = dec_to_bcd(alarm_hour);      // Hours (A1M3=0)
    alarm1_data[3] = 0x80;                         // Day/Date (A1M4=1, DY/DT=0) - ignore day
    
    // Ghi Alarm 1 registers
    for (int i = 0; i < 4; i++) {
        ret = write_register(DS3231_REG_ALARM1_SEC + i, alarm1_data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write alarm register %d", i);
            return ret;
        }
    }
    
    // Enable Alarm 1 Interrupt
    uint8_t ctrl;
    ret = read_register(DS3231_REG_CONTROL, ctrl);
    if (ret != ESP_OK) return ret;
    
    ctrl |= DS3231_CTRL_A1IE | DS3231_CTRL_INTCN;  // Enable A1IE and INTCN
    ret = write_register(DS3231_REG_CONTROL, ctrl);
    
    // Clear alarm flag
    clear_alarm_flag();
    
    ESP_LOGI(TAG, "Alarm 1 configured successfully");
    return ESP_OK;
}

esp_err_t DS3231_RTC::clear_alarm_flag() {
    uint8_t status;
    esp_err_t ret = read_register(DS3231_REG_STATUS, status);
    if (ret != ESP_OK) return ret;
    
    // Clear A1F and A2F bits
    status &= ~(DS3231_STAT_A1F | DS3231_STAT_A2F);
    ret = write_register(DS3231_REG_STATUS, status);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Alarm flags cleared");
    }
    
    return ret;
}

bool DS3231_RTC::is_alarm_triggered() {
    uint8_t status;
    if (read_register(DS3231_REG_STATUS, status) != ESP_OK) {
        return false;
    }
    
    return (status & DS3231_STAT_A1F) != 0;
}

esp_err_t DS3231_RTC::read_temperature(float& temp) {
    uint8_t temp_data[2];
    esp_err_t ret = read_registers(0x11, temp_data, 2);  // Temperature registers
    if (ret != ESP_OK) {
        return ret;
    }
    
    int16_t temp_raw = (temp_data[0] << 8) | temp_data[1];
    temp = temp_raw / 256.0f;
    
    return ESP_OK;
}

// Private helper functions
esp_err_t DS3231_RTC::write_register(uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle_, write_buf, 2, 1000);
}

esp_err_t DS3231_RTC::read_register(uint8_t reg, uint8_t& value) {
    return read_registers(reg, &value, 1);
}

esp_err_t DS3231_RTC::read_registers(uint8_t reg, uint8_t* data, size_t len) {
    return i2c_master_transmit_receive(dev_handle_, &reg, 1, data, len, 1000);
}

uint8_t DS3231_RTC::dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

uint8_t DS3231_RTC::bcd_to_dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

esp_err_t DS3231_RTC::set_datetime(const struct tm* timeinfo) {
    if (timeinfo == nullptr) {
        ESP_LOGE(TAG, "Invalid timeinfo");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting DS3231 datetime: %04d-%02d-%02d %02d:%02d:%02d",
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    
    // Prepare datetime data in BCD format
    uint8_t datetime_data[8];
    datetime_data[0] = DS3231_REG_SECONDS;  // Starting register address
    datetime_data[1] = dec_to_bcd(timeinfo->tm_sec);              // Seconds (0-59)
    datetime_data[2] = dec_to_bcd(timeinfo->tm_min);              // Minutes (0-59)
    datetime_data[3] = dec_to_bcd(timeinfo->tm_hour);             // Hours (0-23)
    datetime_data[4] = dec_to_bcd(timeinfo->tm_wday + 1);         // Day of week (1-7)
    datetime_data[5] = dec_to_bcd(timeinfo->tm_mday);             // Date (1-31)
    datetime_data[6] = dec_to_bcd(timeinfo->tm_mon + 1);          // Month (1-12)
    datetime_data[7] = dec_to_bcd(timeinfo->tm_year - 100);       // Year (0-99, offset from 2000)
    
    // Write register address + 7 bytes of datetime
    esp_err_t ret = i2c_master_transmit(dev_handle_, datetime_data, 8, 1000);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DS3231 datetime set successfully");
    } else {
        ESP_LOGE(TAG, "Failed to set DS3231 datetime: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t DS3231_RTC::get_datetime(struct tm* timeinfo) {
    if (timeinfo == nullptr) {
        ESP_LOGE(TAG, "Invalid timeinfo");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read 7 bytes from SECONDS register
    uint8_t datetime_data[7];
    esp_err_t ret = read_registers(DS3231_REG_SECONDS, datetime_data, 7);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read datetime from DS3231");
        return ret;
    }
    
    // Convert BCD to decimal and populate tm struct
    timeinfo->tm_sec = bcd_to_dec(datetime_data[0] & 0x7F);       // Seconds
    timeinfo->tm_min = bcd_to_dec(datetime_data[1] & 0x7F);       // Minutes
    timeinfo->tm_hour = bcd_to_dec(datetime_data[2] & 0x3F);      // Hours (24h format)
    timeinfo->tm_wday = bcd_to_dec(datetime_data[3] & 0x07) - 1;  // Day of week (0-6)
    timeinfo->tm_mday = bcd_to_dec(datetime_data[4] & 0x3F);      // Date
    timeinfo->tm_mon = bcd_to_dec(datetime_data[5] & 0x1F) - 1;   // Month (0-11)
    timeinfo->tm_year = bcd_to_dec(datetime_data[6]) + 100;       // Year (offset from 1900)
    
    ESP_LOGI(TAG, "DS3231 datetime: %04d-%02d-%02d %02d:%02d:%02d",
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    
    return ESP_OK;
}

esp_err_t DS3231_RTC::sync_from_ntp(const char* ntp_server, const char* timezone) {
    ESP_LOGI(TAG, "Starting NTP time synchronization...");
    ESP_LOGI(TAG, "NTP Server: %s, Timezone: %s", ntp_server, timezone);
    
    // Set timezone
    setenv("TZ", timezone, 1);
    tzset();
    
    // Configure SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, ntp_server);
    esp_sntp_init();
    
    ESP_LOGI(TAG, "Waiting for NTP time sync (max 10 seconds)...");
    
    // Wait for time to be set (max 10 seconds)
    int retry = 0;
    const int retry_count = 20;
    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    if (retry >= retry_count) {
        ESP_LOGE(TAG, "NTP sync timeout!");
        esp_sntp_stop();
        return ESP_ERR_TIMEOUT;
    }
    
    // Get current time from NTP
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    ESP_LOGI(TAG, "NTP time received: %04d-%02d-%02d %02d:%02d:%02d",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    
    // Set DS3231 with NTP time
    esp_err_t ret = set_datetime(&timeinfo);
    
    // Stop SNTP to save power
    esp_sntp_stop();
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DS3231 synchronized with NTP successfully!");
    } else {
        ESP_LOGE(TAG, "Failed to sync DS3231 with NTP");
    }
    
    return ret;
}
