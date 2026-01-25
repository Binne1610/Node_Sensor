#pragma once

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <stdint.h>
#include <time.h>

#define DS3231_I2C_ADDR 0x68

// DS3231 Registers
#define DS3231_REG_SECONDS      0x00
#define DS3231_REG_ALARM1_SEC   0x07
#define DS3231_REG_ALARM2_MIN   0x0B
#define DS3231_REG_CONTROL      0x0E
#define DS3231_REG_STATUS       0x0F

// Control Register bits
#define DS3231_CTRL_A1IE        0x01  // Alarm 1 Interrupt Enable
#define DS3231_CTRL_A2IE        0x02  // Alarm 2 Interrupt Enable
#define DS3231_CTRL_INTCN       0x04  // Interrupt Control
#define DS3231_CTRL_RS1         0x08
#define DS3231_CTRL_RS2         0x10
#define DS3231_CTRL_CONV        0x20
#define DS3231_CTRL_BBSQW       0x40
#define DS3231_CTRL_EOSC        0x80

// Status Register bits
#define DS3231_STAT_A1F         0x01  // Alarm 1 Flag
#define DS3231_STAT_A2F         0x02  // Alarm 2 Flag
#define DS3231_STAT_BSY         0x04
#define DS3231_STAT_EN32KHZ     0x08
#define DS3231_STAT_OSF         0x80

class DS3231_RTC {
public:
    DS3231_RTC(gpio_num_t sda_pin, gpio_num_t scl_pin, gpio_num_t int_pin);
    ~DS3231_RTC();
    
    // Khởi tạo I2C và DS3231
    esp_err_t init();
    
    // Đặt alarm để đánh thức ESP32 sau X giây
    esp_err_t set_alarm_interval(uint32_t seconds);
    
    // Xóa alarm flag sau khi wake
    esp_err_t clear_alarm_flag();
    
    // Kiểm tra alarm có trigger không
    bool is_alarm_triggered();
    
    // Đọc nhiệt độ từ DS3231 (bonus feature)
    esp_err_t read_temperature(float& temp);
    
    // Set thời gian cho DS3231
    esp_err_t set_datetime(const struct tm* timeinfo);
    
    // Đọc thời gian từ DS3231
    esp_err_t get_datetime(struct tm* timeinfo);
    
    // Đồng bộ thời gian từ NTP server (cần WiFi)
    esp_err_t sync_from_ntp(const char* ntp_server = "pool.ntp.org", const char* timezone = "UTC-7");
    
    // Lấy GPIO INT pin (để config ESP32 wakeup)
    gpio_num_t get_int_pin() const { return int_pin_; }

private:
    i2c_master_bus_handle_t bus_handle_;
    i2c_master_dev_handle_t dev_handle_;
    gpio_num_t sda_pin_;
    gpio_num_t scl_pin_;
    gpio_num_t int_pin_;  // SQW/INT pin của DS3231
    
    // I2C helper functions
    esp_err_t write_register(uint8_t reg, uint8_t value);
    esp_err_t read_register(uint8_t reg, uint8_t& value);
    esp_err_t read_registers(uint8_t reg, uint8_t* data, size_t len);
    
    // BCD conversion
    uint8_t dec_to_bcd(uint8_t val);
    uint8_t bcd_to_dec(uint8_t val);
};
