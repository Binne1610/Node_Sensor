#pragma once
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <cstddef>
#include <cstdint>

// Sensor statistics structure
struct SensorStats {
    uint32_t read_count = 0;
    uint32_t error_count = 0;
    uint32_t timeout_count = 0;
    uint32_t crc_error_count = 0;
    uint32_t last_read_time_ms = 0;
    float min_value = 999999.0f;
    float max_value = -999999.0f;
    float total_value = 0.0f;
    
    float get_avg() const {
        return (read_count > 0) ? (total_value / read_count) : 0.0f;
    }
    
    void update(float value) {
        if (value < min_value) min_value = value;
        if (value > max_value) max_value = value;
        total_value += value;
        read_count++;
        last_read_time_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    }
};

class ModbusSensorBase {
protected:
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint8_t slave_addr;
    uint32_t baud_rate;
    const char* TAG;
    
    SensorStats stats;
    
    // CRC16 Modbus calculation
    static uint16_t crc16(const uint8_t *buf, uint16_t len);
    
    // Calculate inter-character gap (3.5 chars)
    static inline uint32_t interchar_gap_ms(uint32_t baud);
    
    // Send Modbus command
    esp_err_t sendCommand(const uint8_t* cmd, size_t len);
    
    // Read exact number of bytes with timeout
    esp_err_t readResponse(uint8_t* resp, size_t len, uint32_t timeout_ms = 500);
    
    // Read Modbus Holding Registers (Function Code 0x03)
    esp_err_t modbusReadHolding(uint16_t reg_addr, uint16_t reg_count, uint8_t* response, size_t resp_len);
    
    // Read Modbus Input Registers (Function Code 0x04)
    esp_err_t modbusReadInput(uint16_t reg_addr, uint16_t reg_count, uint8_t* response, size_t resp_len);
    
    // Retry wrapper
    esp_err_t executeWithRetry(esp_err_t (*func)(void*), void* param, uint8_t max_retries = 3);

public:
    ModbusSensorBase(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, 
                     uint8_t addr, uint32_t baud, const char* tag);
    
    virtual ~ModbusSensorBase() = default;
    
    virtual esp_err_t init();
    
    // Get statistics
    const SensorStats& getStats() const { return stats; }
    void resetStats() { stats = SensorStats(); }
    void printStats() const;
};
