#pragma once
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <vector>
#include <cstddef>
#include <cstdint>

class ES35_SW {
    private:
        /// @brief 
        uart_port_t uart_num;
        gpio_num_t tx_pin;
        gpio_num_t rx_pin;
        uint8_t slave_addr;
        uint32_t baud_rate;
        static const char *TAG;
        
        static uint16_t crc16(const uint8_t *buf, uint16_t len);
        esp_err_t sendCommand(const std::vector<uint8_t> &cmd);
        esp_err_t readResponse(std::vector<uint8_t> &resp, std::size_t len);

    public:
        ES35_SW(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr = 0x04, uint32_t baud = 9600);
        esp_err_t init();
        esp_err_t read_ES35_SW(float& Temp, float& RH);
};