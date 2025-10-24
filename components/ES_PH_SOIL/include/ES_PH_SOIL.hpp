#pragma once
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_err.h>
#include <vector>
#include <cstddef>

class ES_PH_SOIL {
    private:
        uart_port_t uart_num;
        gpio_num_t tx_pin;
        gpio_num_t rx_pin;
        uint8_t slave_addr;
        uint32_t baud_rate;
        static const char *TAG;
        
        uint16_t crc16(const uint8_t *buf, uint16_t len);
        esp_err_t sendCommand(const std::vector<uint8_t> &cmd);
        esp_err_t readResponse(std::vector<uint8_t> &resp, size_t len);

    public:
        ES_PH_SOIL(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr = 0x01, uint32_t baud = 4800);
        esp_err_t init();
        float readPH();
};