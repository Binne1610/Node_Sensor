#pragma once
#include "ModbusSensorBase.hpp"

class NPK : public ModbusSensorBase {
private:
    static const uint16_t NPK_REG_ADDR = 0x001E;
    static const uint16_t NPK_REG_COUNT = 3;

public:
    NPK(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr = 0x01, uint32_t baud = 9600);
    esp_err_t read_NPK(float& N, float& P, float& K);
};