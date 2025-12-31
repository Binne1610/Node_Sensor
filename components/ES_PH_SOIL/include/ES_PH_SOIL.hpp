#pragma once
#include "ModbusSensorBase.hpp"

class ES_PH_SOIL : public ModbusSensorBase {
private:
    static const uint16_t PH_REG_ADDR = 0x0000;
    static const uint16_t PH_REG_COUNT = 1;

public:
    ES_PH_SOIL(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr = 0x03, uint32_t baud = 9600);
    esp_err_t read_PH(float& pH);
};