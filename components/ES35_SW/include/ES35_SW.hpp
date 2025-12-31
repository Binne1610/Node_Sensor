#pragma once
#include "ModbusSensorBase.hpp"

class ES35_SW : public ModbusSensorBase {
private:
    static const uint16_t SW_REG_ADDR = 0x0000;
    static const uint16_t SW_REG_COUNT = 2;

public:
    ES35_SW(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr = 0x04, uint32_t baud = 9600);
    esp_err_t read_ES35_SW(float& Temp, float& RH);
};