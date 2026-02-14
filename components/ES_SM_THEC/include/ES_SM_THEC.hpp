#pragma once
#include "ModbusSensorBase.hpp"

class ES_SM_THEC : public ModbusSensorBase {
private:
    static const uint16_t THEC_REG_ADDR = 0x0000;
    static const uint16_t THEC_REG_COUNT = 3;

public:
    ES_SM_THEC(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr = 0x02, uint32_t baud = 9600);
    esp_err_t read_THEC(float& Temperature, float& RH, float& EC);
};