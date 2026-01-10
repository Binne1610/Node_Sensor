#include "ES35_SW.hpp"
#include <esp_log.h>

ES35_SW::ES35_SW(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud)
    : ModbusSensorBase(uart_port, tx, rx, addr, baud, "ES35_SW") {
}

esp_err_t ES35_SW::read_ES35_SW(float& Temp, float& RH) {
    uint8_t resp[9]; // [addr][func][count][temp_hi][temp_lo][rh_hi][rh_lo][crc_lo][crc_hi]
    
    esp_err_t err = modbusReadHolding(SW_REG_ADDR, SW_REG_COUNT, resp, sizeof(resp));
    if (err != ESP_OK) {
        stats.error_count++;
        return err;
    }
    
    // Check response header
    if (resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 0x04) {
        ESP_LOGE(TAG, "Invalid response header");
        stats.error_count++;
        return ESP_FAIL;
    }
    
    // Parse 2 register values
    uint16_t rh_raw = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
    uint16_t temp_raw = (static_cast<uint16_t>(resp[5]) << 8) | resp[6];
    
    Temp = temp_raw / 10.0f;
    RH = rh_raw / 10.0f;
    
    // Update statistics with average
    stats.update((Temp + RH) / 2.0f);
    
    ESP_LOGD(TAG, "Temp=%.1f°C RH=%.1f%%", Temp, RH);
    return ESP_OK;
}