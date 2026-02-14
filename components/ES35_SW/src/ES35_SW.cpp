#include "ES35_SW.hpp"
#include <esp_log.h>

ES35_SW::ES35_SW(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud)
    : ModbusSensorBase(uart_port, tx, rx, addr, baud, "ES35_SW") {
}

esp_err_t ES35_SW::read_ES35_SW(float& Temp, float& RH) {
    uint8_t resp[9]; // [addr][func][count][temp_hi][temp_lo][rh_hi][rh_lo][crc_lo][crc_hi]
    
    ESP_LOGI(TAG, "Reading ES35_SW from addr 0x%02X, reg 0x%04X, count %u", 
             slave_addr, SW_REG_ADDR, SW_REG_COUNT);
    
    esp_err_t err = modbusReadHolding(SW_REG_ADDR, SW_REG_COUNT, resp, sizeof(resp));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "modbusReadHolding failed: %s", esp_err_to_name(err));
        stats.error_count++;
        return err;
    }
    
    // Log raw response
    ESP_LOGI(TAG, "Raw response: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             resp[0], resp[1], resp[2], resp[3], resp[4], 
             resp[5], resp[6], resp[7], resp[8]);
    
    // Check response header
    // resp[2] should be 0x04 (4 bytes = 2 registers × 2 bytes)
    if (resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 0x04) {
        ESP_LOGE(TAG, "Invalid response header: addr=0x%02X (expect 0x%02X), func=0x%02X (expect 0x03), count=0x%02X (expect 0x04)", 
                 resp[0], slave_addr, resp[1], resp[2]);
        stats.error_count++;
        return ESP_FAIL;
    }
    
    // Parse 2 register values (theo ES35_SW datasheet: reg0=Temp, reg1=Humidity)
    uint16_t temp_raw = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];  // Register 0
    uint16_t rh_raw = (static_cast<uint16_t>(resp[5]) << 8) | resp[6];    // Register 1
    
    Temp = temp_raw / 10.0f;
    RH = rh_raw / 10.0f;
    
    // Update statistics with average
    stats.update((Temp + RH) / 2.0f);
    
    ESP_LOGD(TAG, "Temp=%.1f°C RH=%.1f%%", Temp, RH);
    return ESP_OK;
}