#include "ES_SM_THEC.hpp"
#include <esp_log.h>

ES_SM_THEC::ES_SM_THEC(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud)
    : ModbusSensorBase(uart_port, tx, rx, addr, baud, "ES_SM_THEC") {
}

esp_err_t ES_SM_THEC::read_THEC(float& RH, float& Temperature, float& EC) {
    uint8_t resp[11]; // [addr][func][count][data1_hi][data1_lo][data2_hi][data2_lo][data3_hi][data3_lo][crc_lo][crc_hi]
    
    esp_err_t err = modbusReadHolding(THEC_REG_ADDR, THEC_REG_COUNT, resp, sizeof(resp));
    if (err != ESP_OK) {
        stats.error_count++;
        return err;
    }
    
    // Check response header
    if (resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 0x06) {
        ESP_LOGE(TAG, "Invalid response header");
        stats.error_count++;
        return ESP_FAIL;
    }
    
    // Parse 3 register values
    uint16_t RH_raw = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
    uint16_t Temp_raw = (static_cast<uint16_t>(resp[5]) << 8) | resp[6];
    uint16_t EC_raw = (static_cast<uint16_t>(resp[7]) << 8) | resp[8];
    
    RH = RH_raw / 10.0f;
    Temperature = Temp_raw / 10.0f;
    EC = EC_raw / 100.0f;
    
    // Update statistics with average of 3 values
    stats.update((RH + Temperature + EC) / 3.0f);
    
    ESP_LOGD(TAG, "RH=%.1f%% Temp=%.1f°C EC=%.2f", RH, Temperature, EC);
    return ESP_OK;
}
