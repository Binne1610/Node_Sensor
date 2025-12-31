#include "ES_PH_SOIL.hpp"
#include <esp_log.h>

ES_PH_SOIL::ES_PH_SOIL(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud)
    : ModbusSensorBase(uart_port, tx, rx, addr, baud, "ES_PH_SOIL") {
}

esp_err_t ES_PH_SOIL::read_PH(float& pH) {
    uint8_t resp[7]; // [addr][func][count][data_hi][data_lo][crc_lo][crc_hi]
    
    esp_err_t err = modbusReadHolding(PH_REG_ADDR, PH_REG_COUNT, resp, sizeof(resp));
    if (err != ESP_OK) {
        stats.error_count++;
        return err;
    }
    
    // Check response header
    if (resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 0x02) {
        ESP_LOGE(TAG, "Invalid response header");
        stats.error_count++;
        return ESP_FAIL;
    }
    
    // Parse pH value (register value / 10.0)
    uint16_t pH_raw = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
    pH = pH_raw / 10.0f;
    
    // Update statistics
    stats.update(pH);
    
    ESP_LOGD(TAG, "pH=%.2f (raw=%u)", pH, pH_raw);
    return ESP_OK;
}