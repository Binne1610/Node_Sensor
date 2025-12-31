#include "NPK.hpp"
#include <esp_log.h>

NPK::NPK(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud)
    : ModbusSensorBase(uart_port, tx, rx, addr, baud, "NPK") {
}

esp_err_t NPK::read_NPK(float& N, float& P, float& K) {
    uint8_t resp[11]; // [addr][func][count][N_hi][N_lo][P_hi][P_lo][K_hi][K_lo][crc_lo][crc_hi]
    
    esp_err_t err = modbusReadHolding(NPK_REG_ADDR, NPK_REG_COUNT, resp, sizeof(resp));
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
    uint16_t N_raw = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
    uint16_t P_raw = (static_cast<uint16_t>(resp[5]) << 8) | resp[6];
    uint16_t K_raw = (static_cast<uint16_t>(resp[7]) << 8) | resp[8];
    
    N = N_raw / 10.0f;
    P = P_raw / 10.0f;
    K = K_raw / 10.0f;
    
    // Update statistics with average
    stats.update((N + P + K) / 3.0f);
    
    ESP_LOGD(TAG, "N=%.1f P=%.1f K=%.1f", N, P, K);
    return ESP_OK;
}