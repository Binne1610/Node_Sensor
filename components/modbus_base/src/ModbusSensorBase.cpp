#include "ModbusSensorBase.hpp"
#include "uart_shared.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>
#include <algorithm>

ModbusSensorBase::ModbusSensorBase(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, 
                                   uint8_t addr, uint32_t baud, const char* tag)
    : uart_num(uart_port), tx_pin(tx), rx_pin(rx), slave_addr(addr), baud_rate(baud), TAG(tag) {
    stats = SensorStats(); // Initialize stats
}

esp_err_t ModbusSensorBase::init() {
    // Only configure and install UART driver if not already installed
    if (!g_uart_installed[uart_num]) {
        uart_config_t uart_config;
        uart_config.baud_rate = static_cast<int>(baud_rate);
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.rx_flow_ctrl_thresh = 122;
        uart_config.source_clk = UART_SCLK_DEFAULT;

        ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(uart_num, 2048, 2048, 0, NULL, 0));
        
        g_uart_installed[uart_num] = true;
        ESP_LOGI(TAG, "UART initialized on port %d, TX=%d, RX=%d", uart_num, tx_pin, rx_pin);
    } else {
        ESP_LOGI(TAG, "UART port %d already initialized", uart_num);
    }
    
    return ESP_OK;
}

uint16_t ModbusSensorBase::crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint32_t ModbusSensorBase::interchar_gap_ms(uint32_t baud) {
    uint32_t min_baud = (baud < 1200) ? 1200 : baud;
    float tchar_ms = 1000.0f * 11.0f / static_cast<float>(min_baud);
    float gap = tchar_ms * 3.5f;
    float min_gap = (gap < 2.0f) ? 2.0f : gap;
    return static_cast<uint32_t>(ceilf(min_gap));
}

esp_err_t ModbusSensorBase::sendCommand(const uint8_t* cmd, size_t len) {
    uart_flush_input(uart_num);
    
    int written = uart_write_bytes(uart_num, reinterpret_cast<const char*>(cmd), len);
    if (written < 0 || static_cast<size_t>(written) != len) {
        ESP_LOGE(TAG, "UART write failed (%d/%u bytes)", written, static_cast<unsigned>(len));
        stats.error_count++;
        return ESP_FAIL;
    }
    
    uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t ModbusSensorBase::readResponse(uint8_t* resp, size_t len, uint32_t timeout_ms) {
    uint32_t deadline_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL) + timeout_ms;
    size_t byte_count = 0;
    
    while (byte_count < len) {
        uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
        if (now_ms >= deadline_ms) {
            ESP_LOGW(TAG, "Read timeout (%u/%u bytes)", static_cast<unsigned>(byte_count), static_cast<unsigned>(len));
            stats.timeout_count++;
            return ESP_ERR_TIMEOUT;
        }
        
        uint32_t remain_ms = deadline_ms - now_ms;
        uint32_t slice_ms = (remain_ms < 50u) ? remain_ms : 50u;
        
        int r = uart_read_bytes(uart_num, resp + byte_count, len - byte_count, pdMS_TO_TICKS(slice_ms));
        if (r > 0) byte_count += static_cast<size_t>(r);
    }
    
    return ESP_OK;
}

esp_err_t ModbusSensorBase::modbusReadHolding(uint16_t reg_addr, uint16_t reg_count, 
                                               uint8_t* response, size_t resp_len) {
    // Build command: [addr][0x03][reg_high][reg_low][count_high][count_low][crc_low][crc_high]
    uint8_t cmd[8];
    cmd[0] = slave_addr;
    cmd[1] = 0x03; // Function code: Read Holding Registers
    cmd[2] = static_cast<uint8_t>(reg_addr >> 8);
    cmd[3] = static_cast<uint8_t>(reg_addr & 0xFF);
    cmd[4] = static_cast<uint8_t>(reg_count >> 8);
    cmd[5] = static_cast<uint8_t>(reg_count & 0xFF);
    
    uint16_t crc = crc16(cmd, 6);
    cmd[6] = static_cast<uint8_t>(crc & 0xFF);
    cmd[7] = static_cast<uint8_t>(crc >> 8);
    
    // Send command
    esp_err_t err = sendCommand(cmd, 8);
    if (err != ESP_OK) return err;
    
    // Wait inter-frame gap
    vTaskDelay(pdMS_TO_TICKS(interchar_gap_ms(baud_rate)));
    
    // Read response
    err = readResponse(response, resp_len);
    if (err != ESP_OK) return err;
    
    // Verify CRC
    uint16_t recv_crc = static_cast<uint16_t>(response[resp_len - 2] | (response[resp_len - 1] << 8));
    uint16_t calc_crc = crc16(response, resp_len - 2);
    
    if (recv_crc != calc_crc) {
        ESP_LOGE(TAG, "CRC error: recv=0x%04X calc=0x%04X", recv_crc, calc_crc);
        stats.crc_error_count++;
        return ESP_ERR_INVALID_CRC;
    }
    
    return ESP_OK;
}

esp_err_t ModbusSensorBase::executeWithRetry(esp_err_t (*func)(void*), void* param, uint8_t max_retries) {
    for (uint8_t i = 0; i < max_retries; i++) {
        esp_err_t err = func(param);
        if (err == ESP_OK) return ESP_OK;
        
        if (i < max_retries - 1) {
            ESP_LOGW(TAG, "Retry %d/%d after error", i + 1, max_retries);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    ESP_LOGE(TAG, "Failed after %d retries", max_retries);
    stats.error_count++;
    return ESP_FAIL;
}

void ModbusSensorBase::printStats() const {
    ESP_LOGI(TAG, "Stats: reads=%lu errors=%lu timeouts=%lu crc_err=%lu avg=%.2f min=%.2f max=%.2f",
             stats.read_count, stats.error_count, stats.timeout_count, stats.crc_error_count,
             stats.get_avg(), stats.min_value, stats.max_value);
}
