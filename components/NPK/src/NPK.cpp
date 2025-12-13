#include "NPK.hpp"
#include <esp_log.h>
#include "uart_shared.hpp"
#include <math.h>
#include <esp_timer.h>
#include "esp_check.h"

const char *NPK::TAG = "NPK";

NPK::NPK(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud) {
    this->uart_num = uart_port;
    this->tx_pin = tx;
    this->rx_pin = rx;
    this->slave_addr = addr;
    this->baud_rate = baud;
}

esp_err_t NPK::init() {
    // Chỉ cấu hình và install UART driver nếu chưa được install
    if (!g_uart_installed[uart_num]) {
        uart_config_t uart_config = {
            .baud_rate = static_cast<int>(baud_rate),
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,
            .flags = 0
        };

        ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(uart_num, 2048, 2048, 0, NULL, 0));
        
        g_uart_installed[uart_num] = true;
        ESP_LOGI(TAG, "UART initialized on port %d, TX pin %d, RX pin %d", uart_num, tx_pin, rx_pin);
    } else {
        ESP_LOGI(TAG, "UART port %d already initialized, using existing driver", uart_num);
    }
    
    return ESP_OK;
}

uint16_t NPK::crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF; // Giá trị bắt đầu mặc định của Modbus CRC16
    for (uint16_t i = 0; i < len; ++i) { // duyệt từng byte trong mảng dữ liệu
        crc ^= buf[i]; // XOR byte dữ liệu vào CRC
        for (int j = 0; j < 8; ++j) { // duyệt từng bit trong byte
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc; 
}

// Hàm tính khoảng lặng giữa các frame Modbus (3.5 ký tự)
static inline uint32_t interchar_gap_ms(uint32_t baud){
    float tchar_ms = 1000.0f * 11.0f / (float) (baud < 1200 ? 1200 : baud);
    float gap = tchar_ms * 3.5f;
    return (uint32_t)ceilf(gap < 2.0f ? 2.0f : gap);
}

// Đọc đủ len byte trước deadline_ms
static esp_err_t uart_read_exact(uart_port_t uart, uint8_t* buf, size_t len, uint32_t deadline_ms){
    size_t got = 0;
    while (got < len){
        uint32_t now_ms = (uint32_t)(esp_timer_get_time()/1000ULL);
        if (now_ms >= deadline_ms) break;
        uint32_t remain_ms = deadline_ms - now_ms;
        uint32_t slice_ms = (remain_ms < 50u) ? remain_ms : 50u;
        int r = uart_read_bytes(uart, buf + got, len - got, pdMS_TO_TICKS(slice_ms));
        if (r > 0) got += (size_t)r;
    }
    return (got == len) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// Lệnh 2 tầng truyền và nhận
esp_err_t NPK::sendCommand(const std::vector<uint8_t> &cmd){
    // Xóa RX ring buffer trước khi gửi lệnh mới
    (void)uart_flush_input(uart_num);
    int written = uart_write_bytes(uart_num, reinterpret_cast<const char*>(cmd.data()), cmd.size());
    if (written < 0 || static_cast<size_t>(written) != cmd.size()) {
        ESP_LOGE(TAG, "UART write failed (%d/%u bytes)", written, static_cast<unsigned>(cmd.size()));
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t NPK::readResponse(std::vector<uint8_t> &resp, std::size_t len) {
    resp.resize(len);
    int read = uart_read_bytes(uart_num, resp.data(), len, pdMS_TO_TICKS(500));
    if (read < 0 || static_cast<std::size_t>(read) != len) {
        ESP_LOGE(TAG, "UART read failed (%d/%u bytes)", read, static_cast<unsigned>(len));
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const uint16_t START_REG = 0x00;  // bắt đầu từ thanh ghi 0x0000
static const uint16_t REG_QTY   = 3;     // đọc 3 thanh ghi (N, P, K)

// Helper: gửi và đọc frame nhiều thanh ghi liên tiếp
esp_err_t NPK::read_NPK(float &N, float &P, float &K) {
    // Đọc 3 thanh ghi liên tiếp từ 0x001E (0x1E,0x1F,0x20)
    uint8_t req[8] = {
        slave_addr,
        0x03,
        0x00, 0x1E, // start address
        0x00, 0x03  // quantity = 3
    };
    uint16_t c = crc16(req, 6);
    req[6] = c & 0xFF;
    req[7] = (c >> 8) & 0xFF;

    // Gửi
    ESP_RETURN_ON_ERROR(uart_flush_input(uart_num), TAG, "flush fail");
    int written = uart_write_bytes(uart_num, (const char*)req, sizeof(req));
    if (written != (int)sizeof(req)) {
        ESP_LOGE(TAG, "Write failed (%d/%u)", written, (unsigned)sizeof(req));
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100)), TAG, "wait tx");

    vTaskDelay(pdMS_TO_TICKS(interchar_gap_ms(baud_rate)));

    // Phản hồi mong đợi: 5 + số thanh ghi x 2 (addr, func, bytecount, dataH, dataL, crcL, crcH)
    uint8_t resp[11]; // addr func byteCount data(3) crc(2) = 11
    uint32_t deadline_ms = (uint32_t)(esp_timer_get_time()/1000ULL) + 300;
    esp_err_t err = uart_read_exact(uart_num, resp, sizeof(resp), deadline_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read timeout");
        return ESP_ERR_TIMEOUT;
    }

    // Exception frame?
    if (resp[1] & 0x80) {
        ESP_LOGE(TAG, "Modbus exception code=0x%02X", resp[2]);
        return ESP_FAIL;
    }

    if (resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 0x06) {
        ESP_LOGE(TAG, "Header invalid (%02X %02X %02X)", resp[0], resp[1], resp[2]);
        return ESP_FAIL;
    }

    uint16_t calc = crc16(resp, 9);
    uint16_t rx_crc = (uint16_t)resp[9] | ((uint16_t)resp[10] << 8);
    if (calc != rx_crc) {
        ESP_LOGE(TAG, "CRC mismatch calc=%04X recv=%04X", calc, rx_crc);
        return ESP_FAIL;
    }

    uint16_t N_Raw = (resp[3] << 8) | resp[4];
    uint16_t P_Raw = (resp[5] << 8) | resp[6];
    uint16_t K_Raw = (resp[7] << 8) | resp[8];

    N = N_Raw / 10.0f;
    P = P_Raw / 10.0f;
    K = K_Raw / 10.0f;

    ESP_LOGD(TAG, "N_raw=%u P_raw=%u K_raw=%u N=%.1f P=%.1f K=%.1f", N_Raw, P_Raw, K_Raw, N, P, K);
    return ESP_OK;
}