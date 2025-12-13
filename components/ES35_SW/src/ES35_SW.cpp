#include <ES35_SW.hpp>
#include <esp_log.h>
#include "uart_shared.hpp"
#include <algorithm>
#include <esp_timer.h>
#include <math.h>
#include "esp_check.h"  

const char *ES35_SW::TAG = "ES35_SW";

ES35_SW::ES35_SW(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud) {
    this->uart_num = uart_port;
    this->tx_pin = tx;
    this->rx_pin = rx;
    this->slave_addr = addr;
    this->baud_rate = baud;
}

esp_err_t ES35_SW::init() {
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
            // .flow_ctrl = UART_HW_FLOWCTRL_RTS_CTS // Nếu sử dụng thì phải nối thêm chân RTS và CTS
                                                     // và phải khai báo trong thêm trong hàm uart_set_pin()
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

uint16_t ES35_SW::crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF; // Giá trị bắt đầu mặc định của Modbus CRC16
    for (uint16_t i = 0; i < len; ++i) { // duyệt từng byte trong mảng dữ liệu
        crc ^= buf[i]; // XOR byte dữ liệu vào CRC
        for (int j = 0; j < 8; ++j) { // duyệt từng bit trong byte
            if (crc & 1) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc; 
}

// Hàm tính khoảng lặng giữa các frame Modbus RTU
// Tính khoảng lặng 3.5 ký tự theo baud (≈ 11 bits/char)
static inline uint32_t interchar_gap_ms(uint32_t baud){
    float tchar_ms = 1000.0f * 11.0f / (float)std::max(baud, (uint32_t)1200);
    float gap = tchar_ms * 3.5f;
    return (uint32_t)ceilf(std::max(2.0f, gap)); // tối thiểu 2 ms
}

// Đọc chính xác len byte trước deadline_ms (tính từ bây giờ)
static esp_err_t uart_read_exact(uart_port_t uart, uint8_t* buf, size_t len, uint32_t deadline_ms){
    size_t byte_count = 0;
    while (byte_count < len){
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if (now_ms >= deadline_ms) break;
        uint32_t remain_ms = deadline_ms - now_ms;
        uint32_t slice_ms = (remain_ms < 50u) ? remain_ms : 50u; // tránh std::min lỗi suy luận
        int r = uart_read_bytes(uart, buf + byte_count, len - byte_count, pdMS_TO_TICKS(slice_ms));
        if (r > 0) byte_count += (size_t)r;
    }
    return (byte_count == len) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// Lệnh 2 tầng truyền và nhận
esp_err_t ES35_SW::sendCommand(const std::vector<uint8_t> &cmd){
    // Xóa RX ring buffer trước khi gửi lệnh mới
    (void)uart_flush_input(uart_num);
    int written = uart_write_bytes(uart_num, reinterpret_cast<const char*>(cmd.data()), cmd.size());
    if (written < 0 || (size_t)(written) != cmd.size()) {
        ESP_LOGE(TAG, "UART write failed (%d/%u bytes)", written, (unsigned)(cmd.size()));
        return ESP_FAIL;
    }
    (void)uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t ES35_SW::readResponse(std::vector<uint8_t> &resp, std::size_t len) {
    resp.resize(len);
    uint32_t deadline_ms = (uint32_t)(esp_timer_get_time()/1000ULL) + 500; // 500ms tổng thời gian chờ
    int read = uart_read_exact(uart_num, resp.data(), len, deadline_ms);
    if (read != ESP_OK) {
        int actually = 0;
        (void)uart_get_buffered_data_len(uart_num, (size_t*)&actually);
        ESP_LOGE(TAG, "UART read timeout (%d/%u bytes)", actually, (unsigned)len);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static const uint16_t START_REG = 0x00;  // bắt đầu từ thanh ghi 0x0000
static const uint16_t REG_QTY   = 2;     // đọc 2 thanh ghi (RH, Temp)

// Helper: gửi và đọc frame nhiều thanh ghi liên tiếp
esp_err_t ES35_SW::read_ES35_SW(float &RH, float &Temp) {
    // Cú pháp truy vấn theo khung Modbus RTU: [ADDR] [FUNC] [REG_HI] [REG_LO] [LEN_HI] [LEN_LO] [CRC_LOW] [CRC_HIGH]
    uint8_t req[8] = {
        slave_addr,
        0x03,
        (uint8_t)(START_REG >> 8), (uint8_t)(START_REG & 0xFF),
        (uint8_t)(REG_QTY >> 8),    (uint8_t)(REG_QTY & 0xFF),
        0x00, 0x00 // placeholder cho CRC
    };
    uint16_t c = crc16(req, 6);
    req[6] = (uint8_t)(c & 0xFF);
    req[7] = (uint8_t)(c >> 8);

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
    uint8_t resp[9]; // addr func byteCount data(4) crc(2) = 9
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

    if (resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 0x04) {
        ESP_LOGE(TAG, "Header invalid (%02X %02X %02X)", resp[0], resp[1], resp[2]);
        return ESP_FAIL;
    }

    uint16_t calc = crc16(resp, 7);
    uint16_t rx_crc = (uint16_t)resp[7] | ((uint16_t)resp[8] << 8);
    if (calc != rx_crc) {
        ESP_LOGE(TAG, "CRC mismatch calc=%04X recv=%04X", calc, rx_crc);
        return ESP_FAIL;
    }

    uint16_t temp_raw = ((uint16_t)resp[3] << 8) | resp[4];
    uint16_t rh_raw   = ((uint16_t)resp[5] << 8) | resp[6];
    Temp = temp_raw / 10.0f;
    RH   = rh_raw   / 10.0f;
    
    ESP_LOGD(TAG, "Temp_raw=%u RH_raw=%u Temp=%.1f RH=%.1f", temp_raw, rh_raw, Temp, RH);
    return ESP_OK;
}