#include <ES_SM_THEC.hpp>
#include <esp_log.h>
#include "uart_shared.hpp"
#include <algorithm>
#include <esp_timer.h>
#include <math.h>
#include "esp_check.h" 

const char *ES_SM_THEC::TAG = "ES_SM_THEC";

ES_SM_THEC::ES_SM_THEC(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud) {
    this->uart_num = uart_port;
    this->tx_pin = tx;
    this->rx_pin = rx;
    this->slave_addr = addr;
    this->baud_rate = baud;
}

esp_err_t ES_SM_THEC::init() {
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

uint16_t ES_SM_THEC::crc16(const uint8_t *buf, uint16_t len) {
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
esp_err_t ES_SM_THEC::sendCommand(const std::vector<uint8_t> &cmd){
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

esp_err_t ES_SM_THEC::readResponse(std::vector<uint8_t> &resp, std::size_t len) {
    resp.resize(len);
    int read = uart_read_bytes(uart_num, resp.data(), len, pdMS_TO_TICKS(500));
    if (read < 0 || static_cast<std::size_t>(read) != len) {
        ESP_LOGE(TAG, "UART read failed (%d/%u bytes)", read, static_cast<unsigned>(len));
        return ESP_FAIL;
    }
    return ESP_OK;
}

static const uint16_t START_REG = 0x00;  // bắt đầu từ thanh ghi 0x0000
static const uint16_t REG_QTY   = 3;     // đọc 3 thanh ghi (RH, Temp, EC)

esp_err_t ES_SM_THEC::read_THEC(float& RH, float& Temp, float& EC) {  
    // Cú pháp truy vấn theo khung Modbus RTU: [ADDR] [FUNC] [REG_HI] [REG_LO] [LEN_HI] [LEN_LO] [CRC_LOW] [CRC_HIGH]
    uint8_t req[8] = {
        slave_addr,
        0x03,           // Function code: Read Holding Registers
        (uint8_t)(START_REG >> 8), (uint8_t)(START_REG & 0xFF), // Starting address
        (uint8_t)(REG_QTY >> 8),    (uint8_t)(REG_QTY & 0xFF),  // Number of registers
        0x00, 0x00 // placeholder cho CRC
    };
    // Gắn CRC (little-endian: low byte trước)
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

    // Ghép dữ liệu 16-bit và scale ra giá trị cần đọc
    // RH value
    uint16_t RH_raw = static_cast<uint16_t>(resp[3] << 8) | static_cast<uint16_t>(resp[4]);
    RH = RH_raw / 10.0f;
    
    // Temperature value
    uint16_t Temp_raw = ((uint16_t)resp[5] << 8) | resp[6];
    Temp = Temp_raw / 10.0f;

    // EC value
    uint16_t EC_raw = ((uint16_t)resp[7] << 8) | resp[8];
    EC = EC_raw / 100.0f;

    ESP_LOGD(TAG, "RH_raw=%u Temp_raw=%u EC_raw=%u Temp=%.1f RH=%.1f", Temp_raw, RH_raw, EC_raw, Temp, RH, EC);
    return ESP_OK;
}