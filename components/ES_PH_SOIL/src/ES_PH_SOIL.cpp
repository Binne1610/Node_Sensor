#include <ES_PH_SOIL.hpp>
#include <esp_log.h>

const char *ES_PH_SOIL::TAG = "ES_PH_SOIL";

ES_PH_SOIL::ES_PH_SOIL(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, uint8_t addr, uint32_t baud) {
    this->uart_num = uart_port;
    this->tx_pin = tx;
    this->rx_pin = rx;
    this->slave_addr = addr;
    this->baud_rate = baud;
}

esp_err_t ES_PH_SOIL::init() {
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

    ESP_LOGI(TAG, "UART initialized on port %d, TX pin %d, RX pin %d", uart_num, tx_pin, rx_pin);
    return ESP_OK;
}

uint16_t ES_PH_SOIL::crc16(const uint8_t *buf, uint16_t len) {
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

esp_err_t ES_PH_SOIL::sendCommand(const std::vector<uint8_t> &cmd){
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

esp_err_t ES_PH_SOIL::readResponse(std::vector<uint8_t> &resp, std::size_t len) {
    resp.resize(len);
    int read = uart_read_bytes(uart_num, resp.data(), len, pdMS_TO_TICKS(500));
    if (read < 0 || static_cast<std::size_t>(read) != len) {
        ESP_LOGE(TAG, "UART read failed (%d/%u bytes)", read, static_cast<unsigned>(len));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ES_PH_SOIL::readPH(float& out) {
    // Modbus RTU: Read Holding Registers, 1 register tại 0x0000
    std::vector<uint8_t> cmd = {
        slave_addr,
        0x03,           // Function code: Read Holding Registers
        0x00, 0x00,     // Starting address
        0x00, 0x01      // Number of registers
    };
    // Gắn CRC (little-endian: low byte trước)
    uint16_t c = crc16(cmd.data(), static_cast<uint16_t>(cmd.size()));
    cmd.push_back(static_cast<uint8_t>(c & 0xFF));
    cmd.push_back(static_cast<uint8_t>((c >> 8) & 0xFF));

    if (sendCommand(cmd) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Modbus command");
        return ESP_FAIL;
    }

    // Phản hồi mong đợi: 7 byte (addr, func, bytecount=2, dataH, dataL, crcL, crcH)
    std::vector<uint8_t> resp;
    if (readResponse(resp, 7) != ESP_OK) {
        ESP_LOGE(TAG, "No response or read error from sensor");
        return ESP_FAIL;
    }

    // Kiểm tra cơ bản
    if (resp.size() != 7 || resp[0] != slave_addr || resp[1] != 0x03 || resp[2] != 0x02) {
        ESP_LOGE(TAG, "Invalid response header");
        return ESP_FAIL;
    }

    // Kiểm CRC
    uint16_t calc = crc16(resp.data(), 5); // tính trên 5 byte đầu
    uint16_t rx_crc = static_cast<uint16_t>(resp[5]) | (static_cast<uint16_t>(resp[6]) << 8);
    if (calc != rx_crc) {
        ESP_LOGE(TAG, "Invalid CRC in response");
        return ESP_FAIL;
    }

    // Ghép dữ liệu 16-bit và scale ra pH
    uint16_t ph_raw = static_cast<uint16_t>(resp[3] << 8) | static_cast<uint16_t>(resp[4]);
    out = static_cast<float>(ph_raw) / 100.0f; 
    return ESP_OK;
}