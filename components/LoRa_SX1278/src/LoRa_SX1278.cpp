// /*Các bước cơ bản để “nói chuyện” với LoRa chip

// 1. Khởi tạo SPI trong ESP-IDF.

// 2. Reset module LoRa.

// 3. Đọc/ghi thanh ghi (register) của SX1276 qua SPI (ví dụ: RegOpMode, RegFrfMsb, RegPaConfig, v.v.).

// 4. Cấu hình chế độ P2P:

//     + Tần số (frequency)

//     + Băng thông (BW)

//     + Spreading Factor (SF)

//     Coding Rate (CR)
    
//     + Công suất phát (TxPower)

// 5. Chuyển sang chế độ truyền (TX) hoặc nghe (RX).

// 6. Dùng DIO0 interrupt để biết khi nào gửi/nhận xong.

// */

/*1. Sử dụng Mutex: thêm xSemaphoreTake(spi_mutex, portMAX_DELAY) trước và xSemaphoreGive(spi_mutex) sau để tránh race khi nhiều task.
  2. Chunking: kiểm tra driver/hardware max transfer bytes; nếu vượt quá, chia length thành nhiều chunk.
  => Mutex + Chunking: nếu nhiều task cùng dùng LoRa, kết hợp cả 2 => Dữ liệu lớn được chia nhiều block, mỗi block sẽ gửi trong 1 SPI transaction được bảo vệ bởi mutex.

Reuse buffer: nếu gọi thường xuyên, cấp phát một lần global DMA buffer kích thước max và tái sử dụng thay vì malloc/free mỗi lần.

Non-blocking: dùng spi_device_queue_transmit() + spi_device_get_trans_result() để gửi không block (nếu cần throughput cao) — nhưng cần quản lý nhiều transaction.

Checksums / retries: nếu môi trường noisy, thêm checksum hoặc retry logic khi spi_device_transmit() trả lỗi.

Direct FIFO API: nếu chỉ thao tác với FIFO, dùng register pointer (RegFifoAddrPtr) trước khi write/read để chắc chắn vị trí read/write đúng.
*/

#include <LoRa_SX1278.hpp>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <string.h>

static const char *TAG = "LoRa_SX1278";
static spi_device_handle_t spi_dev = NULL; 
static QueueHandle_t dio0_queue = NULL;

// Forward declarations
static esp_err_t LoRa_SX1278_basicInit();
static esp_err_t LoRa_SX1278_checkVersion(uint8_t *version);

// Định nghĩa các biến global được khai báo extern trong .hpp
SemaphoreHandle_t LoRa_spi_mutex = nullptr;
uint8_t *LoRa_TX_buf = nullptr;
uint8_t *LoRa_RX_buf = nullptr;
uint16_t LoRa_buf_size = 0;

// SPI transfer helper
static esp_err_t spi_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Ghi giá trị 0 vào từng byte của cấu trúc spi_transaction_t
    t.length = length * 8; // Length in bits
    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;
    return spi_device_transmit(spi_dev, &t);
}

// Write value to register
// SX1278: bit MSB của địa chỉ dùng để phân biệt ghi hay đọc: 1 = ghi, 0 = đọc
esp_err_t LoRa_SX1278_writeReg(uint8_t reg, uint8_t value) { // reg: địa chỉ thanh ghi, value: giá trị cần ghi
    uint8_t tx_buffer[2] = { (uint8_t)(reg | 0x80), value }; // Tạo buffer truyền 2 byte 
                                                  // byte đầu (byte 0) là địa chỉ thanh ghi với bit MSB = 1 (ghi), byte thứ hai (byte 1) là giá trị cần ghi
    return spi_transfer(tx_buffer, nullptr, sizeof(tx_buffer)); // spi_transfer(tx, rx, length)
}

// Read value from register
esp_err_t LoRa_SX1278_readReg(uint8_t reg, uint8_t *value) {
    if (value == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t tx_buffer[2] = { (uint8_t)(reg & 0x7F), 0x00 }; // Tạo buffer truyền 2 byte
                                                 // byte đầu (byte 0) là địa chỉ thanh ghi với bit MSB = 0 (đọc), byte thứ hai (byte 1) là byte rỗng để nhận giá trị đọc được
    /*  Full-duplex SPI transfer:
        1. Master gửi đi 2 byte trong tx_buffer 
            + Byte thứ 1 gửi đi = 0 (chỉ cần đọc) 
            + Byte thứ 2, master phải gửi 1 dummy byte (byte rác) để Sx1278 có clock mà đẩy dữ liệu ra MISO
        2. Slave (Sx1278) nhận byte đầu tiên (địa chỉ thanh ghi) và gửi trả lại 1 dummy byte (byte rác) trong rx_buffer[0]
        3. Slave (Sx1278) nhận byte thứ hai (dummy byte) và gửi trả lại giá trị thực của thanh ghi trong rx_buffer[1] */
    uint8_t rx_buffer[2];
    memset(rx_buffer, 0, sizeof(rx_buffer));
    esp_err_t ret = spi_transfer(tx_buffer, rx_buffer, sizeof(tx_buffer));
    if (ret == ESP_OK) {
        *value = rx_buffer[1];
    }
    return ret;
}

// // Write buffer to register (Burst write)
// esp_err_t LoRa_SX1278_writeBuffer(uint8_t reg, const uint8_t *buffer, size_t length) {
//     if (buffer == NULL || length =0 ) return ESP_ERR_INVALID_ARG;

//     // Tạo buffer TX gồm byte địa chỉ + dữ liệu
//     /* 1. Cấp phát bộ nhớ có khả năng DMA -> Dùng heap_caps_malloc() vì:
//     - SPI sử dụng DMA để truyền dữ liệu nhanh hơn CPU
//     - Bộ nhớ dùng cho DMA phải được cấp phát từ vùng nhớ (RAM) có khả năng DMA
//        2. 1 byte đầu địa chỉ + length byte payload (length + 1) */
//     uint8_t *tx_buffer = (uint8_t *)heap_caps_malloc(length + 1, MALLOC_CAP_DMA); 

//     if (tx_buffer == NULL) return ESP_ERR_NO_MEM;
    
//     tx_buffer[0] = reg | 0x80; // Địa chỉ thanh ghi FIFO với bit MSB = 1 (ghi)
//     memcpy(&tx_buffer[1], buffer, length); // Sao chép dữ liệu vào buffer TX

//     esp_err_t ret = spi_transfer(tx_buffer, nullptr, length + 1);
//     heap_caps_free(tx_buffer); // Giải phóng bộ nhớ đã cấp phát -> Tránh memory leak
//     return ret;
// }

// Write FIFO - Queue mode + reuse buffer + chunk
esp_err_t LoRa_SX1278_writeBuffer(uint8_t reg, const uint8_t *buffer, size_t length) {
    if (buffer == NULL || length == 0) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(LoRa_spi_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t ret = ESP_OK;
    size_t chunk_max = LoRa_buf_size - 1; // trừ byte địa chỉ
    size_t offset = 0;
    
    while (offset < length) {
        size_t chunk = length - offset;
        if (chunk > chunk_max)
            chunk = chunk_max;

        // Chuẩn bị buffer truyền
        LoRa_TX_buf[0] = (uint8_t)(reg | 0x80); // Địa chỉ thanh ghi với bit MSB = 1 (ghi)
        memcpy(&LoRa_TX_buf[1], &buffer[offset], chunk);

        // Thiết lập giao dịch SPI
        spi_transaction_t t = {};
        memset(&t, 0, sizeof(t));
        t.length = (chunk + 1) * 8; // tính theo bit
        t.tx_buffer = LoRa_TX_buf;
        t.rx_buffer = NULL;

        // Sử dụng spi_device_transmit thay vì queue
        ret = spi_device_transmit(spi_dev, &t);
        if (ret != ESP_OK) {
            break;
        }

        offset += chunk;
    }
    xSemaphoreGive(LoRa_spi_mutex);
    return ret;
}

// // Read buffer from register
// esp_err_t LoRa_SX1278_readBuffer(uint8_t reg, uint8_t *buffer, size_t length)
// {
//     if (buffer == NULL || length == 0) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // TX = 1 byte address + N dummy bytes
//     size_t tx_len = length + 1;
//     uint8_t *tx_data = heap_caps_malloc(tx_len, MALLOC_CAP_DMA); // Cấp phát bộ nhớ ở vùng DMA-capable memory
//     uint8_t *rx_data = heap_caps_malloc(tx_len, MALLOC_CAP_DMA);

//     if (!tx_data || !rx_data) {               // Kiểm tra cấp phát bộ nhớ
//         if (tx_data) heap_caps_free(tx_data); // Giải phóng bộ nhớ đã cấp phát -> Tránh memory leak
//         if (rx_data) heap_caps_free(rx_data); // Giải phóng bộ nhớ đã cấp phát -> Tránh memory leak
//         return ESP_ERR_NO_MEM;
//     }

//     // Byte đầu tiên là địa chỉ cần đọc (MSB = 0 → read)
//     tx_data[0] = reg & 0x7F;

//     // Các byte còn lại là dummy (0x00)
//     memset(&tx_data[1], 0x00, length);

//     spi_transaction_t trans = {
//         .length = tx_len * 8,   // tính theo bit
//         .tx_buffer = tx_data,
//         .rx_buffer = rx_data,
//     };

//     esp_err_t ret = spi_device_transmit(spi_handle, &trans);

//     if (ret == ESP_OK) {
//         // Byte rx_data[0] là garbage → bỏ
//         memcpy(buffer, &rx_data[1], length);
//     }

//     heap_caps_free(tx_data);
//     heap_caps_free(rx_data);

//     return ret;
// }

// Read FIFO - Queue mode + reuse buffer + chunk
esp_err_t LoRa_SX1278_readBuffer(uint8_t reg, uint8_t *buffer, size_t length) {
    if (buffer == NULL || length == 0) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(LoRa_spi_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t ret = ESP_OK;
    size_t chunk_max = LoRa_buf_size - 1; // trừ byte địa chỉ
    size_t offset = 0;

    while (offset < length) {
        size_t chunk = length - offset;
        if (chunk > chunk_max) chunk = chunk_max;

        // Chuẩn bị buffer truyền
        LoRa_TX_buf[0] = (uint8_t)(reg & 0x7F); // Địa chỉ thanh ghi với bit MSB = 0 (đọc)
        memset(&LoRa_TX_buf[1], 0x00, chunk); // Ghi dummy byte vào buffer truyền

        // Thiết lập giao dịch SPI
        spi_transaction_t t = {};
        memset(&t, 0, sizeof(t));
        t.length = (chunk + 1) * 8; // tính theo bit
        t.tx_buffer = LoRa_TX_buf;
        t.rx_buffer = LoRa_RX_buf;

        // Sử dụng spi_device_transmit thay vì queue
        ret = spi_device_transmit(spi_dev, &t);
        if (ret != ESP_OK) {
            break;
        }

        // ✅ SAO CHÉP dữ liệu từ RX buffer vào output buffer
        // Bỏ byte đầu tiên (dummy byte từ slave khi nhận địa chỉ)
        memcpy(&buffer[offset], &LoRa_RX_buf[1], chunk);

        offset += chunk;
    }
    xSemaphoreGive(LoRa_spi_mutex);
    return ret;
}

// Set LoRa mode
esp_err_t LoRa_SX1278_setMode(uint8_t mode) {
    return LoRa_SX1278_writeReg(0x01, mode);
}

// Get current LoRa mode
uint8_t LoRa_SX1278_getMode() {
    uint8_t mode = 0;
    LoRa_SX1278_readReg(0x01, &mode);
    return mode;
}

// Get RSSI value
int LoRa_SX1278_getRSSI() {
    uint8_t rssi = 0;
    LoRa_SX1278_readReg(0x1A, &rssi);
    return -157 + (int)rssi; // Formula from datasheet
}

// DIO0 interrupt handler
void IRAM_ATTR dio0_isr_handler(void* arg) {
    uint32_t dio0_pin = (uint32_t)arg;
    xQueueSendFromISR(dio0_queue, &dio0_pin, NULL); 
}

// Reset LoRa module
esp_err_t LoRa_SX1278_resetHW() {
    ESP_LOGI(TAG, "=== HARDWARE RESET DEBUG ===");
    
    // Test GPIO trước khi reset
    gpio_set_direction(LoRa_SX1278_PIN_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(LoRa_SX1278_PIN_CS, GPIO_MODE_OUTPUT);
    
    // Test CS pin
    gpio_set_level(LoRa_SX1278_PIN_CS, 1); // CS HIGH (không chọn chip)
    ESP_LOGI(TAG, "CS pin set to HIGH");
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Reset sequence
    ESP_LOGI(TAG, "Pulling RESET LOW...");
    gpio_set_level(LoRa_SX1278_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20)); // Tăng thời gian reset
    
    ESP_LOGI(TAG, "Pulling RESET HIGH...");
    gpio_set_level(LoRa_SX1278_PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(20)); // Tăng thời gian ổn định
    
    ESP_LOGI(TAG, "Reset complete, module should be in sleep mode");
    return ESP_OK;
}

// Initialize LoRa SX1278
esp_err_t LoRa_SX1278_init() {
    esp_err_t ret;

    // Cấp phát bộ nhớ cho mutex và buffer (chỉ khi chưa tồn tại)
    if (LoRa_spi_mutex == NULL) {
        LoRa_spi_mutex = xSemaphoreCreateMutex();
        if (LoRa_spi_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create SPI mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    if (LoRa_TX_buf == NULL) {
        LoRa_buf_size = LoRa_MAX_SPI_TRANSFER_SIZE;
        LoRa_TX_buf = (uint8_t*)heap_caps_malloc(LoRa_buf_size, MALLOC_CAP_DMA);
        LoRa_RX_buf = (uint8_t*)heap_caps_malloc(LoRa_buf_size, MALLOC_CAP_DMA);
        
        if (LoRa_TX_buf == NULL || LoRa_RX_buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate DMA buffers");
            return ESP_ERR_NO_MEM;
        }
    }

    // SPI bus configuration
    spi_bus_config_t buscfg = {};
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = LoRa_SX1278_PIN_MOSI;
    buscfg.miso_io_num = LoRa_SX1278_PIN_MISO;
    buscfg.sclk_io_num = LoRa_SX1278_PIN_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;
    
    // Kiểm tra nếu SPI bus đã được khởi tạo
    ret = spi_bus_initialize(LoRa_SX1278_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        // SPI bus đã tồn tại, bỏ qua lỗi này
        ESP_LOGW(TAG, "SPI bus already initialized, reusing existing bus");
        ret = ESP_OK;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // SPI device configuartion
    spi_device_interface_config_t devcfg = {};
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = LoRa_SX1278_SPI_CLOCK_HZ;
    devcfg.mode = 0;
    devcfg.spics_io_num = LoRa_SX1278_PIN_CS;
    devcfg.queue_size = 3;
    
    // Chỉ add device nếu chưa tồn tại
    if (spi_dev == NULL) {
        ret = spi_bus_add_device(LoRa_SX1278_SPI_HOST, &devcfg, &spi_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGW(TAG, "SPI device already exists, reusing");
    }

    // init GPIO
    gpio_config_t rst_pin_cfg = {};
    memset(&rst_pin_cfg, 0, sizeof(rst_pin_cfg));
    rst_pin_cfg.pin_bit_mask = (1ULL << LoRa_SX1278_PIN_RESET);
    rst_pin_cfg.mode = GPIO_MODE_OUTPUT;
    rst_pin_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    rst_pin_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst_pin_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&rst_pin_cfg);

    gpio_config_t dio0_pin_cfg = {};
    memset(&dio0_pin_cfg, 0, sizeof(dio0_pin_cfg));
    dio0_pin_cfg.pin_bit_mask = (1ULL << LoRa_SX1278_PIN_DIO0);
    dio0_pin_cfg.mode = GPIO_MODE_INPUT;
    dio0_pin_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    dio0_pin_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dio0_pin_cfg.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&dio0_pin_cfg);

    // Reset pin should be output, DIO0 pin should be input
    gpio_set_direction(LoRa_SX1278_PIN_RESET, GPIO_MODE_OUTPUT);
    gpio_set_direction(LoRa_SX1278_PIN_DIO0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LoRa_SX1278_PIN_DIO0, GPIO_PULLUP_ONLY);

    // reset chip
    LoRa_SX1278_resetHW();

    // create queue for DIO0 interrupt (chỉ khi chưa tồn tại)
    if (dio0_queue == NULL) {
        dio0_queue = xQueueCreate(10, sizeof(uint32_t)); // Tạo hàng đợi để chứa tối đa 10 mục, mỗi mục là một giá trị uint32_t
                                                         // ISR sẽ gửi (xQueueSendFromISR) vào queue này khi DIO0 ngắt xảy ra
        if (dio0_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create DIO0 queue");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // install GPIO ISR service (có thể đã được cài đặt)
    ret = gpio_install_isr_service(0); // 0 = default interrupt allocation (không dùng flag đặc biệt)
    if (ret == ESP_ERR_INVALID_STATE) {
        // ISR service đã tồn tại, bỏ qua
        ESP_LOGW(TAG, "GPIO ISR service already installed");
        ret = ESP_OK;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // attach DIO0 interrupt handler
    ret = gpio_isr_handler_add(LoRa_SX1278_PIN_DIO0, dio0_isr_handler, (void*)LoRa_SX1278_PIN_DIO0);
    if (ret == ESP_ERR_INVALID_STATE) {
        // Handler đã tồn tại, xóa rồi thêm lại
        ESP_LOGW(TAG, "DIO0 ISR handler already exists, removing and re-adding");
        gpio_isr_handler_remove(LoRa_SX1278_PIN_DIO0);
        ret = gpio_isr_handler_add(LoRa_SX1278_PIN_DIO0, dio0_isr_handler, (void*)LoRa_SX1278_PIN_DIO0);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DIO0 ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "LoRa SX1278 hardware initialized successfully");
    
    // Configure LoRa modem settings
    ret = LoRa_SX1278_basicInit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa basic init failed");
        return ret;
    }
    
    return ESP_OK;
}

// check version register
esp_err_t LoRa_SX1278_checkVersion(uint8_t *version) {
    // Debug: Đọc nhiều thanh ghi để kiểm tra SPI
    uint8_t test_regs[5];
    ESP_LOGI(TAG, "=== DEBUG: Testing SPI communication ===");
    
    // Test write/read
    ESP_LOGI(TAG, "Step 1: Test writing to RegOpMode...");
    esp_err_t ret = LoRa_SX1278_writeReg(0x01, 0x00); // Sleep mode
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write RegOpMode: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Step 2: Reading back registers...");
    LoRa_SX1278_readReg(0x01, &test_regs[0]); // RegOpMode
    ESP_LOGI(TAG, "RegOpMode (0x01): 0x%02X (expect: 0x00 or 0x08)", test_regs[0]);
    
    LoRa_SX1278_readReg(0x06, &test_regs[1]); // RegFrfMsb
    ESP_LOGI(TAG, "RegFrfMsb (0x06): 0x%02X", test_regs[1]);
    
    LoRa_SX1278_readReg(0x39, &test_regs[2]); // RegSyncWord
    ESP_LOGI(TAG, "RegSyncWord (0x39): 0x%02X", test_regs[2]);
    
    ret = LoRa_SX1278_readReg(0x42, version); // RegVersion address = 0x42
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read version register: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "LoRa SX1278 version (0x42): 0x%02X (expect: 0x12)", *version);
    
    // Kiểm tra pattern 0xFF (MISO floating - không kết nối)
    if (*version == 0xFF && test_regs[0] == 0xFF && test_regs[1] == 0xFF && test_regs[2] == 0xFF) {
        ESP_LOGE(TAG, "=== ALL REGISTERS = 0xFF ===");
        ESP_LOGE(TAG, "This means MISO line is floating (not connected or module not powered)!");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "CHECK:");
        ESP_LOGE(TAG, "  1. Module power: 3.3V connected?");
        ESP_LOGE(TAG, "  2. MISO wire: GPIO13 connected to module MISO?");
        ESP_LOGE(TAG, "  3. MOSI wire: GPIO11 connected to module MOSI?");
        ESP_LOGE(TAG, "  4. SCK wire:  GPIO12 connected to module SCK?");
        ESP_LOGE(TAG, "  5. CS wire:   GPIO10 connected to module NSS/CS?");
        ESP_LOGE(TAG, "  6. RESET wire: GPIO14 connected to module RST?");
        ESP_LOGE(TAG, "  7. Module type: Is this really SX1278/SX1276?");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Using multimeter:");
        ESP_LOGE(TAG, "  - Measure 3.3V on module VCC pin");
        ESP_LOGE(TAG, "  - Check continuity: ESP32 GPIO <-> Module pin");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Kiểm tra tất cả thanh ghi có phải 0x00 không
    if (*version == 0x00 && test_regs[0] == 0x00 && test_regs[1] == 0x00) {
        ESP_LOGE(TAG, "All registers read 0x00 -> Module in RESET or SPI problem");
        ESP_LOGE(TAG, "Check: 1) Wiring (MOSI/MISO/SCK/CS)");
        ESP_LOGE(TAG, "       2) Module power (3.3V)");
        ESP_LOGE(TAG, "       3) Reset pin connection");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    if (*version != 0x12) { // Expected version for SX1278 is 0x12
        ESP_LOGW(TAG, "Unexpected LoRa version: 0x%02X (expected 0x12)", *version);
        ESP_LOGW(TAG, "Module may be SX1276 (0x12) or SX1262 (0x24) or wrong chip");
        // Không return lỗi, cho phép tiếp tục để debug
    } else {
        ESP_LOGI(TAG, "LoRa SX1278 detected successfully!");
    }
    
    return ESP_OK;
}

// basic LoRa init: set LoRa mode, set frequency, configure modem settings, power...
esp_err_t LoRa_SX1278_basicInit() {
    esp_err_t ret;
    uint8_t version;

    // Check version
    ret = LoRa_SX1278_checkVersion(&version);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set LoRa mode
    ret = LoRa_SX1278_writeReg(0x01, 0x80); // RegOpMode: Long Range Mode + Sleep
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LoRa mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set frequency 433.175 MHz
    uint64_t frf = ((uint64_t)433175000 << 19) / 32000000;
    LoRa_SX1278_writeReg(0x06, (uint8_t)(frf>>16));  // RegFrfMsb
    LoRa_SX1278_writeReg(0x07, (uint8_t)(frf>>8));   // RegFrfMid
    LoRa_SX1278_writeReg(0x08, (uint8_t)(frf>>0));   // RegFrfLsb

    // Set SF9, BW=125kHz, CR=4/5 (Tăng SF để cải thiện độ nhạy)
    LoRa_SX1278_writeReg(0x1D, 0x72);  // RegModemConfig1: BW=125kHz, CR=4/5, Explicit Header
    LoRa_SX1278_writeReg(0x1E, 0x94);  // RegModemConfig2 (SF9<<4 | CRC_ON) - SF9 thay vì SF7
    LoRa_SX1278_writeReg(0x26, 0x04);  // RegModemConfig3

    // Set Preamble Length to 8 (default, khớp với Raspberry Pi)
    LoRa_SX1278_writeReg(0x20, 0x00);  // RegPreambleMsb
    LoRa_SX1278_writeReg(0x21, 0x08);  // RegPreambleLsb = 8

    // Set Sync Word (0x34 = LoRaWAN public network, khớp với Raspberry)
    ret = LoRa_SX1278_writeReg(0x39, 0x34); // RegSyncWord
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sync word: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set PA_CONFIG: PA_BOOST with 12dBm (tiết kiệm pin, phù hợp khoảng cách gần-trung bình)
    // RegPaConfig (0x09): bit 7=1 (PA_BOOST), bit 6-4: MaxPower=111 (7), bit 3-0: OutputPower
    // Công thức PA_BOOST: Pout = 17 - (15 - OutputPower)
    // OutputPower=10 -> Pout=12dBm
    ret = LoRa_SX1278_writeReg(0x09, 0xFA); // RegPaConfig: PA_BOOST, MaxPower=7, OutputPower=10 (12dBm)
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PA config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // KHÔNG enable +20dBm mode (module chỉ hỗ trợ max 18dBm)
    // Giữ RegPaDac ở giá trị mặc định 0x84 thay vì 0x87
    ret = LoRa_SX1278_writeReg(0x4D, 0x84); // RegPaDac: Default mode (max 17dBm)
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PA DAC: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set Over Current Protection (OCP) to 100mA (phù hợp cho 12dBm, tiết kiệm pin tối đa)
    // OcpTrim = 11 -> 100mA
    ret = LoRa_SX1278_writeReg(0x0B, 0x2B); // RegOcp: OcpOn=1, OcpTrim=11 (100mA)
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set OCP: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure DIO0 for TxDone interrupt (DIO0=01 in bits 7:6)
    ret = LoRa_SX1278_writeReg(0x40, 0x40); // RegDioMapping1: DIO0=TxDone
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DIO mapping: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "LoRa SX1278 basic initialization complete");
    return ESP_OK;
}

// Simple send: write data to FIFO, set to TX mode, wait for DIO0 interrupt
esp_err_t LoRa_SX1278_send(const uint8_t *data, size_t length) {
    if (data == NULL || length == 0 || length > 255) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Flush any pending interrupts in queue
    uint32_t dummy;
    while (xQueueReceive(dio0_queue, &dummy, 0) == pdTRUE) {
        // Clear queue
    }

    // Set to Standby mode first
    ret = LoRa_SX1278_writeReg(0x01, 0x81); // RegOpMode: Long Range Mode + Standby
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set standby mode: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Clear all IRQ flags
    ret = LoRa_SX1278_writeReg(0x12, 0xFF); // RegIrqFlags: clear all
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear IRQ flags: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set FIFO address pointer to base address
    ret = LoRa_SX1278_writeReg(0x0D, 0x00); // RegFifoAddrPtr = 0
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set FIFO address pointer: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set FIFO TX base address to 0
    ret = LoRa_SX1278_writeReg(0x0E, 0x00); // RegFifoTxBaseAddr = 0
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set FIFO TX base: %s", esp_err_to_name(ret));
        return ret;
    }

    // Write payload length
    ret = LoRa_SX1278_writeReg(0x22, length); // RegPayloadLength
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set payload length: %s", esp_err_to_name(ret));
        return ret;
    }

    // Write data to FIFO
    ESP_LOGI(TAG, "Writing %d bytes to FIFO", length);
    ESP_LOG_BUFFER_HEX(TAG, data, length); // Debug: hiển thị dữ liệu gốc
    ret = LoRa_SX1278_writeBuffer(0x00, data, length); // RegFifo address = 0x00
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write data to FIFO: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Data written to FIFO successfully");

    // Set to TX mode
    ret = LoRa_SX1278_writeReg(0x01, 0x83); // RegOpMode: Long Range Mode + TX
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TX mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for DIO0 interrupt (TX done)
    uint32_t io_num;
    if (xQueueReceive(dio0_queue, &io_num, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout waiting for TX done interrupt");
        // Try to recover by going back to standby
        LoRa_SX1278_writeReg(0x01, 0x81);
        return ESP_ERR_TIMEOUT;
    }

    // Clear IRQ flags after successful TX
    ret = LoRa_SX1278_writeReg(0x12, 0xFF); // Clear all IRQ flags
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear IRQ after TX");
    }

    // Return to Standby mode
    ret = LoRa_SX1278_writeReg(0x01, 0x81); // RegOpMode: Long Range Mode + Standby
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to return to standby mode");
    }

    ESP_LOGI(TAG, "Data sent successfully");
    return ESP_OK;
}

esp_err_t LoRa_SX1278_receive(uint8_t *buffer, size_t *length) {
    // Implementation of receive function
    LoRa_SX1278_writeReg(0x0D, 0x00); // RegFifoAddrPtr = 0
    LoRa_SX1278_writeReg(0x0E, 0x00); // RegFifoRxBaseAddr = 0

    // Set to RX Continuous mode
    LoRa_SX1278_writeReg(0x01, 0x85); // RegOpMode: Long Range Mode + RX Continuous
    return ESP_OK;
}

