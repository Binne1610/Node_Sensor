#ifndef LORA_SX1278_HPP
#define LORA_SX1278_HPP

#pragma once
#include <stdint.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define LoRa_SX1278_SPI_CLOCK_HZ 8000000 // 8MHz
#define LoRa_MAX_SPI_TRANSFER_SIZE 4096 // Maximum SPI transfer size in bytes
#define LoRa_SX1278_SPI_HOST SPI2_HOST

// Extern declarations - định nghĩa thực tế trong .cpp
extern SemaphoreHandle_t LoRa_spi_mutex;
extern uint8_t *LoRa_TX_buf;
extern uint8_t *LoRa_RX_buf;
extern uint16_t LoRa_buf_size;

// Config pins - Khớp với hardware thực tế
// Sensors dùng UART1 (GPIO17/18), LoRa dùng SPI2
// #define LoRa_SX1278_PIN_SCK GPIO_NUM_36    // SPI SCK
// #define LoRa_SX1278_PIN_MISO GPIO_NUM_37   // SPI MISO
// #define LoRa_SX1278_PIN_MOSI GPIO_NUM_35   // SPI MOSI
// #define LoRa_SX1278_PIN_CS GPIO_NUM_45     // SPI CS
// #define LoRa_SX1278_PIN_RESET GPIO_NUM_47  // LoRa Reset
// #define LoRa_SX1278_PIN_DIO0 GPIO_NUM_38   // LoRa DIO0 interrupt

#define LoRa_SX1278_PIN_SCK GPIO_NUM_12    // SPI SCK
#define LoRa_SX1278_PIN_MISO GPIO_NUM_13   // SPI MISO
#define LoRa_SX1278_PIN_MOSI GPIO_NUM_11   // SPI MOSI
#define LoRa_SX1278_PIN_CS GPIO_NUM_10     // SPI CS
#define LoRa_SX1278_PIN_RESET GPIO_NUM_14  // LoRa Reset
#define LoRa_SX1278_PIN_DIO0 GPIO_NUM_9   // LoRa DIO0 interrupt

// C-style API functions
esp_err_t LoRa_SX1278_init(); // Khởi tạo LoRa
esp_err_t LoRa_SX1278_resetHW(); // Reset phần cứng LoRa
esp_err_t LoRa_SX1278_writeReg(uint8_t reg, uint8_t value); // Ghi giá trị vào thanh ghi
esp_err_t LoRa_SX1278_readReg(uint8_t reg, uint8_t *value); // Đọc giá trị từ thanh ghi
esp_err_t LoRa_SX1278_writeBuffer(uint8_t reg, const uint8_t *buffer, size_t length); // Ghi buffer
esp_err_t LoRa_SX1278_readBuffer(uint8_t reg, uint8_t *buffer, size_t length); // Đọc buffer
esp_err_t LoRa_SX1278_setMode(uint8_t mode); // Đặt chế độ hoạt động
uint8_t LoRa_SX1278_getMode(); // Lấy chế độ hiện tại
int LoRa_SX1278_getRSSI(); // Lấy giá trị RSSI
esp_err_t LoRa_SX1278_send(const uint8_t* data, size_t length); // Gửi dữ liệu

// C++ Class Wrapper
class LoRa_SX1278 {
public:
    LoRa_SX1278() = default;
    ~LoRa_SX1278() = default;
    
    esp_err_t init() { return LoRa_SX1278_init(); }
    esp_err_t send(const uint8_t* data, size_t length) { return LoRa_SX1278_send(data, length); }
    esp_err_t resetHW() { return LoRa_SX1278_resetHW(); }
    esp_err_t setMode(uint8_t mode) { return LoRa_SX1278_setMode(mode); }
    uint8_t getMode() { return LoRa_SX1278_getMode(); }
    int getRSSI() { return LoRa_SX1278_getRSSI(); }
};

#endif // LORA_SX1278_HPP