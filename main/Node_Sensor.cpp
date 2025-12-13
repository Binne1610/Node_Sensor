#include "ES_PH_SOIL.hpp"
#include "ES_SM_THEC.hpp"
#include "NPK.hpp"
#include "ES35_SW.hpp"
#include "LoRa_SX1278.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_log.h>

static const char* TAG_APP = "ES_SENSOR";
static const char* TAG_PH  = "PH_SENSOR";
static const char* TAG_THEC= "THEC_SENSOR";
static const char* TAG_NPK = "NPK_SENSOR";
static const char* TAG_SW = "ES35_SW_SENSOR";
static const char* TAG_LORA = "LORA_SX1278";

static SemaphoreHandle_t gUartMutex = nullptr; // bảo vệ truy cập UART/RS485 dùng chung

// BẬT/TẮT quét địa chỉ để tránh spam log khi không cần
#define ENABLE_ADDRESS_SCAN 0

// Function scan Modbus addresses để tìm sensor
void scan_modbus_addresses() {
#if ENABLE_ADDRESS_SCAN
    ESP_LOGI(TAG_APP, "Scanning Modbus addresses...");
    
    // Test các địa chỉ phổ biến
    uint8_t test_addresses[] = {0x01, 0x02, 0x03, 0x04};
    
    for (int i = 0; i < sizeof(test_addresses); i++) {
        ES_PH_SOIL test_sensor(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, test_addresses[i], 9600);
        if (test_sensor.init() == ESP_OK) {
            float test_value = 0.0f;
            ESP_LOGI(TAG_APP, "Testing address 0x%02X...", test_addresses[i]);
            if (test_sensor.read_pH(test_value) == ESP_OK) {
                ESP_LOGI(TAG_APP, "Found sensor at address 0x%02X, value: %.2f", test_addresses[i], test_value);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG_APP, "Address scan completed");
#endif
}

// Helper: thực thi hàm đọc bên trong vùng mutex với timeout
template<typename F>
static esp_err_t uart_locked_exec(F func, TickType_t timeoutMs){
    if (xSemaphoreTake(gUartMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE){
        esp_err_t r = func();
        xSemaphoreGive(gUartMutex);
        return r;
    }
    return ESP_ERR_TIMEOUT;
}

// Task đọc pH
void read_pH(void *parameter) {
    ES_PH_SOIL *pH_sensor = static_cast<ES_PH_SOIL*>(parameter);
    float pH = 0.0f;
    ESP_LOGI(TAG_PH, "pH task started");
    vTaskDelay(pdMS_TO_TICKS(200)); // delay khởi động task

    for(;;) {
        // Timeout 2500ms = 2.5 giây chờ mutex (UART lock)
        esp_err_t err = uart_locked_exec([&]{ return pH_sensor->read_PH(pH); }, 2500);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_PH, "pH: %.2f", pH);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_PH, "UART busy - mutex timeout after 2.5s");
        } else {
            ESP_LOGE(TAG_PH, "Sensor communication error");
        }
        vTaskDelay(pdMS_TO_TICKS(3000)); // thời gian đọc mỗi 3s
    }
}

// Task đọc Temperature/Humidity/EC
void read_THEC(void *parameter) {
    ES_SM_THEC *thec_sensor = static_cast<ES_SM_THEC*>(parameter);

    float Temperature = 0.0f;
    float Humidity    = 0.0f;
    float EC_value    = 0.0f;
    
    ESP_LOGI(TAG_THEC, "THEC task started");
    vTaskDelay(pdMS_TO_TICKS(400)); // delay khởi động task
    for(;;) {
        esp_err_t err = uart_locked_exec([&]{ return thec_sensor->read_THEC(Humidity, Temperature, EC_value); }, 2500);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_THEC, "RH_THEC: %.2f | Temp_THEC: %.2f°C | EC_THEC: %.2f", Humidity, Temperature, EC_value);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_THEC, "UART busy - skipped");
        } else {
            ESP_LOGE(TAG_THEC, "Read error");
        }
        vTaskDelay(pdMS_TO_TICKS(4000)); // thời gian đọc mỗi 4s
    }
}


// Task đọc NPK
void read_NPK(void *parameter) {
    NPK *npk_sensor = static_cast<NPK*>(parameter);

    float N_value = 0.0f;
    float P_value = 0.0f;
    float K_value = 0.0f;
    
    ESP_LOGI(TAG_NPK, "NPK task started");

    for(;;) {
        esp_err_t err = uart_locked_exec([&]{ return npk_sensor->read_NPK(N_value, P_value, K_value); }, 2500);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_NPK, "N: %.2f | P: %.2f | K: %.2f", N_value, P_value, K_value);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_NPK, "UART busy - skipped");
        } else {
            ESP_LOGE(TAG_NPK, "Read error");
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // thời gian đọc mỗi 5s
    }
}

// Task đọc ES35_SW (RH & Temp)
void read_SW(void *parameter) {
    ES35_SW *sw_sensor = static_cast<ES35_SW*>(parameter);

    float RH = 0.0f;
    float Temp = 0.0f;

    ESP_LOGI(TAG_SW, "ES35_SW task started");
    vTaskDelay(pdMS_TO_TICKS(300));
    for(;;) {
        esp_err_t err = uart_locked_exec([&]{ return sw_sensor->read_ES35_SW(RH, Temp); }, 2500);
        if (err == ESP_OK){
            ESP_LOGI(TAG_SW, "RH_SW: %.1f%% | Temp_SW: %.1f°C", RH, Temp);
        } else if (err == ESP_ERR_TIMEOUT){
            ESP_LOGW(TAG_SW, "UART busy - skipped");
        } else {
            ESP_LOGE(TAG_SW, "Read error");
        }
        vTaskDelay(pdMS_TO_TICKS(4000)); // thời gian đọc mỗi 4s
    }
}

// Task truyền dữ liệu qua LoRa SX1278
void lora_task(void *parameter) {
    LoRa_SX1278 *lora = static_cast<LoRa_SX1278*>(parameter);

    ESP_LOGI(TAG_LORA, "LoRa task started");
    vTaskDelay(pdMS_TO_TICKS(500)); // delay khởi động task

    // Khởi tạo LoRa
    if (lora->init() != ESP_OK) {
        ESP_LOGE(TAG_LORA, "LoRa init failed");
        vTaskDelete(nullptr);
        return;
    }

    // Gửi dữ liệu mẫu
    const char *message = "Hello from ESP32 LoRa!";
    for (;;) {
        esp_err_t err = lora->send(reinterpret_cast<const uint8_t*>(message), strlen(message));
        if (err == ESP_OK) {
            ESP_LOGI(TAG_LORA, "Message sent: %s", message);
        } else {
            ESP_LOGE(TAG_LORA, "Failed to send message");
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Gửi mỗi 10s
    }
}

extern "C" void app_main() {
    // Khởi tạo các cảm biến trên cùng UART với địa chỉ khác nhau
    static ES_PH_SOIL pH_sensor   (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x03, 9600);
    static ES_SM_THEC thec_sensor (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x02, 9600);
    static NPK        npk_sensor  (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x01, 9600);
    static ES35_SW    sw_sensor   (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x04, 9600);
    static LoRa_SX1278 lora;      // Khởi tạo LoRa instance

    // Init các sensor
    if (pH_sensor.init()    != ESP_OK){ ESP_LOGE(TAG_APP,"Init pH failed");       return; }
    if (thec_sensor.init()  != ESP_OK){ ESP_LOGE(TAG_APP,"Init THEC failed");     return; }
    if (npk_sensor.init()   != ESP_OK){ ESP_LOGE(TAG_APP,"Init NPK failed");      return; }
    if (sw_sensor.init()    != ESP_OK){ ESP_LOGE(TAG_APP,"Init ES35_SW failed");  return; }

    gUartMutex = xSemaphoreCreateMutex();
    if (gUartMutex == nullptr) {
        ESP_LOGE(TAG_APP, "Create UART mutex failed");
        return;
    }

    ESP_LOGI(TAG_APP, "Sensors initialized. Starting address scan...");
    vTaskDelay(pdMS_TO_TICKS(500)); // Tăng delay để ổn định bus
    
    // Scan địa chỉ trước khi start tasks
    scan_modbus_addresses();
    
    ESP_LOGI(TAG_APP, "Starting sensor reading tasks...");
    xTaskCreate(read_pH,   "read_pH",   4096, &pH_sensor,   4, nullptr);
    xTaskCreate(read_THEC, "read_THEC", 4096, &thec_sensor, 4, nullptr);
    xTaskCreate(read_NPK,  "read_NPK",  4096, &npk_sensor,  4, nullptr);
    xTaskCreate(read_SW,   "read_SW",   4096, &sw_sensor,   4, nullptr);
    xTaskCreate(lora_task, "lora_task", 4096, &lora,        4, nullptr);
    
    ESP_LOGI(TAG_APP, "All tasks created successfully (including LoRa)");
}




// extern "C" void app_main() {
//     ES_PH_SOIL ph_sensor(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18);
//     if (ph_sensor.init() != ESP_OK) {
//         ESP_LOGE("PH_SENSOR", "Init failed");
//         return;
//     }

//     while (1) {
//         float ph_value = 0.0f;
//         if (ph_sensor.read_PH(ph_value) == ESP_OK) {
//             ESP_LOGI("PH_SENSOR", "pH Value: %.2f", ph_value);
//         } else {
//             ESP_LOGE("PH_SENSOR", "Failed to read pH value");
//         }
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

// extern "C" void app_main() {
//     ES_SM_THEC thec_sensor (UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18);
//     if(thec_sensor.init() != ESP_OK) {
//         ESP_LOGE("THEC_SENSOR", "Init failed");
//         return;
//     }

//     while(1) {
//         float RH_value = 0.0f;
//         float Temp_value = 0.0f;
//         float EC_value = 0.0f;

//         if(thec_sensor.read_THEC(RH_value, Temo_value, EC_value) == ESP_OK) {
//             ESP_LOGI("THEC_SENSOR", "RH: %.2f, Temp: %.2f, EC: %.2f", RH_value, Temp_value, EC_value);
//         } else {
//             ESP_LOGE("THEC_SENSOR", "Failed to read THEC values");
//         }
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }