#include "sensor_tasks.hpp"
#include "sensor_data.hpp"
#include "ES_PH_SOIL.hpp"
#include "ES_SM_THEC.hpp"
#include "NPK.hpp"
#include "ES35_SW.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG_PH   = "PH_SENSOR";
static const char* TAG_THEC = "THEC_SENSOR";
static const char* TAG_NPK  = "NPK_SENSOR";
static const char* TAG_SW   = "ES35_SW_SENSOR";
static const char* TAG_SCAN = "ADDR_SCAN";

// BẬT/TẮT quét địa chỉ
#define ENABLE_ADDRESS_SCAN 0

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

// Scan Modbus addresses
void scan_modbus_addresses() {
#if ENABLE_ADDRESS_SCAN
    ESP_LOGI(TAG_SCAN, "Scanning Modbus addresses...");
    
    uint8_t test_addresses[] = {0x01, 0x02, 0x03, 0x04};
    
    for (int i = 0; i < sizeof(test_addresses); i++) {
        ES_PH_SOIL test_sensor(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, test_addresses[i], 9600);
        if (test_sensor.init() == ESP_OK) {
            float test_value = 0.0f;
            ESP_LOGI(TAG_SCAN, "Testing address 0x%02X...", test_addresses[i]);
            if (test_sensor.read_PH(test_value) == ESP_OK) {
                ESP_LOGI(TAG_SCAN, "Found sensor at address 0x%02X, value: %.2f", test_addresses[i], test_value);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG_SCAN, "Address scan completed");
#endif
}

// Task đọc pH
void read_pH_task(void *parameter) {
    ES_PH_SOIL *pH_sensor = static_cast<ES_PH_SOIL*>(parameter);
    float pH = 0.0f;
    ESP_LOGI(TAG_PH, "pH task started");
    vTaskDelay(pdMS_TO_TICKS(200));

    for(;;) {
        esp_err_t err = uart_locked_exec([&]{ return pH_sensor->read_PH(pH); }, 2500);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_PH, "pH: %.2f", pH);
            if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                gSensorData.pH = pH;
                xSemaphoreGive(gDataMutex);
            }
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_PH, "UART busy - mutex timeout");
        } else {
            ESP_LOGE(TAG_PH, "Sensor communication error");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// Task đọc THEC
void read_THEC_task(void *parameter) {
    ES_SM_THEC *thec_sensor = static_cast<ES_SM_THEC*>(parameter);
    float Temperature = 0.0f;
    float Humidity = 0.0f;
    float EC_value = 0.0f;
    
    ESP_LOGI(TAG_THEC, "THEC task started");
    vTaskDelay(pdMS_TO_TICKS(400));

    for(;;) {
        esp_err_t err = uart_locked_exec([&]{ return thec_sensor->read_THEC(Humidity, Temperature, EC_value); }, 2500);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_THEC, "RH_THEC: %.2f | Temp_THEC: %.2f°C | EC_THEC: %.2f", Humidity, Temperature, EC_value);
            if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                gSensorData.RH_THEC = Humidity;
                gSensorData.Temp_THEC = Temperature;
                gSensorData.EC_THEC = EC_value;
                xSemaphoreGive(gDataMutex);
            }
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_THEC, "UART busy - skipped");
        } else {
            ESP_LOGE(TAG_THEC, "Read error");
        }
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

// Task đọc NPK
void read_NPK_task(void *parameter) {
    NPK *npk_sensor = static_cast<NPK*>(parameter);
    float N_value = 0.0f;
    float P_value = 0.0f;
    float K_value = 0.0f;
    
    ESP_LOGI(TAG_NPK, "NPK task started");

    for(;;) {
        esp_err_t err = uart_locked_exec([&]{ return npk_sensor->read_NPK(N_value, P_value, K_value); }, 2500);
        if (err == ESP_OK) {
            ESP_LOGI(TAG_NPK, "N: %.2f | P: %.2f | K: %.2f", N_value, P_value, K_value);
            if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                gSensorData.N_value = N_value;
                gSensorData.P_value = P_value;
                gSensorData.K_value = K_value;
                xSemaphoreGive(gDataMutex);
            }
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_NPK, "UART busy - skipped");
        } else {
            ESP_LOGE(TAG_NPK, "Read error");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Task đọc ES35_SW
void read_SW_task(void *parameter) {
    ES35_SW *sw_sensor = static_cast<ES35_SW*>(parameter);
    float RH = 0.0f;
    float Temp = 0.0f;

    ESP_LOGI(TAG_SW, "ES35_SW task started");
    vTaskDelay(pdMS_TO_TICKS(300));

    for(;;) {
        esp_err_t err = uart_locked_exec([&]{ return sw_sensor->read_ES35_SW(RH, Temp); }, 2500);
        if (err == ESP_OK){
            ESP_LOGI(TAG_SW, "RH_SW: %.1f%% | Temp_SW: %.1f°C", RH, Temp);
            if (xSemaphoreTake(gDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                gSensorData.RH_SW = RH;
                gSensorData.Temp_SW = Temp;
                xSemaphoreGive(gDataMutex);
            }
        } else if (err == ESP_ERR_TIMEOUT){
            ESP_LOGW(TAG_SW, "UART busy - skipped");
        } else {
            ESP_LOGE(TAG_SW, "Read error");
        }
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}
