# Node Sensor - System Architecture Guide

## 📋 Tổng quan hệ thống

Node Sensor là thiết bị IoT nông nghiệp **tiết kiệm pin** sử dụng **ESP32-S3**, đọc dữ liệu từ 4 cảm biến Modbus, **gửi qua LoRa** đến Gateway, và deep sleep để kéo dài tuổi thọ pin.

**Vai trò:** Sensor node (thu thập + truyền LoRa) - **KHÔNG trực tiếp upload Firebase**

**Kiến trúc:** Level 2 Professional (State Machine + Watchdog + Error Recovery + Deep Sleep)

**Hệ thống 2 tầng:**
```
Node Sensor (this project) → LoRa → Gateway_IOT → Firebase
```

---

## 🏗️ Kiến trúc phần cứng

### ESP32-S3 Connections

| Peripheral | GPIO | Protocol | Chức năng |
|------------|------|----------|-----------|
| **Modbus Sensors** | TX=17, RX=18 | UART1 9600 baud | 4 cảm biến (pH, THEC, NPK, SW) |
| **LoRa SX1278** | CS=10, MOSI=11, MISO=13, SCK=12, RST=14, DIO0=9 | SPI2 | Truyền dữ liệu |
| **DS3231 RTC** | SDA=21, SCL=42, INT=35 | I2C 100kHz | Đồng hồ thực + Wake timer |

### Cảm biến Modbus RTU (UART shared)

| Sensor | Slave Address | Registers | Dữ liệu |
|--------|---------------|-----------|---------|
| ES_PH_SOIL | 0x03 | 0x0000 (1 reg) | pH |
| ES_SM_THEC | 0x02 | 0x0000 (3 regs) | Temp, RH, EC |
| NPK | 0x01 | 0x001E (3 regs) | N, P, K |
| ES35_SW | 0x04 | 0x0000 (2 regs) | SW Temp, SW RH |

**Tất cả sensors chia sẻ 1 UART bus** → Đọc tuần tự, không song song

---

## 🔄 State Machine Flow

```
┌─────────────────────────────────────────────────────────────┐
│                     POWER ON / RESET                        │
└──────────────────────┬──────────────────────────────────────┘
                       ↓
              ┌─────────────────┐
              │   INIT STATE    │
              │ - Setup Watchdog│
              │ - Init RTC      │
              │ - NTP Sync (1st)│
              │ - Init Sensors  │
              │ - Init LoRa     │
              └────────┬────────┘
                       ↓
            ┌──────────────────────┐
            │  READ_SENSORS STATE  │
            │ - Read pH            │
            │ - Read THEC          │
            │ - Read NPK           │
            │ - Read SW            │
            │ - Accept ≥2/4 OK     │
            └──────┬───────────────┘
                   ↓
          ┌─────────────────────┐
          │  SEND_DATA STATE    │
          │ - Build LoRa packet │
          │ - Add timestamp     │
          │ - Send via LoRa     │
          └──────┬──────────────┘
                 ↓
        ┌────────────────────┐
        │   SLEEP STATE      │
        │ - Set RTC alarm    │
        │ - ESP32 EXT0 wake  │
        │ - Deep Sleep 5 min │
        └────────┬───────────┘
                 ↓
         (Wake by RTC INT)
                 ↓
        ┌────────────────────┐
        │   INIT STATE       │
        │ (Skip NTP sync)    │
        └────────────────────┘
                 ↓
            (Loop forever)

     ┌─────────────────────────┐
     │  ERROR_RECOVERY STATE   │
     │ - Retry 5s delay        │
     │ - Max 5 errors          │
     │ - Then ESP32 restart    │
     └─────────────────────────┘
```

---

## 📁 Cấu trúc Components

### **Core Components (Đang dùng)**

```
components/
├── system_manager/          ← Main controller (State Machine)
│   ├── include/
│   │   ├── system_states.hpp    (States: INIT, READ_SENSORS, SEND_DATA, SLEEP, ERROR_RECOVERY)
│   │   └── system_manager.hpp   (SystemManager class)
│   └── src/
│       └── system_manager.cpp   (463 dòng - State handlers, NTP sync, Deep sleep)
│
├── modbus_base/             ← Base class cho Modbus sensors
│   ├── include/
│   │   └── ModbusSensorBase.hpp (CRC16, modbusReadHolding, SensorStats)
│   └── src/
│       └── ModbusSensorBase.cpp (UART config, Modbus protocol)
│
├── ES_PH_SOIL/              ← pH sensor (kế thừa ModbusSensorBase)
├── ES_SM_THEC/              ← Temp/RH/EC sensor
├── NPK/                     ← NPK sensor
├── ES35_SW/                 ← Soil sensor
│
├── LoRa_SX1278/             ← LoRa transceiver driver
│   └── src/lora_sx1278.cpp  (SPI communication, 433.175 MHz)
│
├── DS3231_RTC/              ← RTC module
│   ├── include/
│   │   └── DS3231_RTC.hpp   (I2C, Alarm, NTP sync)
│   └── src/
│       └── DS3231_RTC.cpp   (BCD conversion, NTP → RTC)
│
└── WiFiHelper/              ← WiFi connection helper
    └── src/WiFiHelper.cpp   (Connect, Disconnect, NTP sync support)
```

### **Deprecated Components (Đã backup vào .backup/)**

```
.backup/
├── sensor_tasks/     ❌ Multi-task parallel architecture cũ
├── lora_task/        ❌ LoRa transmission task cũ
├── sensor_data/      ❌ Shared data giữa tasks cũ
└── uart_shared/      ❌ UART mutex cũ
```

---

## ⚙️ Chi tiết hoạt động

### 1️⃣ **INIT State**

**Lần boot đầu tiên (boot_count = 1):**
```cpp
1. Setup Watchdog timer (60s timeout)
2. Init DS3231 RTC (I2C)
3. Connect WiFi (SSID: BK_IOT_25)
4. Sync NTP (pool.ntp.org, UTC-7 Vietnam)
5. Set DS3231 datetime từ NTP
6. Disconnect WiFi (tiết kiệm pin)
7. Init 4 Modbus sensors (UART1)
8. Init LoRa SX1278 (SPI2, 433.175 MHz)
9. Transition → READ_SENSORS
```

**Các lần boot sau (wake từ sleep):**
```cpp
1. Setup Watchdog
2. Init RTC (skip NTP sync - đã có thời gian)
3. Clear RTC alarm flag
4. Read RTC temperature (bonus)
5. Init sensors (UART already configured)
6. Init LoRa
7. Transition → READ_SENSORS
```

**Thời gian:** ~5-8s (lần đầu), ~2-3s (các lần sau)

---

### 2️⃣ **READ_SENSORS State**

**Đọc tuần tự (sequential), không parallel:**

```cpp
esp_err_t SystemManager::read_all_sensors() {
    int success_count = 0;
    
    // 1. Đọc pH (0x03)
    if (pH_sensor_->read_PH(pH_value_) == ESP_OK) {
        success_count++;
    }
    
    // 2. Đọc THEC (0x02)
    if (thec_sensor_->read_THEC(temp_value_, rh_value_, ec_value_) == ESP_OK) {
        success_count++;
    }
    
    // 3. Đọc NPK (0x01)
    if (npk_sensor_->read_NPK(n_value_, p_value_, k_value_) == ESP_OK) {
        success_count++;
    }
    
    // 4. Đọc SW (0x04)
    if (sw_sensor_->read_ES35_SW(sw_temp_value_, sw_rh_value_) == ESP_OK) {
        success_count++;
    }
    
    // Chấp nhận nếu ≥2/4 sensor OK
    return (success_count >= 2) ? ESP_OK : ESP_FAIL;
}
```

**Modbus Protocol (mỗi sensor):**
```
1. Gửi request: [Slave_Addr][0x03][Reg_Hi][Reg_Lo][Count_Hi][Count_Lo][CRC16_Lo][CRC16_Hi]
2. Đợi response (timeout 2500ms)
3. Validate CRC16
4. Parse dữ liệu (Big Endian)
5. Convert raw → physical value
```

**Thời gian:** ~1-2 giây (4 sensors × 200-500ms/sensor)

---

### 3️⃣ **SEND_DATA State**

**Build LoRa packet với timestamp:**

```cpp
esp_err_t SystemManager::build_and_send_lora_packet() {
    // 1. Lấy timestamp từ DS3231
    struct tm timeinfo;
    rtc_->get_datetime(&timeinfo);
    snprintf(timestamp, 64, "%04d-%02d-%02dT%02d:%02d:%02d", ...);
    
    // 2. Format packet (256 bytes max)
    snprintf(data, 256, 
             "pH:%.2f;T:%.1f;RH:%.1f;EC:%.1f;N:%.0f;P:%.0f;K:%.0f;SW_T:%.1f;SW_RH:%.1f;time:%s;boot:%lu",
             pH_value_, temp_value_, rh_value_, ec_value_,
             n_value_, p_value_, k_value_,
             sw_temp_value_, sw_rh_value_,
             timestamp, context_.boot_count);
    
    // 3. Gửi qua LoRa
    lora_->send((uint8_t*)data, strlen(data));
}
```

**Packet format example:**
```
pH:6.85;T:28.5;RH:65.2;EC:1.2;N:45;P:23;K:67;SW_T:27.8;SW_RH:68.5;time:2026-01-16T14:30:25;boot:5
```

**LoRa config:**
- Frequency: 433.175 MHz
- Bandwidth: 125 kHz (BW7)
- Spreading Factor: 7 (SF7)
- Coding Rate: 4/5 (CR1)
- Sync Word: 0x34

**Thời gian:** ~1-2 giây

---

### 4️⃣ **SLEEP State**

**Deep Sleep với RTC wakeup:**

```cpp
void SystemManager::enter_deep_sleep() {
    // 1. Cleanup watchdog
    esp_task_wdt_delete(NULL);
    
    // 2. Set DS3231 alarm (5 phút = 300 giây)
    rtc_->set_alarm_interval(300);
    
    // 3. Config ESP32 wake on RTC INT pin
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);  // Wake on LOW
    
    // 4. Fallback timer (nếu RTC fail)
    esp_sleep_enable_timer_wakeup(300 * 1000000ULL);
    
    // 5. Enter deep sleep
    esp_deep_sleep_start();  // ← ESP32 tắt, chỉ RTC chạy
}
```

**Cách DS3231 đánh thức ESP32:**
```
1. DS3231 Alarm 1 trigger sau 300 giây
2. SQW/INT pin → LOW
3. ESP32 GPIO_35 detect LOW → Wake
4. ESP32 boot lại từ đầu (như reset)
5. SystemManager kiểm tra wakeup_reason = ESP_SLEEP_WAKEUP_EXT0
6. Clear alarm flag
7. Tiếp tục từ INIT state
```

**Tiêu thụ điện:**
- **Active (wake):** ~150mA × 15s = 0.625 mAh
- **Sleep:** ~10µA × 285s = 0.0008 mAh
- **1 cycle:** ~0.626 mAh
- **Pin 2500mAh:** 2500 / 0.626 ≈ **4000 cycles ≈ 330 ngày (~1 năm)** ✅

---

## 🛡️ Error Handling & Recovery

### **Watchdog Timer**

```cpp
void SystemManager::setup_watchdog() {
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 60000,  // 60 giây
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
}

// Reset trong mỗi state
void SystemManager::run() {
    while (...) {
        reset_watchdog();  // ← Ngăn watchdog timeout
        // ... handle states
    }
}
```

**Nếu sensor hang > 60s → Watchdog reset ESP32**

---

### **Error Recovery Logic**

```cpp
esp_err_t SystemManager::handle_error_recovery_state() {
    context_.error_count++;
    
    if (context_.error_count >= MAX_ERROR_COUNT) {  // 5 lỗi
        ESP_LOGE(TAG, "Too many errors! Resetting...");
        esp_restart();  // Hard reset
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000));  // Retry sau 5s
    transition_to(SystemState::INIT);  // Thử lại
}
```

**RTC Memory lưu error_count qua deep sleep:**
```cpp
RTC_DATA_ATTR static uint32_t rtc_error_count = 0;  // Không mất khi sleep
```

---

### **Sensor Fault Tolerance**

**Chấp nhận 2/4 sensors OK:**
```cpp
if (success_count >= 2) {
    ESP_LOGI(TAG, "Sensor read OK (%d/4 sensors)", success_count);
    return ESP_OK;  // Vẫn gửi data dù thiếu 1-2 sensor
}
```

**Lý do:** 
- Sensor có thể tạm thời lỗi (nhiễu, kết nối)
- Vẫn có dữ liệu hữu ích để gửi
- Không waste power cycle

---

## 📊 Data Flow

```
┌──────────────┐
│  Modbus      │
│  Sensors     │
│ (pH,THEC,    │
│  NPK,SW)     │
└──────┬───────┘
       │ UART1 (9600 baud)
       ↓
┌──────────────────────┐
│  ModbusSensorBase    │
│  - CRC16 validation  │
│  - Parse registers   │
│  - Convert to float  │
└──────┬───────────────┘
       │
       ↓
┌─────────────────────────┐
│   SystemManager         │
│   (Node Sensor)         │
│   - Store in buffers    │
│   - Add RTC timestamp   │
│   - Format LoRa packet  │
└──────┬──────────────────┘
       │
       ↓
┌──────────────┐
│  LoRa SX1278 │
│  433.175 MHz │
└──────┬───────┘
       │ RF (LoRa wireless)
       ↓
┌────────────────────────┐
│   Gateway_IOT          │
│   - Receive LoRa       │
│   - Parse packet       │
│   - Upload Firebase    │
└──────┬─────────────────┘
       │ HTTP/WiFi
       ↓
┌──────────────────────┐
│   Firebase           │
│   Realtime Database  │
│   (bk-iot-26)        │
└──────────────────────┘
```

**Quan trọng:** 
- ✅ **Node Sensor** chỉ gửi LoRa packet (không connect Firebase)
- ✅ **Gateway_IOT** nhận LoRa → parse → upload Firebase
- ✅ Node không cần WiFi liên tục (chỉ NTP sync lần đầu)

---

## 🔧 Configuration

### **WiFi credentials (system_manager.hpp)**
```cpp
static constexpr const char* WIFI_SSID = "BK_IOT_25";
static constexpr const char* WIFI_PASSWORD = "Bkiot@2025";
static constexpr const char* NTP_SERVER = "pool.ntp.org";
static constexpr const char* TIMEZONE = "UTC-7";  // Vietnam UTC+7
```

### **GPIO Pins (system_manager.hpp)**
```cpp
// DS3231 RTC
static constexpr gpio_num_t RTC_SDA_PIN = GPIO_NUM_21;
static constexpr gpio_num_t RTC_SCL_PIN = GPIO_NUM_42;
static constexpr gpio_num_t RTC_INT_PIN = GPIO_NUM_35;  // Wakeup pin
```

### **Timing (system_manager.hpp)**
```cpp
static constexpr uint32_t WATCHDOG_TIMEOUT_SEC = 60;      // Watchdog timeout
static constexpr uint32_t MAX_ERROR_COUNT = 5;             // Max errors trước reset
static constexpr uint32_t DEFAULT_SLEEP_SEC = 300;         // Deep sleep 5 phút
static constexpr uint32_t ERROR_RETRY_DELAY_MS = 5000;    // Retry delay
```

### **Modbus sensors (SystemManager::init())**
```cpp
pH_sensor_ = new ES_PH_SOIL(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x03, 9600);
thec_sensor_ = new ES_SM_THEC(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x02, 9600);
npk_sensor_ = new NPK(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x01, 9600);
sw_sensor_ = new ES35_SW(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, 0x04, 9600);
```

### **LoRa config (lora_sx1278.cpp)**
```cpp
lora_set_frequency(433175000);        // 433.175 MHz
lora_set_bandwidth(7);                // 125 kHz
lora_set_spreading_factor(7);         // SF7
lora_set_coding_rate(1);              // 4/5
lora_set_sync_word(0x34);             // Private network
```

---

## 🚀 Build & Flash

### **Build project**
```bash
cd d:\IOT_AGRI\Node_Sensor
idf.py build
```

### **Flash to ESP32-S3**
```bash
idf.py -p COM3 flash monitor
```

### **Monitor logs**
```bash
idf.py -p COM3 monitor
```

### **Clean build**
```bash
idf.py fullclean
idf.py build
```

---

## 📝 Log Output Example

```
I (123) NODE_SENSOR: === Node Sensor Starting (Level 2 Architecture) ===
I (145) SYS_MANAGER: === System Manager Init ===
I (156) SYS_MANAGER: First boot or reset
I (167) SYS_MANAGER: Setting up watchdog (60 sec timeout)
I (178) DS3231: DS3231 initialized successfully
I (189) SYS_MANAGER: First boot detected - syncing time from NTP...
I (201) WiFiHelper: Connecting to WiFi SSID: BK_IOT_25
I (3456) WiFiHelper: Connected to AP SSID: BK_IOT_25
I (3467) WiFiHelper: Got IP: 192.168.1.100
I (5123) DS3231: NTP time received: 2026-01-16 14:30:25
I (5234) DS3231: DS3231 datetime set successfully
I (5345) SYS_MANAGER: RTC synced with NTP successfully!
I (5456) WiFiHelper: WiFi disconnected and deinitialized
I (5567) SYS_MANAGER: RTC temperature: 25.50°C
I (5678) ES_PH_SOIL: UART initialized on port 1, TX=17, RX=18
I (6789) SYS_MANAGER: --- System Status ---
I (6890) SYS_MANAGER: Boot: 1, Errors: 0, State: 1
I (6901) SYS_MANAGER: [READ_SENSORS] Reading all sensors...
I (7123) SYS_MANAGER: pH: 6.85
I (7456) SYS_MANAGER: THEC: T=28.5°C, RH=65.2%, EC=1.2
I (7789) SYS_MANAGER: NPK: N=45, P=23, K=67
I (8123) SYS_MANAGER: SW: T=27.8°C, RH=68.5%
I (8234) SYS_MANAGER: Sensor read OK (4/4 sensors)
I (8345) SYS_MANAGER: [SEND_DATA] Sending LoRa packet...
I (8456) SYS_MANAGER: LoRa data: pH:6.85;T:28.5;RH:65.2;EC:1.2;N:45;P:23;K:67;SW_T:27.8;SW_RH:68.5;time:2026-01-16T14:30:25;boot:1
I (9567) SYS_MANAGER: LoRa packet sent successfully
I (9678) SYS_MANAGER: [SLEEP] Preparing for deep sleep...
I (9789) DS3231: DS3231 alarm set for 300 seconds
I (9890) SYS_MANAGER: Goodbye! Sleeping for 300 seconds (RTC alarm wakeup)

... 5 phút sau ...

I (123) NODE_SENSOR: === Node Sensor Starting (Level 2 Architecture) ===
I (145) SYS_MANAGER: Wakeup from DS3231 RTC alarm
I (156) SYS_MANAGER: Not first boot (boot_count=2) - skipping NTP sync
I (167) SYS_MANAGER: [READ_SENSORS] Reading all sensors...
...
```

---

## 🔍 Troubleshooting

### **Lỗi build**
```bash
# Clear build cache
idf.py fullclean
rm -rf build/

# Rebuild
idf.py build
```

### **Sensor không đọc được**
```
1. Kiểm tra wiring: TX=17, RX=18
2. Kiểm tra Modbus address (pH=0x03, THEC=0x02, NPK=0x01, SW=0x04)
3. Kiểm tra baud rate (9600)
4. Monitor UART: idf.py monitor
5. Xem log CRC errors trong SensorStats
```

### **LoRa không gửi**
```
1. Kiểm tra SPI pins (CS=10, MOSI=11, MISO=13, SCK=12)
2. Kiểm tra frequency match với Gateway (433.175 MHz)
3. Kiểm tra SF/BW/CR match
4. Test với simple LoRa test code
```

### **RTC không wake**
```
1. Kiểm tra DS3231 I2C (SDA=21, SCL=42)
2. Kiểm tra INT pin (GPIO 35)
3. Kiểm tra pullup resistor trên INT
4. Test manual wake: esp_sleep_enable_timer_wakeup()
```

### **NTP sync fail**
```
1. Kiểm tra WiFi credentials
2. Ping pool.ntp.org
3. Tăng timeout (15s → 30s)
4. Dùng NTP server khác (time.google.com)
```

---

## 📚 Key Files Reference

| File | Dòng code | Mục đích |
|------|-----------|----------|
| `main/Node_Sensor.cpp` | 18 | Entry point - Tạo SystemManager |
| `system_manager/src/system_manager.cpp` | 463 | State machine core logic |
| `system_manager/include/system_states.hpp` | 30 | State enums & context |
| `modbus_base/src/ModbusSensorBase.cpp` | 195 | Modbus protocol base |
| `DS3231_RTC/src/DS3231_RTC.cpp` | 337 | RTC + NTP sync |
| `WiFiHelper/src/WiFiHelper.cpp` | 156 | WiFi connection |
| `LoRa_SX1278/src/lora_sx1278.cpp` | ~300 | LoRa driver |

---

## 🎯 Best Practices

### **Power Optimization**
- ✅ Deep sleep giữa cycles (10µA)
- ✅ Tắt WiFi sau NTP sync (chỉ dùng lần đầu)
- ✅ **Không cần WiFi liên tục** - chỉ gửi LoRa (tiết kiệm ~100mA)
- ✅ Sequential sensor reading (không waste parallel tasks)
- ✅ Fast wake-read-send-sleep (15s active)

### **Architecture Benefits**
- ✅ **Node Sensor:** Battery powered, LoRa only, no WiFi needed
- ✅ **Gateway_IOT:** Powered from wall, WiFi always on, upload Firebase
- ✅ Phân tách vai trò → Node tiết kiệm pin tối đa

### **Reliability**
- ✅ Watchdog timer (60s)
- ✅ Error recovery (max 5 retry)
- ✅ Sensor fault tolerance (2/4 OK)
- ✅ RTC memory cho state persistence
- ✅ Fallback timer nếu RTC fail

### **Data Quality**
- ✅ CRC16 validation cho Modbus
- ✅ Timestamp chính xác từ NTP
- ✅ Boot count tracking
- ✅ Sensor statistics (min/max/avg)

### **Maintainability**
- ✅ OOP architecture (inheritance)
- ✅ Component-based structure
- ✅ Clear state machine
- ✅ Extensive logging
- ✅ Configuration constants

---

## 📈 Future Improvements

**Có thể thêm:**
- [ ] OTA firmware update
- [ ] SD card logging
- [ ] Battery voltage monitoring
- [ ] Dynamic sleep interval (based on sensor values)
- [ ] LoRaWAN support
- [ ] MQTT fallback
- [ ] Web config portal
- [ ] Multi-language support

---

## 👥 Author

**Project:** Node Sensor Agriculture IoT  
**Architecture:** Level 2 Professional (State Machine)  
**Target:** ESP32-S3  
**Framework:** ESP-IDF v5.5  
**Date:** January 2026  

---

## 📄 License

Dự án nội bộ BK IoT Lab.
