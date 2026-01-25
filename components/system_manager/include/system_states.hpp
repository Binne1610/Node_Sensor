#pragma once
#include <cstdint>

enum class SystemState {
    INIT,
    READ_SENSORS,
    SEND_DATA,
    SLEEP,
    ERROR_RECOVERY
};

enum class ErrorType {
    NONE,
    SENSOR_TIMEOUT,
    LORA_FAILED,
    WATCHDOG_TIMEOUT
};

struct SystemContext {
    SystemState current_state;
    SystemState next_state;
    uint32_t boot_count;
    uint32_t error_count;
    ErrorType last_error;
    uint32_t sleep_interval_sec;
    
    // Multi-Stage Sensor Reading
    uint32_t read_cycle_count;    // Track wake cycles (0-5)
    bool read_full_sensors;       // true = all sensors, false = pH only
};
