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
    
    // BACKUP - Multi-Stage (không dùng, giữ lại để sau có thể enable)
    // uint32_t read_cycle_count;
    // bool read_full_sensors;
};
