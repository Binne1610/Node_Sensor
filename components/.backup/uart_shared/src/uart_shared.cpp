#include "uart_shared.hpp"

// Global shared UART installation state
bool g_uart_installed[UART_NUM_MAX] = {false};
