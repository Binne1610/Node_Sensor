#pragma once
#include <driver/uart.h>

// Global shared UART installation state
extern bool g_uart_installed[UART_NUM_MAX];
