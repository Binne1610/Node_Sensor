#pragma once

class ES_PH_SOIL;
class ES_SM_THEC;
class NPK;
class ES35_SW;

// Task functions
void read_pH_task(void *parameter);
void read_THEC_task(void *parameter);
void read_NPK_task(void *parameter);
void read_SW_task(void *parameter);

// Scan utility
void scan_modbus_addresses();
