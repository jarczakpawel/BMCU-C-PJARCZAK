#pragma once
#include <stdint.h>

void prusa_modbus_rx(uint8_t data);
void prusa_modbus_run();
bool prusa_modbus_bus_idle();
void prusa_modbus_calibration_run();
