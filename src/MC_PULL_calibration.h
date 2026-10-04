#pragma once

void MC_PULL_calibration_boot();
bool MC_PULL_calibration_clear();
#include <stdint.h>
bool MC_PULL_calibration_is_valid(uint8_t ch);
