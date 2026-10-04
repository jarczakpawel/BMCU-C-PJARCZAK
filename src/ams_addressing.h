#pragma once
#include <stdint.h>

#ifndef BAMBU_BUS_AMS_NUM
#define BAMBU_BUS_AMS_NUM 0
#endif

// 0 - 3 = fixed AMS A - D
// 4 = AUTO
#if (BAMBU_BUS_AMS_NUM < 0) || (BAMBU_BUS_AMS_NUM > 4)
#error "BAMBU_BUS_AMS_NUM must be 0..3, or 4 for AUTO"
#endif

#if BAMBU_BUS_AMS_NUM == 4
#define BMCU_AMS_AUTO 1
#define BMCU_LOCAL_AMS_INDEX 0
#else
#define BMCU_AMS_AUTO 0
#define BMCU_LOCAL_AMS_INDEX BAMBU_BUS_AMS_NUM
#endif
