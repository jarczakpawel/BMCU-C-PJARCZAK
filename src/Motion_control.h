#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ams_addressing.h"

void Motion_control_init();
void Motion_control_set_PWM(uint8_t CHx, int PWM);
void Motion_control_run(int error);
bool Motion_control_save_dm_key_none_thresholds(void);

void MC_PULL_detect_channels_inserted();

// Externy
extern float   MC_PULL_V_OFFSET[4];
extern float   MC_PULL_V_MIN[4];
extern float   MC_PULL_V_MAX[4];
extern uint8_t MC_PULL_pct[4];
extern int8_t  MC_PULL_POLARITY[4];
extern float   MC_DM_KEY_NONE_THRESH[4];
extern bool    filament_channel_inserted[4];

// platformio.ini: -DAMS_RETRACT_LEN
#ifndef AMS_RETRACT_LEN
#define AMS_RETRACT_LEN 0.2f
#endif

// platformio.ini: -DBMCU_DM_TWO_MICROSWITCH=1 (DM dual microswitch + autoload assist)
#ifndef BMCU_DM_TWO_MICROSWITCH
#define BMCU_DM_TWO_MICROSWITCH 0
#endif

// platformio.ini: -DBMCU_ONLINE_LED_FILAMENT_RGB=1 (show filament RGB on ONLINE LED when loaded)
#ifndef BMCU_ONLINE_LED_FILAMENT_RGB
#define BMCU_ONLINE_LED_FILAMENT_RGB 0
#endif

#ifndef motion_control_ams_num
#define motion_control_ams_num BMCU_LOCAL_AMS_INDEX
#endif

#ifndef motion_control_pull_back_distance
#define motion_control_pull_back_distance AMS_RETRACT_LEN
#endif

bool Motion_control_filament_present(uint8_t ch);
