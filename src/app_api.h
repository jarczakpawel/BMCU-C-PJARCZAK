#pragma once
#include <stdint.h>
#include "ws2812.h"

extern WS2812_class RGBOUT[4];

static inline __attribute__((always_inline))
void MC_STU_RGB_set(uint8_t ch, uint8_t r, uint8_t g, uint8_t b)
{
    if (ch < 4) RGBOUT[ch].set_RGB(r, g, b, 0);
}

static inline __attribute__((always_inline))
void MC_PULL_ONLINE_RGB_set(uint8_t ch, uint8_t r, uint8_t g, uint8_t b, bool filament = false)
{
    if (ch < 4) RGBOUT[ch].set_RGB_online(r, g, b, 1, filament);
}

void ams_datas_set_need_to_save_filament(uint8_t filament_idx);
void ams_state_claim_loaded(uint8_t filament_ch);
void ams_state_set_loaded(uint8_t filament_ch);
void ams_state_set_unloaded(uint8_t filament_ch);
uint8_t ams_state_get_loaded(void);

float Motion_control_filament_meters(uint8_t ch);
