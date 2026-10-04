#pragma once
#include <stdint.h>

#ifndef BAMBU_BUS_AMS_NUM
#define BAMBU_BUS_AMS_NUM 0
#endif

// === NVM w OSTATNIM sektorze 4KB (CH32V203C8: flash kończy się na 0x08010000) ===
#define FLASH_NVM_BASE_ADDR   ((uint32_t)0x0800F000)   // 4KB sector

#define FLASH_NVM_CAL_ADDR    (FLASH_NVM_BASE_ADDR + 0x000) // 1x256B
#define FLASH_NVM_MOTION_ADDR (FLASH_NVM_BASE_ADDR + 0x100) // 1x256B
#define FLASH_NVM_AMS_ADDR    (FLASH_NVM_BASE_ADDR + 0x200) // 4x256B (0..3) => do +0x5FF

#define FLASH_NVM256_PAGE_SIZE (256u)
#define FLASH_NVM_TOTAL_SIZE    (4096u)
#define FLASH_NVM_PAGE_COUNT    (FLASH_NVM_TOTAL_SIZE / FLASH_NVM256_PAGE_SIZE)
#define NVM256_CRC_OFF         (252u)

// Magici / wersje
static constexpr uint32_t MAGIC_FIL = 0x314C4946u; // 'FIL1'
static constexpr uint32_t MAGIC_CAL = 0x324C4143u; // 'CAL2'
static constexpr uint32_t MAGIC_MOT = 0x31544F4Du; // 'MOT1'
static constexpr uint32_t MAGIC_STA = 0x31415453u; // 'STA1'
static constexpr uint16_t VER_1     = 0x0001u;

struct __attribute__((packed, aligned(4))) NVM256_HDR
{
    uint32_t magic;
    uint16_t ver;
    uint16_t len;
    uint32_t rsv;
};

struct __attribute__((packed, aligned(4))) Flash_FilamentInfo
{
    uint8_t  bambubus_filament_id[8];
    uint8_t  color_R;
    uint8_t  color_G;
    uint8_t  color_B;
    uint8_t  color_A;
    uint16_t temperature_min;
    uint16_t temperature_max;
    char     name[16];
};

void Flash_saves_init(void);
void Flash_background_run(void);
bool Flash_background_idle(void);

// AMS: 4x 256B
bool Flash_AMS_filament_read(uint8_t filament_idx, Flash_FilamentInfo* out);
bool Flash_AMS_filament_write(uint8_t filament_idx, const Flash_FilamentInfo* info);
bool Flash_AMS_filament_clear(uint8_t filament_idx);

bool Flash_AMS_state_read(uint8_t* loaded_channel);
bool Flash_AMS_state_write(uint8_t loaded_channel);

// CAL: 1x 256B
bool Flash_MC_PULL_cal_read(float offs[4], float vmin[4], float vmax[4], int8_t pol[4], float dm_none[4], uint8_t* valid_mask);
bool Flash_MC_PULL_cal_write_all(const float offs[4], const float vmin[4], const float vmax[4], const int8_t pol[4], const float dm_none[4], uint8_t valid_mask);
bool Flash_MC_PULL_cal_clear(void);
bool Flash_NVM_full_clear(void);

// MOT: 1x 256B
bool Flash_Motion_read(void* out, uint16_t bytes);
bool Flash_Motion_write(const void* in, uint16_t bytes);
bool Flash_Motion_clear(void);