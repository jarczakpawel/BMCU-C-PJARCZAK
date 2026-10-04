#pragma once
#include <stdint.h>
#include <stdbool.h>

enum class bambubus_package_type
{
    error = -1,
    none = 0,
    filament_motion_short,
    filament_motion_long,
    online_detect,
    REQx6,
    NFC_detect,
    set_filament_info,
    MC_online,
    read_filament_info,
    set_filament_info_type2,
    version,
    serial_number,
    heartbeat,
    ETC,

    __BambuBus_package_packge_type_size
};

void bambubus_init(void);
void bambubus_heartbeat_seen_fast(void);
extern bambubus_package_type bambubus_run();
uint8_t bambubus_ams_num(void);
uint8_t bambubus_local_ams_index(void);
bool bambubus_registered(void);
bool bambubus_arrange_wait(void);
