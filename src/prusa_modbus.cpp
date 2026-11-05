#include "prusa_modbus.h"
#include "_bus_hardware.h"
#include "Motion_control.h"
#include "app_api.h"
#include "hal/time_hw.h"
#include <stddef.h>

#ifndef BMCU_PRUSA_NODE
#define BMCU_PRUSA_NODE 0
#endif
static_assert(BMCU_PRUSA_NODE >= 0 && BMCU_PRUSA_NODE < 4);

namespace {
constexpr uint8_t unit = 0xE0u + BMCU_PRUSA_NODE;
constexpr uint16_t registers = 12;
uint8_t request[8];
uint8_t used = 0;
uint32_t last_rx = 0;
uint32_t reply_at = 0;
bool reply_pending = false;
uint8_t response[3 + registers * 2 + 2] __attribute__((aligned(4)));

uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 1u) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001u) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

uint16_t read_register(uint16_t index) {
    switch (index) {
    case 0: return 0xB4C3;
    case 1: return 1;
    case 2: return BMCU_PRUSA_NODE;
    case 3: return 4;
    case 4: return ams_state_get_loaded();
    case 5: {
        uint16_t present = 0;
        for (uint8_t i = 0; i < 4; ++i)
            if (Motion_control_filament_present(i)) present |= static_cast<uint16_t>(1u << i);
        return present;
    }
    default: {
        const volatile uint8_t *uid = reinterpret_cast<const volatile uint8_t *>(0x1FFFF7E8u);
        const uint8_t offset = static_cast<uint8_t>((index - 6u) * 2u);
        return static_cast<uint16_t>((static_cast<uint16_t>(uid[offset]) << 8) | uid[offset + 1u]);
    }
    }
}
}

void prusa_modbus_rx(uint8_t data) {
    const uint32_t now = time_ticks32();
    if (used && static_cast<uint32_t>(now - last_rx) > us_to_ticks32(1000u)) used = 0;
    last_rx = now;
    if (reply_pending) return;
    if (!used) {
        if (data != unit) return;
        request[used++] = data;
        return;
    }
    request[used++] = data;
    if (used == 2 && request[1] != 4u) {
        used = 0;
        return;
    }
    if (used != sizeof(request)) return;
    used = 0;

    const uint16_t crc = crc16(request, 6);
    if (request[6] != static_cast<uint8_t>(crc) || request[7] != static_cast<uint8_t>(crc >> 8)) return;
    const uint16_t start = static_cast<uint16_t>((request[2] << 8) | request[3]);
    const uint16_t count = static_cast<uint16_t>((request[4] << 8) | request[5]);
    if (!count || count > registers || start >= registers || count > registers - start) return;

    response[0] = unit;
    response[1] = 4u;
    response[2] = static_cast<uint8_t>(count * 2u);
    for (uint16_t i = 0; i < count; ++i) {
        const uint16_t value = read_register(start + i);
        response[3u + 2u * i] = static_cast<uint8_t>(value >> 8);
        response[4u + 2u * i] = static_cast<uint8_t>(value);
    }
    const uint16_t length = static_cast<uint16_t>(3u + 2u * count);
    const uint16_t reply_crc = crc16(response, length);
    response[length] = static_cast<uint8_t>(reply_crc);
    response[length + 1u] = static_cast<uint8_t>(reply_crc >> 8);
    reply_at = now + us_to_ticks32(200u);
    reply_pending = true;
}

void prusa_modbus_run() {
    if (!reply_pending || !time_reached32(time_ticks32(), reply_at)) return;
    if (bus_uart1_dma_send(response, static_cast<uint16_t>(5u + response[2])))
        reply_pending = false;
}
