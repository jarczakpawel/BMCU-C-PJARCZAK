#include "prusa_modbus.h"
#ifdef BMCU_PRUSA
#include "_bus_hardware.h"
#include "Motion_control.h"
#include "ams.h"
#include "Flash_saves.h"
#include "MC_PULL_calibration.h"
#include "app_api.h"
#include "hal/time_hw.h"
#include <string.h>

namespace {
constexpr uint8_t discovery_address = 0xF0u;
constexpr uint16_t input_registers = 30u;
uint8_t address = 0u;
uint8_t request[20] {};
uint8_t received = 0u;
uint8_t expected = 0u;
uint8_t response[3u + input_registers * 2u + 2u] __attribute__((aligned(4)));
uint8_t response_length = 0u;
uint32_t last_rx = 0u;
uint32_t reply_at = 0u;
uint32_t calibrate_at = 0u;
uint8_t calibration_state = 0u;
bool pending = false;
bool calibrate_pending = false;
bool motor_calibrate_pending = false;
uint8_t attention_channel = 0xFFu;

uint16_t crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit)
            crc = (crc & 1u) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001u) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

void uid_bytes(uint8_t *out) {
    const volatile uint8_t *uid = reinterpret_cast<const volatile uint8_t *>(0x1FFFF7E8u);
    for (uint8_t i = 0; i < 12u; ++i) out[i] = uid[i];
}

void finish_response(uint8_t size, uint32_t delay_us) {
    const uint16_t crc = crc16(response, size);
    response[size++] = static_cast<uint8_t>(crc);
    response[size++] = static_cast<uint8_t>(crc >> 8);
    response_length = size;
    reply_at = time_ticks32() + us_to_ticks32(delay_us);
    pending = true;
}

void exception(uint8_t function, uint8_t code) {
    response[0] = address;
    response[1] = static_cast<uint8_t>(function | 0x80u);
    response[2] = code;
    finish_response(3u, 200u);
}

uint16_t read_register(uint16_t index) {
    switch (index) {
    case 0u: return 0xB4CCu;
    case 1u: return 10u;
    case 2u: return address ? static_cast<uint16_t>(address - 0xE0u) : 0xFFFFu;
    case 3u: return 4u;
    case 4u: return ams_state_get_loaded();
    case 5u: {
        uint16_t bits = 0u;
        for (uint8_t ch = 0; ch < 4u; ++ch)
            if (Motion_control_filament_present(ch)) bits |= static_cast<uint16_t>(1u << ch);
        return bits;
    }
    case 12u: return Motion_control_prusa_retract_mm();
    case 13u: return Motion_control_prusa_led_rgb565();
    case 14u: return Flash_Prusa_settings_status();
    case 15u: return calibration_state;
    case 16u: return Motion_control_prusa_phase();
    case 17u: return Motion_control_prusa_calibration_mask();
    case 22u: return ams_datas_dirty_mask();
    case 23u: return ams_state_prusa_save_pending();
    case 24u: return Motion_control_prusa_switches();
    case 25u: return attention_channel;
    case 26u: return Motion_control_prusa_stop_status();
    case 27u: return Motion_control_prusa_armed_channel();
    case 28u: return Motion_control_prusa_motion_channel();
    case 29u: return Motion_control_prusa_motor_ready_mask();
    default: {
        if (index >= 18u) {
            const auto &fil = ams[BMCU_LOCAL_AMS_INDEX].filament[index - 18u];
            return static_cast<uint16_t>(((fil.color_R >> 3) << 11u) |
                ((fil.color_G >> 2) << 5u) | (fil.color_B >> 3));
        }
        const volatile uint8_t *uid = reinterpret_cast<const volatile uint8_t *>(0x1FFFF7E8u);
        const uint8_t pos = static_cast<uint8_t>((index - 6u) * 2u);
        return static_cast<uint16_t>((static_cast<uint16_t>(uid[pos]) << 8) | uid[pos + 1u]);
    }
    }
}

void process_frame() {
    const uint8_t fn = request[1];
    const uint16_t frame_crc = crc16(request, expected - 2u);
    if (request[expected - 2u] != static_cast<uint8_t>(frame_crc) ||
        request[expected - 1u] != static_cast<uint8_t>(frame_crc >> 8)) return;
    Motion_control_prusa_bus_heartbeat();

    if (request[0] == discovery_address) {
        uint8_t uid[12];
        uid_bytes(uid);
        if (fn == 0x41u) {
            if (address || request[2] > 96u) return;
            const uint8_t bits = request[2];
            const uint8_t whole = bits / 8u;
            const uint8_t partial = bits % 8u;
            if (memcmp(uid, request + 3u, whole)) return;
            if (partial && ((uid[whole] ^ request[3u + whole]) & (0xFFu << (8u - partial)))) return;
            uint32_t value = 0x811C9DC5u;
            for (uint8_t b : uid) value = (value ^ b) * 0x01000193u;
            value ^= static_cast<uint32_t>(request[15]) << 8u | request[16];
            value ^= value >> 16;
            value *= 0x7FEB352Du;
            value ^= value >> 15;
            value *= 0x846CA68Bu;
            value ^= value >> 16;
            response[0] = discovery_address;
            response[1] = fn;
            memcpy(response + 2u, uid, 12u);
            finish_response(14u, 1000u + (value & 127u) * 850u);
            return;
        }
        if (fn == 0x42u || fn == 0x43u) {
            if (memcmp(uid, request + 2u, 12u)) return;
            if (fn == 0x42u && (request[14] < 0xE0u || request[14] > 0xE3u)) return;
            address = fn == 0x42u ? request[14] : 0u;
            response[0] = discovery_address;
            response[1] = fn;
            memcpy(response + 2u, uid, 12u);
            response[14] = address;
            finish_response(15u, 200u);
        }
        return;
    }

    if (!address || request[0] != address) return;
    if (fn == 4u) {
        const uint16_t first = static_cast<uint16_t>(request[2] << 8u | request[3]);
        const uint16_t count = static_cast<uint16_t>(request[4] << 8u | request[5]);
        if (!count || count > input_registers || first >= input_registers || count > input_registers - first) {
            exception(fn, 2u);
            return;
        }
        response[0] = address;
        response[1] = fn;
        response[2] = static_cast<uint8_t>(count * 2u);
        for (uint16_t i = 0; i < count; ++i) {
            const uint16_t value = read_register(first + i);
            response[3u + i * 2u] = static_cast<uint8_t>(value >> 8);
            response[4u + i * 2u] = static_cast<uint8_t>(value);
        }
        finish_response(static_cast<uint8_t>(3u + count * 2u), 200u);
    } else if (fn == 16u) {
        const uint16_t reg = static_cast<uint16_t>(request[2] << 8u | request[3]);
        const uint16_t count = static_cast<uint16_t>(request[4] << 8u | request[5]);
        if (count != 1u || request[6] != 2u) {
            exception(fn, 3u);
            return;
        }
        const uint16_t value = static_cast<uint16_t>(request[7] << 8u | request[8]);
        bool accepted = false;
        if (reg == 0x100u || reg == 0x101u) {
            const uint16_t mm = reg == 0x100u ? value : Motion_control_prusa_retract_mm();
            const uint16_t rgb = reg == 0x101u ? value : Motion_control_prusa_led_rgb565();
            accepted = Motion_control_prusa_settings_save(mm, rgb);
        } else if (reg >= 0x110u && reg < 0x114u &&
            calibration_state != 1u && !calibrate_pending &&
            Motion_control_prusa_phase() != 1u && Motion_control_prusa_phase() != 2u) {
            const uint8_t ch = static_cast<uint8_t>(reg - 0x110u);
            auto &fil = ams[BMCU_LOCAL_AMS_INDEX].filament[ch];
            fil.color_R = static_cast<uint8_t>(((value >> 11u) & 31u) * 255u / 31u);
            fil.color_G = static_cast<uint8_t>(((value >> 5u) & 63u) * 255u / 63u);
            fil.color_B = static_cast<uint8_t>((value & 31u) * 255u / 31u);
            ams_datas_set_need_to_save_filament(ch);
            accepted = true;
        } else if (reg == 0x106u) {
            accepted = Motion_control_prusa_command(value);
        } else if (reg == 0x105u && (value < 4u || value == 0xFFu)) {
            accepted = Motion_control_prusa_arm(static_cast<uint8_t>(value));
        } else if (reg == 0x104u && (value < 4u || value == 0xFFu)) {
            attention_channel = static_cast<uint8_t>(value);
            accepted = true;
        } else if (reg == 0x107u && value == 0xD1CAu) {
            accepted = !calibrate_pending && calibration_state != 1u &&
                Flash_background_idle() && Motion_control_prusa_ready_for_settings() &&
                Motion_control_prusa_armed_channel() == 0xFFu &&
                Motion_control_prusa_phase() != 2u;
            if (accepted) {
                calibration_state = 1u;
                motor_calibrate_pending = true;
                calibrate_pending = true;
                calibrate_at = time_ticks32() + ms_to_ticks32(300u);
            }
        } else if (reg == 0x103u && value == 0xC1EAu &&
            Motion_control_prusa_phase() != 1u && Motion_control_prusa_phase() != 2u) {
            accepted = ams_state_prusa_clear_loaded();
        } else if (reg == 0x102u && value == 0xCA1Bu) {
            accepted = !calibrate_pending && calibration_state != 1u &&
                Flash_background_idle() && Motion_control_prusa_ready_for_settings();
            if (accepted) {
                calibration_state = 1u;
                calibrate_pending = true;
                calibrate_at = time_ticks32() + ms_to_ticks32(300u);
            }
        }
        if (!accepted) {
            exception(fn, 6u);
            return;
        }
        memcpy(response, request, 6u);
        finish_response(6u, 200u);
    }
}
}

void prusa_modbus_rx(uint8_t data) {
    const uint32_t now = time_ticks32();
    if (received && static_cast<uint32_t>(now - last_rx) > us_to_ticks32(1100u)) received = 0u;
    last_rx = now;
    if (!bus_port_to_host.idle) { received = 0u; return; }
    if (pending) {
        if (response[0] == discovery_address && response[1] == 0x41u)
            pending = false;
        else return;
    }
    if (calibrate_pending || calibration_state == 1u) return;
    if (!received) {
        if (data != discovery_address && (!address || data != address)) return;
        request[0] = data;
        received = 1u;
        expected = 0u;
        return;
    }
    if (received >= sizeof(request)) { received = 0u; return; }
    request[received++] = data;
    if (received == 2u) {
        expected = (data == 4u) ? 8u : (data == 16u ? 11u :
            (request[0] == discovery_address && data == 0x41u ? 19u :
             request[0] == discovery_address && (data == 0x42u || data == 0x43u) ? 17u : 0u));
        if (!expected) { received = 0u; return; }
    }
    if (expected && received == expected) {
        process_frame();
        received = 0u;
    }
}

void prusa_modbus_run() {
    if (!pending || !time_reached32(time_ticks32(), reply_at)) return;
    if (bus_uart1_dma_send(response, response_length)) pending = false;
}

bool prusa_modbus_bus_idle() {
    return !received && !pending && bus_port_to_host.idle &&
        static_cast<uint32_t>(time_ticks32() - last_rx) >= us_to_ticks32(900u);
}

void prusa_modbus_calibration_run() {
    if (!calibrate_pending || !time_reached32(time_ticks32(), calibrate_at)) return;
    if (!bus_background_ready()) return;
    calibrate_pending = false;
    const bool motor = motor_calibrate_pending;
    motor_calibrate_pending = false;
    const bool success = motor ? Motion_control_prusa_calibrate_directions() : MC_PULL_calibration_boot(true);
    if (!success && !motor) MC_PULL_calibration_boot(false, false);
    calibration_state = success ? 2u : 3u;
}

uint8_t prusa_modbus_attention_channel() { return attention_channel; }
#endif
