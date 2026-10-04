#include "ahub_bus.h"

#include <string.h>

#include "ch32v20x_rcc.h"
#include "ch32v20x_crc.h"
#include "hal/irq_wch.h"
#include "hal/time_hw.h"
#include "app_api.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "crc_bus.h"
#include "bambu_bus_ams.h"
#include "ams_addressing.h"

typedef uint32_t u32_alias __attribute__((may_alias));

static inline bool ahub_bus_to_local(uint8_t bus_ams, uint8_t *local_ams)
{
    if (!local_ams || bus_ams >= ams_max_number) return false;
#if BMCU_AMS_AUTO
    if (!bambubus_registered() || bambubus_arrange_wait() || bus_ams != bambubus_ams_num()) return false;
    *local_ams = bambubus_local_ams_index();
#else
    *local_ams = bus_ams;
#endif
    return true;
}

static inline uint8_t ahub_local_to_bus(uint8_t local_ams)
{
#if BMCU_AMS_AUTO
    return (local_ams == bambubus_local_ams_index()) ? bambubus_ams_num() : local_ams;
#else
    return local_ams;
#endif
}

#define ahubus_map_port_adr_to_index(port, adr) (((uint8_t)port << 4) + ((uint8_t)adr >> 2))

ahubus_package_type ahubus_get_package_type(uint8_t *package_recv_buf, int length)
{
    if (!package_recv_buf || length < 12 || length > 1280 ||
        ((int)package_recv_buf[2] + 3) * 4 != length) return ahubus_package_type::none;
    if (package_recv_buf[0] != 0x33) return ahubus_package_type::none;

    const uint32_t words = (uint32_t)package_recv_buf[2] + 2u;
    const u32_alias *w = (const u32_alias *)package_recv_buf;

    CRC->CTLR = 1;
    for (uint32_t i = 0; i < words; i++)
        CRC->DATAR = w[i];

    if ((uint32_t)CRC->DATAR != (uint32_t)w[words])
        return ahubus_package_type::none;

    return (ahubus_package_type)package_recv_buf[4];
}
void ahubus_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}

int ahubus_package_add_crc(uint8_t *buf)
{
    if (!buf || ((uint32_t)buf[2] + 3u) * 4u > 1280u) return 0;

    const uint32_t words = (uint32_t)buf[2] + 2u;
    u32_alias *w = (u32_alias *)buf;

    buf[3] = bus_crc8(buf, 3);

    CRC->CTLR = 1;
    for (uint32_t i = 0; i < words; i++)
        CRC->DATAR = w[i];

    w[words] = (uint32_t)CRC->DATAR;
    return (int)((words + 1u) << 2);
}

void ahubus_slave_get_package_heartbeat(uint8_t *buf, int length)
{
    if (!buf || length < 12) return;
#if BMCU_AMS_AUTO
    if (!bambubus_registered() || bambubus_arrange_wait()) return;
#endif
    if (bus_port_to_host.send_data_len != 0) return;
    uint8_t* out = bus_port_to_host.tx_build_buf();

    uint8_t ahubus_ams_numbers = 0;
    out[0] = 0x33;
    out[1] = buf[1];
    out[4] = 0x01;

    uint8_t *EQPT_data_ptr = out + 6;

    for (uint8_t i = 0; i < ams_max_number; i++)
    {
        if (ams[i].online == true)
        {
            EQPT_data_ptr[0] = (uint8_t)(ahub_local_to_bus(i) << 2);
            EQPT_data_ptr[1] = ams[i].ams_type;
            EQPT_data_ptr += 2;
            ahubus_ams_numbers++;
        }
    }

    out[5] = ahubus_ams_numbers;
    if ((ahubus_ams_numbers & 0x01) == 0)
    {
        EQPT_data_ptr[0] = 0x00;
        EQPT_data_ptr[1] = 0x00;
        EQPT_data_ptr += 2;
    }

    out[2] = ((EQPT_data_ptr - out) >> 2) - 2;
    bus_port_to_host.send_data_len = ahubus_package_add_crc(out);
}

enum class ahubus_query_type : uint8_t
{
    ams_name = 0x01,
    filament_info = 0x02,
    filament_stu = 0x04,
    dryer_stu = 0x05,
    all_filament_stu = 0x06,
};

struct ahubus_package_query_head
{
    uint8_t magic_byte;
    uint8_t flag;
    uint8_t length;
    uint8_t crc8;
    uint8_t command;
    ahubus_query_type query_type;
    uint8_t query_adr;
    uint8_t data_struct_count;
    uint8_t data[0];
} __attribute__((packed));

const ahubus_package_query_head ahubus_host_package_query_init = {
    .magic_byte = 0x33,
    .flag = 0x80,
    .command = 0x02,
};

static inline __attribute__((always_inline)) void ahub_pack_filament_stu8(uint8_t* dst, const _filament* f)
{
    uint8_t m = (uint8_t)f->motion;
    if (f->online) m |= 0x80u;

    dst[0] = m;
    dst[1] = f->seal_status;
    dst[2] = (uint8_t)f->compartment_temperature;
    dst[3] = f->compartment_humidity;
    dst[4] = f->dryer_power;
    dst[5] = (uint8_t)f->dryer_temperature;

    uint16_t t;
    memcpy(&t, &f->dryer_time_left, sizeof(t));
    dst[6] = (uint8_t)(t & 0xFFu);
    dst[7] = (uint8_t)(t >> 8);
}

void ahubus_slave_get_package_query(uint8_t *buf, int length)
{
    if (!buf || length < 12) return;
    if (bus_port_to_host.send_data_len != 0) return;

    uint8_t* out = bus_port_to_host.tx_build_buf();

    const ahubus_query_type query_type = (ahubus_query_type)buf[5];
    const uint8_t query_adr_raw = buf[6];
    uint8_t query_adr = query_adr_raw;

    memcpy(out, &ahubus_host_package_query_init, sizeof(ahubus_package_query_head));
    out[5] = (uint8_t)query_type;
    out[6] = query_adr_raw;

    uint8_t *data_ptr = out + 4;

#ifdef xMCU
    query_adr = (uint8_t)(query_adr >> 4);
#endif

    uint8_t local_query_adr = query_adr;
    if (query_type != ahubus_query_type::all_filament_stu)
    {
        if (!ahub_bus_to_local(query_adr, &local_query_adr)) return;
    }
#if BMCU_AMS_AUTO
    else if (!bambubus_registered() || bambubus_arrange_wait()) return;
#endif

    switch (query_type)
    {
    case ahubus_query_type::ams_name:
        memcpy(data_ptr + 4, ams[local_query_adr].name, 8);
        out[2] = 2;
        out[7] = 0x01;
        break;

    case ahubus_query_type::filament_info:
        memcpy(data_ptr + 4,   ams[local_query_adr].filament[0].bambubus_filament_id, 44);
        memcpy(data_ptr + 48,  ams[local_query_adr].filament[1].bambubus_filament_id, 44);
        memcpy(data_ptr + 92,  ams[local_query_adr].filament[2].bambubus_filament_id, 44);
        memcpy(data_ptr + 136, ams[local_query_adr].filament[3].bambubus_filament_id, 44);
        out[2] = 44;
        out[7] = 0x01;
        break;

    case ahubus_query_type::filament_stu:
    {
        bus_now_ams_num = local_query_adr;
        ahub_pack_filament_stu8(data_ptr + 4,  &ams[local_query_adr].filament[0]);
        ahub_pack_filament_stu8(data_ptr + 12, &ams[local_query_adr].filament[1]);
        ahub_pack_filament_stu8(data_ptr + 20, &ams[local_query_adr].filament[2]);
        ahub_pack_filament_stu8(data_ptr + 28, &ams[local_query_adr].filament[3]);
        out[2] = 8;
        out[7] = 0x01;
        break;
    }

    case ahubus_query_type::dryer_stu:
        memcpy(data_ptr + 4,  &(ams[local_query_adr].filament[0].dryer_power), 4);
        memcpy(data_ptr + 8,  &(ams[local_query_adr].filament[1].dryer_power), 4);
        memcpy(data_ptr + 12, &(ams[local_query_adr].filament[2].dryer_power), 4);
        memcpy(data_ptr + 16, &(ams[local_query_adr].filament[3].dryer_power), 4);
        out[2] = 4;
        out[7] = 0x01;
        break;

    case ahubus_query_type::all_filament_stu:
    {
        uint8_t *ams_filament_data_ptr = data_ptr + 4;
        uint8_t ams_data_count = 0;

        for (uint8_t i = 0; i < ams_max_number; i++)
        {
            if (!ams[i].online) continue;

#ifdef xMCU
            ams_filament_data_ptr[0] = (uint8_t)(ahub_local_to_bus(i) << 4);
#else
            ams_filament_data_ptr[0] = ahub_local_to_bus(i);
#endif
            ams_filament_data_ptr[1] = ams[i].online;

            ams_filament_data_ptr[2] = (uint8_t)ams[i].filament[0].motion;
            ams_filament_data_ptr[3] = (uint8_t)ams[i].filament[0].seal_status;
            ams_filament_data_ptr[4] = (uint8_t)ams[i].filament[1].motion;
            ams_filament_data_ptr[5] = (uint8_t)ams[i].filament[1].seal_status;
            ams_filament_data_ptr[6] = (uint8_t)ams[i].filament[2].motion;
            ams_filament_data_ptr[7] = (uint8_t)ams[i].filament[2].seal_status;
            ams_filament_data_ptr[8] = (uint8_t)ams[i].filament[3].motion;
            ams_filament_data_ptr[9] = (uint8_t)ams[i].filament[3].seal_status;

            if (ams[i].filament[0].online) ams_filament_data_ptr[2] |= 0x80u;
            if (ams[i].filament[1].online) ams_filament_data_ptr[4] |= 0x80u;
            if (ams[i].filament[2].online) ams_filament_data_ptr[6] |= 0x80u;
            if (ams[i].filament[3].online) ams_filament_data_ptr[8] |= 0x80u;

            ams_filament_data_ptr += 10;
            ams_data_count++;
        }

        while (((uintptr_t)(ams_filament_data_ptr - data_ptr) & 3u) != 0u) {
            *ams_filament_data_ptr++ = 0x00;
        }

        out[2] = (uint8_t)(((ams_filament_data_ptr - data_ptr) >> 2) - 1u);
        out[7] = ams_data_count;
        break;
    }

    default:
        return;
    }

    bus_port_to_host.send_data_len = ahubus_package_add_crc(out);
}


struct ahubus_sync_req_list_struct
{
    uint8_t ams_num;
    uint8_t filament_channel;
    ahubus_set_type type;
};

struct ahubus_package_set_head
{
    uint8_t magic_byte;
    uint8_t flag;
    uint8_t length;
    uint8_t crc8;
    uint8_t command;
    uint8_t set_type;
    uint8_t set_adr;
    uint8_t data_struct_count;
    uint8_t data[0];
} __attribute__((packed));

const ahubus_package_set_head ahubus_host_package_set_init = {
    .magic_byte = 0x33,
    .flag = 0x80,
    .command = 0x03,
};

void ahubus_slave_get_package_set(uint8_t *buf, int length)
{
    if (!buf || length < 12) return;
    if (bus_port_to_host.send_data_len != 0) return;

    uint8_t* out = bus_port_to_host.tx_build_buf();

    const uint8_t set_type_raw = buf[5];
    const uint8_t set_adr_raw  = buf[6];

    ahubus_set_type set_type = (ahubus_set_type)set_type_raw;
    uint8_t set_adr = set_adr_raw;

    uint8_t *data_ptr = buf + 4;

#ifdef xMCU
    set_adr = (uint8_t)(set_adr >> 4);
#endif

    uint8_t local_set_adr = 0u;
    if (set_type != ahubus_set_type::all_filament_stu)
    {
        if (!ahub_bus_to_local(set_adr, &local_set_adr)) return;
    }
#if BMCU_AMS_AUTO
    else if (!bambubus_registered() || bambubus_arrange_wait()) return;
#endif

    switch (set_type)
    {
    case ahubus_set_type::filament_info:
    {
        if (length < 57) return;
        const uint8_t filament_channel = data_ptr[48];
        if (filament_channel >= 4) return;
        memcpy(&(ams[local_set_adr].filament[filament_channel].bambubus_filament_id), data_ptr + 4, 44);

        if (set_adr == bambubus_ams_num())
            ams_datas_set_need_to_save_filament(filament_channel);

        break;
    }
    case ahubus_set_type::dryer_stu:
    {
        if (length < 17) return;
        const uint8_t dryer_channel = data_ptr[8];
        if (dryer_channel >= 4) return;
        memcpy(&(ams[local_set_adr].filament[dryer_channel].dryer_power), data_ptr + 4, 4);
        break;
    }
    case ahubus_set_type::all_filament_stu:
    {
        const uint8_t data_struct_count = buf[7];
        if (data_struct_count > (length - 12) / 6) return;
        for (uint8_t i = 0; i < data_struct_count; i++) {
            const uint8_t channel = buf[9u + (uint32_t)i * 6u];
            if (channel >= 4u && channel != 0xFFu) return;
        }
        uint8_t *data_struct_ptr = data_ptr + 4;

        for (uint8_t i = 0; i < data_struct_count; i++)
        {
            uint8_t ams_adr = data_struct_ptr[0];
#ifdef xMCU
            ams_adr = (uint8_t)(ams_adr >> 4);
#endif
            uint8_t local_ams_adr = 0u;
            if (!ahub_bus_to_local(ams_adr, &local_ams_adr)) { data_struct_ptr += 6; continue; }

            ams[local_ams_adr].now_filament_num = data_struct_ptr[1];
            ams[local_ams_adr].filament[0].motion = (_filament_motion)(data_struct_ptr[2] & 0x7Fu);
            ams[local_ams_adr].filament[1].motion = (_filament_motion)(data_struct_ptr[3] & 0x7Fu);
            ams[local_ams_adr].filament[2].motion = (_filament_motion)(data_struct_ptr[4] & 0x7Fu);
            ams[local_ams_adr].filament[3].motion = (_filament_motion)(data_struct_ptr[5] & 0x7Fu);

            data_struct_ptr += 6;
        }
        break;
    }
    default:
        return;
    }

    memcpy(out, &ahubus_host_package_set_init, sizeof(ahubus_package_set_head));
    out[5] = set_type_raw;
    out[6] = set_adr_raw;
    out[7] = 0;
    out[2] = 0;

    bus_port_to_host.send_data_len = ahubus_package_add_crc(out);
}

ahubus_package_type ahubus_run()
{
    ahubus_package_type package_type = ahubus_package_type::none;

    static uint32_t deadline = 0;
    const uint32_t now = time_ticks32();

    int rx_len = 0;
    _bus_data_type t = _bus_data_type::none;
    uint8_t *buf = nullptr;

    {
        const uint32_t s = irq_save_wch();
        rx_len = bus_port_to_host.recv_data_len;
        t      = bus_port_to_host.bus_package_type;
        buf    = bus_port_to_host.bus_recv_data_ptr;
        irq_restore_wch(s);
    }

    if (rx_len > 0 && t == _bus_data_type::ahub_bus && bus_port_to_host.send_data_len == 0)
    {
        if (buf != nullptr && rx_len >= 12 && rx_len <= 1280 && buf[0] == 0x33 &&
            (((int)buf[2] + 3) * 4) == rx_len)
        {
            package_type = ahubus_get_package_type(buf, rx_len);

            switch (package_type)
            {
            case ahubus_package_type::heartbeat:
                ahubus_slave_get_package_heartbeat(buf, rx_len);
                deadline = now + ms_to_ticks32(1000u);
                break;

            case ahubus_package_type::query:
                ahubus_slave_get_package_query(buf, rx_len);
                break;

            case ahubus_package_type::set:
                ahubus_slave_get_package_set(buf, rx_len);
                break;

            default:
                break;
            }
        }

        {
            const uint32_t s = irq_save_wch();
            bus_port_to_host.release_packet();
            irq_restore_wch(s);
        }
    }

    if ((int32_t)(now - deadline) > 0)
        package_type = ahubus_package_type::error;

    return package_type;
}

