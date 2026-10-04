#pragma once
#include "crc_bus.h"
#include <string.h>

enum class _bus_data_type : uint8_t
{
    bambubus = 0x3D,
    ahub_bus = 0x33,
    none = 0x00
};

void bambubus_heartbeat_seen_fast(void);

class _bus_port_deal // 中断数据处理
{
public:
    uint8_t send_data_buf[1280] __attribute__((aligned(4)));

private:
    uint8_t tx_dma_buf[1280] __attribute__((aligned(4)));
    uint8_t recv_data_buf[3][1280] __attribute__((aligned(4)));
    uint8_t tx_build_sel = 0;
    uint8_t rx_read = 0, rx_write = 0, rx_count = 0;
    uint16_t rx_length[3] = {};
    _bus_data_type rx_type[3] = {};
    int _index = 0;
    int length = 999;
    uint8_t data_length_index = 0;
    uint8_t data_CRC8_index = 0;
    _bus_data_type irq_package_type = _bus_data_type::none;
    uint8_t *bus_irq_data_ptr = recv_data_buf[0];
    int heartbeat_drop_bytes = 0;
    bool (*port_send_datas)(uint8_t *data, uint16_t len);

public:
    uint8_t * volatile bus_recv_data_ptr = recv_data_buf[0];
    volatile int recv_data_len = 0;
    volatile int send_data_len = 0;
    volatile _bus_data_type bus_package_type = _bus_data_type::none;
    volatile bool idle = true;

    inline __attribute__((always_inline)) uint8_t* tx_build_buf()
    {
        return tx_build_sel ? tx_dma_buf : send_data_buf;
    }

    void init(bool (*_port_send_datas)(uint8_t *data, uint16_t len))
    {
        _index = 0;
        length = 999;
        data_length_index = 0;
        data_CRC8_index = 0;
        irq_package_type = _bus_data_type::none;
        bus_irq_data_ptr = recv_data_buf[0];
        heartbeat_drop_bytes = 0;
        bus_recv_data_ptr = recv_data_buf[0];
        rx_read = rx_write = rx_count = 0;
        idle = true;
        send_data_len = 0;
        recv_data_len = 0;
        tx_build_sel  = 0;
        port_send_datas = _port_send_datas;
    }

    uint32_t rx_dropped = 0;
    volatile uint32_t tx_errors = 0;

    void reset_rx()
    {
        _index = 0;
        heartbeat_drop_bytes = 0;
        irq_package_type = _bus_data_type::none;
    }

    bool receiving() const { return _index != 0 || heartbeat_drop_bytes != 0; }

    void release_packet()
    {
        if (rx_count) {
            rx_read = (uint8_t)((rx_read + 1u) % 3u);
            --rx_count;
        }
        bus_recv_data_ptr = recv_data_buf[rx_read];
        bus_package_type = rx_count ? rx_type[rx_read] : _bus_data_type::none;
        recv_data_len = rx_count ? rx_length[rx_read] : 0;
    }

    void irq(uint8_t data)
    {
        // Heartbeat
        if (__builtin_expect(heartbeat_drop_bytes > 0, 0))
        {
            if (--heartbeat_drop_bytes == 0)
                bambubus_heartbeat_seen_fast();
            return;
        }

        const int BUF_SZ = (int)sizeof(recv_data_buf[0]);
        int idx = _index;

        if (idx == 0)
        {
            if (data == 0x3D || data == 0x33)
            {
                bus_irq_data_ptr[0] = data;
                data_length_index = 4;
                length = data_CRC8_index = 6;
                _index = 1;
                irq_package_type = (_bus_data_type)data;
            }
            return;
        }

        if (idx < 0 || idx >= BUF_SZ)
        {
            _index = 0;
            return;
        }

        uint8_t *buf = bus_irq_data_ptr;
        buf[idx] = data;

        if (idx == 1)
        {
            if (data & 0x80)
            {
                data_length_index = 2;
                data_CRC8_index = 3;
            }
            else
            {
                data_CRC8_index = 6;
                data_length_index = (irq_package_type == _bus_data_type::bambubus) ? 5 : 4;
            }
        }

        if (idx == data_length_index)
        {
            if (irq_package_type == _bus_data_type::bambubus)
            {
                if (data_length_index == 2)
                    length = data;
                else
                    length = (int)buf[4] | ((int)data << 8);
            }
            else if (irq_package_type == _bus_data_type::ahub_bus)
            {
                length = (((int)data) << 2) + 12;
            }

            const int minimum = irq_package_type == _bus_data_type::ahub_bus ? 12 :
                (data_length_index == 2 ? 7 : 15);
            if (length < minimum || length > BUF_SZ)
            {
                _index = 0;
                return;
            }
        }

        if (irq_package_type == _bus_data_type::bambubus &&
            idx == 4 && data_length_index == 2 && length >= 6 &&
            buf[1] == 0xC5u && data == 0x20u)
        {
            const int remain = length - 5;
            _index = 0;
            irq_package_type = _bus_data_type::none;
            if (remain > 0)
                heartbeat_drop_bytes = remain;
            else
                bambubus_heartbeat_seen_fast();
            return;
        }

        if (idx == data_CRC8_index)
        {
            if (data != bus_crc8(buf, (uint32_t)data_CRC8_index))
            {
                _index = 0;
                return;
            }
        }

        ++idx;

        if (idx >= length)
        {
            _index = 0;

            if (rx_count < 2u)
            {
                rx_length[rx_write] = (uint16_t)length;
                rx_type[rx_write] = irq_package_type;
                if (rx_count == 0u) {
                    bus_recv_data_ptr = bus_irq_data_ptr;
                    bus_package_type = irq_package_type;
                    recv_data_len = length;
                }
                ++rx_count;
                rx_write = (uint8_t)((rx_write + 1u) % 3u);
                bus_irq_data_ptr = recv_data_buf[rx_write];
            }
            else ++rx_dropped;
            return;
        }

        _index = idx;
    }

    void send_package()
    {
        const int len = send_data_len;
        if (len > 0 && len <= 1280)
        {
            if (!idle) return;

            uint8_t *tx = tx_build_buf();
            if (port_send_datas(tx, (uint16_t)len)) {
                tx_build_sel ^= 1;
                send_data_len = 0;
            }
        }
    }

    void send_package(uint8_t *data, uint16_t len)
    {
        if (len > 0 && len <= 1280)
        {
            if (!idle) return;
            port_send_datas(data, len);
        }
    }
} __attribute__((aligned(4)));

extern _bus_port_deal bus_port_to_host;
extern void bus_init();
void bus_rx_poll();
bool bus_background_ready();
void bus_shutdown();

#define host_device_type_none 0x0000
#define host_device_type_ahub 0x0001
#define host_device_type_ams 0x0700
extern uint16_t bus_host_device_type;