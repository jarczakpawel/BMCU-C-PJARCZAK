#include "MC_PULL_calibration.h"
#include "ws2812.h"

#include "Flash_saves.h"
#include "Motion_control.h"
#include "ams_addressing.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "ahub_bus.h"
#include "bambu_bus_ams.h"
#include "ADC_DMA.h"
#include "Debug_log.h"
#include <string.h>

WS2812_class SYS_RGB;
WS2812_class RGBOUT[4];

void RGB_init()
{
    SYS_RGB.init(1, GPIOD, GPIO_Pin_1);
    RGBOUT[0].init(2, GPIOA, GPIO_Pin_11);
    RGBOUT[1].init(2, GPIOA, GPIO_Pin_8);
    RGBOUT[2].init(2, GPIOB, GPIO_Pin_1);
    RGBOUT[3].init(2, GPIOB, GPIO_Pin_0);
}

void RGB_update()
{
    if (!(SYS_RGB.is_dirty() ||
          RGBOUT[0].is_dirty() || RGBOUT[1].is_dirty() ||
          RGBOUT[2].is_dirty() || RGBOUT[3].is_dirty()))
        return;

    if (!bus_background_ready()) return;

    static uint32_t last = 0u;

    uint32_t min_gap = time_hw_tpms;
    if (!min_gap) min_gap = 1u;

    const uint32_t now = time_ticks32();
    if (last != 0u && (uint32_t)(now - last) < min_gap)
        return;

    last = now;

    SYS_RGB.updata();
    RGBOUT[0].updata();
    RGBOUT[1].updata();
    RGBOUT[2].updata();
    RGBOUT[3].updata();
}

static uint8_t g_fil_dirty = 0;
static uint8_t g_loaded_ch = 0xFF;
static uint8_t g_state_dirty = 0;
static uint8_t g_state_write_value = 0xFF;
static uint8_t g_loaded_flash_pending = 0;

static inline void ram_to_flashinfo(uint8_t fil, Flash_FilamentInfo* o)
{
    const _filament* f = &ams[BMCU_LOCAL_AMS_INDEX].filament[fil];

    memcpy(o->bambubus_filament_id, f->bambubus_filament_id, sizeof(o->bambubus_filament_id));
    o->color_R = f->color_R;
    o->color_G = f->color_G;
    o->color_B = f->color_B;
    o->color_A = f->color_A;
    o->temperature_min = f->temperature_min;
    o->temperature_max = f->temperature_max;
    memcpy(o->name, f->name, sizeof(o->name));
}

static inline void flashinfo_to_ram(uint8_t fil, const Flash_FilamentInfo* i)
{
    _filament* f = &ams[BMCU_LOCAL_AMS_INDEX].filament[fil];

    memcpy(f->bambubus_filament_id, i->bambubus_filament_id, sizeof(i->bambubus_filament_id));
    f->color_R = i->color_R;
    f->color_G = i->color_G;
    f->color_B = i->color_B;
    f->color_A = i->color_A;
    f->temperature_min = i->temperature_min;
    f->temperature_max = i->temperature_max;

    memset(f->name, 0, sizeof(f->name));
    memcpy(f->name, i->name, sizeof(i->name));
    f->name[sizeof(f->name) - 1u] = 0;
}

bool ams_datas_read()
{
    bool any = false;

    for (uint8_t fil = 0; fil < 4u; fil++)
    {
        Flash_FilamentInfo fi;
        if (Flash_AMS_filament_read(fil, &fi))
        {
            flashinfo_to_ram(fil, &fi);
            any = true;
        }
    }

    return any;
}

void ams_datas_set_need_to_save()
{
    g_fil_dirty = 0x0Fu;
}

void ams_datas_set_need_to_save_filament(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return;
    g_fil_dirty |= (uint8_t)(1u << filament_idx);
}

void ams_state_claim_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu && g_loaded_ch != filament_ch) return;

    if (g_loaded_ch == 0xFFu)
    {
        g_loaded_ch = filament_ch;
        g_loaded_flash_pending = 1u;
    }
}

void ams_state_set_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu && g_loaded_ch != filament_ch) return;

    if (g_loaded_ch == 0xFFu)
    {
        g_loaded_ch = filament_ch;
        g_loaded_flash_pending = 1u;
    }

    if (!g_loaded_flash_pending) return;

    g_state_write_value = filament_ch;
    g_state_dirty = 1u;
    g_loaded_flash_pending = 0u;
}

void ams_state_set_unloaded(uint8_t filament_ch)
{
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
    g_loaded_flash_pending = 0u;
    g_state_write_value = 0xFFu;
    g_state_dirty = 1u;
}

uint8_t ams_state_get_loaded(void)
{
    return g_loaded_ch;
}

static void ams_state_save_run()
{
    if (!g_state_dirty) return;

    if (Flash_AMS_state_write(g_state_write_value))
        g_state_dirty = 0u;
}

void ams_datas_save_run()
{
    if (!g_fil_dirty) return;

    uint8_t fil = 0xFFu;
    for (uint8_t i = 0; i < 4u; i++)
    {
        if (g_fil_dirty & (uint8_t)(1u << i))
        {
            fil = i;
            break;
        }
    }

    if (fil == 0xFFu) return;

    Flash_FilamentInfo now;
    ram_to_flashinfo(fil, &now);

    if (Flash_AMS_filament_write(fil, &now))
        g_fil_dirty &= (uint8_t)~(1u << fil);
}

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    time_hw_init();

    __enable_irq();

    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

    RGB_init();
    delay(10);

    SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
    for (int i = 0; i < 4; i++) RGBOUT[i].set_RGB(0, 0, 0, 0);
    RGB_update();
    delay(50);

    DEBUG_init();
    ams_init();
    Flash_saves_init();

    ADC_DMA_init();
    ADC_DMA_wait_full();

    MC_PULL_calibration_boot();
    ams_datas_read();

    {
        uint8_t ch = 0xFFu;
        if (Flash_AMS_state_read(&ch))
        {
            g_loaded_ch = ch;

            if (ch < 4u)
            {
                _ams* a = &ams[BMCU_LOCAL_AMS_INDEX];

                a->now_filament_num  = ch;
                a->filament_use_flag = 0x04;
                a->pressure          = 0x2B00;

                for (uint8_t i = 0; i < 4u; i++)
                    a->filament[i].motion = _filament_motion::idle;

                a->filament[ch].motion = _filament_motion::on_use;
            }
        }
    }

    Motion_control_init();
    if (g_loaded_ch < 4u && (!Motion_control_filament_present(g_loaded_ch) ||
        !MC_PULL_calibration_is_valid(g_loaded_ch))) {
        _ams* a = &ams[BMCU_LOCAL_AMS_INDEX];
        a->filament[g_loaded_ch].motion = _filament_motion::idle;
        a->now_filament_num = 0xFFu;
        a->filament_use_flag = 0u;
        a->pressure = 0xF9C6u;
        ams_state_set_unloaded(g_loaded_ch);
    }
    bambubus_init();
    bus_init();

    DEBUG("START\n");

    while (1)
    {
        bus_rx_poll();
        const ahubus_package_type   ahub_stu     = ahubus_run();
        const bambubus_package_type bambubus_stu = bambubus_run();
        bus_port_to_host.send_package();

        if (bambubus_stu != bambubus_package_type::none && bambubus_stu != bambubus_package_type::error)
            bus_host_device_type = host_device_type_ams;
        else if (ahub_stu != ahubus_package_type::none && ahub_stu != ahubus_package_type::error)
            bus_host_device_type = host_device_type_ahub;

        const int error = bus_host_device_type == host_device_type_ams
            ? (bambubus_stu == bambubus_package_type::error ? -1 : 0)
            : bus_host_device_type == host_device_type_ahub
                ? (ahub_stu == ahubus_package_type::error ? -1 : 0) : -1;
        if (bambubus_arrange_wait()) SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
        else if (error) SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
        else SYS_RGB.set_RGB(0x38, 0x35, 0x32, 0);

        if (g_state_dirty && bus_background_ready()) ams_state_save_run();
        if (g_fil_dirty && bus_background_ready()) ams_datas_save_run();

        Motion_control_run(error);
        Flash_background_run();
        RGB_update();
    }
}
