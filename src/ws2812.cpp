#include "ws2812.h"
#include "hal/time_hw.h"
#include "hal/irq_wch.h"
#include "_bus_hardware.h"

// WS2812B timing (datasheet):
//  - TH+TL = 1.25us ±600ns
//  - T0H   = 0.4us  ±150ns
//  - T1H   = 0.85us ±150ns
//  - RES low >= 50us
//
// STK = HCLK/8. Przy HCLK=144MHz => STK=18MHz => 55.56ns/tick.
#define WS2812_TBIT_TICKS  (22u)  // 1.222us @18MHz
#define WS2812_T0H_TICKS   (7u)   // 0.389us @18MHz
#define WS2812_T1H_TICKS   (15u)  // 0.833us @18MHz

static uint32_t g_ws2812_rst_ticks = 0;

#define RGB_H(p, m) do{ (p)->BSHR = (uint32_t)(m); }while(0)
#define RGB_L(p, m) do{ (p)->BCR  = (uint32_t)(m); }while(0)

static inline void enable_gpio_clock(GPIO_TypeDef* p)
{
    if      (p == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (p == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (p == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (p == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

static inline __attribute__((always_inline)) void wait_until(uint32_t deadline)
{
    while (((uint32_t)(STK_CNTL - deadline) >> 31) != 0u) { }
}

#if BMCU_ONLINE_LED_FILAMENT_RGB
static inline __attribute__((always_inline)) uint8_t scale8_video(uint8_t v, uint8_t scale)
{
    return (uint8_t)(((uint16_t)v * (uint16_t)scale + 255u) >> 8);
}

static const uint8_t kGamma8[256] =
{
      0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,
      0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   1u,   1u,   1u,   1u,   1u,   1u,   1u,   2u,
      2u,   2u,   2u,   2u,   2u,   3u,   3u,   3u,   3u,   4u,   4u,   4u,   4u,   5u,   5u,   5u,
      6u,   6u,   6u,   7u,   7u,   7u,   8u,   8u,   8u,   9u,   9u,  10u,  10u,  10u,  11u,  11u,
     12u,  12u,  13u,  13u,  14u,  14u,  15u,  15u,  16u,  16u,  17u,  17u,  18u,  18u,  19u,  20u,
     20u,  21u,  21u,  22u,  23u,  23u,  24u,  25u,  25u,  26u,  27u,  28u,  28u,  29u,  30u,  31u,
     31u,  32u,  33u,  34u,  35u,  35u,  36u,  37u,  38u,  39u,  40u,  41u,  42u,  43u,  44u,  45u,
     46u,  47u,  48u,  49u,  50u,  51u,  52u,  54u,  55u,  56u,  57u,  58u,  60u,  61u,  62u,  64u,
     65u,  66u,  68u,  69u,  70u,  72u,  73u,  75u,  76u,  78u,  79u,  81u,  82u,  84u,  85u,  87u,
     89u,  90u,  92u,  94u,  95u,  97u,  99u, 100u, 102u, 104u, 106u, 108u, 109u, 111u, 113u, 115u,
    117u, 119u, 121u, 123u, 125u, 127u, 129u, 131u, 133u, 135u, 137u, 139u, 141u, 143u, 145u, 148u,
    150u, 152u, 154u, 156u, 158u, 161u, 163u, 165u, 167u, 170u, 172u, 174u, 177u, 179u, 181u, 184u,
    186u, 189u, 191u, 193u, 196u, 198u, 201u, 203u, 206u, 208u, 211u, 213u, 216u, 219u, 221u, 224u,
    226u, 229u, 232u, 234u, 237u, 240u, 242u, 245u, 248u, 250u, 253u, 255u, 255u, 255u, 255u, 255u,
    255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
    255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u
};
#endif

void WS2812_class::init(uint8_t _num, GPIO_TypeDef* _port, uint16_t _pin)
{
    dirty = false;

    if (!g_ws2812_rst_ticks) {
        // 50us według datasheet, damy 100us
        g_ws2812_rst_ticks = 100u * time_hw_ticks_per_us();
        if (!g_ws2812_rst_ticks) g_ws2812_rst_ticks = 1u;
    }

    if (_num > MAX_NUM) _num = MAX_NUM;

    num  = _num;
    port = _port;
    pin  = _pin;

    enable_gpio_clock(port);

    GPIO_InitTypeDef gi = {0};
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    gi.GPIO_Mode  = GPIO_Mode_Out_PP;
    gi.GPIO_Pin   = pin;
    GPIO_Init(port, &gi);

    RGB_L(port, pin);
    clear();
}

void WS2812_class::clear(void)
{
    for (uint32_t i = 0; i < (uint32_t)num; i++) last_grb[i] = 0u;
    for (uint32_t i = 0; i < (uint32_t)num; i++) last_online_raw_rgb[i] = 0u;
    for (uint32_t i = 0; i < (uint32_t)num; i++) last_online_is_filament[i] = 0u;
    dirty = true;
}

void WS2812_class::RST(void)
{
    RGB_L(port, pin);
    delayTicks32(g_ws2812_rst_ticks);
}

void WS2812_class::updata(void)
{
    if (!dirty || time_hw_slow || !bus_background_ready()) return;

    GPIO_TypeDef* const p = port;
    const uint32_t      m = (uint32_t)pin;

    uint32_t irq = irq_save_wch();

    // wyrównanie startu do granicy ticka STK
    uint32_t base = STK_CNTL;
    while (STK_CNTL == base) { }
    base = STK_CNTL;

    for (uint32_t led = 0; led < (uint32_t)num; led++)
    {
        // GRB, MSB-first
        uint32_t v = last_grb[led];

        for (uint32_t k = 0; k < 24u; k++)
        {
            const uint32_t th = (v & 0x800000u) ? WS2812_T1H_TICKS : WS2812_T0H_TICKS;
            v <<= 1;

            RGB_H(p, m);
            base += th;
            wait_until(base);

            RGB_L(p, m);
            base += (WS2812_TBIT_TICKS - th);
            wait_until(base);
        }
    }

    dirty = false;
    irq_restore_wch(irq);
    RST();
}

void WS2812_class::set_RGB(uint8_t R, uint8_t G, uint8_t B, uint8_t index)
{
    if (index >= num) return;

    const uint32_t packed = ((uint32_t)G << 16) | ((uint32_t)R << 8) | (uint32_t)B;
    if (last_grb[index] == packed) return;

    last_grb[index] = packed;
    dirty = true;
}

void WS2812_class::set_RGB_online(uint8_t R, uint8_t G, uint8_t B, uint8_t index, bool filament)
{
    if (index >= num) return;

    if (!filament)
    {
        last_online_is_filament[index] = 0u;
        set_RGB(R, G, B, index);
        return;
    }

#if !BMCU_ONLINE_LED_FILAMENT_RGB
    (void)filament;
    set_RGB(R, G, B, index);
#else
    const uint32_t raw = ((uint32_t)R << 16) | ((uint32_t)G << 8) | (uint32_t)B;

    if (last_online_is_filament[index] && (last_online_raw_rgb[index] == raw))
        return;

    last_online_is_filament[index] = 1u;
    last_online_raw_rgb[index]     = raw;

    // profil "kanał 1": gamma + TypicalLEDStrip (G=176, B=240) + cap=32
    uint8_t rr = kGamma8[R];
    uint8_t gg = kGamma8[G];
    uint8_t bb = kGamma8[B];

    gg = scale8_video(gg, 176u);
    bb = scale8_video(bb, 240u);

    rr = scale8_video(rr, 32u);
    gg = scale8_video(gg, 32u);
    bb = scale8_video(bb, 32u);

    set_RGB(rr, gg, bb, index);
#endif
}
