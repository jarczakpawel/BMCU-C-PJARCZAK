#include "hal/time_hw.h"
#include "hal/irq_wch.h"
#include "ch32v20x.h"

uint32_t time_hw_tpus = 1;
uint32_t time_hw_tpms = 1000;
volatile uint32_t time_hw_slow = 0u;
volatile uint32_t time_hw_offset32 = 0u;
volatile uint32_t time_hw_anchor32 = 0u;
static uint64_t time_hw_offset64 = 0u;
static uint64_t time_hw_anchor64 = 0u;

static inline __attribute__((always_inline)) uint64_t ticks64_raw(void)
{
    uint32_t hi1, lo, hi2;
    do {
        hi1 = STK_CNTH;
        lo  = STK_CNTL;
        hi2 = STK_CNTH;
    } while (hi1 != hi2);
    return ((uint64_t)hi1 << 32) | (uint64_t)lo;
}

void time_hw_init(void)
{
    RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_HPRE | RCC_PPRE1 | RCC_PPRE2)) |
        RCC_HPRE_DIV1 | RCC_PPRE1_DIV4 | RCC_PPRE2_DIV2;
    STK_CTLR = 0;
    STK_SR   = 0;
    STK_CNTL = 0;
    STK_CNTH = 0;

    STK_CMPLR = 0xFFFFFFFFu;
    STK_CMPHR = 0xFFFFFFFFu;

    STK_CTLR = (1u << 3) | (1u << 0);   // STRE=1, STCLK=HCLK/8, STE=1

    uint32_t tpus = (SystemCoreClock / 8u) / 1000000u;
    if (!tpus) tpus = 1u;
    time_hw_tpus = tpus;

    uint32_t tpms = tpus * 1000u;
    if (!tpms) tpms = 1u;
    time_hw_tpms = tpms;
}

void time_hw_flash_clock(uint32_t slow)
{
    const uint32_t irq = irq_save_wch();
    slow = slow != 0u;
    if (slow != time_hw_slow) {
        const uint64_t raw = ticks64_raw();
        if (time_hw_slow) time_hw_offset64 += raw - time_hw_anchor64;
        time_hw_anchor64 = raw;
        time_hw_anchor32 = (uint32_t)raw;
        time_hw_offset32 = (uint32_t)time_hw_offset64;
        const uint32_t dividers = slow
            ? RCC_HPRE_DIV2 | RCC_PPRE1_DIV2 | RCC_PPRE2_DIV1
            : RCC_HPRE_DIV1 | RCC_PPRE1_DIV4 | RCC_PPRE2_DIV2;
        RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_HPRE | RCC_PPRE1 | RCC_PPRE2)) | dividers;
        __asm__ volatile("fence iorw, iorw" ::: "memory");
        (void)RCC->CFGR0;
        SystemCoreClock = slow ? 72000000u : 144000000u;
        time_hw_slow = slow;
    }
    irq_restore_wch(irq);
}

uint32_t time_hw_ticks_per_us(void) { return time_hw_tpus; }
uint32_t time_hw_ticks_per_ms(void) { return time_hw_tpms; }

uint64_t time_ticks64(void)
{
    const uint64_t raw = ticks64_raw();
    return raw + time_hw_offset64 + (time_hw_slow ? raw - time_hw_anchor64 : 0u);
}

uint64_t time_us64(void)
{
    const uint64_t t = time_ticks64();
    const uint32_t d = time_hw_tpus;
    if (__builtin_expect(d == 1u, 0)) return t;
    return t / (uint64_t)d;
}

uint64_t time_ms64(void)
{
    const uint64_t t = time_ticks64();
    const uint32_t d = time_hw_tpms;
    if (__builtin_expect(d == 1u, 0)) return t;
    return t / (uint64_t)d;
}

void delay_us(uint32_t us)
{
    if (!us) return;
    uint64_t t = (uint64_t)us * (uint64_t)time_hw_tpus;
    if (t > 0xFFFFFFFFu) t = 0xFFFFFFFFu;
    delayTicks32((uint32_t)t);
}

void delay(uint32_t ms)
{
    if (!ms) return;
    uint64_t t = (uint64_t)ms * (uint64_t)time_hw_tpms;
    if (t > 0xFFFFFFFFu) t = 0xFFFFFFFFu;
    delayTicks32((uint32_t)t);
}
