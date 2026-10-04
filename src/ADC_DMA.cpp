#include "ADC_DMA.h"
#include "hal/time_hw.h"
#include "ch32v20x_adc.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_rcc.h"
#include <stdint.h>

static constexpr uint32_t kCh      = 8;
static constexpr uint32_t kBlock   = 32;
static constexpr uint32_t kNBlocks = ADC_DMA_FILTER_BLOCKS;
static constexpr uint32_t kBufLen  = (kCh * kBlock * 2);
static constexpr uint32_t kHalfLen = (kBufLen / 2);

static bool g_adc_dma_inited = false;
bool ADC_DMA_is_inited() { return g_adc_dma_inited; }

static volatile uint32_t g_dma_buf[kBufLen] __attribute__((aligned(4)));

static uint32_t g_ring_sum[kNBlocks][kCh];
static uint32_t g_acc_sum[kCh];
static uint8_t  g_ring_idx = 0;
static uint8_t  g_blocks_filled = 0;

static float            g_v[2][kCh] __attribute__((aligned(4)));
static volatile uint8_t g_v_rd = 0;
static volatile uint8_t g_acc_dirty = 0;
static uint32_t g_publication = 0u;

static constexpr float kScale32  = 3.3f / (8190.0f *  32.0f);
static constexpr float kScale64  = 3.3f / (8190.0f *  64.0f);
static constexpr float kScale96  = 3.3f / (8190.0f *  96.0f);
static constexpr float kScale128 = 3.3f / (8190.0f * 128.0f);
static float g_scale = kScale32;

static inline __attribute__((always_inline)) void adc_dma_barrier()
{
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

static inline __attribute__((always_inline)) void adc_dma_compiler_barrier()
{
    __asm__ volatile("" ::: "memory");
}

void ADC_DMA_gpio_analog()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Mode  = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                      GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);
}

static inline void filter_reset()
{
    g_ring_idx = 0;
    g_blocks_filled = 0;
    g_scale = kScale32;

    for (uint32_t ch = 0; ch < kCh; ch++) g_acc_sum[ch] = 0;

    for (uint32_t b = 0; b < kNBlocks; b++)
        for (uint32_t ch = 0; ch < kCh; ch++)
            g_ring_sum[b][ch] = 0;

    for (uint32_t ch = 0; ch < kCh; ch++)
    {
        g_v[0][ch] = 0.0f;
        g_v[1][ch] = 0.0f;
    }

    g_v_rd = 0;
    g_acc_dirty = 0;
}

static inline __attribute__((always_inline)) uint32_t adc_pair_sum(uint32_t w)
{
    const uint32_t a1 = (uint16_t)(w & 0xFFFFu);
    const uint32_t a2 = (uint16_t)(w >> 16);
    return a1 + a2;
}

static inline void process_half_update_filter(const uint32_t* p_half)
{
    uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0, s7 = 0;

    const uint32_t* __restrict row = p_half;
    for (uint32_t s = 0; s < kBlock; s++)
    {
        s0 += adc_pair_sum(row[0]);
        s1 += adc_pair_sum(row[1]);
        s2 += adc_pair_sum(row[2]);
        s3 += adc_pair_sum(row[3]);
        s4 += adc_pair_sum(row[4]);
        s5 += adc_pair_sum(row[5]);
        s6 += adc_pair_sum(row[6]);
        s7 += adc_pair_sum(row[7]);
        row += kCh;
    }

    const uint8_t idx = g_ring_idx;
    {
        uint32_t o;
        o = g_ring_sum[idx][0]; g_ring_sum[idx][0] = s0; g_acc_sum[0] = g_acc_sum[0] - o + s0;
        o = g_ring_sum[idx][1]; g_ring_sum[idx][1] = s1; g_acc_sum[1] = g_acc_sum[1] - o + s1;
        o = g_ring_sum[idx][2]; g_ring_sum[idx][2] = s2; g_acc_sum[2] = g_acc_sum[2] - o + s2;
        o = g_ring_sum[idx][3]; g_ring_sum[idx][3] = s3; g_acc_sum[3] = g_acc_sum[3] - o + s3;
        o = g_ring_sum[idx][4]; g_ring_sum[idx][4] = s4; g_acc_sum[4] = g_acc_sum[4] - o + s4;
        o = g_ring_sum[idx][5]; g_ring_sum[idx][5] = s5; g_acc_sum[5] = g_acc_sum[5] - o + s5;
        o = g_ring_sum[idx][6]; g_ring_sum[idx][6] = s6; g_acc_sum[6] = g_acc_sum[6] - o + s6;
        o = g_ring_sum[idx][7]; g_ring_sum[idx][7] = s7; g_acc_sum[7] = g_acc_sum[7] - o + s7;
    }

    uint8_t ni = (uint8_t)(idx + 1u);
    if (ni >= kNBlocks) ni = 0u;
    g_ring_idx = ni;

    if (g_blocks_filled < kNBlocks)
    {
        g_blocks_filled++;
        if      (g_blocks_filled == 1u) g_scale = kScale32;
        else if (g_blocks_filled == 2u) g_scale = kScale64;
        else if (g_blocks_filled == 3u) g_scale = kScale96;
        else                            g_scale = kScale128;
    }

    adc_dma_compiler_barrier();
    g_acc_dirty = 1u;
}

void ADC_DMA_poll()
{
    for (int guard = 0; guard < 4; guard++)
    {
        const uint32_t flags = DMA1->INTFR;

        if (flags & DMA1_FLAG_TE1)
        {
            DMA1->INTFCR = (DMA1_FLAG_TE1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1);
            adc_dma_barrier();
            filter_reset();
            continue;
        }

        if (flags & DMA1_FLAG_HT1)
        {
            DMA1->INTFCR = DMA1_FLAG_HT1;
            adc_dma_barrier();
            const uint32_t* p = (const uint32_t*)(const void*)&g_dma_buf[0];
            process_half_update_filter(p);
            continue;
        }

        if (flags & DMA1_FLAG_TC1)
        {
            DMA1->INTFCR = DMA1_FLAG_TC1;
            adc_dma_barrier();
            const uint32_t* p = (const uint32_t*)(const void*)&g_dma_buf[kHalfLen];
            process_half_update_filter(p);
            continue;
        }

        break;
    }
}

const float *ADC_DMA_get_value()
{
    ADC_DMA_poll();

    if (g_acc_dirty)
    {
        const float scale = g_scale;
        const uint8_t wr = (uint8_t)(g_v_rd ^ 1u);
        float* out = g_v[wr];

        out[0] = (float)g_acc_sum[0] * scale;
        out[1] = (float)g_acc_sum[1] * scale;
        out[2] = (float)g_acc_sum[2] * scale;
        out[3] = (float)g_acc_sum[3] * scale;
        out[4] = (float)g_acc_sum[4] * scale;
        out[5] = (float)g_acc_sum[5] * scale;
        out[6] = (float)g_acc_sum[6] * scale;
        out[7] = (float)g_acc_sum[7] * scale;

        adc_dma_compiler_barrier();
        g_v_rd = wr;
        ++g_publication;
        g_acc_dirty = 0u;
    }

    return g_v[g_v_rd];
}

uint32_t ADC_DMA_generation()
{
    return g_publication;
}

void ADC_DMA_filter_reset()
{
    DMA1->INTFCR = (DMA1_FLAG_GL1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1);
    adc_dma_barrier();
    filter_reset();
}

bool ADC_DMA_ready()
{
    return (g_blocks_filled >= kNBlocks);
}

void ADC_DMA_wait_full()
{
    const uint32_t t0 = time_ticks32();
    const uint32_t tout = ms_to_ticks32(2000u);

    while (g_blocks_filled < kNBlocks)
    {
        ADC_DMA_poll();
        if ((uint32_t)(time_ticks32() - t0) > tout) break;
        delay(1);
    }
}

static inline void adc_calibrate(ADC_TypeDef* a)
{
    ADC_Cmd(a, ENABLE);
    ADC_BufferCmd(a, DISABLE);

    ADC_ResetCalibration(a);
    while (ADC_GetResetCalibrationStatus(a)) {}
    ADC_StartCalibration(a);
    while (ADC_GetCalibrationStatus(a)) {}
}

void ADC_DMA_init()
{
    if (g_adc_dma_inited)
    {
        ADC_DMA_gpio_analog();
        ADC_DMA_filter_reset();
        ADC_DMA_wait_full();
        return;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    ADC_DMA_gpio_analog();

    for (uint32_t i = 0; i < kBufLen; i++) g_dma_buf[i] = 0xFFFFFFFFu;
    filter_reset();

    DMA_DeInit(DMA1_Channel1);
    DMA_Cmd(DMA1_Channel1, DISABLE);

    constexpr uint32_t RDATAR_ADDRESS = 0x4001244Cu;

    DMA_InitTypeDef dma = {0};
    dma.DMA_PeripheralBaseAddr = RDATAR_ADDRESS;
    dma.DMA_MemoryBaseAddr     = (uint32_t)g_dma_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize         = kBufLen;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);

    DMA_SetCurrDataCounter(DMA1_Channel1, kBufLen);

    DMA1->INTFCR = (DMA1_FLAG_GL1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE, DISABLE);

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    ADC_InitTypeDef a1 = {0};
    a1.ADC_Mode               = ADC_Mode_RegSimult;
    a1.ADC_ScanConvMode       = ENABLE;
    a1.ADC_ContinuousConvMode = ENABLE;
    a1.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    a1.ADC_DataAlign          = ADC_DataAlign_Right;
    a1.ADC_NbrOfChannel       = 8;
    a1.ADC_OutputBuffer       = ADC_OutputBuffer_Disable;
    a1.ADC_Pga                = ADC_Pga_1;
    ADC_Init(ADC1, &a1);

    ADC_InitTypeDef a2 = {0};
    a2.ADC_Mode               = ADC_Mode_Independent;
    a2.ADC_ScanConvMode       = ENABLE;
    a2.ADC_ContinuousConvMode = ENABLE;
    a2.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    a2.ADC_DataAlign          = ADC_DataAlign_Right;
    a2.ADC_NbrOfChannel       = 8;
    a2.ADC_OutputBuffer       = ADC_OutputBuffer_Disable;
    a2.ADC_Pga                = ADC_Pga_1;
    ADC_Init(ADC2, &a2);

    for (int i = 0; i < 8; i++)
    {
        ADC_RegularChannelConfig(ADC1, (uint8_t)i, (uint8_t)(i + 1), ADC_SampleTime_71Cycles5);
        ADC_RegularChannelConfig(ADC2, (uint8_t)i, (uint8_t)(i + 1), ADC_SampleTime_71Cycles5);
    }

    adc_calibrate(ADC1);
    adc_calibrate(ADC2);

    ADC_DMACmd(ADC1, ENABLE);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    uint32_t t0 = time_ticks32();
    const uint32_t warm = ms_to_ticks32(350u);

    while ((uint32_t)(time_ticks32() - t0) < warm)
    {
        const uint32_t f = DMA1->INTFR;
        if (f & (DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1)) break;
        if (g_dma_buf[0] != 0xFFFFFFFFu) break;
        delay(1);
    }

    {
        const uint32_t f = DMA1->INTFR;
        g_adc_dma_inited =
            (g_dma_buf[0] != 0xFFFFFFFFu) ||
            (f & (DMA1_FLAG_HT1 | DMA1_FLAG_TC1));
    }

    DMA1->INTFCR = (DMA1_FLAG_GL1 | DMA1_FLAG_HT1 | DMA1_FLAG_TC1 | DMA1_FLAG_TE1);

    if (g_adc_dma_inited)
    {
        ADC_DMA_filter_reset();
        ADC_DMA_wait_full();
    }
}