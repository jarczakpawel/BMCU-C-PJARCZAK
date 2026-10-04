#include "_bus_hardware.h"

#include "ch32v20x.h"
#include "ch32v20x_rcc.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_usart.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_misc.h"
#include "core_riscv.h"
#include "crc_bus.h"
#include "hal/irq_wch.h"
#include "hal/time_hw.h"

uint16_t bus_host_device_type=0x0000;

DMA_InitTypeDef bus_uart1_dma_init_structure;
void bus_uart1_init();
bool bus_uart1_dma_send(uint8_t *data, uint16_t length);


_bus_port_deal bus_port_to_host;

static constexpr uint32_t BUS_BAUD = 1250000u;
static constexpr uint32_t BUS_BITS_PER_CHAR = 11u;
static constexpr uint32_t RX_SIZE = 2048u;
static constexpr uint32_t wire_time_us(uint32_t bytes)
{
    return (bytes * BUS_BITS_PER_CHAR * 1000u + BUS_BAUD / 1000u - 1u) / (BUS_BAUD / 1000u);
}
static volatile uint8_t rx_dma[RX_SIZE] __attribute__((aligned(4)));
static volatile uint32_t rx_wraps = 0u;
static volatile bool rx_fault = false;
static uint32_t rx_consumed = 0u;
static uint32_t rx_last_progress = 0u;
static volatile uint32_t tx_deadline = 0u;
static bool bus_started = false;

static inline __attribute__((always_inline)) uint8_t rx_peek(uint32_t absolute_index)
{
    return rx_dma[absolute_index & (RX_SIZE - 1u)];
}

static uint32_t rx_produced()
{
    const uint32_t irq = irq_save_wch();
    uint32_t count = rx_consumed;
    bool sampled = false;
    for (uint8_t retry = 0u; retry < 4u; retry++) {
        const uint32_t before = DMA1->INTFR & DMA1_FLAG_TC5;
        const uint32_t remaining = DMA1_Channel5->CNTR;
        const uint32_t after = DMA1->INTFR & DMA1_FLAG_TC5;
        if (before != after || !remaining || remaining > RX_SIZE) continue;
        count = (rx_wraps + (after ? 1u : 0u)) * RX_SIZE + RX_SIZE - remaining;
        sampled = true;
        break;
    }
    if (!sampled) count = rx_consumed;
    __asm__ volatile("fence iorw, iorw" ::: "memory");
    irq_restore_wch(irq);
    return count;
}

static void bus_uart_reset()
{
    const uint16_t baud = USART1->BRR;
    const uint16_t control1 = USART1->CTLR1;
    const uint16_t control2 = USART1->CTLR2;
    const uint16_t control3 = USART1->CTLR3;
    USART_DeInit(USART1);
    USART1->BRR = baud;
    USART1->CTLR2 = control2;
    USART1->CTLR3 = control3;
    USART1->CTLR1 = control1;
}

static void bus_tx_finish(bool aborted)
{
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    USART1->CTLR3 &= (uint16_t)~USART_DMAReq_Tx;
    DMA1_Channel4->CFGR &= (uint16_t)~DMA_CFGR1_EN;
    DMA1->INTFCR = DMA1_FLAG_GL4;
    if (aborted) USART_Cmd(USART1, DISABLE);
    GPIOA->BCR = GPIO_Pin_12;
    USART_ClearITPendingBit(USART1, USART_IT_TC);
    USART1->CTLR1 |= USART_Mode_Rx;
    if (aborted) {
        bus_uart_reset();
        rx_fault = true;
        ++bus_port_to_host.tx_errors;
        USART_Cmd(USART1, ENABLE);
    }
    bus_port_to_host.idle = true;
}

void bus_rx_poll()
{
    if (!bus_started) return;
    const uint64_t now64 = time_ticks64();
    const uint32_t now = (uint32_t)now64;
    {
        const uint32_t irq = irq_save_wch();
        if (!bus_port_to_host.idle) {
            const bool done = DMA1_Channel4->CNTR == 0u && (USART1->STATR & USART_FLAG_TC);
            const bool failed = (DMA1->INTFR & DMA1_FLAG_TE4) != 0u;
            if (done || failed || time_reached32(now, tx_deadline))
                bus_tx_finish(failed || !done);
        }
        irq_restore_wch(irq);
    }
    if (rx_fault) {
        const uint32_t irq = irq_save_wch();
        if (!bus_port_to_host.idle) {
            irq_restore_wch(irq);
            return;
        }
        USART1->CTLR1 &= (uint16_t)~USART_Mode_Rx;
        DMA1_Channel5->CFGR &= (uint16_t)~DMA_CFGR1_EN;
        USART_Cmd(USART1, DISABLE);
        bus_uart_reset();
        DMA1->INTFCR = DMA1_FLAG_GL5;
        DMA1_Channel5->MADDR = (uint32_t)rx_dma;
        DMA1_Channel5->CNTR = RX_SIZE;
        rx_wraps = rx_consumed = 0u;
        rx_fault = false;
        rx_last_progress = now;
        bus_port_to_host.reset_rx();
        ++bus_port_to_host.rx_dropped;
        DMA1_Channel5->CFGR |= DMA_CFGR1_EN;
        USART1->CTLR1 |= USART_Mode_Rx;
        USART_Cmd(USART1, ENABLE);
        irq_restore_wch(irq);
    }
    const uint32_t produced = rx_produced();
    if (rx_fault) return;
    if ((uint32_t)(produced - rx_consumed) >= RX_SIZE) {
        rx_fault = true;
        return;
    }
    if (produced != rx_consumed) rx_last_progress = now;
    while (rx_consumed != produced && !rx_fault) {
        // skip heartbeat
        const uint32_t available = (uint32_t)(produced - rx_consumed);
        if (__builtin_expect(available >= 5u, 1) &&
            rx_peek(rx_consumed + 0u) == 0x3Du &&
            rx_peek(rx_consumed + 1u) == 0xC5u)
        {
            const uint32_t hb_len = rx_peek(rx_consumed + 2u);
            if (hb_len >= 6u && hb_len <= 255u &&
                rx_peek(rx_consumed + 4u) == 0x20u &&
                available >= hb_len)
            {
                rx_consumed += hb_len;
                bambubus_heartbeat_seen_fast();
                continue;
            }
        }

        if ((rx_consumed & 127u) == 0u &&
            (uint32_t)(rx_produced() - rx_consumed) >= RX_SIZE) {
            rx_fault = true;
            break;
        }
        const uint8_t data = rx_dma[rx_consumed & (RX_SIZE - 1u)];
        ++rx_consumed;
        bus_port_to_host.irq(data);
    }
    if (!rx_fault && bus_port_to_host.receiving() &&
        (uint32_t)(now - rx_last_progress) >= us_to_ticks32(wire_time_us(1282u)) &&
        rx_consumed == rx_produced() && !(USART1->STATR & USART_FLAG_RXNE)) {
        bus_port_to_host.reset_rx();
        ++bus_port_to_host.rx_dropped;
    }
}

bool bus_background_ready()
{
    if (!bus_started) return true;
    bus_rx_poll();
    return bus_port_to_host.idle && !bus_port_to_host.send_data_len &&
        !bus_port_to_host.recv_data_len && !bus_port_to_host.receiving() &&
        rx_consumed == rx_produced() && !rx_fault &&
        !(USART1->STATR & USART_FLAG_RXNE);
}

void bus_shutdown()
{
    if (!bus_started) return;
    const uint32_t deadline = time_ticks32() + us_to_ticks32(wire_time_us(1288u));
    uint32_t remaining = 1000000u;
    while (!bus_port_to_host.idle && !time_reached32(time_ticks32(), deadline) && --remaining)
        bus_rx_poll();
    const uint32_t irq = irq_save_wch();
    if (!bus_port_to_host.idle) bus_tx_finish(true);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    USART_ITConfig(USART1, USART_IT_PE, DISABLE);
    USART_ITConfig(USART1, USART_IT_ERR, DISABLE);
    USART1->CTLR3 &= (uint16_t)~(USART_DMAReq_Rx | USART_DMAReq_Tx);
    DMA1_Channel5->CFGR &= (uint16_t)~DMA_CFGR1_EN;
    DMA1_Channel4->CFGR &= (uint16_t)~DMA_CFGR1_EN;
    DMA1->INTFCR = DMA1_FLAG_GL4 | DMA1_FLAG_GL5;
    USART_Cmd(USART1, DISABLE);
    GPIOA->BCR = GPIO_Pin_12;
    bus_started = false;
    irq_restore_wch(irq);
}

#define uart1_port_irq(data) bus_port_to_host.irq(data)
#define uart1_port_idle bus_port_to_host.idle
#define bus_port_to_host_send_func bus_uart1_dma_send

void bus_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
    bus_crc_init();
    bus_port_to_host.init(bus_port_to_host_send_func);
    bus_uart1_init();
}

void bus_uart1_init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* USART1 TX-->A.9   RX-->A.10   DE-->A.12*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // DE
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BCR = GPIO_Pin_12;

    USART_InitStructure.USART_BaudRate = BUS_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure DMA1 channel 4 for USART1 TX
    bus_uart1_dma_init_structure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    bus_uart1_dma_init_structure.DMA_MemoryBaseAddr = (uint32_t)0;
    bus_uart1_dma_init_structure.DMA_DIR = DMA_DIR_PeripheralDST;
    bus_uart1_dma_init_structure.DMA_Mode = DMA_Mode_Normal;
    bus_uart1_dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    bus_uart1_dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    bus_uart1_dma_init_structure.DMA_Priority = DMA_Priority_VeryHigh;
    bus_uart1_dma_init_structure.DMA_M2M = DMA_M2M_Disable;
    bus_uart1_dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    bus_uart1_dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    bus_uart1_dma_init_structure.DMA_BufferSize = 0;
    DMA_Init(DMA1_Channel4, &bus_uart1_dma_init_structure);
    DMA_ClearFlag(DMA1_FLAG_GL4);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    DMA_InitTypeDef rx = bus_uart1_dma_init_structure;
    rx.DMA_MemoryBaseAddr = (uint32_t)rx_dma;
    rx.DMA_DIR = DMA_DIR_PeripheralSRC;
    rx.DMA_Mode = DMA_Mode_Circular;
    rx.DMA_BufferSize = RX_SIZE;
    DMA_Init(DMA1_Channel5, &rx);
    DMA_ClearFlag(DMA1_FLAG_GL5);
    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC | DMA_IT_TE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    rx_wraps = rx_consumed = 0u;
    rx_fault = false;
    rx_last_progress = time_ticks32();
    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    bus_started = true;
    USART_Cmd(USART1, ENABLE);
}

bool bus_uart1_dma_send(unsigned char *data, uint16_t length)
{
    const uint32_t irq = irq_save_wch();
    if (!data || !length || length > 1280u || !bus_started || !bus_port_to_host.idle) {
        irq_restore_wch(irq);
        return false;
    }
    USART1->CTLR1 &= (uint16_t)~USART_Mode_Rx;

    tx_deadline = time_ticks32() + ms_to_ticks32(100u);
    bus_port_to_host.idle = false;

    DMA1_Channel4->CFGR &= (uint16_t)(~DMA_CFGR1_EN);

    DMA1_Channel4->MADDR = (uint32_t)data;
    DMA1_Channel4->CNTR  = length;

    // DE = TX
    GPIOA->BSHR = GPIO_Pin_12;

    // wyczyść TC
    USART_ClearITPendingBit(USART1, USART_IT_TC);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    DMA_ClearFlag(DMA1_FLAG_GL4);

    USART1->CTLR3 |= USART_DMAReq_Tx;
    __asm__ volatile("fence iorw, iorw" ::: "memory");
    DMA1_Channel4->CFGR |= DMA_CFGR1_EN;
    irq_restore_wch(irq);
    return true;
}

extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    const uint16_t status = USART1->STATR;
    if (!bus_port_to_host.idle && (status & USART_FLAG_TC))
    {
        // DE = RX
        // TX done
        if (DMA1_Channel4->CNTR == 0u) bus_tx_finish((DMA1->INTFR & DMA1_FLAG_TE4) != 0u);
        else USART_ClearITPendingBit(USART1, USART_IT_TC);
    }
}

extern "C" void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel5_IRQHandler(void)
{
    const uint32_t flags = DMA1->INTFR;
    if (flags & DMA1_FLAG_TC5) {
        DMA1->INTFCR = DMA1_FLAG_TC5;
        ++rx_wraps;
    }
    if (flags & DMA1_FLAG_TE5) {
        DMA1->INTFCR = DMA1_FLAG_TE5;
        rx_fault = true;
    }
}

extern "C" void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA1->INTFR & DMA1_FLAG_TE4) {
        if (!bus_port_to_host.idle) bus_tx_finish(true);
        else DMA1->INTFCR = DMA1_FLAG_GL4;
    }
}
