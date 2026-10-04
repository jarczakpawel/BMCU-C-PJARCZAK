#include "Debug_log.h"
#include <string.h>
#include <stddef.h>

#include "ch32v20x_rcc.h"
#include "ch32v20x_gpio.h"
#include "ch32v20x_usart.h"
#include "ch32v20x_dma.h"
#include "ch32v20x_misc.h"

/* ===== IRQ ===== */
extern "C" void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        (void)USART_ReceiveData(USART3);
    }
}

static DMA_InitTypeDef g_dma;
static uint8_t g_dbg_inited = 0;

/* ===== UART3 + DMA TX ===== */
static void Debug_uart3_dma_init(uint32_t baudrate)
{
    GPIO_InitTypeDef  gpio = {0};
    USART_InitTypeDef us   = {0};
    NVIC_InitTypeDef  nv   = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* USART3: TX=PB10, RX=PB11 */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    us.USART_BaudRate            = baudrate;
    us.USART_WordLength          = USART_WordLength_9b;
    us.USART_StopBits            = USART_StopBits_1;
    us.USART_Parity              = USART_Parity_Even;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &us);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    nv.NVIC_IRQChannel                   = USART3_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 1;
    nv.NVIC_IRQChannelSubPriority        = 1;
    nv.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nv);

    /* DMA1 Channel2: USART3 TX */
    g_dma.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DATAR;
    g_dma.DMA_MemoryBaseAddr     = (uint32_t)0;
    g_dma.DMA_DIR                = DMA_DIR_PeripheralDST;
    g_dma.DMA_Mode               = DMA_Mode_Normal;
    g_dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    g_dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    g_dma.DMA_Priority           = DMA_Priority_Low;
    g_dma.DMA_M2M                = DMA_M2M_Disable;
    g_dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    g_dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    g_dma.DMA_BufferSize         = 0;

    USART_Cmd(USART3, ENABLE);

    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);
}

void Debug_log_init(void)
{
    if (g_dbg_inited) return;
    Debug_uart3_dma_init(Debug_log_baudrate);
    g_dbg_inited = 1;
}

uint64_t Debug_log_count64(void) { return 0ULL; }
void Debug_log_time(void) { }

void Debug_log_write(const void *data)
{
    int n = (int)strlen((const char*)data);
    Debug_log_write_num(data, n);
}

void Debug_log_write_num(const void *data, int num)
{
    if (num <= 0) return;
    if (!g_dbg_inited) Debug_log_init();

    /* wait previous TX complete */
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) { }

    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);

    g_dma.DMA_MemoryBaseAddr = (uint32_t)data;
    g_dma.DMA_BufferSize     = (uint16_t)num;
    DMA_Init(DMA1_Channel2, &g_dma);

    /* avoid TC-race: clear before start */
    USART_ClearFlag(USART3, USART_FLAG_TC);

    DMA_Cmd(DMA1_Channel2, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

    /* wait DMA/UART complete */
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) { }
}

__attribute__((used))
int _write(int fd, char *buf, int size)
{
#ifdef Debug_log_on
    (void)fd;
    Debug_log_write_num(buf, size);
#else
    (void)fd; (void)buf; (void)size;
#endif
    return size;
}

__attribute__((used))
void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
        return (void *)(-1);

    curbrk += incr;
    return curbrk - incr;
}
