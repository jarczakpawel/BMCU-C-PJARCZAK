#pragma once

#include <stdbool.h>
#include <stdint.h>

enum { ADC_DMA_FILTER_BLOCKS = 4u };

void  ADC_DMA_init(void);
bool  ADC_DMA_is_inited(void);

void  ADC_DMA_gpio_analog(void);

void  ADC_DMA_poll(void);
const float* ADC_DMA_get_value(void);
uint32_t ADC_DMA_generation(void);

void  ADC_DMA_filter_reset(void);
bool  ADC_DMA_ready(void);
void  ADC_DMA_wait_full(void);
