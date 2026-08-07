#include "micros.h"

#include <stdint.h>

#define REG32(address) (*(volatile uint32_t *)(address))

#define RCC_BASE       0x40021000UL
#define TIM2_BASE      0x40000000UL

#define RCC_APB1ENR    REG32(RCC_BASE + 0x1CUL)

#define TIM2_CR1       REG32(TIM2_BASE + 0x00UL)
#define TIM2_EGR       REG32(TIM2_BASE + 0x14UL)
#define TIM2_CNT       REG32(TIM2_BASE + 0x24UL)
#define TIM2_PSC       REG32(TIM2_BASE + 0x28UL)
#define TIM2_ARR       REG32(TIM2_BASE + 0x2CUL)

#define RCC_APB1ENR_TIM2EN (1UL << 0)

#define TIM_CR1_CEN         (1UL << 0)
#define TIM_CR1_URS         (1UL << 2)
#define TIM_EGR_UG          (1UL << 0)

#define TIMESTAMP_HZ        1000000UL

micros_status_t micros_init(uint32_t tim2_clock_hz)
{
    uint32_t divider;

    if ((tim2_clock_hz == 0UL) ||
        ((tim2_clock_hz % TIMESTAMP_HZ) != 0UL))
    {
        return MICROS_INVALID_CLOCK;
    }

    divider = tim2_clock_hz / TIMESTAMP_HZ;

    if ((divider == 0UL) || (divider > 65536UL))
    {
        return MICROS_PRESCALER_OUT_OF_RANGE;
    }

    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC_APB1ENR;

    TIM2_CR1 = 0UL;
    TIM2_PSC = divider - 1UL;
    TIM2_ARR = 0xFFFFFFFFUL;
    TIM2_CNT = 0UL;

    TIM2_CR1 = TIM_CR1_URS;
    TIM2_EGR = TIM_EGR_UG;
    TIM2_CR1 = TIM_CR1_URS | TIM_CR1_CEN;

    return MICROS_OK;
}

uint32_t micros(void)
{
    return TIM2_CNT;
}

void delay_ms(uint32_t milliseconds)
{
    uint32_t start = micros();
    uint32_t duration_us = milliseconds * 1000UL;

    while ((uint32_t)(micros() - start) < duration_us)
    {
        __asm volatile ("nop");
    }
}