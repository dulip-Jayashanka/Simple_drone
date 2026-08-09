#include "micros.h"

#include <stdint.h>

#define REG32(address) (*(volatile uint32_t *)(address))
#define REG8(address)  (*(volatile uint8_t *)(address))

#define RCC_BASE       0x40021000UL
#define TIM2_BASE      0x40000000UL

#define RCC_APB1ENR    REG32(RCC_BASE + 0x1CUL)

#define TIM2_CR1       REG32(TIM2_BASE + 0x00UL)
#define TIM2_DIER      REG32(TIM2_BASE + 0x0CUL)
#define TIM2_SR        REG32(TIM2_BASE + 0x10UL)
#define TIM2_EGR       REG32(TIM2_BASE + 0x14UL)
#define TIM2_CNT       REG32(TIM2_BASE + 0x24UL)
#define TIM2_PSC       REG32(TIM2_BASE + 0x28UL)
#define TIM2_ARR       REG32(TIM2_BASE + 0x2CUL)

#define NVIC_ISER0     REG32(0xE000E100UL)
#define NVIC_IPR_BASE  0xE000E400UL

#define RCC_APB1ENR_TIM2EN (1UL << 0)

#define TIM_CR1_CEN         (1UL << 0)
#define TIM_CR1_URS         (1UL << 2)
#define TIM_DIER_UIE        (1UL << 0)
#define TIM_SR_UIF          (1UL << 0)
#define TIM_EGR_UG          (1UL << 0)

#define TIM2_IRQ_NUMBER     28UL
#define TIM2_IRQ_PRIORITY   0x60U

#define TIMESTAMP_HZ        1000000UL
#define TIM2_COUNTER_PERIOD 65536UL

/*
 * RM0008 describes TIM2-TIM5 on STM32F1 as 16-bit auto-reload timers.
 * The old code programmed ARR=0xFFFFFFFF and returned CNT directly, so the
 * timestamp wrapped every 65.536 ms at 1 MHz. This high word extends TIM2 to
 * a software 32-bit microsecond clock without changing the public API.
 */
static volatile uint32_t tim2_overflow_base_us;

static uint32_t enter_critical(void)
{
    uint32_t primask;

    __asm volatile (
        "mrs %0, primask\n"
        "cpsid i\n"
        : "=r" (primask)
        :
        : "memory");

    return primask;
}

static void exit_critical(uint32_t primask)
{
    if ((primask & 1UL) == 0UL)
    {
        __asm volatile ("cpsie i" ::: "memory");
    }
}

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
    TIM2_DIER = 0UL;
    TIM2_SR = 0UL;

    TIM2_PSC = divider - 1UL;

    /* TIM2 is 16-bit on this STM32F1 device. */
    TIM2_ARR = 0xFFFFUL;
    TIM2_CNT = 0UL;

    tim2_overflow_base_us = 0UL;

    /* Load PSC/ARR immediately, then remove the generated update flag. */
    TIM2_CR1 = TIM_CR1_URS;
    TIM2_EGR = TIM_EGR_UG;
    TIM2_SR &= ~TIM_SR_UIF;

    /* TIM2 update interrupt extends the 16-bit counter in software. */
    REG8(NVIC_IPR_BASE + TIM2_IRQ_NUMBER) =
        TIM2_IRQ_PRIORITY;
    NVIC_ISER0 = (1UL << TIM2_IRQ_NUMBER);

    TIM2_DIER = TIM_DIER_UIE;
    TIM2_CR1 = TIM_CR1_URS | TIM_CR1_CEN;

    return MICROS_OK;
}

uint32_t micros(void)
{
    uint32_t primask;
    uint32_t base;
    uint32_t count;
    uint32_t status;

    /*
     * Keep the high-word and 16-bit CNT sample coherent. If an overflow has
     * happened but the TIM2 IRQ is pending, compensate for it locally; the
     * ISR will later update the stored base and clear UIF.
     */
    primask = enter_critical();

    base = tim2_overflow_base_us;
    count = TIM2_CNT & 0xFFFFUL;
    status = TIM2_SR;

    if ((status & TIM_SR_UIF) != 0UL)
    {
        base += TIM2_COUNTER_PERIOD;
        count = TIM2_CNT & 0xFFFFUL;
    }

    exit_critical(primask);

    return base + count;
}

void delay_ms(uint32_t milliseconds)
{
    /*
     * Delay in 1 ms chunks. Besides avoiding multiplication overflow, this
     * continuously services micros() and remains correct across TIM2 wraps.
     */
    while (milliseconds != 0UL)
    {
        uint32_t start = micros();

        while ((uint32_t)(micros() - start) < 1000UL)
        {
            __asm volatile ("nop");
        }

        milliseconds--;
    }
}

void TIM2_IRQHandler(void)
{
    if ((TIM2_SR & TIM_SR_UIF) != 0UL)
    {
        /* UIF is cleared by software writing 0. */
        TIM2_SR &= ~TIM_SR_UIF;
        tim2_overflow_base_us += TIM2_COUNTER_PERIOD;
    }
}
