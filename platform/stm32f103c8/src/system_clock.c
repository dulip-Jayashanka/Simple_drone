#include "system_clock.h"

#include <stdbool.h>
#include <stdint.h>

/* STM32F103 peripheral base addresses from RM0008. */
#define RCC_BASE_ADDRESS              (0x40021000UL)
#define FLASH_INTERFACE_BASE_ADDRESS  (0x40022000UL)

/*
 * Only the first two RCC registers are required in this phase.
 * Their order must match the hardware register map:
 *
 *   CR   at RCC base + 0x00
 *   CFGR at RCC base + 0x04
 */
typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t CFGR;
} rcc_registers_t;

/* FLASH_ACR is the first register in the Flash interface register map. */
typedef struct
{
    volatile uint32_t ACR;
} flash_registers_t;

#define RCC    ((rcc_registers_t *)RCC_BASE_ADDRESS)
#define FLASH  ((flash_registers_t *)FLASH_INTERFACE_BASE_ADDRESS)

/* RCC_CR bits. */
#define RCC_CR_HSEON                  (1UL << 16)
#define RCC_CR_HSERDY                 (1UL << 17)
#define RCC_CR_PLLON                  (1UL << 24)
#define RCC_CR_PLLRDY                 (1UL << 25)

/* RCC_CFGR system-clock selection fields. */
#define RCC_CFGR_SW_MASK              (3UL << 0)
#define RCC_CFGR_SW_PLL               (2UL << 0)
#define RCC_CFGR_SWS_MASK             (3UL << 2)
#define RCC_CFGR_SWS_PLL              (2UL << 2)

/* RCC_CFGR bus-prescaler fields. */
#define RCC_CFGR_HPRE_MASK            (15UL << 4)
#define RCC_CFGR_HPRE_DIV1            (0UL << 4)
#define RCC_CFGR_PPRE1_MASK           (7UL << 8)
#define RCC_CFGR_PPRE1_DIV2           (4UL << 8)
#define RCC_CFGR_PPRE2_MASK           (7UL << 11)
#define RCC_CFGR_PPRE2_DIV1           (0UL << 11)
#define RCC_CFGR_ADCPRE_MASK          (3UL << 14)
#define RCC_CFGR_ADCPRE_DIV6          (2UL << 14)

/* RCC_CFGR PLL fields. */
#define RCC_CFGR_PLLSRC               (1UL << 16)
#define RCC_CFGR_PLLXTPRE             (1UL << 17)
#define RCC_CFGR_PLLMUL_MASK          (15UL << 18)
#define RCC_CFGR_PLLMUL_X9            (7UL << 18)

/* FLASH_ACR bits. */
#define FLASH_ACR_LATENCY_MASK        (7UL << 0)
#define FLASH_ACR_LATENCY_2WS         (2UL << 0)
#define FLASH_ACR_PRFTBE              (1UL << 4)

/*
 * This is an iteration limit, not an exact time in microseconds.
 * It prevents a missing crystal or failed PLL from trapping the firmware
 * forever. At the reset-default 8 MHz HSI clock, it gives the oscillator
 * substantially more time than its normal startup interval.
 */
#define CLOCK_READY_TIMEOUT_ITERATIONS  (1000000UL)

#define RESET_HSI_CLOCK_HZ             (8000000UL)
#define CONFIGURED_SYSCLK_HZ           (72000000UL)
#define CONFIGURED_HCLK_HZ             (72000000UL)
#define CONFIGURED_PCLK1_HZ            (36000000UL)
#define CONFIGURED_PCLK2_HZ            (72000000UL)
#define CONFIGURED_ADCCLK_HZ           (12000000UL)

/*
 * These variables describe the clock tree that is actually known to be
 * active. Their initial values match the reset-default HSI clock tree.
 */
static uint32_t system_clock_hz = RESET_HSI_CLOCK_HZ;
static uint32_t ahb_clock_hz = RESET_HSI_CLOCK_HZ;
static uint32_t apb1_clock_hz = RESET_HSI_CLOCK_HZ;
static uint32_t apb2_clock_hz = RESET_HSI_CLOCK_HZ;
static uint32_t adc_clock_hz = RESET_HSI_CLOCK_HZ / 2UL;

static bool wait_for_register_value(
    volatile const uint32_t *register_address,
    uint32_t mask,
    uint32_t expected_value)
{
    uint32_t iteration;

    for (iteration = 0UL;
         iteration < CLOCK_READY_TIMEOUT_ITERATIONS;
         iteration++)
    {
        if ((*register_address & mask) == expected_value)
        {
            return true;
        }
    }

    /*
     * Check once more after the final iteration so a hardware transition
     * occurring at the timeout boundary is not incorrectly reported as a
     * failure.
     */
    return ((*register_address & mask) == expected_value);
}

static void record_reset_hsi_clock_tree(void)
{
    system_clock_hz = RESET_HSI_CLOCK_HZ;
    ahb_clock_hz = RESET_HSI_CLOCK_HZ;
    apb1_clock_hz = RESET_HSI_CLOCK_HZ;
    apb2_clock_hz = RESET_HSI_CLOCK_HZ;
    adc_clock_hz = RESET_HSI_CLOCK_HZ / 2UL;
}

system_clock_status_t system_clock_init(void)
{
    uint32_t cfgr;

    /*
     * Reset_Handler has just entered main(), so HSI is the current SYSCLK
     * and the PLL is disabled. Start with software state matching that
     * hardware reset state.
     */
    record_reset_hsi_clock_tree();

    /*
     * 1. Start the external 8 MHz crystal.
     */
    RCC->CR |= RCC_CR_HSEON;

    if (!wait_for_register_value(
            &RCC->CR,
            RCC_CR_HSERDY,
            RCC_CR_HSERDY))
    {
        RCC->CR &= ~RCC_CR_HSEON;
        return SYSTEM_CLOCK_HSE_TIMEOUT;
    }

    /*
     * 2. Prepare Flash before increasing SYSCLK above 48 MHz.
     *
     * PRFTBE = 1: enable prefetch
     * LATENCY = 010: two wait states for 48 MHz < SYSCLK <= 72 MHz
     */
    FLASH->ACR =
        FLASH_ACR_PRFTBE |
        FLASH_ACR_LATENCY_2WS;

    /*
     * 3. Build the new RCC_CFGR value while preserving unrelated fields.
     *
     * HPRE    = 0000: AHB  = SYSCLK / 1
     * PPRE1   = 100 : APB1 = HCLK / 2
     * PPRE2   = 000 : APB2 = HCLK / 1
     * ADCPRE  = 10  : ADC  = PCLK2 / 6
     * PLLSRC  = 1   : HSE is PLL input
     * PLLXTPRE= 0   : HSE is not divided by 2
     * PLLMUL  = 0111: PLL input x 9
     *
     * SW remains 00 here, so HSI remains SYSCLK during configuration.
     */
    cfgr = RCC->CFGR;

    cfgr &= ~(
        RCC_CFGR_SW_MASK |
        RCC_CFGR_HPRE_MASK |
        RCC_CFGR_PPRE1_MASK |
        RCC_CFGR_PPRE2_MASK |
        RCC_CFGR_ADCPRE_MASK |
        RCC_CFGR_PLLSRC |
        RCC_CFGR_PLLXTPRE |
        RCC_CFGR_PLLMUL_MASK);

    cfgr |=
        RCC_CFGR_HPRE_DIV1 |
        RCC_CFGR_PPRE1_DIV2 |
        RCC_CFGR_PPRE2_DIV1 |
        RCC_CFGR_ADCPRE_DIV6 |
        RCC_CFGR_PLLSRC |
        RCC_CFGR_PLLMUL_X9;

    RCC->CFGR = cfgr;

    /*
     * 4. Start the PLL and wait for it to lock.
     */
    RCC->CR |= RCC_CR_PLLON;

    if (!wait_for_register_value(
            &RCC->CR,
            RCC_CR_PLLRDY,
            RCC_CR_PLLRDY))
    {
        RCC->CR &= ~RCC_CR_PLLON;
        return SYSTEM_CLOCK_PLL_TIMEOUT;
    }

    /*
     * 5. Request PLL as SYSCLK.
     */
    RCC->CFGR =
        (RCC->CFGR & ~RCC_CFGR_SW_MASK) |
        RCC_CFGR_SW_PLL;

    /*
     * SWS is controlled by hardware. Do not report success until it says
     * that PLL is actually supplying SYSCLK.
     */
    if (!wait_for_register_value(
            &RCC->CFGR,
            RCC_CFGR_SWS_MASK,
            RCC_CFGR_SWS_PLL))
    {
        /*
         * Request HSI again. Do not disable the PLL here: if the clock
         * transition happened exactly at the timeout boundary, keeping the
         * PLL running is safer than removing a possible active clock source.
         */
        RCC->CFGR &= ~RCC_CFGR_SW_MASK;
        return SYSTEM_CLOCK_SWITCH_TIMEOUT;
    }

    /*
     * 6. The hardware has confirmed the complete 72 MHz clock tree.
     */
    system_clock_hz = CONFIGURED_SYSCLK_HZ;
    ahb_clock_hz = CONFIGURED_HCLK_HZ;
    apb1_clock_hz = CONFIGURED_PCLK1_HZ;
    apb2_clock_hz = CONFIGURED_PCLK2_HZ;
    adc_clock_hz = CONFIGURED_ADCCLK_HZ;

    return SYSTEM_CLOCK_OK;
}

uint32_t system_clock_get_sysclk_hz(void)
{
    return system_clock_hz;
}

uint32_t system_clock_get_hclk_hz(void)
{
    return ahb_clock_hz;
}

uint32_t system_clock_get_pclk1_hz(void)
{
    return apb1_clock_hz;
}

uint32_t system_clock_get_pclk2_hz(void)
{
    return apb2_clock_hz;
}

uint32_t system_clock_get_adcclk_hz(void)
{
    return adc_clock_hz;
}
