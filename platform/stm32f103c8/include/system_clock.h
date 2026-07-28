#ifndef SYSTEM_CLOCK_H
#define SYSTEM_CLOCK_H

#include <stdint.h>

/*
 * Result returned by system_clock_init().
 *
 * If initialization fails, the STM32 remains on its reset-default
 * 8 MHz internal HSI system clock.
 */
typedef enum
{
    SYSTEM_CLOCK_OK = 0,
    SYSTEM_CLOCK_HSE_TIMEOUT,
    SYSTEM_CLOCK_PLL_TIMEOUT,
    SYSTEM_CLOCK_SWITCH_TIMEOUT
} system_clock_status_t;

/*
 * Configure the STM32F103C8T6 clock tree for an 8 MHz external crystal:
 *
 *   HSE     =  8 MHz
 *   PLL     = HSE x 9
 *   SYSCLK  = 72 MHz
 *   HCLK    = 72 MHz
 *   PCLK1   = 36 MHz
 *   PCLK2   = 72 MHz
 *   ADCCLK  = 12 MHz
 */
system_clock_status_t system_clock_init(void);

/*
 * Return the frequencies established by the most recent call to
 * system_clock_init(). Before a successful initialization, these getters
 * report the reset-default HSI clock tree.
 */
uint32_t system_clock_get_sysclk_hz(void);
uint32_t system_clock_get_hclk_hz(void);
uint32_t system_clock_get_pclk1_hz(void);
uint32_t system_clock_get_pclk2_hz(void);
uint32_t system_clock_get_adcclk_hz(void);

#endif /* SYSTEM_CLOCK_H */