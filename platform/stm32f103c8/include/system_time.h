#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdint.h>

/*
 * PHASE 2.1.9: NEW FILE
 *
 * Public interface for the STM32F103 SysTick-based system timer.
 *
 * The system timer generates one interrupt every millisecond and
 * maintains a continuously increasing 32-bit millisecond counter.
 */


/*
 * Result returned by system_time_init().
 */
typedef enum
{
    /*
     * SysTick was configured successfully.
     */
    SYSTEM_TIME_OK = 0,

    /*
     * The supplied HCLK frequency was zero or could not produce
     * an exact 1 ms SysTick period.
     */
    SYSTEM_TIME_INVALID_CLOCK,

    /*
     * The calculated SysTick reload value was larger than the
     * 24-bit SysTick reload register can hold.
     */
    SYSTEM_TIME_RELOAD_OUT_OF_RANGE
} system_time_status_t;


/*
 * Configure the Cortex-M3 SysTick timer to generate one interrupt
 * every millisecond.
 *
 * hclk_hz must be the current AHB/core clock frequency.
 *
 * For the normal STM32F103 clock configuration:
 *
 *     HCLK = 72,000,000 Hz
 *
 * The resulting calculation is:
 *
 *     clocks per millisecond = 72,000,000 / 1,000
 *                            = 72,000
 *
 *     reload value = 72,000 - 1
 *                  = 71,999
 */
system_time_status_t system_time_init(uint32_t hclk_hz);


/*
 * Return the number of milliseconds since system_time_init()
 * successfully enabled SysTick.
 *
 * The value naturally wraps from 0xFFFFFFFF to zero after
 * approximately 49.7 days.
 */
uint32_t millis(void);


/*
 * SysTick exception handler.
 *
 * This function is declared here mainly for clarity. It is called
 * automatically through the Cortex-M3 exception vector table.
 *
 * Application code must not call it directly.
 */
void SysTick_Handler(void);

#endif /* SYSTEM_TIME_H */