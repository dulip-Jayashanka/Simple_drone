#include "system_time.h"

#include <stdint.h>

/*
 * PHASE 2.1.9: NEW FILE
 *
 * Bare-metal Cortex-M3 SysTick implementation.
 */


/* ================================================================
 * Memory-mapped register helper
 * ================================================================ */

#define REG32(address) \
    (*(volatile uint32_t *)(address))


/* ================================================================
 * Cortex-M3 SysTick registers
 * ================================================================ */

/*
 * SysTick register addresses in the Cortex-M3 System Control Space.
 */
#define SYSTICK_CTRL_ADDRESS      0xE000E010UL
#define SYSTICK_LOAD_ADDRESS      0xE000E014UL
#define SYSTICK_VAL_ADDRESS       0xE000E018UL

#define SYSTICK_CTRL              REG32(SYSTICK_CTRL_ADDRESS)
#define SYSTICK_LOAD              REG32(SYSTICK_LOAD_ADDRESS)
#define SYSTICK_VAL               REG32(SYSTICK_VAL_ADDRESS)


/* ================================================================
 * Cortex-M3 System Handler Priority register
 * ================================================================ */

/*
 * SCB_SHPR3 controls the priorities of:
 *
 * - SysTick
 * - PendSV
 *
 * SysTick priority occupies bits 31:24.
 */
#define SCB_SHPR3_ADDRESS         0xE000ED20UL
#define SCB_SHPR3                 REG32(SCB_SHPR3_ADDRESS)

#define SCB_SHPR3_SYSTICK_SHIFT   24UL
#define SCB_SHPR3_SYSTICK_MASK    \
    (0xFFUL << SCB_SHPR3_SYSTICK_SHIFT)

/*
 * STM32F103 implements four priority bits.
 *
 * 0xF0 corresponds to the lowest programmable interrupt priority.
 * This allows later safety-critical interrupts, such as motor or
 * communication interrupts, to interrupt the SysTick handler.
 */
#define SYSTICK_PRIORITY_LOWEST   0xF0UL


/* ================================================================
 * SysTick CTRL register bits
 * ================================================================ */

/*
 * ENABLE:
 *
 * 0 = SysTick counter disabled
 * 1 = SysTick counter enabled
 */
#define SYSTICK_CTRL_ENABLE       (1UL << 0)

/*
 * TICKINT:
 *
 * 0 = Reaching zero does not generate a SysTick exception
 * 1 = Reaching zero generates a SysTick exception
 */
#define SYSTICK_CTRL_TICKINT      (1UL << 1)

/*
 * CLKSOURCE:
 *
 * 0 = External SysTick clock, which is HCLK/8 on STM32F103
 * 1 = Processor clock, which is HCLK
 *
 * This implementation selects HCLK.
 */
#define SYSTICK_CTRL_CLKSOURCE    (1UL << 2)


/* ================================================================
 * SysTick timing constants
 * ================================================================ */

/*
 * One thousand timer interrupts per second creates a 1 ms period.
 */
#define SYSTEM_TIME_TICKS_PER_SECOND  1000UL

/*
 * SysTick LOAD is a 24-bit register.
 *
 * Largest reload value:
 *
 *     0x00FFFFFF
 */
#define SYSTICK_RELOAD_MAX            0x00FFFFFFUL

/*
 * The counter must receive at least one clock per tick.
 */
#define SYSTICK_MIN_CLOCKS_PER_TICK   1UL

/*
 * A reload value of 0x00FFFFFF represents 0x01000000
 * counter clocks because the timer counts reload + 1 clocks.
 */
#define SYSTICK_MAX_CLOCKS_PER_TICK   \
    (SYSTICK_RELOAD_MAX + 1UL)


/* ================================================================
 * Millisecond counter
 * ================================================================ */

/*
 * Updated by SysTick_Handler() and read by millis().
 *
 * volatile is necessary because the value changes asynchronously
 * inside an interrupt handler.
 *
 * It is intentionally a global symbol so it can be inspected using
 * GDB during Phase 2.1.9 testing. Application code should use
 * millis() instead of accessing this variable directly.
 */
volatile uint32_t g_milliseconds;


/* ================================================================
 * System-time initialization
 * ================================================================ */

system_time_status_t system_time_init(uint32_t hclk_hz)
{
    uint32_t clocks_per_millisecond;
    uint32_t reload_value;
    uint32_t handler_priorities;

    /*
     * Stop SysTick before changing its configuration.
     *
     * This also disables its interrupt temporarily.
     */
    SYSTICK_CTRL = 0UL;

    /*
     * HCLK must be a valid nonzero frequency.
     */
    if (hclk_hz == 0UL)
    {
        return SYSTEM_TIME_INVALID_CLOCK;
    }

    /*
     * Require an exact division by 1,000.
     *
     * This prevents silently creating an inaccurate millisecond
     * period when the supplied clock frequency is unsuitable.
     */
    if ((hclk_hz % SYSTEM_TIME_TICKS_PER_SECOND) != 0UL)
    {
        return SYSTEM_TIME_INVALID_CLOCK;
    }

    clocks_per_millisecond =
        hclk_hz / SYSTEM_TIME_TICKS_PER_SECOND;

    /*
     * Make sure the calculated period can be represented by the
     * 24-bit SysTick reload register.
     */
    if ((clocks_per_millisecond <
         SYSTICK_MIN_CLOCKS_PER_TICK) ||
        (clocks_per_millisecond >
         SYSTICK_MAX_CLOCKS_PER_TICK))
    {
        return SYSTEM_TIME_RELOAD_OUT_OF_RANGE;
    }

    /*
     * SysTick counts from LOAD down to zero.
     *
     * Therefore:
     *
     *     number of clocks = LOAD + 1
     *
     * For HCLK = 72 MHz:
     *
     *     clocks_per_millisecond = 72,000
     *     reload_value           = 71,999
     */
    reload_value = clocks_per_millisecond - 1UL;

    /*
     * Reset the software time counter before enabling interrupts.
     */
    g_milliseconds = 0UL;

    /*
     * Program the 24-bit reload value.
     */
    SYSTICK_LOAD = reload_value & SYSTICK_RELOAD_MAX;

    /*
     * Writing any value to VAL clears:
     *
     * - The current SysTick counter value.
     * - The COUNTFLAG status bit.
     *
     * The first countdown begins from LOAD after SysTick is enabled.
     */
    SYSTICK_VAL = 0UL;

    /*
     * Configure SysTick at the lowest programmable priority.
     *
     * Preserve the other fields in SCB_SHPR3.
     */
    handler_priorities = SCB_SHPR3;
    handler_priorities &= (uint32_t)(~SCB_SHPR3_SYSTICK_MASK);
    handler_priorities |=
        (SYSTICK_PRIORITY_LOWEST << SCB_SHPR3_SYSTICK_SHIFT);
    SCB_SHPR3 = handler_priorities;

    /*
     * Start SysTick with:
     *
     * - HCLK as its clock source.
     * - SysTick exception enabled.
     * - Counter enabled.
     */
    SYSTICK_CTRL =
        SYSTICK_CTRL_CLKSOURCE |
        SYSTICK_CTRL_TICKINT |
        SYSTICK_CTRL_ENABLE;

    return SYSTEM_TIME_OK;
}


/* ================================================================
 * Millisecond access
 * ================================================================ */

uint32_t millis(void)
{
    /*
     * An aligned 32-bit load is atomic on the Cortex-M3.
     *
     * Therefore, it is not necessary to disable interrupts while
     * reading this 32-bit counter.
     */
    return g_milliseconds;
}


/* ================================================================
 * SysTick exception handler
 * ================================================================ */

void SysTick_Handler(void)
{
    /*
     * Keep this handler extremely short.
     *
     * Do not blink the LED, process CAN messages, calculate PID
     * control, or wait inside this interrupt handler.
     *
     * The handler only records that another millisecond passed.
     */
    g_milliseconds++;
}