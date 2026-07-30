#include "fault_handlers.h"
#include "fault_test.h"
#include "motor_outputs.h"
#include "system_clock.h"

#include <stdint.h>

/*
 * Existing clock diagnostic variables from Phase 2.1.4.
 */
volatile system_clock_status_t g_clock_status;
volatile uint32_t g_sysclk_hz;
volatile uint32_t g_hclk_hz;
volatile uint32_t g_pclk1_hz;
volatile uint32_t g_pclk2_hz;
volatile uint32_t g_adcclk_hz;

/*
 * Existing debugger-visible main-loop indicator.
 */
volatile uint32_t g_main_loop_reached;

int main(void)
{
    /*
     * Reset_Handler has already initialized .data and .bss.
     *
     * motor_outputs_force_safe() existed in Phase 2.1.5.
     *
     * PHASE 2.1.6 — UPDATED BEHAVIOUR:
     *
     * It now configures PA6, PA7, PB0 and PB1 as physical
     * GPIO outputs and holds them LOW.
     */
    motor_outputs_force_safe();

    /*
     * Existing Phase 2.1.5 call.
     *
     * Clear an old fault record before beginning normal startup.
     */
    fault_record_clear();


    /* ============================================================
     * PHASE 2.1.6 — NEW CODE START
     *
     * First safety check.
     *
     * Do not begin clock initialization unless all four motor
     * pins are already confirmed safe.
     * ============================================================ */

    if (!motor_outputs_are_safe())
    {
        /*
         * Remain here if the initial GPIO safety configuration
         * did not succeed.
         *
         * Because motor_outputs_force_safe() already attempted to
         * drive the pins LOW, continuing initialization would be
         * unsafe.
         */
        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    /* ============================================================
     * PHASE 2.1.6 — NEW CODE END
     * ============================================================ */


    /*
     * Existing Phase 2.1.4 clock initialization.
     */
    g_clock_status = system_clock_init();

    g_sysclk_hz = system_clock_get_sysclk_hz();
    g_hclk_hz = system_clock_get_hclk_hz();
    g_pclk1_hz = system_clock_get_pclk1_hz();
    g_pclk2_hz = system_clock_get_pclk2_hz();
    g_adcclk_hz = system_clock_get_adcclk_hz();

    /*
     * Existing clock-failure safety handling.
     */
    if (g_clock_status != SYSTEM_CLOCK_OK)
    {
        /*
         * PHASE 2.1.6 EFFECT:
         *
         * This existing function call now performs a real GPIO
         * shutdown instead of calling the old empty weak hook.
         */
        motor_outputs_force_safe();

        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    /*
     * Existing Phase 2.1.5 controlled fault-test hook.
     *
     * FAULT_TEST_MODE=0:
     *     Returns normally.
     *
     * FAULT_TEST_MODE=1:
     *     Triggers the Default Handler test.
     *
     * FAULT_TEST_MODE=2:
     *     Triggers the NMI test.
     *
     * FAULT_TEST_MODE=3:
     *     Triggers the HardFault test.
     */
    fault_test_run();


    /* ============================================================
     * PHASE 2.1.6 — NEW CODE START
     *
     * Second safety check.
     *
     * Recheck after clock initialization and the optional
     * controlled fault-test hook.
     * ============================================================ */

    g_motor_outputs_safe =
        motor_outputs_are_safe() ? 1UL : 0UL;

    if (g_motor_outputs_safe == 0UL)
    {
        /*
         * Try to restore the safe GPIO state immediately.
         */
        motor_outputs_force_safe();

        /*
         * Do not allow the firmware to enter its normal main loop
         * when the four pins cannot be verified safe.
         */
        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    /* ============================================================
     * PHASE 2.1.6 — NEW CODE END
     * ============================================================ */


    /*
     * Existing debugger-visible indication that startup succeeded.
     */
    g_main_loop_reached = 1UL;

    for (;;)
    {
        /*
         * Normal idle main loop.
         *
         * PWM, ESC control, CAN communication and control commands
         * are not implemented during Phase 2.1.6.
         */
        __asm volatile ("nop");
    }
}