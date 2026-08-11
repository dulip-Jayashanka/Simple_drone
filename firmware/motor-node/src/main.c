#include "fault_handlers.h"
#include "fault_test.h"
#include "motor_outputs.h"
#include "system_clock.h"
#include "status_led.h"
#include "system_time.h"
#include "uart_diag.h"
#include "motor_node_state.h"

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
volatile system_time_status_t g_system_time_status;
volatile uart_diag_status_t g_uart_diag_status;
volatile uint32_t g_uart_diag_output_ok;
#define STATUS_LED_TOGGLE_INTERVAL_MS  500UL
#define UART_DIAG_BAUD_RATE             115200UL

static void uart_diag_record_result(bool success)
{
    if (!success)
    {
        g_uart_diag_output_ok = 0UL;
    }
}

static void uart_diag_write_value_line(const char *label,
                                       uint32_t value,
                                       const char *unit)
{
    uart_diag_record_result(
        uart_diag_write_string(label));

    uart_diag_record_result(
        uart_diag_write_uint32(value));

    uart_diag_record_result(
        uart_diag_write_string(unit));

    uart_diag_record_result(
        uart_diag_write_string("\r\n"));
}

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
    uint32_t previous_heartbeat_ms;
    uint32_t current_time_ms;

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
    status_led_init();

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
    if (g_clock_status != SYSTEM_CLOCK_OK)
    {
        motor_outputs_force_safe();

        for (;;)
        {
            __asm volatile ("nop");
        }
    }


        /*
    * PHASE 2.1.10: UART diagnostics initialization.
    *
    * USART1 is connected to PCLK2, so it must be initialized only
    * after the clock tree is configured successfully.
    */
    g_uart_diag_status =
        uart_diag_init(g_pclk2_hz,
                    UART_DIAG_BAUD_RATE);

    g_uart_diag_output_ok =
        (g_uart_diag_status == UART_DIAG_OK) ?
        1UL :
        0UL;

    /*
    * UART failure is not a motor-safety failure.
    *
    * If UART initialization fails, firmware continues with the
    * motors safe. The debugger-visible status records the failure.
    */
    if (g_uart_diag_status == UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[BOOT] Motor node starting"));

        uart_diag_record_result(
            uart_diag_write_line(
                "[CLOCK] Initialization OK"));

        uart_diag_write_value_line(
            "[CLOCK] SYSCLK = ",
            g_sysclk_hz,
            " Hz");

        uart_diag_write_value_line(
            "[CLOCK] HCLK   = ",
            g_hclk_hz,
            " Hz");

        uart_diag_write_value_line(
            "[CLOCK] PCLK1  = ",
            g_pclk1_hz,
            " Hz");

        uart_diag_write_value_line(
            "[CLOCK] PCLK2  = ",
            g_pclk2_hz,
            " Hz");
    }

    

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
        * Report the safety failure afterward.
        */

        if (g_uart_diag_status == UART_DIAG_OK)
        {
            uart_diag_record_result(
                uart_diag_write_line(
                    "[ERROR] Motor outputs are not safe"));
        }

        /*
         * Do not allow the firmware to enter its normal main loop
         * when the four pins cannot be verified safe.
         */
        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    if (g_uart_diag_status == UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[SAFE] Motor outputs are LOW"));
    }
    



    /* ============================================================
     * PHASE 2.1.6 — NEW CODE END
     * ============================================================ */


    /*
     * Existing debugger-visible indication that startup succeeded.
     */

      /* ============================================================
     * PHASE 2.1.9: NEW CODE START
     *
     * The system clock is now confirmed as valid.
     *
     * Pass the actual HCLK frequency to system_time_init() so it
     * can calculate the correct 1 ms SysTick reload value.
     * ============================================================ */

    g_system_time_status =
        system_time_init(system_clock_get_hclk_hz());

    /*
     * If SysTick cannot create the required 1 ms time base, remain
     * in the safe state.
     *
     * The motor outputs remain LOW and the LED remains OFF.
     */
    if (g_system_time_status != SYSTEM_TIME_OK)
    {
        /*
        * Safety actions always happen before diagnostic output.
        */
        motor_outputs_force_safe();
        status_led_off();

        if (g_uart_diag_status == UART_DIAG_OK)
        {
            uart_diag_record_result(
                uart_diag_write_line(
                    "[ERROR] SysTick initialization failed"));
        }

        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    if (g_uart_diag_status == UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[TIME] SysTick initialized"));
    }

    /* ============================================================
    * PHASE 2.1.11 — NEW CODE START
    * ============================================================ */

    if (!motor_node_state_init())
    {
        motor_outputs_force_safe();
        status_led_off();

        if (g_uart_diag_status == UART_DIAG_OK)
        {
            uart_diag_record_result(
                uart_diag_write_line(
                    "[ERROR] Failed to enter DISARMED"));
        }

        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    if (g_uart_diag_status == UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[STATE] Entered DISARMED"));
    }

    /* ============================================================
    * PHASE 2.1.11 — NEW CODE END
    * ============================================================ */

    /* ============================================================
     * PHASE 2.1.9: NEW CODE END
     * ============================================================ */
    
    g_main_loop_reached = 1UL;

    status_led_on();

    previous_heartbeat_ms = millis();

    if (g_uart_diag_status == UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[READY] Main loop started"));
    }


    for (;;)
    {
         /* ========================================================
         * PHASE 2.1.9: NEW CODE START
         *
         * Non-blocking LED heartbeat.
         * ======================================================== */

        /*
         * Read millis() once during this main-loop iteration.
         */

         /* Place the Phase 2.1.11 enforcement block HERE,
       before heartbeat and other application processing. */

        if (!motor_node_state_process())
        {
            motor_outputs_force_safe();
            status_led_off();

            if (g_uart_diag_status == UART_DIAG_OK)
            {
                uart_diag_record_result(
                    uart_diag_write_line(
                        "[ERROR] DISARMED safety enforcement failed"));
            }

            for (;;)
            {
                __asm volatile ("nop");
            }
        }

         
        current_time_ms = millis();

        /*
         * Unsigned subtraction makes this check work correctly even
         * when the 32-bit millisecond counter wraps from:
         *
         *     0xFFFFFFFF -> 0x00000000
         */
        if ((uint32_t)
            (current_time_ms - previous_heartbeat_ms) >=
            STATUS_LED_TOGGLE_INTERVAL_MS)
        {
            /*
             * Begin measuring the next interval from the time at
             * which this toggle was processed.
             */
            previous_heartbeat_ms = current_time_ms;

            /*
             * The LED GPIO operation remains outside the interrupt
             * handler. SysTick_Handler() only increments the counter.
             */
            status_led_toggle();
        }

        /* ========================================================
         * PHASE 2.1.9: NEW CODE END
         * ======================================================== */

        /*
         * The main loop remains free to perform later operations.
         *
         * Future examples:
         *
         *     can_process_received_messages();
         *     motor_command_timeout_check();
         *     motor_safety_check();
         *     telemetry_process();
         *
         * No blocking delay is used here.
         */

        __asm volatile ("nop");
    }
}