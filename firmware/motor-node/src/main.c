#include "command_receiver.h"
#include "fault_handlers.h"
#include "fault_test.h"
#include "motor_node_state.h"
#include "motor_outputs.h"
#include "system_clock.h"
#include "status_led.h"
#include "system_time.h"
#include "uart2_link.h"
#include "uart_diag.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * Existing clock diagnostic variables.
 */
volatile system_clock_status_t
    g_clock_status;

volatile uint32_t
    g_sysclk_hz;

volatile uint32_t
    g_hclk_hz;

volatile uint32_t
    g_pclk1_hz;

volatile uint32_t
    g_pclk2_hz;

volatile uint32_t
    g_adcclk_hz;


/*
 * Existing debugger-visible application state.
 */
volatile uint32_t
    g_main_loop_reached;

volatile system_time_status_t
    g_system_time_status;

volatile uart_diag_status_t
    g_uart_diag_status;

volatile uint32_t
    g_uart_diag_output_ok;


/*
 * FMCOM Phase 6.1.
 *
 * Dedicated USART2 motor-command receive transport.
 */
volatile uart2_link_status_t
    g_motor_command_uart_status;


#define STATUS_LED_TOGGLE_INTERVAL_MS \
    500UL

#define UART_DIAG_BAUD_RATE \
    115200UL


#ifndef MOTOR_NODE_LINK_BAUD

#define MOTOR_NODE_LINK_BAUD \
    230400UL

#endif


static void
uart_diag_record_result(
    bool success)
{
    if (!success)
    {
        g_uart_diag_output_ok =
            0UL;
    }
}


static void
uart_diag_write_value_line(
    const char *label,
    uint32_t value,
    const char *unit)
{
    uart_diag_record_result(
        uart_diag_write_string(
            label));


    uart_diag_record_result(
        uart_diag_write_uint32(
            value));


    uart_diag_record_result(
        uart_diag_write_string(
            unit));


    uart_diag_record_result(
        uart_diag_write_string(
            "\r\n"));
}


int
main(void)
{
    uint32_t
        previous_heartbeat_ms;

    uint32_t
        current_time_ms;


    /*
     * ========================================================
     * EARLIEST MOTOR-SAFE STATE
     * ========================================================
     *
     * Reset_Handler has already initialized .data and .bss.
     *
     * Existing motor_outputs_force_safe() configures:
     *
     *     M1 -> PA6
     *     M2 -> PA7
     *     M3 -> PB0
     *     M4 -> PB1
     *
     * and holds all four outputs LOW.
     */

    motor_outputs_force_safe();


    /*
     * Clear previous fault record before normal startup.
     */
    fault_record_clear();


    /*
     * Physical safety verification must succeed before continuing.
     */
    if (!motor_outputs_are_safe())
    {
        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    status_led_init();


    /*
     * ========================================================
     * SYSTEM CLOCK
     * ========================================================
     */

    g_clock_status =
        system_clock_init();


    g_sysclk_hz =
        system_clock_get_sysclk_hz();


    g_hclk_hz =
        system_clock_get_hclk_hz();


    g_pclk1_hz =
        system_clock_get_pclk1_hz();


    g_pclk2_hz =
        system_clock_get_pclk2_hz();


    g_adcclk_hz =
        system_clock_get_adcclk_hz();


    if (g_clock_status !=
        SYSTEM_CLOCK_OK)
    {
        motor_outputs_force_safe();


        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    /*
     * ========================================================
     * USART1 DIAGNOSTICS
     * ========================================================
     *
     * USART1 remains completely separate from the new USART2
     * flight-controller motor-command link.
     */

    g_uart_diag_status =
        uart_diag_init(
            g_pclk2_hz,
            UART_DIAG_BAUD_RATE);


    g_uart_diag_output_ok =
        (g_uart_diag_status ==
         UART_DIAG_OK) ?
        1UL :
        0UL;


    if (g_uart_diag_status ==
        UART_DIAG_OK)
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


    /*
     * Existing controlled fault test.
     */
    fault_test_run();


    /*
     * Reverify physical safe state after startup/fault-test path.
     */
    g_motor_outputs_safe =
        motor_outputs_are_safe() ?
        1UL :
        0UL;


    if (g_motor_outputs_safe ==
        0UL)
    {
        motor_outputs_force_safe();


        if (g_uart_diag_status ==
            UART_DIAG_OK)
        {
            uart_diag_record_result(
                uart_diag_write_line(
                    "[ERROR] Motor outputs are not safe"));
        }


        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[SAFE] Motor outputs are LOW"));
    }


    /*
     * ========================================================
     * SYSTEM TIME
     * ========================================================
     */

    g_system_time_status =
        system_time_init(
            system_clock_get_hclk_hz());


    if (g_system_time_status !=
        SYSTEM_TIME_OK)
    {
        motor_outputs_force_safe();

        status_led_off();


        if (g_uart_diag_status ==
            UART_DIAG_OK)
        {
            uart_diag_record_result(
                uart_diag_write_line(
                    "[ERROR] SysTick initialization failed"));
        }


        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[TIME] SysTick initialized"));
    }


    /*
     * ========================================================
     * MOTOR-NODE SAFETY STATE
     * ========================================================
     *
     * Phase 6.1 deliberately retains the existing DISARMED-only
     * safety behavior.
     */

    if (!motor_node_state_init())
    {
        motor_outputs_force_safe();

        status_led_off();


        if (g_uart_diag_status ==
            UART_DIAG_OK)
        {
            uart_diag_record_result(
                uart_diag_write_line(
                    "[ERROR] Failed to enter DISARMED"));
        }


        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[STATE] Entered DISARMED"));
    }


    /*
     * ========================================================
     * FMCOM PHASE 6.1
     * COMMAND RECEIVER
     * ========================================================
     *
     * Initialize the software parser before enabling the UART RX
     * interrupt.
     */

    command_receiver_init();


    /*
     * Motor-node side:
     *
     *     USART2 TX = disabled
     *     USART2 RX = enabled
     *
     * Reverse status transmission can be added in a later phase.
     */
    g_motor_command_uart_status =
        uart2_link_init(
            g_pclk1_hz,
            MOTOR_NODE_LINK_BAUD,
            false,
            true);


    if (g_motor_command_uart_status !=
        UART2_LINK_OK)
    {
        /*
         * Communication is required when this phase is compiled.
         *
         * Fail closed: physical motor outputs stay safe.
         */
        motor_outputs_force_safe();

        status_led_off();


        if (g_uart_diag_status ==
            UART_DIAG_OK)
        {
            uart_diag_record_result(
                uart_diag_write_line(
                    "[ERROR] USART2 motor-command receiver init failed"));
        }


        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_write_value_line(
            "[LINK] USART2 motor-command RX = ",
            MOTOR_NODE_LINK_BAUD,
            " baud");


        uart_diag_record_result(
            uart_diag_write_line(
                "[LINK] Receive/validate only; PWM remains disabled"));
    }


    /*
     * Startup completed.
     */
    g_main_loop_reached =
        1UL;


    status_led_on();


    previous_heartbeat_ms =
        millis();


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[READY] Main loop started"));
    }


    /*
     * ========================================================
     * MAIN LOOP
     * ========================================================
     */

    for (;;)
    {
        /*
         * ----------------------------------------------------
         * 1. EXISTING LOCAL MOTOR SAFETY FIRST
         * ----------------------------------------------------
         *
         * command_receiver cannot bypass this state machine.
         */
        if (!motor_node_state_process())
        {
            motor_outputs_force_safe();

            status_led_off();


            if (g_uart_diag_status ==
                UART_DIAG_OK)
            {
                uart_diag_record_result(
                    uart_diag_write_line(
                        "[ERROR] DISARMED safety enforcement failed"));
            }


            for (;;)
            {
                __asm volatile (
                    "nop");
            }
        }


        /*
         * Read time once for this application iteration.
         */
        current_time_ms =
            millis();


        /*
         * ----------------------------------------------------
         * 2. FMCOM COMMAND RX / VALIDATION
         * ----------------------------------------------------
         *
         * command_receiver_process():
         *
         *     drains USART2 RX ring
         *     locates packet boundaries
         *     validates protocol
         *     validates CRC
         *     validates motor ranges
         *     validates sequence freshness
         *     publishes only accepted command
         *
         * IMPORTANT:
         *
         * No motor-output function is called.
         *
         * Therefore receipt of even a valid packet cannot cause
         * an ESC or motor output during Phase 6.1.
         */
        (void)command_receiver_process(
            current_time_ms);


        /*
         * ----------------------------------------------------
         * 3. EXISTING NON-BLOCKING LED HEARTBEAT
         * ----------------------------------------------------
         */

        if ((uint32_t)(
                current_time_ms -
                previous_heartbeat_ms) >=
            STATUS_LED_TOGGLE_INTERVAL_MS)
        {
            previous_heartbeat_ms =
                current_time_ms;


            status_led_toggle();
        }


        /*
         * Future motor-node phases will add:
         *
         *     command_watchdog_process()
         *
         *     arm/disarm transition logic
         *
         *     motor_output_limits
         *
         *     TIM3 PWM update
         *
         * None is intentionally implemented here.
         */

        __asm volatile (
            "nop");
    }
}