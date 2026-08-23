#ifndef MOTOR_NODE_LINK_ENABLE
#define MOTOR_NODE_LINK_ENABLE 0
#endif

#if (MOTOR_NODE_LINK_ENABLE != 0) && \
    (MOTOR_NODE_LINK_ENABLE != 1)
#error "MOTOR_NODE_LINK_ENABLE must be 0 or 1"
#endif


#ifndef MOTOR_NODE_LINK_UART_DEBUG
#define MOTOR_NODE_LINK_UART_DEBUG 0
#endif

#if (MOTOR_NODE_LINK_UART_DEBUG != 0) && \
    (MOTOR_NODE_LINK_UART_DEBUG != 1)
#error "MOTOR_NODE_LINK_UART_DEBUG must be 0 or 1"
#endif


#ifndef MOTOR_NODE_LINK_UART_RATE_HZ
#define MOTOR_NODE_LINK_UART_RATE_HZ 10
#endif


#if MOTOR_NODE_LINK_UART_DEBUG && \
    !MOTOR_NODE_LINK_ENABLE
#error "MOTOR_NODE_LINK_UART_DEBUG requires MOTOR_NODE_LINK_ENABLE=1"
#endif


#if MOTOR_NODE_LINK_UART_DEBUG && \
    ((MOTOR_NODE_LINK_UART_RATE_HZ < 1) || \
     (MOTOR_NODE_LINK_UART_RATE_HZ > 100))
#error "MOTOR_NODE_LINK_UART_RATE_HZ must be from 1 to 100"
#endif


#if MOTOR_NODE_LINK_UART_DEBUG

#define MOTOR_NODE_LINK_UART_PERIOD_MS \
    (1000UL / \
     (uint32_t)MOTOR_NODE_LINK_UART_RATE_HZ)

#endif


#if MOTOR_NODE_LINK_ENABLE
#include "command_receiver.h"
#include "uart2_link.h"
#endif

#include "fault_handlers.h"
#include "fault_test.h"
#include "motor_node_state.h"
#include "motor_outputs.h"
#include "system_clock.h"
#include "status_led.h"
#include "system_time.h"
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


#if MOTOR_NODE_LINK_ENABLE

/*
 * FMCOM Phase 6.1.
 *
 * Dedicated USART2 motor-command receive transport.
 *
 * This state exists only in a MOTOR_NODE_LINK_ENABLE=1 build.
 */
volatile uart2_link_status_t
    g_motor_command_uart_status;


#ifndef MOTOR_NODE_LINK_BAUD

#define MOTOR_NODE_LINK_BAUD \
    230400UL

#endif

#endif


#define STATUS_LED_TOGGLE_INTERVAL_MS \
    500UL

#define UART_DIAG_BAUD_RATE \
    115200UL


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


#if MOTOR_NODE_LINK_UART_DEBUG

/*
 * ============================================================
 * RECEIVED MOTOR-COMMAND UART DEBUG
 * ============================================================
 *
 * USART1 human-readable observer for commands that have already
 * been received over USART2 and accepted by command_receiver.
 *
 * Printed M1..M4 values are protocol-domain values:
 *
 *     0 ... 1000
 *
 * They are NOT PWM pulse widths.
 *
 * This debug helper does not change the command receiver, state
 * machine, safety enforcement or motor outputs.
 */
static void
motor_node_link_uart_write_received(
    const command_receiver_output_t *output)
{
    if (output ==
        (const command_receiver_output_t *)0)
    {
        return;
    }


    uart_diag_record_result(
        uart_diag_write_string(
            "[MOTOR RX] seq="));


    uart_diag_record_result(
        uart_diag_write_uint32(
            output->sequence));


    uart_diag_record_result(
        uart_diag_write_string(
            " M="));


    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)output->m1));


    uart_diag_record_result(
        uart_diag_write_char(
            ','));


    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)output->m2));


    uart_diag_record_result(
        uart_diag_write_char(
            ','));


    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)output->m3));


    uart_diag_record_result(
        uart_diag_write_char(
            ','));


    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)output->m4));


    uart_diag_record_result(
        uart_diag_write_string(
            " rx_ms="));


    uart_diag_record_result(
        uart_diag_write_uint32(
            output->received_timestamp_ms));


    uart_diag_record_result(
        uart_diag_write_string(
            "\r\n"));
}

#endif


int
main(void)
{
    uint32_t
        previous_heartbeat_ms;

    uint32_t
        current_time_ms;


#if MOTOR_NODE_LINK_UART_DEBUG

    uint32_t
        motor_node_link_previous_uart_ms;

    uint32_t
        accepted_command_count;

    bool
        motor_node_link_uart_timestamp_valid;

    command_receiver_output_t
        motor_node_link_uart_output;

#endif


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
     * USART1 remains the existing human-readable diagnostic link.
     * The optional FC-to-motor-node command path uses USART2 and is
     * compiled only when MOTOR_NODE_LINK_ENABLE=1.
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
     * safety behavior whether the communication feature is enabled
     * or disabled.
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


#if MOTOR_NODE_LINK_ENABLE

    /*
     * ========================================================
     * FMCOM PHASE 6.1
     * OPTIONAL COMMAND RECEIVER
     * ========================================================
     *
     * This complete block is excluded from a normal build when:
     *
     *     MOTOR_NODE_LINK_ENABLE=0
     *
     * Initialize the software parser before enabling USART2 RX.
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
         * A link-enabled build requires a correctly initialized
         * receive path. Fail closed if it cannot be configured.
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


#if MOTOR_NODE_LINK_UART_DEBUG

    motor_node_link_previous_uart_ms =
        0UL;


    accepted_command_count =
        0UL;


    motor_node_link_uart_timestamp_valid =
        false;


    motor_node_link_uart_output =
        (command_receiver_output_t){0};

#endif


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


#if MOTOR_NODE_LINK_UART_DEBUG

        uart_diag_write_value_line(
            "[MOTOR RX] USART1 debug rate = ",
            MOTOR_NODE_LINK_UART_RATE_HZ,
            " Hz");

#endif
    }

#endif


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
         * Optional communication processing cannot bypass this
         * state machine.
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


#if MOTOR_NODE_LINK_ENABLE

        /*
         * ----------------------------------------------------
         * 2. OPTIONAL FMCOM COMMAND RX / VALIDATION
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

#if MOTOR_NODE_LINK_UART_DEBUG

        accepted_command_count =
            command_receiver_process(
                current_time_ms);


        /*
         * USART1 debug is only an observer of newly accepted
         * received commands. The receive/validation path above is
         * identical regardless of whether debug is enabled.
         */
        if ((accepted_command_count !=
             0UL) &&
            (g_uart_diag_status ==
             UART_DIAG_OK) &&
            ((!motor_node_link_uart_timestamp_valid) ||
             ((uint32_t)(
                  current_time_ms -
                  motor_node_link_previous_uart_ms) >=
              MOTOR_NODE_LINK_UART_PERIOD_MS)) &&
            command_receiver_get_latest(
                &motor_node_link_uart_output))
        {
            motor_node_link_uart_write_received(
                &motor_node_link_uart_output);


            motor_node_link_previous_uart_ms =
                current_time_ms;


            motor_node_link_uart_timestamp_valid =
                true;
        }

#else

        (void)command_receiver_process(
            current_time_ms);

#endif

#endif


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
