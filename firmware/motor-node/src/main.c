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


/*
 * ============================================================
 * PHASE 6.2 COMMAND WATCHDOG
 * ============================================================
 */

#ifndef MOTOR_COMMAND_WATCHDOG_ENABLE
#define MOTOR_COMMAND_WATCHDOG_ENABLE 0
#endif

#if (MOTOR_COMMAND_WATCHDOG_ENABLE != 0) && \
    (MOTOR_COMMAND_WATCHDOG_ENABLE != 1)
#error "MOTOR_COMMAND_WATCHDOG_ENABLE must be 0 or 1"
#endif


#ifndef MOTOR_COMMAND_TIMEOUT_MS
#define MOTOR_COMMAND_TIMEOUT_MS 20UL
#endif


#if MOTOR_NODE_LINK_UART_DEBUG && \
    !MOTOR_NODE_LINK_ENABLE
#error "MOTOR_NODE_LINK_UART_DEBUG requires MOTOR_NODE_LINK_ENABLE=1"
#endif


#if MOTOR_COMMAND_WATCHDOG_ENABLE && \
    !MOTOR_NODE_LINK_ENABLE
#error "MOTOR_COMMAND_WATCHDOG_ENABLE requires MOTOR_NODE_LINK_ENABLE=1"
#endif


#if MOTOR_NODE_LINK_UART_DEBUG && \
    ((MOTOR_NODE_LINK_UART_RATE_HZ < 1) || \
     (MOTOR_NODE_LINK_UART_RATE_HZ > 100))
#error "MOTOR_NODE_LINK_UART_RATE_HZ must be from 1 to 100"
#endif


#if MOTOR_COMMAND_WATCHDOG_ENABLE && \
    ((MOTOR_COMMAND_TIMEOUT_MS < 1) || \
     (MOTOR_COMMAND_TIMEOUT_MS > 1000))
#error "MOTOR_COMMAND_TIMEOUT_MS must be from 1 to 1000"
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


#if MOTOR_COMMAND_WATCHDOG_ENABLE

#include "command_watchdog.h"

#endif


#include "fault_handlers.h"
#include "fault_test.h"
#include "motor_node_state.h"
#include "motor_outputs.h"
#include "status_led.h"
#include "system_clock.h"
#include "system_time.h"
#include "uart_diag.h"

#include <stdbool.h>
#include <stdint.h>


#define STATUS_LED_TOGGLE_INTERVAL_MS \
    500UL


#define UART_DIAG_BAUD_RATE \
    115200UL


#if MOTOR_NODE_LINK_ENABLE

#ifndef MOTOR_NODE_LINK_BAUD

#define MOTOR_NODE_LINK_BAUD \
    230400UL

#endif

#endif


/*
 * ============================================================
 * EXISTING CLOCK / PLATFORM DIAGNOSTICS
 * ============================================================
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


volatile uint32_t
    g_main_loop_reached;


volatile system_time_status_t
    g_system_time_status;


volatile uart_diag_status_t
    g_uart_diag_status;


volatile uint32_t
    g_uart_diag_output_ok;


#if MOTOR_NODE_LINK_ENABLE

volatile uart2_link_status_t
    g_motor_command_uart_status;

#endif


/*
 * ============================================================
 * USART1 DIAGNOSTIC HELPERS
 * ============================================================
 */

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


/*
 * ============================================================
 * RECEIVED MOTOR-COMMAND UART DEBUG
 * ============================================================
 */

#if MOTOR_NODE_LINK_UART_DEBUG

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


/*
 * ============================================================
 * FATAL FAIL-CLOSED HELPER
 * ============================================================
 */

static __attribute__((noreturn))
void
halt_with_safe_outputs(
    const char *message)
{
    motor_outputs_force_safe();


    status_led_off();


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                message));
    }


    for (;;)
    {
        __asm volatile (
            "nop");
    }
}


/*
 * ============================================================
 * MAIN
 * ============================================================
 */

int
main(void)
{
    uint32_t
        previous_heartbeat_ms;


    uint32_t
        current_time_ms;


#if MOTOR_NODE_LINK_UART_DEBUG || \
    MOTOR_COMMAND_WATCHDOG_ENABLE

    uint32_t
        accepted_command_count;


    command_receiver_output_t
        received_command;

#endif


#if MOTOR_NODE_LINK_UART_DEBUG

    uint32_t
        motor_node_link_previous_uart_ms;


    bool
        motor_node_link_uart_timestamp_valid;

#endif


#if MOTOR_COMMAND_WATCHDOG_ENABLE

    command_watchdog_status_t
        previous_watchdog_status;


    command_watchdog_status_t
        watchdog_status;

#endif


    /*
     * ========================================================
     * EARLIEST MOTOR-SAFE STATE
     * ========================================================
     */

    motor_outputs_force_safe();


    fault_record_clear();


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
        halt_with_safe_outputs(
            "[ERROR] Motor outputs are not safe");
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
        halt_with_safe_outputs(
            "[ERROR] SysTick initialization failed");
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
     * PHASE 6.2 MOTOR-NODE SAFETY STATE
     * ========================================================
     *
     * Starts in DISARMED.
     *
     * DISARMED remains enum value zero.
     *
     * Phase 6.2 adds:
     *
     *     ARMED
     *     FAILSAFE
     *
     * but PWM is still not implemented.
     */

    if (!motor_node_state_init())
    {
        halt_with_safe_outputs(
            "[ERROR] Failed to initialize motor-node safety state");
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
     * FMCOM MOTOR COMMAND LINK
     * ========================================================
     */

#if MOTOR_NODE_LINK_ENABLE

    command_receiver_init();


    /*
     * Motor-node communication direction:
     *
     *     USART2 TX = disabled
     *     USART2 RX = enabled
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
        halt_with_safe_outputs(
            "[ERROR] USART2 motor-command receiver init failed");
    }


    /*
     * ========================================================
     * PHASE 6.2 COMMAND WATCHDOG INITIALIZATION
     * ========================================================
     */

#if MOTOR_COMMAND_WATCHDOG_ENABLE

    if (!command_watchdog_init(
            MOTOR_COMMAND_TIMEOUT_MS))
    {
        halt_with_safe_outputs(
            "[ERROR] Command watchdog initialization failed");
    }

#endif


#if MOTOR_NODE_LINK_UART_DEBUG

    motor_node_link_previous_uart_ms =
        0UL;


    motor_node_link_uart_timestamp_valid =
        false;

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


#if MOTOR_COMMAND_WATCHDOG_ENABLE

        uart_diag_write_value_line(
            "[WATCHDOG] Valid-command timeout = ",
            MOTOR_COMMAND_TIMEOUT_MS,
            " ms");


        uart_diag_record_result(
            uart_diag_write_line(
                "[WATCHDOG] WAITING until first accepted fresh command"));

#endif
    }

#endif


    /*
     * ========================================================
     * STARTUP COMPLETE
     * ========================================================
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
         * 1. LOCAL MOTOR SAFETY STATE FIRST
         * ----------------------------------------------------
         *
         * Phase 6.2:
         *
         *     DISARMED  -> safe LOW
         *     ARMED     -> safe LOW
         *     FAILSAFE  -> safe LOW
         *
         * ARMED becomes a real PWM-driving state only later.
         */
        if (!motor_node_state_process())
        {
            halt_with_safe_outputs(
                "[ERROR] Motor-node state safety enforcement failed");
        }


        current_time_ms =
            millis();


        /*
         * ----------------------------------------------------
         * 2. COMMAND WATCHDOG
         * ----------------------------------------------------
         *
         * IMPORTANT ORDERING:
         *
         * Check the age of the PREVIOUS accepted command before
         * processing newly arrived UART bytes.
         *
         * Example:
         *
         *     last valid command       = 1000 ms
         *     timeout                  = 20 ms
         *     current loop             = 1025 ms
         *     a new UART frame exists
         *
         * We must still recognize that a real 25 ms communication
         * break occurred.
         */

#if MOTOR_COMMAND_WATCHDOG_ENABLE

        previous_watchdog_status =
            command_watchdog_get_status();


        watchdog_status =
            command_watchdog_process(
                current_time_ms);


        /*
         * Only act once when entering TIMED_OUT.
         */
        if ((watchdog_status ==
             COMMAND_WATCHDOG_TIMED_OUT) &&
            (previous_watchdog_status !=
             COMMAND_WATCHDOG_TIMED_OUT))
        {
            /*
             * A watchdog timeout proves that the previous command
             * communication session has been broken.
             *
             * Forget the old sequence baseline now.
             *
             * This allows:
             *
             *     old FC sequence = 50000
             *     FC loses power
             *     watchdog timeout
             *     FC reboots
             *     new FC sequence = 0
             *
             * without permanently rejecting the restarted FC as
             * stale.
             */
            command_receiver_reset_sequence_history();


            if (g_uart_diag_status ==
                UART_DIAG_OK)
            {
                uart_diag_record_result(
                    uart_diag_write_line(
                        "[WATCHDOG] Command timeout; sequence history reset"));
            }


            /*
             * Communication timeout becomes a FAILSAFE condition
             * only when the motor node was logically ARMED.
             *
             * DISARMED remains DISARMED.
             */
            if (motor_node_is_armed())
            {
                if (!motor_node_state_enter_failsafe(
                        MOTOR_NODE_FAILSAFE_COMMAND_TIMEOUT))
                {
                    halt_with_safe_outputs(
                        "[ERROR] FAILSAFE could not verify safe outputs");
                }


                if (g_uart_diag_status ==
                    UART_DIAG_OK)
                {
                    uart_diag_record_result(
                        uart_diag_write_line(
                            "[STATE] Entered FAILSAFE: command timeout"));
                }
            }
        }

#endif


        /*
         * ----------------------------------------------------
         * 3. USART2 COMMAND RX / VALIDATION
         * ----------------------------------------------------
         */

#if MOTOR_NODE_LINK_ENABLE


#if MOTOR_NODE_LINK_UART_DEBUG || \
    MOTOR_COMMAND_WATCHDOG_ENABLE

        accepted_command_count =
            command_receiver_process(
                current_time_ms);


        /*
         * command_receiver_get_latest() is reached only when at
         * least one complete command was accepted during this
         * process call.
         */
        if ((accepted_command_count !=
             0UL) &&
            command_receiver_get_latest(
                &received_command))
        {

#if MOTOR_COMMAND_WATCHDOG_ENABLE

            /*
             * This is the ONLY watchdog refresh path.
             *
             * Because command_receiver has already accepted this
             * command, it has already passed:
             *
             *     sync
             *     version
             *     type
             *     length
             *     CRC
             *     M1..M4 range
             *     sequence freshness
             *
             * Therefore:
             *
             * bad CRC             -> NO refresh
             * duplicate sequence  -> NO refresh
             * stale sequence      -> NO refresh
             * random bytes        -> NO refresh
             */
            (void)command_watchdog_note_valid_command(
                received_command.sequence,
                received_command.received_timestamp_ms);

#endif


#if MOTOR_NODE_LINK_UART_DEBUG

            /*
             * Existing received-command USART1 observer.
             *
             * It remains diagnostic only and does not participate
             * in safety/watchdog decisions.
             */
            if ((g_uart_diag_status ==
                 UART_DIAG_OK) &&
                ((!motor_node_link_uart_timestamp_valid) ||
                 ((uint32_t)(
                      current_time_ms -
                      motor_node_link_previous_uart_ms) >=
                  MOTOR_NODE_LINK_UART_PERIOD_MS)))
            {
                motor_node_link_uart_write_received(
                    &received_command);


                motor_node_link_previous_uart_ms =
                    current_time_ms;


                motor_node_link_uart_timestamp_valid =
                    true;
            }

#endif
        }


#else

        /*
         * Original Phase 6.1 path when neither watchdog nor UART
         * receive debug needs to inspect accepted_count.
         */
        (void)command_receiver_process(
            current_time_ms);

#endif


#endif


        /*
         * ----------------------------------------------------
         * 4. EXISTING LED HEARTBEAT
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
         * ====================================================
         * PHASE 6.2 IMPORTANT LIMIT
         * ====================================================
         *
         * There is intentionally NO automatic arm operation here.
         *
         * Ordinary M1..M4 MOTOR_COMMAND frames are not an ARM
         * command.
         *
         * motor_node_state_request_arm() is now available, but
         * nothing in this phase calls it from the wire protocol.
         *
         * Therefore actual hardware operation remains:
         *
         *     DISARMED
         *     motor pins LOW
         *
         * unless a future explicit arm-control layer requests the
         * logical transition.
         *
         * Future phases:
         *
         *     explicit ARM/DISARM protocol integration
         *     motor output limits
         *     TIM3 PWM
         *     ESC mapping
         */

        __asm volatile (
            "nop");
    }
}