#ifndef MOTOR_NODE_LINK_ENABLE
#define MOTOR_NODE_LINK_ENABLE 0
#endif

#ifndef MOTOR_NODE_LINK_UART_DEBUG
#define MOTOR_NODE_LINK_UART_DEBUG 0
#endif

#ifndef MOTOR_NODE_LINK_UART_RATE_HZ
#define MOTOR_NODE_LINK_UART_RATE_HZ 10
#endif

#ifndef MOTOR_COMMAND_WATCHDOG_ENABLE
#define MOTOR_COMMAND_WATCHDOG_ENABLE 0
#endif

#ifndef MOTOR_COMMAND_TIMEOUT_MS
#define MOTOR_COMMAND_TIMEOUT_MS 20UL
#endif

#ifndef MOTOR_PWM_ENABLE
#define MOTOR_PWM_ENABLE 0
#endif

#ifndef MOTOR_PWM_TEST_MODE
#define MOTOR_PWM_TEST_MODE 0
#endif

#ifndef MOTOR_ACTUATOR_TEST_MODE
#define MOTOR_ACTUATOR_TEST_MODE 0
#endif


#if MOTOR_NODE_LINK_UART_DEBUG

#define MOTOR_NODE_LINK_UART_PERIOD_MS \
    (1000UL / \
     (uint32_t)MOTOR_NODE_LINK_UART_RATE_HZ)

#endif


#if MOTOR_PWM_ENABLE && \
    (MOTOR_PWM_TEST_MODE || \
     MOTOR_ACTUATOR_TEST_MODE)

#define MOTOR_PHASE63_ISOLATED_TEST 1

#else

#define MOTOR_PHASE63_ISOLATED_TEST 0

#endif


#if MOTOR_NODE_LINK_ENABLE
#include "command_receiver.h"
#include "uart2_link.h"
#endif


#if MOTOR_COMMAND_WATCHDOG_ENABLE
#include "command_watchdog.h"
#include "motor_command_gate.h"
#endif


#if MOTOR_PWM_ENABLE
#include "motor_actuator.h"
#include "motor_pwm.h"
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


#define STATUS_LED_TOGGLE_INTERVAL_MS 500UL
#define UART_DIAG_BAUD_RATE           115200UL


#if MOTOR_NODE_LINK_ENABLE
#ifndef MOTOR_NODE_LINK_BAUD
#define MOTOR_NODE_LINK_BAUD          230400UL
#endif
#endif


/*
 * ============================================================
 * DEBUGGER-VISIBLE PLATFORM STATE
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


#if MOTOR_PWM_ENABLE
volatile uint32_t
    g_motor_pwm_timer_clock_hz;
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
        uart_diag_write_char(','));

    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)output->m2));

    uart_diag_record_result(
        uart_diag_write_char(','));

    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)output->m3));

    uart_diag_record_result(
        uart_diag_write_char(','));

    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)output->m4));

    uart_diag_record_result(
        uart_diag_write_string(
            " req="));

    uart_diag_record_result(
        uart_diag_write_uint32(
            (uint32_t)
            output->requested_state));

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
 *
 * Normal Phase 6.3 DISARMED/FAILSAFE keeps valid ESC-safe PWM.
 * This helper is deliberately stronger: PWM is disconnected and the
 * existing PA6/PA7/PB0/PB1 GPIO-LOW policy is restored.
 */

static __attribute__((noreturn))
void
halt_with_safe_outputs(
    const char *message)
{
#if MOTOR_PWM_ENABLE
    motor_pwm_hard_disable();
#else
    motor_outputs_force_safe();
#endif


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


#if MOTOR_PWM_ENABLE && \
    !MOTOR_PHASE63_ISOLATED_TEST

static bool
apply_current_runtime_output(
    const command_receiver_output_t *command)
{
    bool
        success;


    if ((command !=
         (const command_receiver_output_t *)0) &&
        command->valid &&
        motor_node_is_armed()
#if MOTOR_COMMAND_WATCHDOG_ENABLE
        && command_watchdog_is_healthy()
#endif
       )
    {
        success =
            motor_actuator_apply(
                command->m1,
                command->m2,
                command->m3,
                command->m4);
    }
    else
    {
        success =
            motor_actuator_set_safe();
    }


    if (success)
    {
        return true;
    }


    /*
     * A failed actuator write/configuration is itself a local output
     * failure. Latch FAILSAFE and ask that state to establish safe
     * output. If even that fails, the caller will hard-stop.
     */
    return
        motor_node_state_enter_failsafe(
            MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);
}

#endif


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
     * EARLIEST HARD-SAFE STATE
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


    fault_test_run();


    g_motor_outputs_safe =
        motor_outputs_are_safe() ?
        1UL :
        0UL;


    if (g_motor_outputs_safe ==
        0UL)
    {
        halt_with_safe_outputs(
            "[ERROR] Motor outputs are not hard-safe after startup");
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[SAFE] Startup motor outputs are GPIO LOW"));
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
     * PHASE 6.3 PWM / ACTUATOR INITIALIZATION
     * ========================================================
     */

#if MOTOR_PWM_ENABLE

    g_motor_pwm_timer_clock_hz =
        motor_pwm_timer_clock_from_apb1(
            g_hclk_hz,
            g_pclk1_hz);


    if (g_motor_pwm_timer_clock_hz ==
        0UL)
    {
        halt_with_safe_outputs(
            "[ERROR] Invalid APB1/TIM3 clock relationship");
    }


    if (!motor_pwm_init(
            g_motor_pwm_timer_clock_hz))
    {
        halt_with_safe_outputs(
            "[ERROR] TIM3 motor PWM initialization failed");
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_write_value_line(
            "[PWM] TIM3 clock = ",
            g_motor_pwm_timer_clock_hz,
            " Hz");

        uart_diag_write_value_line(
            "[PWM] ESC rate = ",
            MOTOR_ESC_PWM_HZ,
            " Hz");

        uart_diag_write_value_line(
            "[PWM] ESC safe = ",
            MOTOR_ESC_SAFE_US,
            " us");
    }


#if MOTOR_PWM_TEST_MODE

    if (!motor_pwm_set_us(
            (uint16_t)MOTOR_PWM_TEST_M1_US,
            (uint16_t)MOTOR_PWM_TEST_M2_US,
            (uint16_t)MOTOR_PWM_TEST_M3_US,
            (uint16_t)MOTOR_PWM_TEST_M4_US))
    {
        halt_with_safe_outputs(
            "[ERROR] Raw PWM bench-test value rejected");
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[PWM TEST] Isolated raw TIM3 output owner active"));
    }


#elif MOTOR_ACTUATOR_TEST_MODE

    if (!motor_actuator_init())
    {
        halt_with_safe_outputs(
            "[ERROR] Motor actuator initialization failed");
    }


    if (!motor_actuator_apply(
            (uint16_t)MOTOR_ACTUATOR_TEST_M1,
            (uint16_t)MOTOR_ACTUATOR_TEST_M2,
            (uint16_t)MOTOR_ACTUATOR_TEST_M3,
            (uint16_t)MOTOR_ACTUATOR_TEST_M4))
    {
        halt_with_safe_outputs(
            "[ERROR] Motor actuator bench-test command rejected");
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
        uart_diag_record_result(
            uart_diag_write_line(
                "[ACTUATOR TEST] Isolated command-to-PWM owner active"));
    }


#else

    if (!motor_actuator_init())
    {
        halt_with_safe_outputs(
            "[ERROR] Motor actuator initialization failed");
    }

#endif

#endif /* MOTOR_PWM_ENABLE */


    /*
     * ========================================================
     * NORMAL STATE / COMMUNICATION OWNERSHIP
     * ========================================================
     *
     * Raw PWM and actuator test modes intentionally stop before this
     * section so no second subsystem can fight the test owner for CCRx.
     */

#if !MOTOR_PHASE63_ISOLATED_TEST

    if (!motor_node_state_init())
    {
        halt_with_safe_outputs(
            "[ERROR] Failed to initialize motor-node safety state");
    }


    if (g_uart_diag_status ==
        UART_DIAG_OK)
    {
#if MOTOR_PWM_ENABLE
        uart_diag_record_result(
            uart_diag_write_line(
                "[STATE] Entered DISARMED with ESC-safe PWM"));
#else
        uart_diag_record_result(
            uart_diag_write_line(
                "[STATE] Entered DISARMED with GPIO LOW"));
#endif
    }


#if MOTOR_NODE_LINK_ENABLE

    command_receiver_init();


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


#if MOTOR_COMMAND_WATCHDOG_ENABLE

    if (!command_watchdog_init(
            MOTOR_COMMAND_TIMEOUT_MS))
    {
        halt_with_safe_outputs(
            "[ERROR] Command watchdog initialization failed");
    }


    if (!motor_command_gate_init())
    {
        halt_with_safe_outputs(
            "[ERROR] Motor command state gate initialization failed");
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

#if MOTOR_PWM_ENABLE
        uart_diag_record_result(
            uart_diag_write_line(
                "[LINK] Valid ARMED commands drive TIM3 PWM"));
#else
        uart_diag_record_result(
            uart_diag_write_line(
                "[LINK] Receive/validate only; PWM disabled"));
#endif

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

#endif /* MOTOR_NODE_LINK_ENABLE */

#endif /* !MOTOR_PHASE63_ISOLATED_TEST */


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


    for (;;)
    {
        current_time_ms =
            millis();


#if !MOTOR_PHASE63_ISOLATED_TEST

        /*
         * ----------------------------------------------------
         * 1. ENFORCE / VERIFY CURRENT LOCAL STATE FIRST
         * ----------------------------------------------------
         */
        if (!motor_node_state_process())
        {
            halt_with_safe_outputs(
                "[ERROR] Motor-node state output enforcement failed");
        }


        /*
         * ----------------------------------------------------
         * 2. WATCHDOG PREVIOUS COMMAND AGE BEFORE NEW RX
         * ----------------------------------------------------
         *
         * This preserves genuine communication gaps even if a new
         * frame is already waiting in the UART software buffer.
         */

#if MOTOR_COMMAND_WATCHDOG_ENABLE

        previous_watchdog_status =
            command_watchdog_get_status();


        watchdog_status =
            command_watchdog_process(
                current_time_ms);


        if ((watchdog_status ==
             COMMAND_WATCHDOG_TIMED_OUT) &&
            (previous_watchdog_status !=
             COMMAND_WATCHDOG_TIMED_OUT))
        {
            command_receiver_reset_sequence_history();

            motor_command_gate_reset_session();


            if (g_uart_diag_status ==
                UART_DIAG_OK)
            {
                uart_diag_record_result(
                    uart_diag_write_line(
                        "[WATCHDOG] Timeout; command session reset"));
            }


            if (motor_node_is_armed())
            {
                if (!motor_node_state_enter_failsafe(
                        MOTOR_NODE_FAILSAFE_COMMAND_TIMEOUT))
                {
                    halt_with_safe_outputs(
                        "[ERROR] FAILSAFE could not establish safe output");
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
         * 3. USART2 FRAME VALIDATION / PUBLICATION
         * ----------------------------------------------------
         */

#if MOTOR_NODE_LINK_ENABLE

#if MOTOR_NODE_LINK_UART_DEBUG || \
    MOTOR_COMMAND_WATCHDOG_ENABLE

        accepted_command_count =
            command_receiver_process(
                current_time_ms);


        if ((accepted_command_count !=
             0UL) &&
            command_receiver_get_latest(
                &received_command))
        {

#if MOTOR_COMMAND_WATCHDOG_ENABLE

            /*
             * The watchdog is refreshed only after the frame has
             * passed sync/version/type/length/CRC/range/state/sequence
             * validation in command_receiver.
             */
            if (!command_watchdog_note_valid_command(
                    received_command.sequence,
                    received_command.received_timestamp_ms))
            {
                halt_with_safe_outputs(
                    "[ERROR] Valid command could not refresh watchdog");
            }


            /*
             * Only after watchdog refresh does the requested state
             * gate evaluate DISARM/ARM. This keeps command freshness
             * and the local state decision in an explicit order.
             */
            (void)motor_command_gate_apply(
                &received_command,
                command_watchdog_is_healthy());

#endif


#if MOTOR_PWM_ENABLE

            /*
             * Actual M1..M4 may reach PWM only if the local motor-node
             * state is ARMED and the command watchdog is currently
             * healthy. Every other state writes the ESC-safe pulse.
             */
            if (!apply_current_runtime_output(
                    &received_command))
            {
                halt_with_safe_outputs(
                    "[ERROR] Motor actuator output failure");
            }

#endif


#if MOTOR_NODE_LINK_UART_DEBUG

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

        (void)command_receiver_process(
            current_time_ms);

#endif

#endif /* MOTOR_NODE_LINK_ENABLE */

#endif /* !MOTOR_PHASE63_ISOLATED_TEST */


        /*
         * ----------------------------------------------------
         * LED HEARTBEAT — ACTIVE IN NORMAL AND ISOLATED TEST MODE
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


        __asm volatile (
            "nop");
    }
}
