#include "motor_node_state.h"

#include "motor_outputs.h"

#ifndef MOTOR_PWM_ENABLE
#define MOTOR_PWM_ENABLE 0
#endif

#if MOTOR_PWM_ENABLE
#include "motor_actuator.h"
#endif


/*
 * DISARMED is zero so reset-time .bss remains fail-safe.
 */
volatile motor_node_state_t
    g_motor_node_state =
        MOTOR_NODE_STATE_DISARMED;


volatile motor_node_failsafe_reason_t
    g_motor_node_failsafe_reason =
        MOTOR_NODE_FAILSAFE_NONE;


volatile uint32_t
    g_motor_node_state_initialized;


volatile uint32_t
    g_disarmed_enforcement_count;


volatile uint32_t
    g_armed_safe_enforcement_count;


volatile uint32_t
    g_failsafe_enforcement_count;


volatile uint32_t
    g_disarmed_safety_failure_count;


volatile uint32_t
    g_armed_safety_failure_count;


volatile uint32_t
    g_failsafe_safety_failure_count;


volatile uint32_t
    g_unexpected_state_count;


volatile uint32_t
    g_arm_request_count;


volatile uint32_t
    g_arm_accept_count;


volatile uint32_t
    g_arm_reject_count;


volatile uint32_t
    g_disarm_request_count;


volatile uint32_t
    g_failsafe_entry_count;


/*
 * ============================================================
 * FAILSAFE LATCH
 * ============================================================
 */

static void
latch_failsafe_state(
    motor_node_failsafe_reason_t reason)
{
    if (reason ==
        MOTOR_NODE_FAILSAFE_NONE)
    {
        reason =
            MOTOR_NODE_FAILSAFE_UNEXPECTED_STATE;
    }


    /*
     * Preserve the reason that originally caused FAILSAFE entry.
     */
    if (g_motor_node_state !=
        MOTOR_NODE_STATE_FAILSAFE)
    {
        g_motor_node_state =
            MOTOR_NODE_STATE_FAILSAFE;


        g_motor_node_failsafe_reason =
            reason;


        g_failsafe_entry_count++;
    }
}


/*
 * ============================================================
 * NORMAL DISARMED OUTPUT POLICY
 * ============================================================
 *
 * Phase 6.3 deliberately distinguishes two safe-output concepts:
 *
 *     normal ESC-safe state
 *         TIM3 remains active at MOTOR_ESC_SAFE_US
 *
 *     hard/fatal safe state
 *         PA6/PA7/PB0/PB1 are ordinary GPIO LOW
 *
 * With MOTOR_PWM_ENABLE=0 the previous Phase 6.2 GPIO-LOW policy is
 * preserved exactly.
 */

static bool
disarmed_outputs_enforce(void)
{
    bool
        outputs_safe;


#if MOTOR_PWM_ENABLE

    outputs_safe =
        motor_actuator_set_safe();


    if (!outputs_safe)
    {
        /*
         * If normal ESC-safe PWM itself cannot be established, fall
         * back to the strongest hardware shutdown immediately.
         */
        motor_outputs_force_safe();
    }

#else

    motor_outputs_force_safe();


    outputs_safe =
        motor_outputs_are_safe();

#endif


    g_disarmed_enforcement_count++;


    if (!outputs_safe)
    {
        g_disarmed_safety_failure_count++;


        return false;
    }


    return true;
}


/*
 * ============================================================
 * ARMED OUTPUT POLICY
 * ============================================================
 *
 * In Phase 6.3 ARMED no longer overwrites the current motor command
 * with a safe value on every loop. The fresh validated command path
 * in main.c owns M1..M4 updates while ARMED.
 *
 * State processing only verifies that the actuator/PWM boundary is
 * still ready. The historical debugger counter name is retained for
 * compatibility; in PWM builds it counts ARMED output-readiness
 * checks rather than repeated GPIO-safe writes.
 */

static bool
armed_outputs_enforce(void)
{
    bool
        outputs_ready;


#if MOTOR_PWM_ENABLE

    outputs_ready =
        motor_actuator_is_ready();


    if (!outputs_ready)
    {
        motor_outputs_force_safe();
    }

#else

    motor_outputs_force_safe();


    outputs_ready =
        motor_outputs_are_safe();

#endif


    g_armed_safe_enforcement_count++;


    if (!outputs_ready)
    {
        g_armed_safety_failure_count++;


        return false;
    }


    return true;
}


/*
 * ============================================================
 * FAILSAFE OUTPUT POLICY
 * ============================================================
 *
 * Normal command-timeout FAILSAFE uses the ESC-safe PWM value when
 * PWM is available. A PWM/output subsystem failure itself falls back
 * to hard GPIO LOW.
 */

static bool
failsafe_outputs_enforce(void)
{
    bool
        outputs_safe;


#if MOTOR_PWM_ENABLE

    outputs_safe =
        motor_actuator_set_safe();


    if (!outputs_safe)
    {
        motor_outputs_force_safe();
    }

#else

    motor_outputs_force_safe();


    outputs_safe =
        motor_outputs_are_safe();

#endif


    g_failsafe_enforcement_count++;


    if (!outputs_safe)
    {
        g_failsafe_safety_failure_count++;


        return false;
    }


    return true;
}


/*
 * ============================================================
 * INITIALIZATION
 * ============================================================
 */

bool
motor_node_state_init(void)
{
    bool
        outputs_safe;


    g_motor_node_state_initialized =
        0UL;


    g_disarmed_enforcement_count =
        0UL;


    g_armed_safe_enforcement_count =
        0UL;


    g_failsafe_enforcement_count =
        0UL;


    g_disarmed_safety_failure_count =
        0UL;


    g_armed_safety_failure_count =
        0UL;


    g_failsafe_safety_failure_count =
        0UL;


    g_unexpected_state_count =
        0UL;


    g_arm_request_count =
        0UL;


    g_arm_accept_count =
        0UL;


    g_arm_reject_count =
        0UL;


    g_disarm_request_count =
        0UL;


    g_failsafe_entry_count =
        0UL;


    g_motor_node_state =
        MOTOR_NODE_STATE_DISARMED;


    g_motor_node_failsafe_reason =
        MOTOR_NODE_FAILSAFE_NONE;


    outputs_safe =
        disarmed_outputs_enforce();


    if (!outputs_safe)
    {
        latch_failsafe_state(
            MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


        return false;
    }


    g_motor_node_state_initialized =
        1UL;


    return true;
}


/*
 * ============================================================
 * PERIODIC STATE PROCESSING
 * ============================================================
 */

bool
motor_node_state_process(void)
{
    bool
        outputs_safe;


    if (g_motor_node_state_initialized ==
        0UL)
    {
        motor_outputs_force_safe();


        return false;
    }


    switch (g_motor_node_state)
    {
        case MOTOR_NODE_STATE_DISARMED:

            outputs_safe =
                disarmed_outputs_enforce();


            if (!outputs_safe)
            {
                latch_failsafe_state(
                    MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);
            }


            return
                outputs_safe;


        case MOTOR_NODE_STATE_ARMED:

            outputs_safe =
                armed_outputs_enforce();


            if (!outputs_safe)
            {
                latch_failsafe_state(
                    MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);
            }


            return
                outputs_safe;


        case MOTOR_NODE_STATE_FAILSAFE:

            return
                failsafe_outputs_enforce();


        default:

            g_unexpected_state_count++;


            latch_failsafe_state(
                MOTOR_NODE_FAILSAFE_UNEXPECTED_STATE);


            return
                failsafe_outputs_enforce();
    }
}


/*
 * ============================================================
 * ARM REQUEST
 * ============================================================
 */

bool
motor_node_state_request_arm(
    bool command_stream_healthy)
{
    bool
        outputs_safe;


    g_arm_request_count++;


    /*
     * Direct FAILSAFE -> ARMED remains impossible.
     */
    if ((g_motor_node_state_initialized ==
         0UL) ||
        (g_motor_node_state !=
         MOTOR_NODE_STATE_DISARMED) ||
        !command_stream_healthy)
    {
        g_arm_reject_count++;


        return false;
    }


    /*
     * Re-establish/verify the normal DISARMED physical state before
     * granting the logical ARM transition. The state gate separately
     * requires an all-zero ARM transition packet.
     */
    outputs_safe =
        disarmed_outputs_enforce();


    if (!outputs_safe)
    {
        g_arm_reject_count++;


        latch_failsafe_state(
            MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


        return false;
    }


    g_motor_node_state =
        MOTOR_NODE_STATE_ARMED;


    g_motor_node_failsafe_reason =
        MOTOR_NODE_FAILSAFE_NONE;


    g_arm_accept_count++;


    return true;
}


/*
 * ============================================================
 * DISARM REQUEST
 * ============================================================
 */

bool
motor_node_state_request_disarm(void)
{
    bool
        outputs_safe;


    g_disarm_request_count++;


    if (g_motor_node_state_initialized ==
        0UL)
    {
        motor_outputs_force_safe();


        return false;
    }


    /*
     * DISARM first establishes the normal physical safe policy and
     * only then accepts the logical transition. In a PWM build this
     * is four ESC-safe pulses; without PWM it remains hard GPIO LOW.
     */
    outputs_safe =
        disarmed_outputs_enforce();


    if (!outputs_safe)
    {
        latch_failsafe_state(
            MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


        return false;
    }


    /*
     * Explicit DISARM remains the permitted FAILSAFE recovery path.
     */
    g_motor_node_state =
        MOTOR_NODE_STATE_DISARMED;


    g_motor_node_failsafe_reason =
        MOTOR_NODE_FAILSAFE_NONE;


    return true;
}


/*
 * ============================================================
 * FAILSAFE ENTRY
 * ============================================================
 */

bool
motor_node_state_enter_failsafe(
    motor_node_failsafe_reason_t reason)
{
    latch_failsafe_state(
        reason);


    return
        failsafe_outputs_enforce();
}


/*
 * ============================================================
 * STATE HELPERS
 * ============================================================
 */

bool
motor_node_is_disarmed(void)
{
    return
        (g_motor_node_state_initialized !=
         0UL) &&
        (g_motor_node_state ==
         MOTOR_NODE_STATE_DISARMED);
}


bool
motor_node_is_armed(void)
{
    return
        (g_motor_node_state_initialized !=
         0UL) &&
        (g_motor_node_state ==
         MOTOR_NODE_STATE_ARMED);
}


bool
motor_node_is_failsafe(void)
{
    return
        g_motor_node_state ==
        MOTOR_NODE_STATE_FAILSAFE;
}
