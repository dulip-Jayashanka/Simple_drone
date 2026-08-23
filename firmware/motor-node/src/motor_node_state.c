#include "motor_node_state.h"

#include "motor_outputs.h"


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
 *
 * This helper changes only the logical state/reason.
 *
 * Physical safe-output enforcement is performed separately so the
 * caller can report whether that hardware operation succeeded.
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
     * Preserve the original entry reason while already in FAILSAFE.
     *
     * This makes the debugger-visible reason identify the event that
     * originally caused the state transition.
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
 * DISARMED OUTPUT POLICY
 * ============================================================
 */

static bool
disarmed_outputs_enforce(void)
{
    motor_outputs_force_safe();


    g_disarmed_enforcement_count++;


    if (!motor_outputs_are_safe())
    {
        g_disarmed_safety_failure_count++;


        return false;
    }


    return true;
}


/*
 * ============================================================
 * ARMED OUTPUT POLICY — PHASE 6.2
 * ============================================================
 *
 * IMPORTANT:
 *
 * ARMED is only a logical application state in this phase.
 *
 * PWM has NOT been implemented yet.
 *
 * Therefore ARMED continues forcing PA6, PA7, PB0 and PB1 into
 * the existing verified-safe LOW configuration.
 *
 * Phase 6.3 will replace only the physical ARMED output policy.
 */
static bool
armed_phase62_outputs_enforce(void)
{
    motor_outputs_force_safe();


    g_armed_safe_enforcement_count++;


    if (!motor_outputs_are_safe())
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
 */

static bool
failsafe_outputs_enforce(void)
{
    motor_outputs_force_safe();


    g_failsafe_enforcement_count++;


    if (!motor_outputs_are_safe())
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


    /*
     * Every power-on/reset explicitly returns to DISARMED.
     */
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
        /*
         * Initialization never completed successfully.
         *
         * Still request the physical safe state before returning
         * failure.
         */
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

            /*
             * Phase 6.2 still holds physical motor outputs LOW.
             */
            outputs_safe =
                armed_phase62_outputs_enforce();


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

            /*
             * An impossible/corrupt application state is not silently
             * converted back to normal operation.
             *
             * It becomes a latched FAILSAFE.
             */
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
     * Arm only from initialized DISARMED state and only while the
     * command stream is currently healthy.
     *
     * FAILSAFE therefore cannot directly transition to ARMED.
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
     * Verify physical safe state once more before changing the
     * logical application state.
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


    /*
     * Logical ARMED only.
     *
     * Phase 6.2 still has no PWM.
     */
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
     * Explicit disarm always attempts physical shutdown before
     * accepting the logical transition.
     */
    motor_outputs_force_safe();


    outputs_safe =
        motor_outputs_are_safe();


    if (!outputs_safe)
    {
        latch_failsafe_state(
            MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


        return false;
    }


    /*
     * Explicit DISARM is the permitted recovery from FAILSAFE.
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
    /*
     * Logical state is latched first, then physical safety is
     * immediately enforced.
     */
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
    /*
     * FAILSAFE can also be entered because initialization itself
     * failed, so this helper does not require initialized == 1.
     */
    return
        g_motor_node_state ==
        MOTOR_NODE_STATE_FAILSAFE;
}