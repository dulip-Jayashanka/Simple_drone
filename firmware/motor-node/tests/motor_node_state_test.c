#include "motor_node_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * HOST-SIDE MOTOR OUTPUT MOCK
 * ============================================================
 */

static bool
    mock_force_safe_succeeds;


static bool
    mock_outputs_safe;


static uint32_t
    mock_force_safe_call_count;


void
motor_outputs_force_safe(void)
{
    mock_force_safe_call_count++;


    if (mock_force_safe_succeeds)
    {
        mock_outputs_safe =
            true;
    }
}


bool
motor_outputs_are_safe(void)
{
    return
        mock_outputs_safe;
}


bool
motor_outputs_safe_state_was_requested(void)
{
    return
        mock_force_safe_call_count !=
        0UL;
}


static void
mock_reset(
    bool force_safe_succeeds)
{
    mock_force_safe_succeeds =
        force_safe_succeeds;


    mock_outputs_safe =
        false;


    mock_force_safe_call_count =
        0UL;
}


/*
 * ============================================================
 * TESTS
 * ============================================================
 */

static void
test_initialization_enters_disarmed(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    assert(
        motor_node_is_disarmed());


    assert(
        !motor_node_is_armed());


    assert(
        !motor_node_is_failsafe());


    assert(
        g_motor_node_state ==
        MOTOR_NODE_STATE_DISARMED);


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_NONE);


    assert(
        g_motor_node_state_initialized ==
        1UL);


    assert(
        g_disarmed_enforcement_count ==
        1UL);


    assert(
        g_disarmed_safety_failure_count ==
        0UL);


    assert(
        mock_force_safe_call_count ==
        1UL);
}


static void
test_disarmed_is_continuously_enforced(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    /*
     * Simulate another code path disturbing the output state.
     */
    mock_outputs_safe =
        false;


    assert(
        motor_node_state_process());


    assert(
        mock_outputs_safe);


    assert(
        g_disarmed_enforcement_count ==
        2UL);


    assert(
        mock_force_safe_call_count ==
        2UL);
}


static void
test_arm_requires_healthy_command_stream(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    assert(
        !motor_node_state_request_arm(
            false));


    assert(
        motor_node_is_disarmed());


    assert(
        g_arm_request_count ==
        1UL);


    assert(
        g_arm_accept_count ==
        0UL);


    assert(
        g_arm_reject_count ==
        1UL);
}


static void
test_arm_is_logical_only_in_phase62(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    assert(
        motor_node_is_armed());


    assert(
        g_arm_accept_count ==
        1UL);


    assert(
        mock_outputs_safe);


    /*
     * Disturb the mock output and prove ARMED Phase 6.2 still
     * forces the physical motor output back to safe.
     */
    mock_outputs_safe =
        false;


    assert(
        motor_node_state_process());


    assert(
        motor_node_is_armed());


    assert(
        mock_outputs_safe);


    assert(
        g_armed_safe_enforcement_count ==
        1UL);
}


static void
test_explicit_disarm_returns_to_disarmed(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    assert(
        motor_node_state_request_disarm());


    assert(
        motor_node_is_disarmed());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_NONE);


    assert(
        g_disarm_request_count ==
        1UL);


    assert(
        mock_outputs_safe);
}


static void
test_failsafe_is_latched_until_explicit_disarm(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    assert(
        motor_node_state_enter_failsafe(
            MOTOR_NODE_FAILSAFE_COMMAND_TIMEOUT));


    assert(
        motor_node_is_failsafe());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_COMMAND_TIMEOUT);


    assert(
        g_failsafe_entry_count ==
        1UL);


    /*
     * A healthy command stream returning is not sufficient to arm
     * directly from FAILSAFE.
     */
    assert(
        !motor_node_state_request_arm(
            true));


    assert(
        motor_node_is_failsafe());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_COMMAND_TIMEOUT);


    /*
     * Regular processing keeps FAILSAFE latched.
     */
    assert(
        motor_node_state_process());


    assert(
        motor_node_is_failsafe());


    /*
     * Explicit disarm is allowed to recover to DISARMED.
     */
    assert(
        motor_node_state_request_disarm());


    assert(
        motor_node_is_disarmed());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_NONE);
}


static void
test_unknown_state_enters_failsafe(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    g_motor_node_state =
        (motor_node_state_t)99;


    assert(
        motor_node_state_process());


    assert(
        motor_node_is_failsafe());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_UNEXPECTED_STATE);


    assert(
        g_unexpected_state_count ==
        1UL);


    assert(
        g_failsafe_entry_count ==
        1UL);


    assert(
        mock_outputs_safe);
}


static void
test_initial_safety_failure_enters_failsafe(void)
{
    mock_reset(
        false);


    assert(
        !motor_node_state_init());


    assert(
        !motor_node_is_disarmed());


    assert(
        motor_node_is_failsafe());


    assert(
        g_motor_node_state_initialized ==
        0UL);


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


    assert(
        g_disarmed_safety_failure_count ==
        1UL);
}


static void
test_armed_output_failure_enters_failsafe(void)
{
    mock_reset(
        true);


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    /*
     * From this point the safe-output request will no longer
     * successfully restore the mock hardware.
     */
    mock_force_safe_succeeds =
        false;


    mock_outputs_safe =
        false;


    assert(
        !motor_node_state_process());


    assert(
        motor_node_is_failsafe());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


    assert(
        g_armed_safety_failure_count ==
        1UL);
}


int
main(void)
{
    test_initialization_enters_disarmed();

    test_disarmed_is_continuously_enforced();

    test_arm_requires_healthy_command_stream();

    test_arm_is_logical_only_in_phase62();

    test_explicit_disarm_returns_to_disarmed();

    test_failsafe_is_latched_until_explicit_disarm();

    test_unknown_state_enters_failsafe();

    test_initial_safety_failure_enters_failsafe();

    test_armed_output_failure_enters_failsafe();


    return 0;
}