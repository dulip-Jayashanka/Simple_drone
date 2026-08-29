#include "motor_command_gate.h"
#include "motor_node_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


static motor_node_state_t
    stub_state;


static bool
    stub_transition_success;


bool
motor_node_state_request_arm(
    bool command_stream_healthy)
{
    if ((!stub_transition_success) ||
        (!command_stream_healthy) ||
        (stub_state !=
         MOTOR_NODE_STATE_DISARMED))
    {
        return false;
    }


    stub_state =
        MOTOR_NODE_STATE_ARMED;


    return true;
}


bool
motor_node_state_request_disarm(void)
{
    if (!stub_transition_success)
    {
        return false;
    }


    stub_state =
        MOTOR_NODE_STATE_DISARMED;


    return true;
}


bool
motor_node_is_disarmed(void)
{
    return
        stub_state ==
        MOTOR_NODE_STATE_DISARMED;
}


bool
motor_node_is_armed(void)
{
    return
        stub_state ==
        MOTOR_NODE_STATE_ARMED;
}


bool
motor_node_is_failsafe(void)
{
    return
        stub_state ==
        MOTOR_NODE_STATE_FAILSAFE;
}


static command_receiver_output_t
make_command(
    motor_link_requested_state_t requested_state,
    uint16_t motor_value)
{
    command_receiver_output_t command;


    command =
        (command_receiver_output_t){0};


    command.valid = true;

    command.requested_state =
        requested_state;

    command.m1 = motor_value;
    command.m2 = motor_value;
    command.m3 = motor_value;
    command.m4 = motor_value;


    return command;
}


static void
reset_all(void)
{
    stub_state =
        MOTOR_NODE_STATE_DISARMED;

    stub_transition_success =
        true;


    assert(
        motor_command_gate_init());
}


static void
test_requires_disarm_before_arm(void)
{
    command_receiver_output_t command;


    reset_all();


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            0U);


    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_ARM_REJECT_NO_DISARM_SEEN);

    assert(
        stub_state ==
        MOTOR_NODE_STATE_DISARMED);


    /*
     * DISARM is honored even if its M fields are non-zero.
     */
    command =
        make_command(
            MOTOR_LINK_REQUEST_DISARMED,
            900U);


    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);

    assert(
        motor_command_gate_disarm_seen());


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            0U);


    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);

    assert(
        stub_state ==
        MOTOR_NODE_STATE_ARMED);
}


static void
test_zero_required_only_for_arm_transition(void)
{
    command_receiver_output_t command;


    reset_all();


    command =
        make_command(
            MOTOR_LINK_REQUEST_DISARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            100U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_ARM_REJECT_NONZERO_COMMAND);

    assert(
        stub_state ==
        MOTOR_NODE_STATE_DISARMED);


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            700U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);

    assert(
        stub_state ==
        MOTOR_NODE_STATE_ARMED);
}


static void
test_watchdog_and_failsafe_rules(void)
{
    command_receiver_output_t command;


    reset_all();


    command =
        make_command(
            MOTOR_LINK_REQUEST_DISARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            false) ==
        MOTOR_COMMAND_GATE_ARM_REJECT_WATCHDOG);

    assert(
        stub_state ==
        MOTOR_NODE_STATE_DISARMED);


    stub_state =
        MOTOR_NODE_STATE_FAILSAFE;


    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_ARM_REJECT_FAILSAFE);

    assert(
        stub_state ==
        MOTOR_NODE_STATE_FAILSAFE);


    command =
        make_command(
            MOTOR_LINK_REQUEST_DISARMED,
            800U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);

    assert(
        stub_state ==
        MOTOR_NODE_STATE_DISARMED);
}


static void
test_session_reset_reestablishes_interlock(void)
{
    command_receiver_output_t command;


    reset_all();


    command =
        make_command(
            MOTOR_LINK_REQUEST_DISARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);


    motor_command_gate_reset_session();


    assert(
        !motor_command_gate_disarm_seen());


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_ARM_REJECT_NO_DISARM_SEEN);


    command =
        make_command(
            MOTOR_LINK_REQUEST_DISARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);


    command =
        make_command(
            MOTOR_LINK_REQUEST_ARMED,
            0U);

    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_OK);
}


static void
test_transition_failure_is_reported(void)
{
    command_receiver_output_t command;


    reset_all();

    stub_state =
        MOTOR_NODE_STATE_ARMED;

    stub_transition_success =
        false;


    command =
        make_command(
            MOTOR_LINK_REQUEST_DISARMED,
            0U);


    assert(
        motor_command_gate_apply(
            &command,
            true) ==
        MOTOR_COMMAND_GATE_STATE_TRANSITION_FAILED);


    assert(
        !motor_command_gate_disarm_seen());
}


int
main(void)
{
    test_requires_disarm_before_arm();
    test_zero_required_only_for_arm_transition();
    test_watchdog_and_failsafe_rules();
    test_session_reset_reestablishes_interlock();
    test_transition_failure_is_reported();


    puts(
        "motor_command_gate_test: PASS");


    return 0;
}
