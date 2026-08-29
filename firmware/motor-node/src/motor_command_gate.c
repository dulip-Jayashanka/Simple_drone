#include "motor_command_gate.h"

#include "motor_node_state.h"

#include <stdbool.h>
#include <stdint.h>


volatile motor_command_gate_status_t
    g_motor_command_gate_status =
        MOTOR_COMMAND_GATE_NOT_INITIALIZED;


volatile motor_command_gate_diag_t
    g_motor_command_gate_diag;


static bool
    motor_command_gate_initialized;


static bool
commands_are_zero(
    const command_receiver_output_t *command)
{
    return
        (command->m1 == 0U) &&
        (command->m2 == 0U) &&
        (command->m3 == 0U) &&
        (command->m4 == 0U);
}


static motor_command_gate_status_t
record_status(
    motor_command_gate_status_t status)
{
    g_motor_command_gate_status =
        status;


    g_motor_command_gate_diag
        .last_status =
        (uint32_t)
        status;


    return status;
}


bool
motor_command_gate_init(void)
{
    g_motor_command_gate_diag =
        (motor_command_gate_diag_t){0};


    motor_command_gate_initialized =
        true;


    g_motor_command_gate_diag
        .init_count =
        1UL;


    return
        record_status(
            MOTOR_COMMAND_GATE_OK) ==
        MOTOR_COMMAND_GATE_OK;
}


void
motor_command_gate_reset_session(void)
{
    if (!motor_command_gate_initialized)
    {
        return;
    }


    g_motor_command_gate_diag
        .disarm_seen_since_session_start =
        0UL;


    g_motor_command_gate_diag
        .session_reset_count++;
}


motor_command_gate_status_t
motor_command_gate_apply(
    const command_receiver_output_t *command,
    bool command_stream_healthy)
{
    g_motor_command_gate_diag
        .apply_count++;


    if (!motor_command_gate_initialized)
    {
        return
            record_status(
                MOTOR_COMMAND_GATE_NOT_INITIALIZED);
    }


    if ((command ==
         (const command_receiver_output_t *)0) ||
        !command->valid)
    {
        return
            record_status(
                MOTOR_COMMAND_GATE_INVALID_ARGUMENT);
    }


    g_motor_command_gate_diag
        .last_requested_state =
        (uint32_t)
        command->requested_state;


    if (command->requested_state ==
        MOTOR_LINK_REQUEST_DISARMED)
    {
        g_motor_command_gate_diag
            .disarm_request_count++;


        if ((!motor_node_is_disarmed()) &&
            (!motor_node_state_request_disarm()))
        {
            g_motor_command_gate_diag
                .state_transition_failure_count++;


            return
                record_status(
                    MOTOR_COMMAND_GATE_STATE_TRANSITION_FAILED);
        }


        g_motor_command_gate_diag
            .disarm_seen_since_session_start =
            1UL;


        g_motor_command_gate_diag
            .disarm_accept_count++;


        return
            record_status(
                MOTOR_COMMAND_GATE_OK);
    }


    if (command->requested_state !=
        MOTOR_LINK_REQUEST_ARMED)
    {
        return
            record_status(
                MOTOR_COMMAND_GATE_INVALID_REQUESTED_STATE);
    }


    g_motor_command_gate_diag
        .arm_request_count++;


    if (motor_node_is_failsafe())
    {
        g_motor_command_gate_diag
            .arm_reject_failsafe_count++;


        return
            record_status(
                MOTOR_COMMAND_GATE_ARM_REJECT_FAILSAFE);
    }


    if (!command_stream_healthy)
    {
        g_motor_command_gate_diag
            .arm_reject_watchdog_count++;


        return
            record_status(
                MOTOR_COMMAND_GATE_ARM_REJECT_WATCHDOG);
    }


    if (motor_node_is_armed())
    {
        g_motor_command_gate_diag
            .arm_already_armed_count++;


        return
            record_status(
                MOTOR_COMMAND_GATE_OK);
    }


    if (g_motor_command_gate_diag
            .disarm_seen_since_session_start ==
        0UL)
    {
        g_motor_command_gate_diag
            .arm_reject_no_disarm_seen_count++;


        return
            record_status(
                MOTOR_COMMAND_GATE_ARM_REJECT_NO_DISARM_SEEN);
    }


    if (!commands_are_zero(
            command))
    {
        g_motor_command_gate_diag
            .arm_reject_nonzero_command_count++;


        return
            record_status(
                MOTOR_COMMAND_GATE_ARM_REJECT_NONZERO_COMMAND);
    }


    if (!motor_node_state_request_arm(
            true))
    {
        g_motor_command_gate_diag
            .state_transition_failure_count++;


        return
            record_status(
                MOTOR_COMMAND_GATE_STATE_TRANSITION_FAILED);
    }


    g_motor_command_gate_diag
        .arm_accept_count++;


    return
        record_status(
            MOTOR_COMMAND_GATE_OK);
}


bool
motor_command_gate_disarm_seen(void)
{
    return
        motor_command_gate_initialized &&
        (g_motor_command_gate_diag
             .disarm_seen_since_session_start !=
         0UL);
}
