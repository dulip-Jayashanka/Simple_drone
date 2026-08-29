#include "motor_node_state.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


static bool
    stub_actuator_ready;


static bool
    stub_actuator_safe_success;


static uint32_t
    stub_actuator_safe_count;


static uint32_t
    stub_hard_safe_count;


static bool
    stub_hard_outputs_safe;


bool
motor_actuator_set_safe(void)
{
    stub_actuator_safe_count++;


    return
        stub_actuator_ready &&
        stub_actuator_safe_success;
}


bool
motor_actuator_is_ready(void)
{
    return
        stub_actuator_ready;
}


void
motor_outputs_force_safe(void)
{
    stub_hard_safe_count++;


    stub_hard_outputs_safe =
        true;
}


bool
motor_outputs_are_safe(void)
{
    return
        stub_hard_outputs_safe;
}


bool
motor_outputs_safe_state_was_requested(void)
{
    return
        stub_hard_safe_count !=
        0UL;
}


static void
reset_stubs(void)
{
    stub_actuator_ready =
        true;


    stub_actuator_safe_success =
        true;


    stub_actuator_safe_count =
        0UL;


    stub_hard_safe_count =
        0UL;


    stub_hard_outputs_safe =
        false;
}


static void
test_pwm_init_enters_disarmed_with_esc_safe_output(void)
{
    reset_stubs();


    assert(
        motor_node_state_init());


    assert(
        motor_node_is_disarmed());


    assert(
        stub_actuator_safe_count ==
        1UL);


    assert(
        stub_hard_safe_count ==
        0UL);


    assert(
        g_disarmed_enforcement_count ==
        1UL);
}


static void
test_disarmed_continuously_reasserts_esc_safe_pwm(void)
{
    reset_stubs();


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_process());


    assert(
        stub_actuator_safe_count ==
        2UL);


    assert(
        stub_hard_safe_count ==
        0UL);
}


static void
test_armed_state_does_not_overwrite_active_motor_command(void)
{
    uint32_t
        safe_count_before_process;


    reset_stubs();


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    assert(
        motor_node_is_armed());


    safe_count_before_process =
        stub_actuator_safe_count;


    assert(
        motor_node_state_process());


    /*
     * ARMED process only checks actuator readiness. It must not write
     * the safe pulse over a fresh M1..M4 command.
     */
    assert(
        stub_actuator_safe_count ==
        safe_count_before_process);


    assert(
        g_armed_safe_enforcement_count ==
        1UL);
}


static void
test_explicit_disarm_reestablishes_esc_safe_pwm(void)
{
    uint32_t
        safe_count_before_disarm;


    reset_stubs();


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    safe_count_before_disarm =
        stub_actuator_safe_count;


    assert(
        motor_node_state_request_disarm());


    assert(
        motor_node_is_disarmed());


    assert(
        stub_actuator_safe_count ==
        (safe_count_before_disarm +
         1UL));


    assert(
        stub_hard_safe_count ==
        0UL);
}


static void
test_failsafe_immediately_reestablishes_esc_safe_pwm(void)
{
    uint32_t
        safe_count_before_failsafe;


    reset_stubs();


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    safe_count_before_failsafe =
        stub_actuator_safe_count;


    assert(
        motor_node_state_enter_failsafe(
            MOTOR_NODE_FAILSAFE_COMMAND_TIMEOUT));


    assert(
        motor_node_is_failsafe());


    assert(
        stub_actuator_safe_count ==
        (safe_count_before_failsafe +
         1UL));


    assert(
        stub_hard_safe_count ==
        0UL);


    assert(
        !motor_node_state_request_arm(
            true));


    assert(
        motor_node_is_failsafe());


    assert(
        motor_node_state_request_disarm());


    assert(
        motor_node_is_disarmed());
}


static void
test_esc_safe_failure_falls_back_to_hard_low(void)
{
    reset_stubs();


    assert(
        motor_node_state_init());


    stub_actuator_safe_success =
        false;


    assert(
        !motor_node_state_process());


    assert(
        motor_node_is_failsafe());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


    assert(
        stub_hard_safe_count ==
        1UL);
}


static void
test_armed_actuator_loss_enters_failsafe_and_hard_low(void)
{
    reset_stubs();


    assert(
        motor_node_state_init());


    assert(
        motor_node_state_request_arm(
            true));


    stub_actuator_ready =
        false;


    assert(
        !motor_node_state_process());


    assert(
        motor_node_is_failsafe());


    assert(
        g_motor_node_failsafe_reason ==
        MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE);


    assert(
        stub_hard_safe_count ==
        1UL);
}


int
main(void)
{
    test_pwm_init_enters_disarmed_with_esc_safe_output();

    test_disarmed_continuously_reasserts_esc_safe_pwm();

    test_armed_state_does_not_overwrite_active_motor_command();

    test_explicit_disarm_reestablishes_esc_safe_pwm();

    test_failsafe_immediately_reestablishes_esc_safe_pwm();

    test_esc_safe_failure_falls_back_to_hard_low();

    test_armed_actuator_loss_enters_failsafe_and_hard_low();


    puts(
        "motor_node_state_pwm_test: PASS");


    return 0;
}
