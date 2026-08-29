#include "motor_actuator.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


static bool
    stub_pwm_initialized;


static bool
    stub_pwm_set_success;


static uint32_t
    stub_pwm_set_call_count;


static uint32_t
    stub_pwm_safe_call_count;


static uint32_t
    stub_pwm_hard_disable_count;


static uint16_t
    captured_m1_us;

static uint16_t
    captured_m2_us;

static uint16_t
    captured_m3_us;

static uint16_t
    captured_m4_us;


volatile motor_pwm_status_t
    g_motor_pwm_status;


volatile motor_pwm_diag_t
    g_motor_pwm_diag;


uint32_t
motor_pwm_timer_clock_from_apb1(
    uint32_t hclk_hz,
    uint32_t pclk1_hz)
{
    (void)hclk_hz;
    (void)pclk1_hz;

    return 0UL;
}


bool
motor_pwm_calculate_timer_config(
    uint32_t timer_clock_hz,
    uint32_t pwm_hz,
    uint32_t *prescaler_out,
    uint32_t *auto_reload_out)
{
    (void)timer_clock_hz;
    (void)pwm_hz;
    (void)prescaler_out;
    (void)auto_reload_out;

    return false;
}


bool
motor_pwm_pulse_is_valid(
    uint16_t pulse_us)
{
    return
        ((uint32_t)pulse_us >=
         MOTOR_ESC_SAFE_US) &&
        ((uint32_t)pulse_us <=
         MOTOR_ESC_MAX_US);
}


bool
motor_pwm_init(
    uint32_t timer_clock_hz)
{
    (void)timer_clock_hz;

    return false;
}


bool
motor_pwm_set_us(
    uint16_t m1_us,
    uint16_t m2_us,
    uint16_t m3_us,
    uint16_t m4_us)
{
    stub_pwm_set_call_count++;


    if (!stub_pwm_set_success)
    {
        return false;
    }


    captured_m1_us =
        m1_us;

    captured_m2_us =
        m2_us;

    captured_m3_us =
        m3_us;

    captured_m4_us =
        m4_us;


    return true;
}


bool
motor_pwm_set_safe(void)
{
    stub_pwm_safe_call_count++;


    if (!stub_pwm_set_success)
    {
        return false;
    }


    captured_m1_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    captured_m2_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    captured_m3_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    captured_m4_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;


    return true;
}


void
motor_pwm_hard_disable(void)
{
    stub_pwm_hard_disable_count++;


    stub_pwm_initialized =
        false;
}


bool
motor_pwm_is_initialized(void)
{
    return
        stub_pwm_initialized;
}


bool
motor_pwm_configuration_is_valid(void)
{
    return
        stub_pwm_initialized;
}


static void
reset_stubs(void)
{
    stub_pwm_initialized =
        true;

    stub_pwm_set_success =
        true;


    stub_pwm_set_call_count =
        0UL;

    stub_pwm_safe_call_count =
        0UL;

    stub_pwm_hard_disable_count =
        0UL;


    captured_m1_us =
        0U;

    captured_m2_us =
        0U;

    captured_m3_us =
        0U;

    captured_m4_us =
        0U;
}


static void
test_mapping_known_points(void)
{
    uint16_t
        pulse_us;


    assert(
        motor_actuator_command_to_pulse_us(
            0U,
            &pulse_us));

    assert(
        pulse_us ==
        (uint16_t)
        MOTOR_ESC_MIN_US);


    assert(
        motor_actuator_command_to_pulse_us(
            250U,
            &pulse_us));

    assert(
        pulse_us ==
        (uint16_t)(
            MOTOR_ESC_MIN_US +
            ((250UL *
              (MOTOR_ESC_MAX_US -
               MOTOR_ESC_MIN_US)) /
             MOTOR_LINK_COMMAND_MAX)));


    assert(
        motor_actuator_command_to_pulse_us(
            500U,
            &pulse_us));

    assert(
        pulse_us ==
        (uint16_t)(
            MOTOR_ESC_MIN_US +
            ((500UL *
              (MOTOR_ESC_MAX_US -
               MOTOR_ESC_MIN_US)) /
             MOTOR_LINK_COMMAND_MAX)));


    assert(
        motor_actuator_command_to_pulse_us(
            750U,
            &pulse_us));

    assert(
        pulse_us ==
        (uint16_t)(
            MOTOR_ESC_MIN_US +
            ((750UL *
              (MOTOR_ESC_MAX_US -
               MOTOR_ESC_MIN_US)) /
             MOTOR_LINK_COMMAND_MAX)));


    assert(
        motor_actuator_command_to_pulse_us(
            1000U,
            &pulse_us));

    assert(
        pulse_us ==
        (uint16_t)
        MOTOR_ESC_MAX_US);


    assert(
        !motor_actuator_command_to_pulse_us(
            1001U,
            &pulse_us));


    assert(
        !motor_actuator_command_to_pulse_us(
            0U,
            (uint16_t *)0));
}


static void
test_init_establishes_safe_pwm(void)
{
    reset_stubs();


    assert(
        motor_actuator_init());


    assert(
        motor_actuator_is_ready());


    assert(
        stub_pwm_safe_call_count ==
        1UL);


    assert(
        captured_m1_us ==
        (uint16_t)
        MOTOR_ESC_SAFE_US);


    assert(
        g_motor_actuator_diag
            .init_count ==
        1UL);
}


static void
test_init_requires_pwm(void)
{
    reset_stubs();


    stub_pwm_initialized =
        false;


    assert(
        !motor_actuator_init());


    assert(
        !motor_actuator_is_ready());


    assert(
        g_motor_actuator_status ==
        MOTOR_ACTUATOR_STATUS_PWM_NOT_READY);
}


static void
test_mixed_commands_reach_expected_pwm(void)
{
    reset_stubs();


    assert(
        motor_actuator_init());


    assert(
        motor_actuator_apply(
            0U,
            250U,
            500U,
            750U));


    assert(
        stub_pwm_set_call_count ==
        1UL);


    assert(
        captured_m1_us ==
        1000U);

    assert(
        captured_m2_us ==
        1250U);

    assert(
        captured_m3_us ==
        1500U);

    assert(
        captured_m4_us ==
        1750U);


    assert(
        g_motor_actuator_diag
            .last_m4_command ==
        750U);
}


static void
test_invalid_command_is_rejected_before_pwm(void)
{
    reset_stubs();


    assert(
        motor_actuator_init());


    assert(
        !motor_actuator_apply(
            0U,
            250U,
            500U,
            1001U));


    assert(
        stub_pwm_set_call_count ==
        0UL);


    assert(
        g_motor_actuator_diag
            .invalid_command_count ==
        1UL);
}


static void
test_pwm_failure_is_propagated(void)
{
    reset_stubs();


    assert(
        motor_actuator_init());


    stub_pwm_set_success =
        false;


    assert(
        !motor_actuator_apply(
            100U,
            200U,
            300U,
            400U));


    assert(
        g_motor_actuator_diag
            .pwm_failure_count ==
        1UL);
}


static void
test_hard_disable_revokes_ready_state(void)
{
    reset_stubs();


    assert(
        motor_actuator_init());


    motor_actuator_hard_disable();


    assert(
        stub_pwm_hard_disable_count ==
        1UL);


    assert(
        !motor_actuator_is_ready());
}


int
main(void)
{
    test_mapping_known_points();

    test_init_establishes_safe_pwm();

    test_init_requires_pwm();

    test_mixed_commands_reach_expected_pwm();

    test_invalid_command_is_rejected_before_pwm();

    test_pwm_failure_is_propagated();

    test_hard_disable_revokes_ready_state();


    puts(
        "motor_actuator_test: PASS");


    return 0;
}
