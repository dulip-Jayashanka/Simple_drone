#include "motor_pwm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


/*
 * motor_pwm.c contains the embedded hard-disable path. These stubs
 * satisfy that dependency for host tests; the pure timer/pulse helpers
 * exercised below never touch STM32 memory-mapped registers.
 */
static bool
    stub_outputs_safe;


void
motor_outputs_force_safe(void)
{
    stub_outputs_safe =
        true;
}


bool
motor_outputs_are_safe(void)
{
    return
        stub_outputs_safe;
}


bool
motor_outputs_safe_state_was_requested(void)
{
    return
        stub_outputs_safe;
}


static void
test_apb1_timer_clock_rule(void)
{
    assert(
        motor_pwm_timer_clock_from_apb1(
            72000000UL,
            36000000UL) ==
        72000000UL);


    assert(
        motor_pwm_timer_clock_from_apb1(
            72000000UL,
            72000000UL) ==
        72000000UL);


    assert(
        motor_pwm_timer_clock_from_apb1(
            8000000UL,
            8000000UL) ==
        8000000UL);


    assert(
        motor_pwm_timer_clock_from_apb1(
            0UL,
            36000000UL) ==
        0UL);


    assert(
        motor_pwm_timer_clock_from_apb1(
            72000000UL,
            0UL) ==
        0UL);


    assert(
        motor_pwm_timer_clock_from_apb1(
            36000000UL,
            72000000UL) ==
        0UL);


    assert(
        motor_pwm_timer_clock_from_apb1(
            72000000UL,
            30000000UL) ==
        0UL);
}


static void
test_72mhz_400hz_configuration(void)
{
    uint32_t
        prescaler;

    uint32_t
        auto_reload;


    prescaler =
        0UL;

    auto_reload =
        0UL;


    assert(
        motor_pwm_calculate_timer_config(
            72000000UL,
            400UL,
            &prescaler,
            &auto_reload));


    assert(
        prescaler ==
        71UL);


    assert(
        auto_reload ==
        2499UL);
}


static void
test_timer_configuration_rejects_invalid_inputs(void)
{
    uint32_t
        prescaler;

    uint32_t
        auto_reload;


    assert(
        !motor_pwm_calculate_timer_config(
            0UL,
            400UL,
            &prescaler,
            &auto_reload));


    assert(
        !motor_pwm_calculate_timer_config(
            72000000UL,
            0UL,
            &prescaler,
            &auto_reload));


    assert(
        !motor_pwm_calculate_timer_config(
            72000001UL,
            400UL,
            &prescaler,
            &auto_reload));


    assert(
        !motor_pwm_calculate_timer_config(
            72000000UL,
            333UL,
            &prescaler,
            &auto_reload));


    assert(
        !motor_pwm_calculate_timer_config(
            72000000UL,
            400UL,
            (uint32_t *)0,
            &auto_reload));
}


static void
test_physical_pulse_validation(void)
{
    assert(
        motor_pwm_pulse_is_valid(
            (uint16_t)
            MOTOR_ESC_SAFE_US));


    assert(
        motor_pwm_pulse_is_valid(
            (uint16_t)
            MOTOR_ESC_MIN_US));


    assert(
        motor_pwm_pulse_is_valid(
            (uint16_t)
            MOTOR_ESC_MAX_US));


    if (MOTOR_ESC_SAFE_US > 1UL)
    {
        assert(
            !motor_pwm_pulse_is_valid(
                (uint16_t)(
                    MOTOR_ESC_SAFE_US -
                    1UL)));
    }


    if (MOTOR_ESC_MAX_US <
        UINT16_MAX)
    {
        assert(
            !motor_pwm_pulse_is_valid(
                (uint16_t)(
                    MOTOR_ESC_MAX_US +
                    1UL)));
    }
}


int
main(void)
{
    test_apb1_timer_clock_rule();

    test_72mhz_400hz_configuration();

    test_timer_configuration_rejects_invalid_inputs();

    test_physical_pulse_validation();


    puts(
        "motor_pwm_test: PASS");


    return 0;
}
