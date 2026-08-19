#include "pid.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

static bool close_to(
    float actual,
    float expected,
    float tolerance)
{
    return
        fabsf(actual - expected) <=
        tolerance;
}

static pid_config_t make_config(
    float kp,
    float ki,
    float kd,
    float integral_limit,
    float output_limit,
    float derivative_cutoff_hz)
{
    pid_config_t config;

    config.kp = kp;
    config.ki = ki;
    config.kd = kd;

    config.integral_min =
        -integral_limit;

    config.integral_max =
        integral_limit;

    config.output_min =
        -output_limit;

    config.output_max =
        output_limit;

    config.derivative_cutoff_hz =
        derivative_cutoff_hz;

    return config;
}

static void test_invalid_configuration_is_rejected(void)
{
    pid_controller_t controller;
    pid_config_t config;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f,
            1.0f,
            1.0f,
            0.0f);

    config.kp = -1.0f;

    assert(!pid_init(
        &controller,
        &config));
}

static void test_proportional_term(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            2.0f,
            0.0f,
            0.0f,
            1.0f,
            10.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    assert(pid_update(
        &controller,
        1.0f,
        0.6f,
        0.002f,
        &output));

    assert(close_to(
        output.error,
        0.4f,
        0.000001f));

    assert(close_to(
        output.p_term,
        0.8f,
        0.000001f));

    assert(close_to(
        output.i_term,
        0.0f,
        0.000001f));

    assert(close_to(
        output.d_term,
        0.0f,
        0.000001f));

    assert(close_to(
        output.output,
        0.8f,
        0.000001f));
}

static void test_negative_error_produces_negative_output(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f,
            1.0f,
            10.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    assert(pid_update(
        &controller,
        0.0f,
        0.5f,
        0.002f,
        &output));

    assert(close_to(
        output.output,
        -0.5f,
        0.000001f));
}

static void test_integral_accumulates_and_clamps(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;
    unsigned int i;

    config =
        make_config(
            0.0f,
            1.0f,
            0.0f,
            0.01f,
            1.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    for (i = 0U; i < 5U; i++)
    {
        assert(pid_update(
            &controller,
            1.0f,
            0.0f,
            0.002f,
            &output));
    }

    assert(close_to(
        output.i_term,
        0.01f,
        0.000001f));

    assert(pid_update(
        &controller,
        1.0f,
        0.0f,
        0.002f,
        &output));

    assert(close_to(
        output.i_term,
        0.01f,
        0.000001f));

    assert(
        (output.flags &
         PID_OUTPUT_INTEGRAL_CLAMPED) != 0UL);
}

static void test_conditional_anti_windup(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;
    unsigned int i;

    config =
        make_config(
            0.0f,
            10.0f,
            0.0f,
            10.0f,
            1.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    for (i = 0U; i < 5U; i++)
    {
        assert(pid_update(
            &controller,
            1.0f,
            0.0f,
            0.02f,
            &output));
    }

    assert(close_to(
        output.i_term,
        1.0f,
        0.000001f));

    /*
     * The next positive-error integration attempt would drive the output
     * beyond +1.0, so the old integral must be held.
     */
    assert(pid_update(
        &controller,
        1.0f,
        0.0f,
        0.02f,
        &output));

    assert(close_to(
        output.i_term,
        1.0f,
        0.000001f));

    assert(
        (output.flags &
         PID_OUTPUT_INTEGRAL_HELD) != 0UL);

    /*
     * Opposite error must be allowed to unwind the integral.
     */
    assert(pid_update(
        &controller,
        -1.0f,
        0.0f,
        0.02f,
        &output));

    assert(close_to(
        output.i_term,
        0.8f,
        0.000001f));
}

static void test_derivative_on_measurement_has_damping_sign(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            0.0f,
            0.0f,
            0.1f,
            1.0f,
            100.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    /*
     * First measurement primes derivative history.
     */
    assert(pid_update(
        &controller,
        1.0f,
        0.2f,
        0.002f,
        &output));

    assert(close_to(
        output.d_term,
        0.0f,
        0.000001f));

    /*
     * Measurement rises:
     *
     *     (0.4 - 0.2) / 0.002 = +100 rad/s^2
     *
     * D = -0.1 * 100 = -10
     */
    assert(pid_update(
        &controller,
        1.0f,
        0.4f,
        0.002f,
        &output));

    assert(close_to(
        output.raw_measurement_derivative,
        100.0f,
        0.001f));

    assert(close_to(
        output.d_term,
        -10.0f,
        0.001f));

    /*
     * Measurement falls, so D should become positive.
     */
    assert(pid_update(
        &controller,
        1.0f,
        0.2f,
        0.002f,
        &output));

    assert(output.d_term > 0.0f);
}

static void test_setpoint_step_does_not_create_derivative_kick(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            0.0f,
            0.0f,
            1.0f,
            1.0f,
            100.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    assert(pid_update(
        &controller,
        0.0f,
        0.0f,
        0.002f,
        &output));

    /*
     * The setpoint changes, but the measurement does not.
     *
     * Derivative-on-measurement should therefore remain zero.
     */
    assert(pid_update(
        &controller,
        1.0f,
        0.0f,
        0.002f,
        &output));

    assert(close_to(
        output.raw_measurement_derivative,
        0.0f,
        0.000001f));

    assert(close_to(
        output.d_term,
        0.0f,
        0.000001f));
}

static void test_derivative_filter_reduces_step_magnitude(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            0.0f,
            0.0f,
            1.0f,
            1.0f,
            200.0f,
            10.0f);

    assert(pid_init(
        &controller,
        &config));

    assert(pid_update(
        &controller,
        0.0f,
        0.0f,
        0.01f,
        &output));

    assert(pid_update(
        &controller,
        0.0f,
        1.0f,
        0.01f,
        &output));

    assert(close_to(
        output.raw_measurement_derivative,
        100.0f,
        0.001f));

    assert(
        output.filtered_measurement_derivative >
        0.0f);

    assert(
        output.filtered_measurement_derivative <
        100.0f);

    assert(output.d_term < 0.0f);

    assert(
        fabsf(output.d_term) <
        100.0f);
}

static void test_output_saturation(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            10.0f,
            0.0f,
            0.0f,
            1.0f,
            1.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    assert(pid_update(
        &controller,
        1.0f,
        0.0f,
        0.002f,
        &output));

    assert(close_to(
        output.output,
        1.0f,
        0.000001f));

    assert(
        (output.flags &
         PID_OUTPUT_SATURATED_HIGH) != 0UL);
}

static void test_reset_clears_dynamic_state(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            0.0f,
            1.0f,
            1.0f,
            1.0f,
            100.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    assert(pid_update(
        &controller,
        1.0f,
        0.0f,
        0.002f,
        &output));

    assert(pid_update(
        &controller,
        1.0f,
        0.1f,
        0.002f,
        &output));

    pid_reset(&controller);

    assert(pid_update(
        &controller,
        0.0f,
        0.5f,
        0.002f,
        &output));

    assert(close_to(
        output.i_term,
        -0.001f,
        0.000001f));

    assert(close_to(
        output.d_term,
        0.0f,
        0.000001f));

    assert(
        (output.flags &
         PID_OUTPUT_DERIVATIVE_PRIMED) != 0UL);
}

static void test_invalid_dt_is_rejected_without_state_update(void)
{
    pid_controller_t controller;
    pid_output_t output;
    pid_config_t config;

    config =
        make_config(
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            10.0f,
            0.0f);

    assert(pid_init(
        &controller,
        &config));

    assert(!pid_update(
        &controller,
        1.0f,
        0.0f,
        0.0f,
        &output));

    assert(
        (output.flags &
         PID_OUTPUT_DT_INVALID) != 0UL);

    /*
     * The rejected update must not prime derivative history.
     */
    assert(pid_update(
        &controller,
        1.0f,
        0.0f,
        0.002f,
        &output));

    assert(
        (output.flags &
         PID_OUTPUT_DERIVATIVE_PRIMED) != 0UL);
}

int main(void)
{
    test_invalid_configuration_is_rejected();
    test_proportional_term();
    test_negative_error_produces_negative_output();
    test_integral_accumulates_and_clamps();
    test_conditional_anti_windup();
    test_derivative_on_measurement_has_damping_sign();
    test_setpoint_step_does_not_create_derivative_kick();
    test_derivative_filter_reduces_step_magnitude();
    test_output_saturation();
    test_reset_clears_dynamic_state();
    test_invalid_dt_is_rejected_without_state_update();

    puts("pid_test: PASS");

    return 0;
}