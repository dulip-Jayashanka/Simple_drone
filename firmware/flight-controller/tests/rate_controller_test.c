#include "rate_controller.h"

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

static pid_config_t make_axis_config(
    float kp,
    float ki,
    float kd)
{
    pid_config_t config;

    config.kp = kp;
    config.ki = ki;
    config.kd = kd;

    config.integral_min = -1.0f;
    config.integral_max = 1.0f;

    config.output_min = -10.0f;
    config.output_max = 10.0f;

    config.derivative_cutoff_hz = 0.0f;

    return config;
}

static rate_controller_config_t make_config(
    float kp,
    float ki,
    float kd)
{
    rate_controller_config_t config;

    config.roll =
        make_axis_config(kp, ki, kd);

    config.pitch =
        make_axis_config(kp, ki, kd);

    config.yaw =
        make_axis_config(kp, ki, kd);

    return config;
}

static rate_controller_input_t make_input(
    uint32_t sequence)
{
    rate_controller_input_t input;

    input =
        (rate_controller_input_t){0};

    input.sequence = sequence;

    input.timestamp_us =
        sequence * 2000UL;

    input.sample_interval_us =
        2000UL;

    input.measurements_valid = true;
    input.dt_s = 0.002f;

    return input;
}

static void test_three_axes_update_together(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f);

    assert(rate_controller_init(&config));

    input = make_input(1UL);

    input.desired_roll_rate_rad_s =
        0.40f;

    input.desired_pitch_rate_rad_s =
        -0.30f;

    input.desired_yaw_rate_rad_s =
        0.20f;

    input.measured_roll_rate_rad_s =
        0.10f;

    input.measured_pitch_rate_rad_s =
        -0.10f;

    input.measured_yaw_rate_rad_s =
        0.05f;

    assert(rate_controller_update(
        &input,
        &output));

    assert(
        (output.flags &
         RATE_CONTROL_VALID) != 0UL);

    assert(close_to(
        output.roll.output,
        0.30f,
        0.000001f));

    assert(close_to(
        output.pitch.output,
        -0.20f,
        0.000001f));

    assert(close_to(
        output.yaw.output,
        0.15f,
        0.000001f));
}

static void test_axis_independence(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f);

    assert(rate_controller_init(&config));

    input = make_input(1UL);

    input.desired_roll_rate_rad_s =
        0.5f;

    assert(rate_controller_update(
        &input,
        &output));

    assert(close_to(
        output.roll.output,
        0.5f,
        0.000001f));

    assert(close_to(
        output.pitch.output,
        0.0f,
        0.000001f));

    assert(close_to(
        output.yaw.output,
        0.0f,
        0.000001f));
}

static void test_zero_setpoint_opposes_measured_rate(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f);

    assert(rate_controller_init(&config));

    input = make_input(1UL);

    /*
     * Desired rates remain zero.
     */
    input.measured_roll_rate_rad_s =
        0.3f;

    input.measured_pitch_rate_rad_s =
        -0.2f;

    input.measured_yaw_rate_rad_s =
        0.1f;

    assert(rate_controller_update(
        &input,
        &output));

    assert(close_to(
        output.roll.output,
        -0.3f,
        0.000001f));

    assert(close_to(
        output.pitch.output,
        0.2f,
        0.000001f));

    assert(close_to(
        output.yaw.output,
        -0.1f,
        0.000001f));
}

static void test_duplicate_sequence_is_rejected(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f);

    assert(rate_controller_init(&config));

    input = make_input(7UL);

    assert(rate_controller_update(
        &input,
        &output));

    assert(!rate_controller_update(
        &input,
        &output));

    assert(
        (output.flags &
         RATE_CONTROL_DUPLICATE_SEQUENCE) != 0UL);
}

static void test_sequence_gap_resets_derivative_history(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            0.0f,
            0.0f,
            1.0f);

    assert(rate_controller_init(&config));

    input = make_input(1UL);

    input.measured_roll_rate_rad_s =
        0.0f;

    assert(rate_controller_update(
        &input,
        &output));

    /*
     * Sequence 2 is skipped.
     *
     * Even though the measurement jumps from 0 to 1, derivative history
     * should be reset, so no artificial D spike is generated.
     */
    input = make_input(3UL);

    input.measured_roll_rate_rad_s =
        1.0f;

    assert(rate_controller_update(
        &input,
        &output));

    assert(
        (output.flags &
         RATE_CONTROL_SEQUENCE_GAP) != 0UL);

    assert(close_to(
        output.roll.d_term,
        0.0f,
        0.000001f));
}

static void test_invalid_measurement_state_is_rejected(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f);

    assert(rate_controller_init(&config));

    input = make_input(1UL);
    input.measurements_valid = false;

    assert(!rate_controller_update(
        &input,
        &output));

    assert(
        (output.flags &
         RATE_CONTROL_INPUT_INVALID) != 0UL);
}

static void test_invalid_dt_is_rejected(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            1.0f,
            0.0f,
            0.0f);

    assert(rate_controller_init(&config));

    input = make_input(1UL);
    input.dt_s = 0.0f;

    assert(!rate_controller_update(
        &input,
        &output));

    assert(
        (output.flags &
         RATE_CONTROL_DT_INVALID) != 0UL);
}

static void test_reset_clears_sequence_history(void)
{
    rate_controller_config_t config;
    rate_controller_input_t input;
    rate_controller_output_t output;

    config =
        make_config(
            1.0f,
            1.0f,
            0.0f);

    assert(rate_controller_init(&config));

    input = make_input(10UL);

    input.desired_roll_rate_rad_s =
        1.0f;

    assert(rate_controller_update(
        &input,
        &output));

    rate_controller_reset();

    /*
     * The same sequence can be accepted after a complete controller reset.
     */
    assert(rate_controller_update(
        &input,
        &output));

    assert(
        (output.flags &
         RATE_CONTROL_DUPLICATE_SEQUENCE) == 0UL);

    assert(close_to(
        output.roll.i_term,
        0.002f,
        0.000001f));
}

int main(void)
{
    test_three_axes_update_together();
    test_axis_independence();
    test_zero_setpoint_opposes_measured_rate();
    test_duplicate_sequence_is_rejected();
    test_sequence_gap_resets_derivative_history();
    test_invalid_measurement_state_is_rejected();
    test_invalid_dt_is_rejected();
    test_reset_clears_sequence_history();

    puts("rate_controller_test: PASS");

    return 0;
}