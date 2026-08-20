#include "attitude_controller.h"

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


static attitude_controller_config_t
make_config(void)
{
    attitude_controller_config_t config;

    /*
     * Host-test values only.
     *
     * These values are selected to make expected mathematical
     * results simple to calculate.
     */
    config.roll_gain_per_s =
        2.0f;

    config.pitch_gain_per_s =
        3.0f;


    config.max_roll_rate_rad_s =
        1.0f;

    config.max_pitch_rate_rad_s =
        1.5f;


    return config;
}


static attitude_controller_input_t
make_input(
    uint32_t sequence)
{
    attitude_controller_input_t input;

    input =
        (attitude_controller_input_t){0};


    input.sequence =
        sequence;


    /*
     * Simulate approximately 100 Hz outer-loop timestamps.
     */
    input.timestamp_us =
        sequence * 10000UL;


    input.attitude_valid =
        true;


    return input;
}


/*
 * ------------------------------------------------------------
 * LEVEL + LEVEL = ZERO RATE
 * ------------------------------------------------------------
 */

static void
test_zero_error_produces_zero_rates(void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    /*
     * Default desired and estimated angles are all zero.
     */
    assert(
        attitude_controller_update(
            &input,
            &output));


    assert(
        (output.flags &
         ATTITUDE_CONTROL_VALID) !=
        0UL);


    assert(
        close_to(
            output
                .desired_roll_rate_rad_s,
            0.0f,
            0.000001f));


    assert(
        close_to(
            output
                .desired_pitch_rate_rad_s,
            0.0f,
            0.000001f));
}


/*
 * ------------------------------------------------------------
 * POSITIVE ROLL ANGLE -> NEGATIVE RATE REQUEST
 * ------------------------------------------------------------
 */

static void
test_positive_roll_attitude_is_corrected_negative(
    void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    /*
     * Target roll = 0
     * Actual roll = +0.20 rad
     *
     * error =
     *     0 - 0.20
     *     = -0.20
     *
     * roll gain = 2
     *
     * desired rate =
     *     2 * -0.20
     *     = -0.40 rad/s
     */
    input.estimated_roll_rad =
        0.20f;


    assert(
        attitude_controller_update(
            &input,
            &output));


    assert(
        close_to(
            output.roll_error_rad,
            -0.20f,
            0.000001f));


    assert(
        close_to(
            output
                .desired_roll_rate_rad_s,
            -0.40f,
            0.000001f));
}


/*
 * ------------------------------------------------------------
 * NEGATIVE ROLL ANGLE -> POSITIVE RATE REQUEST
 * ------------------------------------------------------------
 */

static void
test_negative_roll_attitude_is_corrected_positive(
    void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    input.estimated_roll_rad =
        -0.20f;


    assert(
        attitude_controller_update(
            &input,
            &output));


    assert(
        close_to(
            output
                .desired_roll_rate_rad_s,
            0.40f,
            0.000001f));
}


/*
 * ------------------------------------------------------------
 * PITCH SIGN + AXIS INDEPENDENCE
 * ------------------------------------------------------------
 */

static void
test_pitch_sign_and_axis_independence(
    void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    input.estimated_pitch_rad =
        0.10f;


    assert(
        attitude_controller_update(
            &input,
            &output));


    /*
     * Roll has no error.
     */
    assert(
        close_to(
            output
                .desired_roll_rate_rad_s,
            0.0f,
            0.000001f));


    /*
     * Pitch:
     *
     *     0 - 0.10 = -0.10 rad
     *
     * gain = 3
     *
     * rate = -0.30 rad/s
     */
    assert(
        close_to(
            output.pitch_error_rad,
            -0.10f,
            0.000001f));


    assert(
        close_to(
            output
                .desired_pitch_rate_rad_s,
            -0.30f,
            0.000001f));
}


/*
 * ------------------------------------------------------------
 * NON-ZERO REQUESTED ATTITUDE
 * ------------------------------------------------------------
 */

static void
test_requested_nonzero_attitude(void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    /*
     * Roll:
     *
     * target = 0.30
     * actual = 0.10
     * error  = 0.20
     * gain   = 2
     * rate   = 0.40
     */
    input.desired_roll_rad =
        0.30f;

    input.estimated_roll_rad =
        0.10f;


    /*
     * Pitch:
     *
     * target = -0.20
     * actual = -0.05
     * error  = -0.15
     * gain   = 3
     * rate   = -0.45
     */
    input.desired_pitch_rad =
        -0.20f;

    input.estimated_pitch_rad =
        -0.05f;


    assert(
        attitude_controller_update(
            &input,
            &output));


    assert(
        close_to(
            output
                .desired_roll_rate_rad_s,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output
                .desired_pitch_rate_rad_s,
            -0.45f,
            0.000001f));
}


/*
 * ------------------------------------------------------------
 * RATE LIMITS
 * ------------------------------------------------------------
 */

static void
test_rate_limits(void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    input.desired_roll_rad =
        2.0f;

    input.desired_pitch_rad =
        -2.0f;


    assert(
        attitude_controller_update(
            &input,
            &output));


    /*
     * Roll raw:
     *
     * 2 * 2 = +4 rad/s
     *
     * but max roll rate = +1 rad/s.
     */
    assert(
        close_to(
            output
                .raw_roll_rate_rad_s,
            4.0f,
            0.000001f));


    assert(
        close_to(
            output
                .desired_roll_rate_rad_s,
            1.0f,
            0.000001f));


    /*
     * Pitch raw:
     *
     * 3 * -2 = -6 rad/s
     *
     * but max pitch rate = -1.5 rad/s.
     */
    assert(
        close_to(
            output
                .raw_pitch_rate_rad_s,
            -6.0f,
            0.000001f));


    assert(
        close_to(
            output
                .desired_pitch_rate_rad_s,
            -1.5f,
            0.000001f));


    assert(
        (output.flags &
         ATTITUDE_CONTROL_ROLL_RATE_LIMITED) !=
        0UL);


    assert(
        (output.flags &
         ATTITUDE_CONTROL_PITCH_RATE_LIMITED) !=
        0UL);
}


/*
 * ------------------------------------------------------------
 * INVALID ATTITUDE
 * ------------------------------------------------------------
 */

static void
test_invalid_attitude_is_rejected(void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    input.attitude_valid =
        false;


    assert(
        !attitude_controller_update(
            &input,
            &output));


    assert(
        (output.flags &
         ATTITUDE_CONTROL_INPUT_INVALID) !=
        0UL);
}


/*
 * ------------------------------------------------------------
 * NaN INPUT
 * ------------------------------------------------------------
 */

static void
test_nonfinite_input_is_rejected(void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(1UL);


    input.estimated_roll_rad =
        NAN;


    assert(
        !attitude_controller_update(
            &input,
            &output));


    assert(
        (output.flags &
         ATTITUDE_CONTROL_INPUT_INVALID) !=
        0UL);
}


/*
 * ------------------------------------------------------------
 * DUPLICATE EULER SAMPLE
 * ------------------------------------------------------------
 */

static void
test_duplicate_sequence_is_rejected(void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(25UL);


    assert(
        attitude_controller_update(
            &input,
            &output));


    /*
     * Same Euler sample must not be processed again.
     */
    assert(
        !attitude_controller_update(
            &input,
            &output));


    assert(
        (output.flags &
         ATTITUDE_CONTROL_DUPLICATE_SEQUENCE) !=
        0UL);
}


/*
 * ------------------------------------------------------------
 * RESET
 * ------------------------------------------------------------
 */

static void
test_reset_allows_fresh_sequence(void)
{
    attitude_controller_config_t config;
    attitude_controller_input_t input;
    attitude_controller_output_t output;


    config =
        make_config();


    assert(
        attitude_controller_init(
            &config));


    input =
        make_input(7UL);


    assert(
        attitude_controller_update(
            &input,
            &output));


    attitude_controller_reset();


    /*
     * Same sequence may now be treated as fresh because
     * sequence history was intentionally reset.
     */
    assert(
        attitude_controller_update(
            &input,
            &output));
}


/*
 * ------------------------------------------------------------
 * INVALID CONFIG
 * ------------------------------------------------------------
 */

static void
test_invalid_config_is_rejected(void)
{
    attitude_controller_config_t config;


    config =
        make_config();


    /*
     * A zero maximum rate is not a valid operating
     * configuration.
     */
    config.max_roll_rate_rad_s =
        0.0f;


    assert(
        !attitude_controller_init(
            &config));
}


/*
 * ------------------------------------------------------------
 * HOST TEST ENTRY
 * ------------------------------------------------------------
 */

int main(void)
{
    test_zero_error_produces_zero_rates();

    test_positive_roll_attitude_is_corrected_negative();

    test_negative_roll_attitude_is_corrected_positive();

    test_pitch_sign_and_axis_independence();

    test_requested_nonzero_attitude();

    test_rate_limits();

    test_invalid_attitude_is_rejected();

    test_nonfinite_input_is_rejected();

    test_duplicate_sequence_is_rejected();

    test_reset_allows_fresh_sequence();

    test_invalid_config_is_rejected();


    puts(
        "attitude_controller_test: PASS");


    return 0;
}