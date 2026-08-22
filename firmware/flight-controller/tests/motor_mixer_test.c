#include "motor_mixer.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>


static bool
close_to(
    float actual,
    float expected,
    float tolerance)
{
    return
        fabsf(
            actual -
            expected) <=
        tolerance;
}


static motor_mixer_input_t
make_input(void)
{
    motor_mixer_input_t
        input;


    input =
        (motor_mixer_input_t){0};


    input.sequence =
        10UL;


    input.timestamp_us =
        20000UL;


    input.sample_interval_us =
        2000UL;


    input.control_valid =
        true;


    input.collective =
        0.50f;


    return
        input;
}


static void
test_collective_only(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        (output.flags &
         MOTOR_MIXER_VALID) !=
        0UL);


    assert(
        close_to(
            output.m1,
            0.50f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.50f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.50f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.50f,
            0.000001f));


    assert(
        close_to(
            output.differential_scale,
            1.0f,
            0.000001f));


    assert(
        close_to(
            output.collective_used,
            0.50f,
            0.000001f));
}


static void
test_positive_roll_mix(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.roll_correction =
        0.10f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    /*
     * +roll = left side goes down.
     *
     * Therefore:
     *
     *     left motors  M1/M4 decrease
     *     right motors M2/M3 increase
     */
    assert(
        close_to(
            output.m1,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.40f,
            0.000001f));
}


static void
test_negative_roll_mix(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.roll_correction =
        -0.10f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        close_to(
            output.m1,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.60f,
            0.000001f));
}


static void
test_positive_pitch_mix(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.pitch_correction =
        0.10f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    /*
     * +pitch = nose/front goes up.
     *
     * Front motors increase.
     * Rear motors decrease.
     */
    assert(
        close_to(
            output.m1,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.40f,
            0.000001f));
}


static void
test_negative_pitch_mix(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.pitch_correction =
        -0.10f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        close_to(
            output.m1,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.60f,
            0.000001f));
}


static void
test_positive_yaw_mix(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.yaw_correction =
        0.10f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    /*
     * +yaw = nose turns right.
     *
     * Viewed from above:
     *
     *     M1 = CCW
     *     M3 = CCW
     *
     * Increasing M1/M3 produces an opposite clockwise
     * reaction torque on the airframe.
     */
    assert(
        close_to(
            output.m1,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.40f,
            0.000001f));
}


static void
test_negative_yaw_mix(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.yaw_correction =
        -0.10f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        close_to(
            output.m1,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.60f,
            0.000001f));
}


static void
test_combined_mix(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.roll_correction =
        0.05f;


    input.pitch_correction =
        -0.03f;


    input.yaw_correction =
        0.01f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    /*
     * C = 0.50
     * R = 0.05
     * P = -0.03
     * Y = 0.01
     *
     * M1 = 0.43
     * M2 = 0.51
     * M3 = 0.59
     * M4 = 0.47
     */

    assert(
        close_to(
            output.raw_m1,
            0.43f,
            0.000001f));


    assert(
        close_to(
            output.raw_m2,
            0.51f,
            0.000001f));


    assert(
        close_to(
            output.raw_m3,
            0.59f,
            0.000001f));


    assert(
        close_to(
            output.raw_m4,
            0.47f,
            0.000001f));


    assert(
        close_to(
            output.m1,
            0.43f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.51f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.59f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.47f,
            0.000001f));
}


static void
test_collective_shift_high_preserves_differential(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.collective =
        0.90f;


    input.roll_correction =
        0.20f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        (output.flags &
         MOTOR_MIXER_COLLECTIVE_SHIFTED) !=
        0UL);


    assert(
        (output.flags &
         MOTOR_MIXER_DIFFERENTIAL_SCALED) ==
        0UL);


    /*
     * Before desaturation:
     *
     *     M1 = 0.70
     *     M2 = 1.10
     *     M3 = 1.10
     *     M4 = 0.70
     */

    assert(
        close_to(
            output.raw_m1,
            0.70f,
            0.000001f));


    assert(
        close_to(
            output.raw_m2,
            1.10f,
            0.000001f));


    assert(
        close_to(
            output.raw_m3,
            1.10f,
            0.000001f));


    assert(
        close_to(
            output.raw_m4,
            0.70f,
            0.000001f));


    /*
     * Collective is shifted down from 0.90 to 0.80.
     */

    assert(
        close_to(
            output.collective_used,
            0.80f,
            0.000001f));


    assert(
        close_to(
            output.m1,
            0.60f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            1.00f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            1.00f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.60f,
            0.000001f));


    /*
     * Original right-left differential remains 0.40.
     */

    assert(
        close_to(
            output.m2 -
            output.m1,
            0.40f,
            0.000001f));
}


static void
test_collective_shift_low_preserves_differential(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.collective =
        0.10f;


    input.roll_correction =
        -0.20f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        (output.flags &
         MOTOR_MIXER_COLLECTIVE_SHIFTED) !=
        0UL);


    assert(
        close_to(
            output.collective_used,
            0.20f,
            0.000001f));


    assert(
        close_to(
            output.m1,
            0.40f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.00f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.00f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.40f,
            0.000001f));
}


static void
test_impossible_differential_is_scaled(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.roll_correction =
        0.80f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    /*
     * Differential values:
     *
     *     -0.8
     *     +0.8
     *     +0.8
     *     -0.8
     *
     * span = 1.6
     *
     * available range = 1.0
     *
     * scale = 1 / 1.6 = 0.625
     */

    assert(
        (output.flags &
         MOTOR_MIXER_DIFFERENTIAL_SCALED) !=
        0UL);


    assert(
        close_to(
            output.differential_scale,
            0.625f,
            0.000001f));


    assert(
        close_to(
            output.m1,
            0.00f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            1.00f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            1.00f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.00f,
            0.000001f));
}


static void
test_collective_is_clamped(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.collective =
        1.20f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        (output.flags &
         MOTOR_MIXER_COLLECTIVE_CLAMPED) !=
        0UL);


    assert(
        close_to(
            output.m1,
            1.0f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            1.0f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            1.0f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            1.0f,
            0.000001f));


    input =
        make_input();


    input.collective =
        -0.20f;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        (output.flags &
         MOTOR_MIXER_COLLECTIVE_CLAMPED) !=
        0UL);


    assert(
        close_to(
            output.m1,
            0.0f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.0f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.0f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.0f,
            0.000001f));
}


static void
test_invalid_source_is_rejected(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.control_valid =
        false;


    assert(
        !motor_mixer_update(
            &input,
            &output));


    assert(
        output.flags ==
        MOTOR_MIXER_SOURCE_INVALID);


    /*
     * Rejected output stays deterministic and zero.
     */

    assert(
        close_to(
            output.m1,
            0.0f,
            0.000001f));


    assert(
        close_to(
            output.m2,
            0.0f,
            0.000001f));


    assert(
        close_to(
            output.m3,
            0.0f,
            0.000001f));


    assert(
        close_to(
            output.m4,
            0.0f,
            0.000001f));
}


static void
test_nonfinite_input_is_rejected(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.roll_correction =
        NAN;


    assert(
        !motor_mixer_update(
            &input,
            &output));


    assert(
        output.flags ==
        MOTOR_MIXER_INPUT_INVALID);


    input =
        make_input();


    input.yaw_correction =
        INFINITY;


    assert(
        !motor_mixer_update(
            &input,
            &output));


    assert(
        output.flags ==
        MOTOR_MIXER_INPUT_INVALID);
}


static void
test_derived_numeric_overflow_is_rejected(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    /*
     * Each input value is finite, but the resulting arithmetic
     * can overflow a float.
     */

    input.roll_correction =
        3.0e38f;


    input.pitch_correction =
        3.0e38f;


    input.yaw_correction =
        3.0e38f;


    assert(
        !motor_mixer_update(
            &input,
            &output));


    assert(
        (output.flags &
         MOTOR_MIXER_NUMERIC_ERROR) !=
        0UL);
}


static void
test_null_pointers_are_rejected(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    assert(
        !motor_mixer_update(
            (const motor_mixer_input_t *)0,
            &output));


    assert(
        output.flags ==
        MOTOR_MIXER_INPUT_INVALID);


    assert(
        !motor_mixer_update(
            &input,
            (motor_mixer_output_t *)0));
}


static void
test_metadata_is_copied(void)
{
    motor_mixer_input_t
        input;

    motor_mixer_output_t
        output;


    input =
        make_input();


    input.sequence =
        1234UL;


    input.timestamp_us =
        567890UL;


    input.sample_interval_us =
        2003UL;


    assert(
        motor_mixer_update(
            &input,
            &output));


    assert(
        output.sequence ==
        1234UL);


    assert(
        output.timestamp_us ==
        567890UL);


    assert(
        output.sample_interval_us ==
        2003UL);
}


int
main(void)
{
    test_collective_only();

    test_positive_roll_mix();

    test_negative_roll_mix();

    test_positive_pitch_mix();

    test_negative_pitch_mix();

    test_positive_yaw_mix();

    test_negative_yaw_mix();

    test_combined_mix();

    test_collective_shift_high_preserves_differential();

    test_collective_shift_low_preserves_differential();

    test_impossible_differential_is_scaled();

    test_collective_is_clamped();

    test_invalid_source_is_rejected();

    test_nonfinite_input_is_rejected();

    test_derived_numeric_overflow_is_rejected();

    test_null_pointers_are_rejected();

    test_metadata_is_copied();


    puts(
        "motor_mixer_test: PASS");


    return 0;
}