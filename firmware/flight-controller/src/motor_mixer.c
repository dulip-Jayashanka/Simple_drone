#include "motor_mixer.h"

#include <stdbool.h>
#include <stdint.h>


typedef union
{
    float as_float;

    uint32_t as_uint32;

} mixer_float_bits_t;


/*
 * ============================================================
 * FINITE FLOAT CHECK
 * ============================================================
 *
 * Avoid dependency on the standard math library in the
 * freestanding STM32 firmware.
 */
static bool
value_is_finite(
    float value)
{
    mixer_float_bits_t
        bits;


    bits.as_float =
        value;


    return
        (bits.as_uint32 &
         0x7F800000UL) !=
        0x7F800000UL;
}


/*
 * ============================================================
 * NORMALIZED MOTOR COMMAND CLAMP
 * ============================================================
 */

static float
clamp_motor_command(
    float value,
    bool *clamped)
{
    if (value >
        MOTOR_MIXER_COMMAND_MAX)
    {
        if (clamped !=
            (bool *)0)
        {
            *clamped =
                true;
        }


        return
            MOTOR_MIXER_COMMAND_MAX;
    }


    if (value <
        MOTOR_MIXER_COMMAND_MIN)
    {
        if (clamped !=
            (bool *)0)
        {
            *clamped =
                true;
        }


        return
            MOTOR_MIXER_COMMAND_MIN;
    }


    return
        value;
}


/*
 * ============================================================
 * FOUR-VALUE MINIMUM
 * ============================================================
 */

static float
minimum4(
    float a,
    float b,
    float c,
    float d)
{
    float
        minimum;


    minimum =
        a;


    if (b <
        minimum)
    {
        minimum =
            b;
    }


    if (c <
        minimum)
    {
        minimum =
            c;
    }


    if (d <
        minimum)
    {
        minimum =
            d;
    }


    return
        minimum;
}


/*
 * ============================================================
 * FOUR-VALUE MAXIMUM
 * ============================================================
 */

static float
maximum4(
    float a,
    float b,
    float c,
    float d)
{
    float
        maximum;


    maximum =
        a;


    if (b >
        maximum)
    {
        maximum =
            b;
    }


    if (c >
        maximum)
    {
        maximum =
            c;
    }


    if (d >
        maximum)
    {
        maximum =
            d;
    }


    return
        maximum;
}


/*
 * ============================================================
 * COPY INPUT FOR DIAGNOSTICS
 * ============================================================
 */

static void
copy_input(
    const motor_mixer_input_t *input,
    motor_mixer_output_t *output)
{
    output->sequence =
        input->sequence;


    output->timestamp_us =
        input->timestamp_us;


    output->sample_interval_us =
        input->sample_interval_us;


    output->collective_input =
        input->collective;


    output->roll_correction =
        input->roll_correction;


    output->pitch_correction =
        input->pitch_correction;


    output->yaw_correction =
        input->yaw_correction;
}


/*
 * ============================================================
 * INPUT VALUE VALIDATION
 * ============================================================
 */

static bool
input_values_are_valid(
    const motor_mixer_input_t *input)
{
    return
        value_is_finite(
            input->collective) &&

        value_is_finite(
            input->roll_correction) &&

        value_is_finite(
            input->pitch_correction) &&

        value_is_finite(
            input->yaw_correction);
}


/*
 * ============================================================
 * MOTOR MIXER UPDATE
 * ============================================================
 */

bool
motor_mixer_update(
    const motor_mixer_input_t *input,
    motor_mixer_output_t *output)
{
    uint32_t
        flags;


    float
        collective;

    float
        differential_scale;

    float
        collective_shift;


    float
        d1;

    float
        d2;

    float
        d3;

    float
        d4;


    float
        differential_min;

    float
        differential_max;

    float
        differential_span;


    float
        candidate_m1;

    float
        candidate_m2;

    float
        candidate_m3;

    float
        candidate_m4;


    float
        candidate_min;

    float
        candidate_max;


    bool
        collective_clamped;

    bool
        output_clamped;


    /*
     * The output pointer itself cannot be reported through
     * output flags when it is NULL.
     */
    if (output ==
        (motor_mixer_output_t *)0)
    {
        return false;
    }


    /*
     * Deterministic rejected-output state.
     */
    *output =
        (motor_mixer_output_t){0};


    if (input ==
        (const motor_mixer_input_t *)0)
    {
        output->flags =
            MOTOR_MIXER_INPUT_INVALID;


        return false;
    }


    /*
     * Preserve the source metadata and requested values even
     * when the update is rejected.
     *
     * This is useful during GDB inspection.
     */
    copy_input(
        input,
        output);


    /*
     * The mixer must never convert an invalid controller result
     * into a valid motor command.
     */
    if (!input->control_valid)
    {
        output->flags =
            MOTOR_MIXER_SOURCE_INVALID;


        return false;
    }


    if (!input_values_are_valid(
            input))
    {
        output->flags =
            MOTOR_MIXER_INPUT_INVALID;


        return false;
    }


    flags =
        0UL;


    /*
     * ========================================================
     * LOCKED X-FRAME DIFFERENTIAL ALLOCATION
     * ========================================================
     *
     * M1 = C - R + P + Y
     *
     * M2 = C + R + P - Y
     *
     * M3 = C + R - P + Y
     *
     * M4 = C - R - P - Y
     *
     *
     * Calculate only the differential part here.
     *
     * Collective is intentionally added separately so that it
     * can later be shifted without changing roll/pitch/yaw
     * motor differences.
     */


    d1 =
        (-input->roll_correction) +
        input->pitch_correction +
        input->yaw_correction;


    d2 =
        input->roll_correction +
        input->pitch_correction -
        input->yaw_correction;


    d3 =
        input->roll_correction -
        input->pitch_correction +
        input->yaw_correction;


    d4 =
        (-input->roll_correction) -
        input->pitch_correction -
        input->yaw_correction;


    /*
     * Large but finite inputs can still overflow while the
     * differential equations are evaluated.
     */
    if (!value_is_finite(d1) ||
        !value_is_finite(d2) ||
        !value_is_finite(d3) ||
        !value_is_finite(d4))
    {
        output->flags =
            MOTOR_MIXER_NUMERIC_ERROR;


        return false;
    }


    /*
     * ========================================================
     * ORIGINAL RAW REQUEST
     * ========================================================
     *
     * Keep the requested values BEFORE desaturation.
     *
     * They may legitimately be below 0 or above 1.
     */

    output->raw_m1 =
        input->collective +
        d1;


    output->raw_m2 =
        input->collective +
        d2;


    output->raw_m3 =
        input->collective +
        d3;


    output->raw_m4 =
        input->collective +
        d4;


    if (!value_is_finite(
            output->raw_m1) ||

        !value_is_finite(
            output->raw_m2) ||

        !value_is_finite(
            output->raw_m3) ||

        !value_is_finite(
            output->raw_m4))
    {
        output->flags =
            MOTOR_MIXER_NUMERIC_ERROR;


        return false;
    }


    /*
     * ========================================================
     * COLLECTIVE NORMALIZATION
     * ========================================================
     */

    collective_clamped =
        false;


    collective =
        clamp_motor_command(
            input->collective,
            &collective_clamped);


    if (collective_clamped)
    {
        flags |=
            MOTOR_MIXER_COLLECTIVE_CLAMPED;
    }


    /*
     * ========================================================
     * DIFFERENTIAL-SPAN CHECK
     * ========================================================
     *
     * Example:
     *
     *     minimum differential = -0.8
     *
     *     maximum differential = +0.8
     *
     *     span = 1.6
     *
     *
     * The complete normalized motor range is only:
     *
     *     1.0 - 0.0 = 1.0
     *
     *
     * No common collective shift can fit a 1.6-wide
     * differential request into a 1.0-wide output range.
     */

    differential_min =
        minimum4(
            d1,
            d2,
            d3,
            d4);


    differential_max =
        maximum4(
            d1,
            d2,
            d3,
            d4);


    differential_span =
        differential_max -
        differential_min;


    if (!value_is_finite(
            differential_span))
    {
        output->flags =
            flags |
            MOTOR_MIXER_NUMERIC_ERROR;


        return false;
    }


    differential_scale =
        1.0f;


    /*
     * ========================================================
     * DIFFERENTIAL DESATURATION
     * ========================================================
     *
     * If the differential request itself cannot fit, uniformly
     * scale the complete differential allocation.
     *
     * Because all four differential terms are scaled by the
     * same value, the requested roll/pitch/yaw allocation is
     * preserved proportionally.
     */

    if (differential_span >
        (MOTOR_MIXER_COMMAND_MAX -
         MOTOR_MIXER_COMMAND_MIN))
    {
        differential_scale =
            (MOTOR_MIXER_COMMAND_MAX -
             MOTOR_MIXER_COMMAND_MIN) /
            differential_span;


        d1 *=
            differential_scale;


        d2 *=
            differential_scale;


        d3 *=
            differential_scale;


        d4 *=
            differential_scale;


        flags |=
            MOTOR_MIXER_DIFFERENTIAL_SCALED;
    }


    output->differential_scale =
        differential_scale;


    /*
     * ========================================================
     * TRY REQUESTED COLLECTIVE
     * ========================================================
     */

    candidate_m1 =
        collective +
        d1;


    candidate_m2 =
        collective +
        d2;


    candidate_m3 =
        collective +
        d3;


    candidate_m4 =
        collective +
        d4;


    if (!value_is_finite(
            candidate_m1) ||

        !value_is_finite(
            candidate_m2) ||

        !value_is_finite(
            candidate_m3) ||

        !value_is_finite(
            candidate_m4))
    {
        output->flags =
            flags |
            MOTOR_MIXER_NUMERIC_ERROR;


        return false;
    }


    candidate_min =
        minimum4(
            candidate_m1,
            candidate_m2,
            candidate_m3,
            candidate_m4);


    candidate_max =
        maximum4(
            candidate_m1,
            candidate_m2,
            candidate_m3,
            candidate_m4);


    /*
     * ========================================================
     * COLLECTIVE SHIFT DESATURATION
     * ========================================================
     *
     * Example:
     *
     *     raw:
     *
     *         0.70
     *         1.10
     *         1.10
     *         0.70
     *
     *
     * Instead of clipping only the two high motors:
     *
     *         0.70
     *         1.00
     *         1.00
     *         0.70
     *
     *
     * move the complete set downward:
     *
     *         0.60
     *         1.00
     *         1.00
     *         0.60
     *
     *
     * The original differential remains 0.40.
     *
     * Only collective was sacrificed.
     */

    collective_shift =
        0.0f;


    if (candidate_max >
        MOTOR_MIXER_COMMAND_MAX)
    {
        collective_shift =
            MOTOR_MIXER_COMMAND_MAX -
            candidate_max;
    }
    else if (candidate_min <
             MOTOR_MIXER_COMMAND_MIN)
    {
        collective_shift =
            MOTOR_MIXER_COMMAND_MIN -
            candidate_min;
    }


    if (collective_shift !=
        0.0f)
    {
        collective +=
            collective_shift;


        candidate_m1 +=
            collective_shift;


        candidate_m2 +=
            collective_shift;


        candidate_m3 +=
            collective_shift;


        candidate_m4 +=
            collective_shift;


        flags |=
            MOTOR_MIXER_COLLECTIVE_SHIFTED;
    }


    output->collective_used =
        collective;


    /*
     * ========================================================
     * FINAL OUTPUT GUARD
     * ========================================================
     *
     * Normally the previous two desaturation stages already
     * guarantee a valid range.
     *
     * This final clamp protects against floating-point
     * round-off and unforeseen edge cases.
     */

    output_clamped =
        false;


    output->m1 =
        clamp_motor_command(
            candidate_m1,
            &output_clamped);


    output->m2 =
        clamp_motor_command(
            candidate_m2,
            &output_clamped);


    output->m3 =
        clamp_motor_command(
            candidate_m3,
            &output_clamped);


    output->m4 =
        clamp_motor_command(
            candidate_m4,
            &output_clamped);


    if (output_clamped)
    {
        flags |=
            MOTOR_MIXER_OUTPUT_CLAMPED;
    }


    /*
     * Final numerical check.
     */
    if (!value_is_finite(
            output->m1) ||

        !value_is_finite(
            output->m2) ||

        !value_is_finite(
            output->m3) ||

        !value_is_finite(
            output->m4) ||

        !value_is_finite(
            output->collective_used) ||

        !value_is_finite(
            output->differential_scale))
    {
        output->flags =
            flags |
            MOTOR_MIXER_NUMERIC_ERROR;


        output->m1 =
            0.0f;


        output->m2 =
            0.0f;


        output->m3 =
            0.0f;


        output->m4 =
            0.0f;


        return false;
    }


    output->flags =
        flags |
        MOTOR_MIXER_VALID;


    return true;
}