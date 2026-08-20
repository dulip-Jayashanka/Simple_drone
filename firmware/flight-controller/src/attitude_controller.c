#include "attitude_controller.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * ------------------------------------------------------------
 * FLOAT VALIDATION
 * ------------------------------------------------------------
 *
 * Same bare-metal style used by the existing PID/rate-control
 * implementation.
 */

typedef union
{
    float as_float;
    uint32_t as_uint32;

} attitude_float_bits_t;


/*
 * ------------------------------------------------------------
 * PRIVATE CONTROLLER STATE
 * ------------------------------------------------------------
 *
 * There is no integral or derivative history because the first
 * outer attitude controller is proportional-only.
 *
 * We only retain:
 *
 *     configuration
 *     initialization state
 *     last processed Euler sequence
 */

typedef struct
{
    bool initialized;

    bool sequence_valid;
    uint32_t last_sequence;

    attitude_controller_config_t config;

} attitude_controller_state_t;


static attitude_controller_state_t
    controller_state;


/*
 * ------------------------------------------------------------
 * HELPERS
 * ------------------------------------------------------------
 */

static bool value_is_finite(
    float value)
{
    attitude_float_bits_t bits;

    bits.as_float =
        value;

    return
        (bits.as_uint32 & 0x7F800000UL) !=
        0x7F800000UL;
}


static float clamp_float(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


static bool config_is_valid(
    const attitude_controller_config_t *config)
{
    if (config ==
        (const attitude_controller_config_t *)0)
    {
        return false;
    }


    /*
     * Reject NaN and infinity.
     */
    if (!value_is_finite(
            config->roll_gain_per_s) ||
        !value_is_finite(
            config->pitch_gain_per_s) ||
        !value_is_finite(
            config->max_roll_rate_rad_s) ||
        !value_is_finite(
            config->max_pitch_rate_rad_s))
    {
        return false;
    }


    /*
     * Negative proportional gains are not used by this
     * controller convention.
     */
    if ((config->roll_gain_per_s < 0.0f) ||
        (config->pitch_gain_per_s < 0.0f))
    {
        return false;
    }


    /*
     * Rate caps must be strictly positive.
     */
    if (!(config->max_roll_rate_rad_s > 0.0f) ||
        !(config->max_pitch_rate_rad_s > 0.0f))
    {
        return false;
    }


    return true;
}


static bool input_values_are_valid(
    const attitude_controller_input_t *input)
{
    return
        value_is_finite(
            input->desired_roll_rad) &&

        value_is_finite(
            input->desired_pitch_rad) &&

        value_is_finite(
            input->estimated_roll_rad) &&

        value_is_finite(
            input->estimated_pitch_rad);
}


static void copy_input_metadata(
    const attitude_controller_input_t *input,
    attitude_controller_output_t *output)
{
    output->sequence =
        input->sequence;

    output->timestamp_us =
        input->timestamp_us;


    output->desired_roll_rad =
        input->desired_roll_rad;

    output->desired_pitch_rad =
        input->desired_pitch_rad;


    output->estimated_roll_rad =
        input->estimated_roll_rad;

    output->estimated_pitch_rad =
        input->estimated_pitch_rad;
}


/*
 * ------------------------------------------------------------
 * INITIALIZATION
 * ------------------------------------------------------------
 */

bool attitude_controller_init(
    const attitude_controller_config_t *config)
{
    /*
     * Start from a completely known state.
     */
    controller_state =
        (attitude_controller_state_t){0};


    if (!config_is_valid(config))
    {
        return false;
    }


    /*
     * Store the validated configuration.
     */
    controller_state.config =
        *config;


    controller_state.initialized =
        true;


    return true;
}


/*
 * ------------------------------------------------------------
 * RESET
 * ------------------------------------------------------------
 */

void attitude_controller_reset(void)
{
    if (!controller_state.initialized)
    {
        return;
    }


    /*
     * No I or D terms exist in this outer-loop implementation.
     *
     * Therefore only freshness/sequence history needs reset.
     */
    controller_state.sequence_valid =
        false;

    controller_state.last_sequence =
        0UL;
}


/*
 * ------------------------------------------------------------
 * OUTER ATTITUDE UPDATE
 * ------------------------------------------------------------
 */

bool attitude_controller_update(
    const attitude_controller_input_t *input,
    attitude_controller_output_t *output)
{
    uint32_t flags;

    float roll_error;
    float pitch_error;

    float raw_roll_rate;
    float raw_pitch_rate;

    float final_roll_rate;
    float final_pitch_rate;


    /*
     * Output storage is mandatory.
     */
    if (output ==
        (attitude_controller_output_t *)0)
    {
        return false;
    }


    /*
     * Never leave stale data in the caller's output structure.
     */
    *output =
        (attitude_controller_output_t){0};


    /*
     * Validate input pointer.
     */
    if (input ==
        (const attitude_controller_input_t *)0)
    {
        output->flags =
            ATTITUDE_CONTROL_INPUT_INVALID;

        return false;
    }


    /*
     * Copy metadata before later validation so failed updates
     * remain easy to inspect through GDB.
     */
    copy_input_metadata(
        input,
        output);


    /*
     * Controller must have been initialized with a valid
     * configuration.
     */
    if (!controller_state.initialized)
    {
        output->flags =
            ATTITUDE_CONTROL_NOT_INITIALIZED;

        return false;
    }


    /*
     * The caller must explicitly confirm that the attitude
     * estimate is valid.
     *
     * Also reject NaN/infinity.
     */
    if (!input->attitude_valid ||
        !input_values_are_valid(input))
    {
        output->flags =
            ATTITUDE_CONTROL_INPUT_INVALID;

        return false;
    }


    /*
     * Euler angles are intentionally calculated more slowly
     * than the approximately 500 Hz quaternion update.
     *
     * Therefore outer-loop sequence numbers do NOT need to be
     * consecutive.
     *
     * Example:
     *
     *     100
     *     105
     *     110
     *
     * is valid for a 100 Hz Euler/outer loop while the sensor
     * operates near 500 Hz.
     *
     * We only reject processing the exact same Euler sample
     * twice.
     */
    if (controller_state.sequence_valid &&
        (input->sequence ==
         controller_state.last_sequence))
    {
        output->flags =
            ATTITUDE_CONTROL_DUPLICATE_SEQUENCE;

        return false;
    }


    /*
     * --------------------------------------------------------
     * ROLL ANGLE ERROR
     * --------------------------------------------------------
     *
     *     error =
     *         target - actual
     */
    roll_error =
        input->desired_roll_rad -
        input->estimated_roll_rad;


    /*
     * --------------------------------------------------------
     * PITCH ANGLE ERROR
     * --------------------------------------------------------
     */
    pitch_error =
        input->desired_pitch_rad -
        input->estimated_pitch_rad;


    /*
     * --------------------------------------------------------
     * PROPORTIONAL OUTER CONTROL
     * --------------------------------------------------------
     *
     * Angle error is converted to desired angular rate.
     */

    raw_roll_rate =
        controller_state.config
            .roll_gain_per_s *
        roll_error;


    raw_pitch_rate =
        controller_state.config
            .pitch_gain_per_s *
        pitch_error;


    /*
     * Protect against unexpected floating-point overflow or
     * corruption.
     */
    if (!value_is_finite(roll_error) ||
        !value_is_finite(pitch_error) ||
        !value_is_finite(raw_roll_rate) ||
        !value_is_finite(raw_pitch_rate))
    {
        output->flags =
            ATTITUDE_CONTROL_NUMERIC_ERROR;

        return false;
    }


    flags =
        0UL;


    /*
     * --------------------------------------------------------
     * ROLL-RATE LIMIT
     * --------------------------------------------------------
     */

    final_roll_rate =
        clamp_float(
            raw_roll_rate,
            -controller_state.config
                .max_roll_rate_rad_s,
            controller_state.config
                .max_roll_rate_rad_s);


    /*
     * --------------------------------------------------------
     * PITCH-RATE LIMIT
     * --------------------------------------------------------
     */

    final_pitch_rate =
        clamp_float(
            raw_pitch_rate,
            -controller_state.config
                .max_pitch_rate_rad_s,
            controller_state.config
                .max_pitch_rate_rad_s);


    /*
     * Record whether either rate request was limited.
     */
    if (final_roll_rate !=
        raw_roll_rate)
    {
        flags |=
            ATTITUDE_CONTROL_ROLL_RATE_LIMITED;
    }


    if (final_pitch_rate !=
        raw_pitch_rate)
    {
        flags |=
            ATTITUDE_CONTROL_PITCH_RATE_LIMITED;
    }


    /*
     * Publish diagnostic intermediate values.
     */
    output->roll_error_rad =
        roll_error;

    output->pitch_error_rad =
        pitch_error;


    output->raw_roll_rate_rad_s =
        raw_roll_rate;

    output->raw_pitch_rate_rad_s =
        raw_pitch_rate;


    /*
     * These are the values consumed by the existing
     * rate_controller.c inner loop.
     */
    output->desired_roll_rate_rad_s =
        final_roll_rate;

    output->desired_pitch_rate_rad_s =
        final_pitch_rate;


    /*
     * Only after the complete calculation succeeds do we
     * commit sequence history.
     */
    controller_state.last_sequence =
        input->sequence;

    controller_state.sequence_valid =
        true;


    output->flags =
        flags |
        ATTITUDE_CONTROL_VALID;


    return true;
}