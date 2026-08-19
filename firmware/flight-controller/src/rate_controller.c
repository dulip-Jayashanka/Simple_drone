#include "rate_controller.h"

#include <stdbool.h>
#include <stdint.h>

typedef union
{
    float as_float;
    uint32_t as_uint32;
} rate_float_bits_t;

typedef struct
{
    bool initialized;

    bool sequence_valid;
    uint32_t last_sequence;

    pid_controller_t roll;
    pid_controller_t pitch;
    pid_controller_t yaw;
} rate_controller_state_t;

static rate_controller_state_t controller_state;

static bool value_is_finite(float value)
{
    rate_float_bits_t bits;

    bits.as_float = value;

    return
        (bits.as_uint32 & 0x7F800000UL) !=
        0x7F800000UL;
}

static void copy_input_metadata(
    const rate_controller_input_t *input,
    rate_controller_output_t *output)
{
    output->sequence =
        input->sequence;

    output->timestamp_us =
        input->timestamp_us;

    output->sample_interval_us =
        input->sample_interval_us;

    output->dt_s =
        input->dt_s;

    output->desired_roll_rate_rad_s =
        input->desired_roll_rate_rad_s;

    output->desired_pitch_rate_rad_s =
        input->desired_pitch_rate_rad_s;

    output->desired_yaw_rate_rad_s =
        input->desired_yaw_rate_rad_s;

    output->measured_roll_rate_rad_s =
        input->measured_roll_rate_rad_s;

    output->measured_pitch_rate_rad_s =
        input->measured_pitch_rate_rad_s;

    output->measured_yaw_rate_rad_s =
        input->measured_yaw_rate_rad_s;
}

static bool input_values_are_valid(
    const rate_controller_input_t *input)
{
    return
        value_is_finite(
            input->desired_roll_rate_rad_s) &&
        value_is_finite(
            input->desired_pitch_rate_rad_s) &&
        value_is_finite(
            input->desired_yaw_rate_rad_s) &&
        value_is_finite(
            input->measured_roll_rate_rad_s) &&
        value_is_finite(
            input->measured_pitch_rate_rad_s) &&
        value_is_finite(
            input->measured_yaw_rate_rad_s);
}

static bool pid_output_is_saturated(
    const pid_output_t *output)
{
    return
        (output->flags &
         (PID_OUTPUT_SATURATED_HIGH |
          PID_OUTPUT_SATURATED_LOW)) != 0UL;
}

bool rate_controller_init(
    const rate_controller_config_t *config)
{
    pid_controller_t roll;
    pid_controller_t pitch;
    pid_controller_t yaw;

    controller_state =
        (rate_controller_state_t){0};

    if (config ==
        (const rate_controller_config_t *)0)
    {
        return false;
    }

    /*
     * Initialize local PID objects first.
     *
     * The global controller is marked initialized only when all three
     * configurations are valid.
     */
    if (!pid_init(&roll, &config->roll) ||
        !pid_init(&pitch, &config->pitch) ||
        !pid_init(&yaw, &config->yaw))
    {
        return false;
    }

    controller_state.roll = roll;
    controller_state.pitch = pitch;
    controller_state.yaw = yaw;

    controller_state.initialized = true;

    return true;
}

void rate_controller_reset(void)
{
    if (!controller_state.initialized)
    {
        return;
    }

    pid_reset(&controller_state.roll);
    pid_reset(&controller_state.pitch);
    pid_reset(&controller_state.yaw);

    controller_state.sequence_valid = false;
    controller_state.last_sequence = 0UL;
}

bool rate_controller_update(
    const rate_controller_input_t *input,
    rate_controller_output_t *output)
{
    uint32_t flags;
    bool sequence_gap;

    pid_controller_t roll_trial;
    pid_controller_t pitch_trial;
    pid_controller_t yaw_trial;

    pid_output_t roll_output;
    pid_output_t pitch_output;
    pid_output_t yaw_output;

    if (output == (rate_controller_output_t *)0)
    {
        return false;
    }

    *output = (rate_controller_output_t){0};

    if (input ==
        (const rate_controller_input_t *)0)
    {
        output->flags =
            RATE_CONTROL_INPUT_INVALID;

        return false;
    }

    copy_input_metadata(input, output);

    if (!controller_state.initialized)
    {
        output->flags =
            RATE_CONTROL_NOT_INITIALIZED;

        return false;
    }

    if (!input->measurements_valid ||
        !input_values_are_valid(input))
    {
        output->flags =
            RATE_CONTROL_INPUT_INVALID;

        return false;
    }

    if (!value_is_finite(input->dt_s) ||
        !(input->dt_s >= PID_DT_MIN_S) ||
        !(input->dt_s <= PID_DT_MAX_S))
    {
        output->flags =
            RATE_CONTROL_DT_INVALID;

        return false;
    }

    /*
     * Prevent accidental repeated integration of one sensor sample.
     */
    if (controller_state.sequence_valid &&
        (input->sequence ==
         controller_state.last_sequence))
    {
        output->flags =
            RATE_CONTROL_DUPLICATE_SEQUENCE;

        return false;
    }

    flags = 0UL;

    roll_output = (pid_output_t){0};
    pitch_output = (pid_output_t){0};
    yaw_output = (pid_output_t){0};

    sequence_gap =
        controller_state.sequence_valid &&
        (input->sequence !=
         (controller_state.last_sequence + 1UL));

    if (sequence_gap)
    {
        flags |= RATE_CONTROL_SEQUENCE_GAP;
    }

    /*
     * Work on local PID copies.
     *
     * If any axis fails, none of the real roll/pitch/yaw PID states are
     * modified. This keeps the three-axis update coherent.
     */
    roll_trial =
        controller_state.roll;

    pitch_trial =
        controller_state.pitch;

    yaw_trial =
        controller_state.yaw;

    if (sequence_gap)
    {
        /*
         * Preserve integrators but prevent a D-term spike across the missed
         * sequence.
         */
        pid_reset_derivative(&roll_trial);
        pid_reset_derivative(&pitch_trial);
        pid_reset_derivative(&yaw_trial);
    }

    if (!pid_update(
            &roll_trial,
            input->desired_roll_rate_rad_s,
            input->measured_roll_rate_rad_s,
            input->dt_s,
            &roll_output) ||
        !pid_update(
            &pitch_trial,
            input->desired_pitch_rate_rad_s,
            input->measured_pitch_rate_rad_s,
            input->dt_s,
            &pitch_output) ||
        !pid_update(
            &yaw_trial,
            input->desired_yaw_rate_rad_s,
            input->measured_yaw_rate_rad_s,
            input->dt_s,
            &yaw_output))
    {
        output->roll = roll_output;
        output->pitch = pitch_output;
        output->yaw = yaw_output;

        output->flags =
            flags |
            RATE_CONTROL_PID_FAILURE;

        return false;
    }

    /*
     * All three axis calculations succeeded.
     *
     * Commit the three dynamic PID states together.
     */
    controller_state.roll = roll_trial;
    controller_state.pitch = pitch_trial;
    controller_state.yaw = yaw_trial;

    controller_state.last_sequence =
        input->sequence;

    controller_state.sequence_valid = true;

    if (pid_output_is_saturated(&roll_output))
    {
        flags |=
            RATE_CONTROL_ROLL_SATURATED;
    }

    if (pid_output_is_saturated(&pitch_output))
    {
        flags |=
            RATE_CONTROL_PITCH_SATURATED;
    }

    if (pid_output_is_saturated(&yaw_output))
    {
        flags |=
            RATE_CONTROL_YAW_SATURATED;
    }

    output->roll = roll_output;
    output->pitch = pitch_output;
    output->yaw = yaw_output;

    output->flags =
        flags |
        RATE_CONTROL_VALID;

    return true;
}