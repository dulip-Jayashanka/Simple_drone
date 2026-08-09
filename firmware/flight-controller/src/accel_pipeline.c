#include "accel_pipeline.h"

#include <stdbool.h>
#include <stdint.h>

/* Six-face calibration measured from this MPU6500 at +/-4 g. */
#define ACCEL_X_BIAS_COUNTS       23.3847f
#define ACCEL_Y_BIAS_COUNTS        9.1664f
#define ACCEL_Z_BIAS_COUNTS     (-111.2419f)

#define ACCEL_X_COUNTS_PER_G    8165.3419f
#define ACCEL_Y_COUNTS_PER_G    8187.6939f
#define ACCEL_Z_COUNTS_PER_G    8233.3779f

#define STANDARD_GRAVITY_MS2       9.80665f

/* Second-order 20 Hz Butterworth at exactly 500 samples/second. */
#define BUTTERWORTH_B0             0.0133592f
#define BUTTERWORTH_B1             0.0267184f
#define BUTTERWORTH_B2             0.0133592f
#define BUTTERWORTH_A1           (-1.64745998f)
#define BUTTERWORTH_A2             0.70089678f

/* Leave a small margin before the signed 16-bit endpoints. */
#define ACCEL_RAW_SATURATION_LIMIT 32760

typedef struct
{
    float x1;
    float x2;
    float y1;
    float y2;
    bool initialized;
} biquad_state_t;

typedef struct
{
    int16_t x[3];
    int16_t y[3];
    int16_t z[3];
    uint8_t next;
    uint8_t count;

    biquad_state_t filter_x;
    biquad_state_t filter_y;
    biquad_state_t filter_z;

    uint32_t last_sequence;
    bool sequence_valid;
} accel_pipeline_state_t;

static accel_pipeline_state_t pipeline_state;

static int16_t median3(int16_t a, int16_t b, int16_t c)
{
    int16_t temporary;

    if (a > b)
    {
        temporary = a;
        a = b;
        b = temporary;
    }

    if (b > c)
    {
        temporary = b;
        b = c;
        c = temporary;
    }

    if (a > b)
    {
        b = a;
    }

    return b;
}

static float biquad_process(
    biquad_state_t *state,
    float input)
{
    float output;

    if (!state->initialized)
    {
        /* Start in steady state; especially important for the Z axis. */
        state->x1 = input;
        state->x2 = input;
        state->y1 = input;
        state->y2 = input;
        state->initialized = true;

        return input;
    }

    output =
        (BUTTERWORTH_B0 * input) +
        (BUTTERWORTH_B1 * state->x1) +
        (BUTTERWORTH_B2 * state->x2) -
        (BUTTERWORTH_A1 * state->y1) -
        (BUTTERWORTH_A2 * state->y2);

    state->x2 = state->x1;
    state->x1 = input;
    state->y2 = state->y1;
    state->y1 = output;

    return output;
}

static bool raw_is_saturated(const accel_pipeline_input_t *input)
{
    return
        (input->raw_x >= ACCEL_RAW_SATURATION_LIMIT) ||
        (input->raw_x <= -ACCEL_RAW_SATURATION_LIMIT) ||
        (input->raw_y >= ACCEL_RAW_SATURATION_LIMIT) ||
        (input->raw_y <= -ACCEL_RAW_SATURATION_LIMIT) ||
        (input->raw_z >= ACCEL_RAW_SATURATION_LIMIT) ||
        (input->raw_z <= -ACCEL_RAW_SATURATION_LIMIT);
}

void accel_pipeline_init(void)
{
    pipeline_state = (accel_pipeline_state_t){0};
}

bool accel_pipeline_process(
    const accel_pipeline_input_t *input,
    accel_pipeline_output_t *output)
{
    uint8_t slot;

    if ((input == (const accel_pipeline_input_t *)0) ||
        (output == (accel_pipeline_output_t *)0))
    {
        return false;
    }

    *output = (accel_pipeline_output_t){0};
    output->sequence = input->sequence;
    output->timestamp_us = input->timestamp_us;
    output->raw_x = input->raw_x;
    output->raw_y = input->raw_y;
    output->raw_z = input->raw_z;

    if (pipeline_state.sequence_valid &&
        (input->sequence == pipeline_state.last_sequence))
    {
        output->flags = ACCEL_PIPELINE_DUPLICATE_SEQUENCE;
        return false;
    }

    if (raw_is_saturated(input))
    {
        output->flags = ACCEL_PIPELINE_INPUT_SATURATED;
        return false;
    }

    pipeline_state.last_sequence = input->sequence;
    pipeline_state.sequence_valid = true;

    if (pipeline_state.count == 0U)
    {
        /* Repeat the first sample so startup output is immediately defined. */
        pipeline_state.x[0] = input->raw_x;
        pipeline_state.x[1] = input->raw_x;
        pipeline_state.x[2] = input->raw_x;

        pipeline_state.y[0] = input->raw_y;
        pipeline_state.y[1] = input->raw_y;
        pipeline_state.y[2] = input->raw_y;

        pipeline_state.z[0] = input->raw_z;
        pipeline_state.z[1] = input->raw_z;
        pipeline_state.z[2] = input->raw_z;

        pipeline_state.count = 3U;
        pipeline_state.next = 1U;
    }
    else
    {
        slot = pipeline_state.next;

        pipeline_state.x[slot] = input->raw_x;
        pipeline_state.y[slot] = input->raw_y;
        pipeline_state.z[slot] = input->raw_z;

        pipeline_state.next = (uint8_t)((slot + 1U) % 3U);
    }

    output->median_x = median3(
        pipeline_state.x[0],
        pipeline_state.x[1],
        pipeline_state.x[2]);

    output->median_y = median3(
        pipeline_state.y[0],
        pipeline_state.y[1],
        pipeline_state.y[2]);

    output->median_z = median3(
        pipeline_state.z[0],
        pipeline_state.z[1],
        pipeline_state.z[2]);

    output->calibrated_x_g =
        ((float)output->median_x - ACCEL_X_BIAS_COUNTS) /
        ACCEL_X_COUNTS_PER_G;

    output->calibrated_y_g =
        ((float)output->median_y - ACCEL_Y_BIAS_COUNTS) /
        ACCEL_Y_COUNTS_PER_G;

    output->calibrated_z_g =
        ((float)output->median_z - ACCEL_Z_BIAS_COUNTS) /
        ACCEL_Z_COUNTS_PER_G;

    output->filtered_x_g = biquad_process(
        &pipeline_state.filter_x,
        output->calibrated_x_g);

    output->filtered_y_g = biquad_process(
        &pipeline_state.filter_y,
        output->calibrated_y_g);

    output->filtered_z_g = biquad_process(
        &pipeline_state.filter_z,
        output->calibrated_z_g);

    output->filtered_x_ms2 =
        output->filtered_x_g * STANDARD_GRAVITY_MS2;

    output->filtered_y_ms2 =
        output->filtered_y_g * STANDARD_GRAVITY_MS2;

    output->filtered_z_ms2 =
        output->filtered_z_g * STANDARD_GRAVITY_MS2;

    output->flags =
        ACCEL_PIPELINE_VALID |
        ACCEL_PIPELINE_MEDIAN_READY |
        ACCEL_PIPELINE_FILTER_INITIALIZED;

    return true;
}
