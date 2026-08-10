#include "gyro_pipeline.h"

#include <stdbool.h>
#include <stdint.h>

/* Stationary zero-rate biases from the completed 20-minute MPU6500 run. */
#define GYRO_FIXED_X_BIAS_COUNTS       140.64f
#define GYRO_FIXED_Y_BIAS_COUNTS       181.20f
#define GYRO_FIXED_Z_BIAS_COUNTS         8.51f

/* MPU6500 gyro sensitivity for the configured +/-500 degrees/second range. */
#define GYRO_COUNTS_PER_DPS             65.5f
#define DEGREES_TO_RADIANS               0.01745329252f

#define GYRO_STARTUP_SETTLE_US     5000000UL
#define GYRO_BIAS_COLLECTION_US    5000000UL
#define GYRO_BIAS_MINIMUM_SAMPLES     2400UL

/* 10 counts standard deviation is approximately 0.153 degrees/second. */
#define GYRO_BIAS_MAX_VARIANCE_COUNTS2  100LL

#define GYRO_EXPECTED_DT_MIN_US        1800UL
#define GYRO_EXPECTED_DT_MAX_US        2200UL

/* Leave a small margin before the signed 16-bit endpoints. */
#define GYRO_RAW_SATURATION_LIMIT       32760

typedef enum
{
    GYRO_CALIBRATION_FIXED = 0,
    GYRO_CALIBRATION_SETTLING,
    GYRO_CALIBRATION_COLLECTING,
    GYRO_CALIBRATION_READY
} gyro_calibration_phase_t;

typedef struct
{
    gyro_calibration_phase_t calibration_phase;
    uint32_t phase_start_timestamp_us;
    bool phase_timestamp_valid;

    int64_t calibration_sum_x;
    int64_t calibration_sum_y;
    int64_t calibration_sum_z;
    int64_t calibration_sum_squares_x;
    int64_t calibration_sum_squares_y;
    int64_t calibration_sum_squares_z;
    uint32_t calibration_sample_count;
    uint32_t calibration_restart_count;

    float bias_x_counts;
    float bias_y_counts;
    float bias_z_counts;

    uint32_t last_sequence;
    bool sequence_valid;

    uint32_t previous_timestamp_us;
    bool timestamp_valid;
} gyro_pipeline_state_t;

static gyro_pipeline_state_t pipeline_state;

static bool raw_is_saturated(const gyro_pipeline_input_t *input)
{
    return
        (input->raw_x >= GYRO_RAW_SATURATION_LIMIT) ||
        (input->raw_x <= -GYRO_RAW_SATURATION_LIMIT) ||
        (input->raw_y >= GYRO_RAW_SATURATION_LIMIT) ||
        (input->raw_y <= -GYRO_RAW_SATURATION_LIMIT) ||
        (input->raw_z >= GYRO_RAW_SATURATION_LIMIT) ||
        (input->raw_z <= -GYRO_RAW_SATURATION_LIMIT);
}

static void copy_input_to_output(
    const gyro_pipeline_input_t *input,
    gyro_pipeline_output_t *output)
{
    output->sequence = input->sequence;
    output->timestamp_us = input->timestamp_us;
    output->raw_x = input->raw_x;
    output->raw_y = input->raw_y;
    output->raw_z = input->raw_z;

    output->bias_x_counts = pipeline_state.bias_x_counts;
    output->bias_y_counts = pipeline_state.bias_y_counts;
    output->bias_z_counts = pipeline_state.bias_z_counts;

    output->calibration_sample_count =
        pipeline_state.calibration_sample_count;
    output->calibration_restart_count =
        pipeline_state.calibration_restart_count;
}

static void start_settling(uint32_t timestamp_us)
{
    pipeline_state.calibration_phase =
        GYRO_CALIBRATION_SETTLING;
    pipeline_state.phase_start_timestamp_us = timestamp_us;
    pipeline_state.phase_timestamp_valid = true;

    pipeline_state.calibration_sum_x = 0;
    pipeline_state.calibration_sum_y = 0;
    pipeline_state.calibration_sum_z = 0;
    pipeline_state.calibration_sum_squares_x = 0;
    pipeline_state.calibration_sum_squares_y = 0;
    pipeline_state.calibration_sum_squares_z = 0;
    pipeline_state.calibration_sample_count = 0UL;
}

static void start_collection(uint32_t timestamp_us)
{
    pipeline_state.calibration_phase =
        GYRO_CALIBRATION_COLLECTING;
    pipeline_state.phase_start_timestamp_us = timestamp_us;
    pipeline_state.phase_timestamp_valid = true;

    pipeline_state.calibration_sum_x = 0;
    pipeline_state.calibration_sum_y = 0;
    pipeline_state.calibration_sum_z = 0;
    pipeline_state.calibration_sum_squares_x = 0;
    pipeline_state.calibration_sum_squares_y = 0;
    pipeline_state.calibration_sum_squares_z = 0;
    pipeline_state.calibration_sample_count = 0UL;
}

static void accumulate_calibration_sample(
    const gyro_pipeline_input_t *input)
{
    int64_t x;
    int64_t y;
    int64_t z;

    x = (int64_t)input->raw_x;
    y = (int64_t)input->raw_y;
    z = (int64_t)input->raw_z;

    pipeline_state.calibration_sum_x += x;
    pipeline_state.calibration_sum_y += y;
    pipeline_state.calibration_sum_z += z;

    pipeline_state.calibration_sum_squares_x += x * x;
    pipeline_state.calibration_sum_squares_y += y * y;
    pipeline_state.calibration_sum_squares_z += z * z;
    pipeline_state.calibration_sample_count++;
}

static bool axis_is_stable(int64_t sum, int64_t sum_squares)
{
    int64_t sample_count;
    int64_t variance_numerator;
    int64_t maximum_variance_numerator;

    sample_count =
        (int64_t)pipeline_state.calibration_sample_count;

    /*
     * Variance = (N * sum(x^2) - sum(x)^2) / N^2.
     * Keeping this comparison in integer form avoids sqrt and rounding.
     */
    variance_numerator =
        (sample_count * sum_squares) - (sum * sum);

    maximum_variance_numerator =
        GYRO_BIAS_MAX_VARIANCE_COUNTS2 *
        sample_count * sample_count;

    return variance_numerator <= maximum_variance_numerator;
}

static bool calibration_is_stable(void)
{
    return
        axis_is_stable(
            pipeline_state.calibration_sum_x,
            pipeline_state.calibration_sum_squares_x) &&
        axis_is_stable(
            pipeline_state.calibration_sum_y,
            pipeline_state.calibration_sum_squares_y) &&
        axis_is_stable(
            pipeline_state.calibration_sum_z,
            pipeline_state.calibration_sum_squares_z);
}

static void complete_startup_calibration(void)
{
    float sample_count;

    sample_count = (float)pipeline_state.calibration_sample_count;

    pipeline_state.bias_x_counts =
        (float)pipeline_state.calibration_sum_x / sample_count;
    pipeline_state.bias_y_counts =
        (float)pipeline_state.calibration_sum_y / sample_count;
    pipeline_state.bias_z_counts =
        (float)pipeline_state.calibration_sum_z / sample_count;

    pipeline_state.calibration_phase = GYRO_CALIBRATION_READY;

    /* The first post-calibration sample starts a fresh dt history. */
    pipeline_state.timestamp_valid = false;
}

static bool process_startup_calibration(
    const gyro_pipeline_input_t *input,
    gyro_pipeline_output_t *output)
{
    uint32_t elapsed_us;

    if (!pipeline_state.phase_timestamp_valid)
    {
        start_settling(input->timestamp_us);
    }

    if (pipeline_state.calibration_phase ==
        GYRO_CALIBRATION_SETTLING)
    {
        elapsed_us =
            input->timestamp_us -
            pipeline_state.phase_start_timestamp_us;

        if (elapsed_us < GYRO_STARTUP_SETTLE_US)
        {
            output->flags = GYRO_BIAS_SETTLING;
            return false;
        }

        start_collection(input->timestamp_us);
    }

    if (pipeline_state.calibration_phase ==
        GYRO_CALIBRATION_COLLECTING)
    {
        accumulate_calibration_sample(input);

        elapsed_us =
            input->timestamp_us -
            pipeline_state.phase_start_timestamp_us;

        if (elapsed_us < GYRO_BIAS_COLLECTION_US)
        {
            output->flags = GYRO_BIAS_CALIBRATING;
            output->calibration_sample_count =
                pipeline_state.calibration_sample_count;
            return false;
        }

        if (pipeline_state.calibration_sample_count <
            GYRO_BIAS_MINIMUM_SAMPLES)
        {
            pipeline_state.calibration_restart_count++;
            start_settling(input->timestamp_us);

            output->flags = GYRO_BIAS_SETTLING;
            output->calibration_restart_count =
                pipeline_state.calibration_restart_count;
            return false;
        }

        if (!calibration_is_stable())
        {
            output->calibration_sample_count =
                pipeline_state.calibration_sample_count;

            pipeline_state.calibration_restart_count++;
            start_settling(input->timestamp_us);

            output->flags =
                GYRO_BIAS_SETTLING |
                GYRO_BIAS_UNSTABLE;
            output->calibration_restart_count =
                pipeline_state.calibration_restart_count;
            return false;
        }

        complete_startup_calibration();

        output->bias_x_counts = pipeline_state.bias_x_counts;
        output->bias_y_counts = pipeline_state.bias_y_counts;
        output->bias_z_counts = pipeline_state.bias_z_counts;
        output->calibration_sample_count =
            pipeline_state.calibration_sample_count;
        output->flags =
            GYRO_BIAS_READY |
            GYRO_BIAS_FROM_STARTUP;

        return false;
    }

    return true;
}

void gyro_pipeline_init(bool startup_bias_calibration_enabled)
{
    pipeline_state = (gyro_pipeline_state_t){0};

    if (startup_bias_calibration_enabled)
    {
        pipeline_state.calibration_phase =
            GYRO_CALIBRATION_SETTLING;
    }
    else
    {
        pipeline_state.calibration_phase =
            GYRO_CALIBRATION_FIXED;

        pipeline_state.bias_x_counts =
            GYRO_FIXED_X_BIAS_COUNTS;
        pipeline_state.bias_y_counts =
            GYRO_FIXED_Y_BIAS_COUNTS;
        pipeline_state.bias_z_counts =
            GYRO_FIXED_Z_BIAS_COUNTS;
    }
}

bool gyro_pipeline_process(
    const gyro_pipeline_input_t *input,
    gyro_pipeline_output_t *output)
{
    uint32_t dt_us;
    uint32_t validation_flags;

    if ((input == (const gyro_pipeline_input_t *)0) ||
        (output == (gyro_pipeline_output_t *)0))
    {
        return false;
    }

    *output = (gyro_pipeline_output_t){0};
    copy_input_to_output(input, output);
    validation_flags = 0UL;

    if (pipeline_state.sequence_valid &&
        (input->sequence == pipeline_state.last_sequence))
    {
        output->flags = GYRO_DUPLICATE_SEQUENCE;
        return false;
    }

    if (pipeline_state.sequence_valid &&
        (input->sequence != (pipeline_state.last_sequence + 1UL)))
    {
        validation_flags |= GYRO_SEQUENCE_GAP;
    }

    if (raw_is_saturated(input))
    {
        output->flags =
            validation_flags |
            GYRO_INPUT_SATURATED;
        return false;
    }

    pipeline_state.last_sequence = input->sequence;
    pipeline_state.sequence_valid = true;

    if ((pipeline_state.calibration_phase ==
         GYRO_CALIBRATION_SETTLING) ||
        (pipeline_state.calibration_phase ==
         GYRO_CALIBRATION_COLLECTING))
    {
        if (!process_startup_calibration(input, output))
        {
            output->flags |= validation_flags;
            return false;
        }
    }

    output->flags =
        validation_flags |
        GYRO_BIAS_READY;

    if (pipeline_state.calibration_phase ==
        GYRO_CALIBRATION_FIXED)
    {
        output->flags |= GYRO_BIAS_FROM_FIXED;
    }
    else
    {
        output->flags |= GYRO_BIAS_FROM_STARTUP;
    }

    if (pipeline_state.timestamp_valid)
    {
        dt_us =
            input->timestamp_us -
            pipeline_state.previous_timestamp_us;

        output->sample_interval_us = dt_us;
        output->dt_s = (float)dt_us * 0.000001f;
        output->flags |= GYRO_DT_VALID;

        if ((dt_us < GYRO_EXPECTED_DT_MIN_US) ||
            (dt_us > GYRO_EXPECTED_DT_MAX_US))
        {
            output->flags |= GYRO_TIMING_WARNING;
        }
    }

    pipeline_state.previous_timestamp_us = input->timestamp_us;
    pipeline_state.timestamp_valid = true;

    output->bias_x_counts = pipeline_state.bias_x_counts;
    output->bias_y_counts = pipeline_state.bias_y_counts;
    output->bias_z_counts = pipeline_state.bias_z_counts;

    output->corrected_x_counts =
        (float)input->raw_x - pipeline_state.bias_x_counts;
    output->corrected_y_counts =
        (float)input->raw_y - pipeline_state.bias_y_counts;
    output->corrected_z_counts =
        (float)input->raw_z - pipeline_state.bias_z_counts;

    output->x_dps = output->corrected_x_counts / GYRO_COUNTS_PER_DPS;
    output->y_dps = output->corrected_y_counts / GYRO_COUNTS_PER_DPS;
    output->z_dps = output->corrected_z_counts / GYRO_COUNTS_PER_DPS;

    output->x_rad_s = output->x_dps * DEGREES_TO_RADIANS;
    output->y_rad_s = output->y_dps * DEGREES_TO_RADIANS;
    output->z_rad_s = output->z_dps * DEGREES_TO_RADIANS;

    output->flags |= GYRO_PIPELINE_VALID;

    return true;
}
