#include "gyro_pipeline.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static bool close_to(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static bool process_sample(
    uint32_t sequence,
    uint32_t timestamp_us,
    int16_t x,
    int16_t y,
    int16_t z,
    gyro_pipeline_output_t *output)
{
    gyro_pipeline_input_t input;

    input.sequence = sequence;
    input.timestamp_us = timestamp_us;
    input.raw_x = x;
    input.raw_y = y;
    input.raw_z = z;

    return gyro_pipeline_process(&input, output);
}

static void test_fixed_bias_and_units(void)
{
    gyro_pipeline_output_t output;

    gyro_pipeline_init(false);

    assert(process_sample(1U, 1000U, 141, 181, 9, &output));
    assert((output.flags & GYRO_PIPELINE_VALID) != 0U);
    assert((output.flags & GYRO_BIAS_FROM_FIXED) != 0U);
    assert((output.flags & GYRO_DT_VALID) == 0U);

    assert(close_to(output.bias_x_counts, 140.64f, 0.001f));
    assert(close_to(output.bias_y_counts, 181.20f, 0.001f));
    assert(close_to(output.bias_z_counts, 8.51f, 0.001f));

    assert(close_to(output.corrected_x_counts, 0.36f, 0.001f));
    assert(close_to(output.x_dps, 0.36f / 65.5f, 0.00001f));
    assert(close_to(
        output.x_rad_s,
        (0.36f / 65.5f) * 0.01745329252f,
        0.000001f));
}

static void test_dt_and_timing_warning(void)
{
    gyro_pipeline_output_t output;

    gyro_pipeline_init(false);

    assert(process_sample(1U, 100000U, 141, 181, 9, &output));
    assert(process_sample(2U, 102000U, 141, 181, 9, &output));

    assert(output.sample_interval_us == 2000U);
    assert(close_to(output.dt_s, 0.002f, 0.0000001f));
    assert((output.flags & GYRO_DT_VALID) != 0U);
    assert((output.flags & GYRO_TIMING_WARNING) == 0U);

    assert(process_sample(3U, 105000U, 141, 181, 9, &output));
    assert(output.sample_interval_us == 3000U);
    assert((output.flags & GYRO_TIMING_WARNING) != 0U);
}

static void test_timestamp_wraparound(void)
{
    gyro_pipeline_output_t output;

    gyro_pipeline_init(false);

    assert(process_sample(
        1U, UINT32_MAX - 999U, 141, 181, 9, &output));
    assert(process_sample(2U, 1000U, 141, 181, 9, &output));

    assert(output.sample_interval_us == 2000U);
    assert((output.flags & GYRO_TIMING_WARNING) == 0U);
}

static void test_sequence_gap_is_reported(void)
{
    gyro_pipeline_output_t output;

    gyro_pipeline_init(false);

    assert(process_sample(1U, 1000U, 141, 181, 9, &output));
    assert(process_sample(3U, 3000U, 141, 181, 9, &output));
    assert((output.flags & GYRO_SEQUENCE_GAP) != 0U);
}

static void test_rejected_samples_do_not_advance_timing(void)
{
    gyro_pipeline_output_t output;

    gyro_pipeline_init(false);

    assert(process_sample(10U, 10000U, 141, 181, 9, &output));

    assert(!process_sample(10U, 12000U, 500, 500, 500, &output));
    assert((output.flags & GYRO_DUPLICATE_SEQUENCE) != 0U);

    assert(!process_sample(11U, 12000U, 32767, 0, 0, &output));
    assert((output.flags & GYRO_INPUT_SATURATED) != 0U);

    assert(process_sample(11U, 12000U, 141, 181, 9, &output));
    assert(output.sample_interval_us == 2000U);
}

static void test_startup_bias_calibration(void)
{
    gyro_pipeline_output_t output;
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t i;

    gyro_pipeline_init(true);

    sequence = 1U;
    timestamp_us = 100U;

    assert(!process_sample(
        sequence++, timestamp_us, 150, 190, 10, &output));
    assert((output.flags & GYRO_BIAS_SETTLING) != 0U);

    timestamp_us += 5000000U;
    assert(!process_sample(
        sequence++, timestamp_us, 150, 190, 10, &output));
    assert((output.flags & GYRO_BIAS_CALIBRATING) != 0U);

    for (i = 1U; i <= 2500U; i++)
    {
        timestamp_us += 2000U;
        assert(!process_sample(
            sequence++, timestamp_us, 150, 190, 10, &output));
    }

    assert((output.flags & GYRO_BIAS_READY) != 0U);
    assert((output.flags & GYRO_BIAS_FROM_STARTUP) != 0U);
    assert((output.flags & GYRO_PIPELINE_VALID) == 0U);
    assert(output.calibration_sample_count == 2501U);
    assert(close_to(output.bias_x_counts, 150.0f, 0.001f));
    assert(close_to(output.bias_y_counts, 190.0f, 0.001f));
    assert(close_to(output.bias_z_counts, 10.0f, 0.001f));

    timestamp_us += 2000U;
    assert(process_sample(
        sequence++, timestamp_us, 151, 188, 13, &output));
    assert((output.flags & GYRO_BIAS_FROM_STARTUP) != 0U);
    assert((output.flags & GYRO_DT_VALID) == 0U);
    assert(close_to(output.corrected_x_counts, 1.0f, 0.001f));
    assert(close_to(output.corrected_y_counts, -2.0f, 0.001f));
    assert(close_to(output.corrected_z_counts, 3.0f, 0.001f));

    timestamp_us += 2000U;
    assert(process_sample(
        sequence, timestamp_us, 151, 188, 13, &output));
    assert((output.flags & GYRO_DT_VALID) != 0U);
    assert(output.sample_interval_us == 2000U);
}

static void test_unstable_startup_bias_restarts(void)
{
    gyro_pipeline_output_t output;
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t i;
    int16_t x;

    gyro_pipeline_init(true);

    sequence = 1U;
    timestamp_us = 0U;

    assert(!process_sample(
        sequence++, timestamp_us, 150, 190, 10, &output));

    timestamp_us += 5000000U;
    assert(!process_sample(
        sequence++, timestamp_us, 100, 190, 10, &output));

    for (i = 1U; i <= 2500U; i++)
    {
        timestamp_us += 2000U;
        x = ((i & 1U) == 0U) ? 100 : 200;

        assert(!process_sample(
            sequence++, timestamp_us, x, 190, 10, &output));
    }

    assert((output.flags & GYRO_BIAS_UNSTABLE) != 0U);
    assert((output.flags & GYRO_BIAS_SETTLING) != 0U);
    assert((output.flags & GYRO_BIAS_READY) == 0U);
    assert(output.calibration_restart_count == 1U);
}

int main(void)
{
    test_fixed_bias_and_units();
    test_dt_and_timing_warning();
    test_timestamp_wraparound();
    test_sequence_gap_is_reported();
    test_rejected_samples_do_not_advance_timing();
    test_startup_bias_calibration();
    test_unstable_startup_bias_restarts();

    puts("gyro_pipeline_test: PASS");
    return 0;
}
