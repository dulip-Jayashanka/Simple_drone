#include "accel_pipeline.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

_Static_assert(sizeof(accel_pipeline_input_t) == 16U,
               "Unexpected pipeline input size");
_Static_assert(sizeof(accel_pipeline_output_t) == 60U,
               "Unexpected pipeline output size");

static bool close_to(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static accel_pipeline_output_t process_sample(
    uint32_t sequence,
    int16_t x,
    int16_t y,
    int16_t z)
{
    accel_pipeline_input_t input;
    accel_pipeline_output_t output;

    input.sequence = sequence;
    input.timestamp_us = sequence * 2000U;
    input.raw_x = x;
    input.raw_y = y;
    input.raw_z = z;

    assert(accel_pipeline_process(&input, &output));
    assert((output.flags & ACCEL_PIPELINE_VALID) != 0U);

    return output;
}

static void test_calibration_centres(void)
{
    accel_pipeline_output_t output;

    accel_pipeline_init();
    output = process_sample(1U, 8189, 9, -111);
    assert(close_to(output.calibrated_x_g, 1.0f, 0.001f));
    assert(close_to(output.calibrated_y_g, 0.0f, 0.001f));
    assert(close_to(output.calibrated_z_g, 0.0f, 0.001f));

    accel_pipeline_init();
    output = process_sample(1U, -8142, 9, -111);
    assert(close_to(output.calibrated_x_g, -1.0f, 0.001f));

    accel_pipeline_init();
    output = process_sample(1U, 23, 8197, -111);
    assert(close_to(output.calibrated_x_g, 0.0f, 0.001f));
    assert(close_to(output.calibrated_y_g, 1.0f, 0.001f));
    assert(close_to(output.calibrated_z_g, 0.0f, 0.001f));

    accel_pipeline_init();
    output = process_sample(1U, 23, -8179, -111);
    assert(close_to(output.calibrated_y_g, -1.0f, 0.001f));

    accel_pipeline_init();
    output = process_sample(1U, 23, 9, 8122);
    assert(close_to(output.calibrated_x_g, 0.0f, 0.001f));
    assert(close_to(output.calibrated_y_g, 0.0f, 0.001f));
    assert(close_to(output.calibrated_z_g, 1.0f, 0.001f));
    assert(close_to(output.filtered_z_ms2, 9.80665f, 0.01f));

    accel_pipeline_init();
    output = process_sample(1U, 23, 9, -8345);
    assert(close_to(output.calibrated_z_g, -1.0f, 0.001f));
    assert(close_to(output.filtered_z_ms2, -9.80665f, 0.01f));
}

static void test_median_rejects_single_spike(void)
{
    accel_pipeline_output_t output;

    accel_pipeline_init();
    (void)process_sample(1U, 100, 200, 300);
    (void)process_sample(2U, 101, 201, 301);
    output = process_sample(3U, 20000, -20000, 15000);

    assert(output.median_x == 101);
    assert(output.median_y == 200);
    assert(output.median_z == 301);
}

static void test_rejected_sample_does_not_change_state(void)
{
    accel_pipeline_input_t input;
    accel_pipeline_output_t output;

    accel_pipeline_init();
    (void)process_sample(10U, 10, 20, 30);

    input.sequence = 10U;
    input.timestamp_us = 22000U;
    input.raw_x = 1000;
    input.raw_y = 1000;
    input.raw_z = 1000;

    assert(!accel_pipeline_process(&input, &output));
    assert((output.flags & ACCEL_PIPELINE_DUPLICATE_SEQUENCE) != 0U);

    input.sequence = 11U;
    input.raw_x = 32767;
    assert(!accel_pipeline_process(&input, &output));
    assert((output.flags & ACCEL_PIPELINE_INPUT_SATURATED) != 0U);

    output = process_sample(11U, 11, 21, 31);
    assert(output.median_x == 10);
    assert(output.median_y == 20);
    assert(output.median_z == 30);
}

int main(void)
{
    test_calibration_centres();
    test_median_rejects_single_spike();
    test_rejected_sample_does_not_change_state();

    puts("accel_pipeline_test: PASS");
    return 0;
}
