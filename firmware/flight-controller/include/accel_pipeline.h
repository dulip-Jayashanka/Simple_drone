#ifndef ACCEL_PIPELINE_H
#define ACCEL_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

/* Output quality/state flags. */
#define ACCEL_PIPELINE_VALID              (1UL << 0)
#define ACCEL_PIPELINE_MEDIAN_READY       (1UL << 1)
#define ACCEL_PIPELINE_FILTER_INITIALIZED (1UL << 2)
#define ACCEL_PIPELINE_INPUT_SATURATED    (1UL << 3)
#define ACCEL_PIPELINE_DUPLICATE_SEQUENCE (1UL << 4)

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
} accel_pipeline_input_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t flags;

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    int16_t median_x;
    int16_t median_y;
    int16_t median_z;

    float calibrated_x_g;
    float calibrated_y_g;
    float calibrated_z_g;

    float filtered_x_g;
    float filtered_y_g;
    float filtered_z_g;

    float filtered_x_ms2;
    float filtered_y_ms2;
    float filtered_z_ms2;
} accel_pipeline_output_t;

/* Clear all median and Butterworth histories. */
void accel_pipeline_init(void);

/*
 * Process one new, valid raw accelerometer sample.
 *
 * false means the sample was rejected (duplicate sequence or saturation).
 * Rejected samples never modify median or Butterworth state.
 */
bool accel_pipeline_process(
    const accel_pipeline_input_t *input,
    accel_pipeline_output_t *output);

#endif /* ACCEL_PIPELINE_H */
