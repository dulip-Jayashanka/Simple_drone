#ifndef GYRO_PIPELINE_H
#define GYRO_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

/* Output quality, calibration source and timing-state flags. */
#define GYRO_PIPELINE_VALID              (1UL << 0)
#define GYRO_BIAS_READY                  (1UL << 1)
#define GYRO_BIAS_FROM_STARTUP           (1UL << 2)
#define GYRO_BIAS_FROM_FIXED             (1UL << 3)
#define GYRO_BIAS_SETTLING               (1UL << 4)
#define GYRO_BIAS_CALIBRATING            (1UL << 5)
#define GYRO_INPUT_SATURATED             (1UL << 6)
#define GYRO_DUPLICATE_SEQUENCE           (1UL << 7)
#define GYRO_TIMING_WARNING               (1UL << 8)
#define GYRO_DT_VALID                     (1UL << 9)
#define GYRO_BIAS_UNSTABLE                (1UL << 10)
#define GYRO_SEQUENCE_GAP                 (1UL << 11)

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
} gyro_pipeline_input_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t sample_interval_us;
    uint32_t flags;

    uint32_t calibration_sample_count;
    uint32_t calibration_restart_count;

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    float dt_s;

    float bias_x_counts;
    float bias_y_counts;
    float bias_z_counts;

    float corrected_x_counts;
    float corrected_y_counts;
    float corrected_z_counts;

    float x_dps;
    float y_dps;
    float z_dps;

    float x_rad_s;
    float y_rad_s;
    float z_rad_s;
} gyro_pipeline_output_t;

/*
 * Clear all pipeline state and choose the zero-rate bias source.
 *
 * false: immediately use the fixed biases measured from the 20-minute run.
 * true:  wait five seconds, then average five seconds of startup samples.
 */
void gyro_pipeline_init(bool startup_bias_calibration_enabled);

/*
 * Process one valid raw gyroscope sample.
 *
 * true means a bias-corrected angular-rate output is ready.
 * false means the sample was rejected or startup calibration is still active.
 * In every case, output describes the latest pipeline state.
 */
bool gyro_pipeline_process(
    const gyro_pipeline_input_t *input,
    gyro_pipeline_output_t *output);

#endif /* GYRO_PIPELINE_H */
