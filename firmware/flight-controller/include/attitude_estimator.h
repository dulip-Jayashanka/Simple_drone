#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "imu_body_frame.h"

#include <stdbool.h>
#include <stdint.h>

#define ATTITUDE_VALID                    (1UL << 0)
#define ATTITUDE_INITIALIZED              (1UL << 1)
#define ATTITUDE_ACCEL_CORRECTION_USED    (1UL << 2)
#define ATTITUDE_ACCEL_REJECTED           (1UL << 3)
#define ATTITUDE_GYRO_INVALID             (1UL << 4)
#define ATTITUDE_ACCEL_INVALID            (1UL << 5)
#define ATTITUDE_DT_INVALID               (1UL << 6)
#define ATTITUDE_SEQUENCE_MISMATCH        (1UL << 7)
#define ATTITUDE_QUATERNION_NORMALIZED    (1UL << 8)
#define ATTITUDE_YAW_RELATIVE_ONLY        (1UL << 9)
#define ATTITUDE_NUMERIC_ERROR            (1UL << 10)
#define ATTITUDE_EULER_UPDATED            (1UL << 11)

#define ATTITUDE_ESTIMATOR_DEFAULT_KP 2.0f

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t sample_interval_us;
    uint32_t flags;

    uint32_t accepted_update_count;
    uint32_t rejected_update_count;
    uint32_t accel_rejection_count;

    float dt_s;
    float accel_magnitude_g;

    float body_rate_x_rad_s;
    float body_rate_y_rad_s;
    float body_rate_z_rad_s;

    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} attitude_estimator_output_t;

/* Clear the quaternion, counters and Euler-output scheduler. */
void attitude_estimator_init(
    float proportional_gain,
    uint32_t euler_rate_hz);

/*
 * Process one synchronized body-frame IMU sample.
 *
 * true means a valid attitude was initialized or propagated for this sample.
 * false means the estimator is still waiting or rejected this update. The
 * output is filled in both cases so flags and retained state can be inspected.
 */
bool attitude_estimator_update(
    const imu_body_sample_t *input,
    attitude_estimator_output_t *output);

#endif /* ATTITUDE_ESTIMATOR_H */