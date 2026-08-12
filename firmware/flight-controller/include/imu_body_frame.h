#ifndef IMU_BODY_FRAME_H
#define IMU_BODY_FRAME_H

#include "accel_pipeline.h"
#include "gyro_pipeline.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t accel_sequence;
    uint32_t gyro_sequence;
    uint32_t timestamp_us;
    uint32_t sample_interval_us;
    uint32_t accel_flags;
    uint32_t gyro_flags;

    float dt_s;

    /* Accelerometer specific force in the drone body frame, in g. */
    float specific_force_x_g;
    float specific_force_y_g;
    float specific_force_z_g;

    /* Body angular rates p, q and r, in radians/second. */
    float angular_rate_x_rad_s;
    float angular_rate_y_rad_s;
    float angular_rate_z_rad_s;
} imu_body_sample_t;

/*
 * Convert processed sensor-frame outputs into the drone body frame:
 *
 *   +X = nose/forward
 *   +Y = drone right
 *   +Z = down
 *
 * Positive rotations are right-wing-down roll, nose-up pitch and
 * nose-right yaw.
 */
bool imu_body_frame_from_pipeline(
    const accel_pipeline_output_t *accel,
    const gyro_pipeline_output_t *gyro,
    imu_body_sample_t *body);

#endif /* IMU_BODY_FRAME_H */