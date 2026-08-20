#include "imu_body_frame.h"

/*
 * Physical roll and pitch tests established the component signs used by
 * the current body-frame conversion. Apply the same mapping to both
 * accelerometer and gyroscope vectors so estimator attitude and measured
 * body rates use the same convention:
 *
 *   sensor X -> body +X
 *   sensor Y -> body -Y
 *   sensor Z -> body -Z
 *
 * These signs are experimentally verified for roll/pitch on the current
 * hardware. The yaw convention must be physically validated separately
 * before the final yaw mixer signs are defined.
 */
#define SENSOR_TO_BODY_X_SIGN ( 1.0f)
#define SENSOR_TO_BODY_Y_SIGN (-1.0f)
#define SENSOR_TO_BODY_Z_SIGN (-1.0f)

bool imu_body_frame_from_pipeline(
    const accel_pipeline_output_t *accel,
    const gyro_pipeline_output_t *gyro,
    imu_body_sample_t *body)
{
    if ((accel == (const accel_pipeline_output_t *)0) ||
        (gyro == (const gyro_pipeline_output_t *)0) ||
        (body == (imu_body_sample_t *)0))
    {
        return false;
    }

    *body = (imu_body_sample_t){0};

    body->accel_sequence = accel->sequence;
    body->gyro_sequence = gyro->sequence;
    body->timestamp_us = gyro->timestamp_us;
    body->sample_interval_us = gyro->sample_interval_us;
    body->accel_flags = accel->flags;
    body->gyro_flags = gyro->flags;
    body->dt_s = gyro->dt_s;

    body->specific_force_x_g =
        SENSOR_TO_BODY_X_SIGN * accel->filtered_x_g;
    body->specific_force_y_g =
        SENSOR_TO_BODY_Y_SIGN * accel->filtered_y_g;
    body->specific_force_z_g =
        SENSOR_TO_BODY_Z_SIGN * accel->filtered_z_g;

    body->angular_rate_x_rad_s =
        SENSOR_TO_BODY_X_SIGN * gyro->x_rad_s;
    body->angular_rate_y_rad_s =
        SENSOR_TO_BODY_Y_SIGN * gyro->y_rad_s;
    body->angular_rate_z_rad_s =
        SENSOR_TO_BODY_Z_SIGN * gyro->z_rad_s;

    return true;
}