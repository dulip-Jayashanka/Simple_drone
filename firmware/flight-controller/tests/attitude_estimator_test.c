#include "attitude_estimator.h"
#include "imu_body_frame.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD_F (PI_F / 180.0f)

static bool close_to(
    float actual,
    float expected,
    float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static imu_body_sample_t level_sample(
    uint32_t sequence)
{
    imu_body_sample_t sample = {0};

    sample.accel_sequence = sequence;
    sample.gyro_sequence = sequence;
    sample.timestamp_us = sequence * 2000UL;
    sample.sample_interval_us = 2000UL;

    sample.accel_flags =
        ACCEL_PIPELINE_VALID;

    sample.gyro_flags =
        GYRO_PIPELINE_VALID |
        GYRO_BIAS_READY |
        GYRO_DT_VALID;

    sample.dt_s = 0.002f;

    /*
     * Level drone using +Z down:
     * stationary specific force points upward.
     */
    sample.specific_force_x_g = 0.0f;
    sample.specific_force_y_g = 0.0f;
    sample.specific_force_z_g = -1.0f;

    return sample;
}

static float quaternion_norm(
    const attitude_estimator_output_t *output)
{
    return sqrtf(
        (output->quaternion_w *
         output->quaternion_w) +
        (output->quaternion_x *
         output->quaternion_x) +
        (output->quaternion_y *
         output->quaternion_y) +
        (output->quaternion_z *
         output->quaternion_z));
}

static void test_body_mapping(void)
{
    accel_pipeline_output_t accel = {0};
    gyro_pipeline_output_t gyro = {0};
    imu_body_sample_t body;

    accel.sequence = 7UL;
    accel.timestamp_us = 100UL;
    accel.flags = ACCEL_PIPELINE_VALID;
    accel.filtered_x_g = 1.0f;
    accel.filtered_y_g = 2.0f;
    accel.filtered_z_g = 3.0f;

    gyro.sequence = 7UL;
    gyro.timestamp_us = 100UL;
    gyro.sample_interval_us = 2000UL;
    gyro.flags =
        GYRO_PIPELINE_VALID |
        GYRO_BIAS_READY |
        GYRO_DT_VALID;

    gyro.dt_s = 0.002f;
    gyro.x_rad_s = 4.0f;
    gyro.y_rad_s = 5.0f;
    gyro.z_rad_s = 6.0f;

    assert(imu_body_frame_from_pipeline(
        &accel,
        &gyro,
        &body));

    assert(close_to(
        body.specific_force_x_g,
        -1.0f,
        1.0e-6f));

    assert(close_to(
        body.specific_force_y_g,
        -2.0f,
        1.0e-6f));

    assert(close_to(
        body.specific_force_z_g,
        3.0f,
        1.0e-6f));

    assert(close_to(
        body.angular_rate_x_rad_s,
        -4.0f,
        1.0e-6f));

    assert(close_to(
        body.angular_rate_y_rad_s,
        -5.0f,
        1.0e-6f));

    assert(close_to(
        body.angular_rate_z_rad_s,
        6.0f,
        1.0e-6f));
}

static void test_level_initialization(void)
{
    imu_body_sample_t sample =
        level_sample(1UL);

    attitude_estimator_output_t output;

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        100UL);

    assert(attitude_estimator_update(
        &sample,
        &output));

    assert(
        (output.flags & ATTITUDE_VALID) != 0UL);

    assert(
        (output.flags &
         ATTITUDE_INITIALIZED) != 0UL);

    assert(
        (output.flags &
         ATTITUDE_YAW_RELATIVE_ONLY) != 0UL);

    assert(close_to(
        output.roll_deg,
        0.0f,
        0.2f));

    assert(close_to(
        output.pitch_deg,
        0.0f,
        0.2f));

    assert(close_to(
        output.yaw_deg,
        0.0f,
        0.2f));

    assert(close_to(
        quaternion_norm(&output),
        1.0f,
        1.0e-4f));
}

static void test_tilt_initialization(void)
{
    imu_body_sample_t sample;
    attitude_estimator_output_t output;

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        100UL);

    sample = level_sample(1UL);

    /* Positive 30-degree roll. */
    sample.specific_force_y_g = -0.5f;
    sample.specific_force_z_g = -0.8660254f;

    assert(attitude_estimator_update(
        &sample,
        &output));

    assert(close_to(
        output.roll_deg,
        30.0f,
        0.3f));

    assert(close_to(
        output.pitch_deg,
        0.0f,
        0.3f));

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        100UL);

    sample = level_sample(1UL);

    /* Positive 30-degree pitch. */
    sample.specific_force_x_g = 0.5f;
    sample.specific_force_z_g = -0.8660254f;

    assert(attitude_estimator_update(
        &sample,
        &output));

    assert(close_to(
        output.roll_deg,
        0.0f,
        0.3f));

    assert(close_to(
        output.pitch_deg,
        30.0f,
        0.3f));
}

static attitude_estimator_output_t
integrate_one_axis(
    float rate_x,
    float rate_y,
    float rate_z)
{
    imu_body_sample_t sample =
        level_sample(1UL);

    attitude_estimator_output_t output;
    uint32_t index;

    /*
     * Kp=0 isolates pure gyro integration
     * for this test.
     */
    attitude_estimator_init(0.0f, 500UL);

    assert(attitude_estimator_update(
        &sample,
        &output));

    sample.accel_flags = 0UL;

    sample.angular_rate_x_rad_s = rate_x;
    sample.angular_rate_y_rad_s = rate_y;
    sample.angular_rate_z_rad_s = rate_z;

    for (index = 0UL;
         index < 500UL;
         index++)
    {
        sample.accel_sequence++;
        sample.gyro_sequence++;
        sample.timestamp_us += 2000UL;

        assert(attitude_estimator_update(
            &sample,
            &output));
    }

    return output;
}

static void test_positive_body_rates(void)
{
    attitude_estimator_output_t output;

    output = integrate_one_axis(
        90.0f * DEG_TO_RAD_F,
        0.0f,
        0.0f);

    assert(close_to(
        output.roll_deg,
        90.0f,
        0.5f));

    output = integrate_one_axis(
        0.0f,
        90.0f * DEG_TO_RAD_F,
        0.0f);

    assert(close_to(
        output.pitch_deg,
        90.0f,
        0.5f));

    output = integrate_one_axis(
        0.0f,
        0.0f,
        90.0f * DEG_TO_RAD_F);

    assert(close_to(
        output.yaw_deg,
        90.0f,
        0.5f));

    assert(
        (output.flags &
         ATTITUDE_YAW_RELATIVE_ONLY) != 0UL);

    assert(close_to(
        quaternion_norm(&output),
        1.0f,
        1.0e-4f));
}

static void test_accel_rejection_and_recovery(void)
{
    imu_body_sample_t sample =
        level_sample(1UL);

    attitude_estimator_output_t output;

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        500UL);

    assert(attitude_estimator_update(
        &sample,
        &output));

    sample.accel_sequence++;
    sample.gyro_sequence++;
    sample.timestamp_us += 2000UL;

    sample.specific_force_z_g = -2.0f;

    assert(attitude_estimator_update(
        &sample,
        &output));

    assert(
        (output.flags &
         ATTITUDE_ACCEL_REJECTED) != 0UL);

    assert(
        (output.flags &
         ATTITUDE_ACCEL_CORRECTION_USED) == 0UL);

    sample.accel_sequence++;
    sample.gyro_sequence++;
    sample.timestamp_us += 2000UL;

    sample.specific_force_z_g = -1.0f;

    assert(attitude_estimator_update(
        &sample,
        &output));

    assert(
        (output.flags &
         ATTITUDE_ACCEL_CORRECTION_USED) != 0UL);

    assert(
        (output.flags &
         ATTITUDE_ACCEL_REJECTED) == 0UL);
}

static void test_accel_correction_direction(void)
{
    imu_body_sample_t sample =
        level_sample(1UL);

    attitude_estimator_output_t output;
    uint32_t index;

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        100UL);

    /* Initialize at positive 20-degree roll. */
    sample.specific_force_y_g = -0.34202015f;
    sample.specific_force_z_g = -0.93969262f;

    assert(attitude_estimator_update(
        &sample,
        &output));

    assert(close_to(
        output.roll_deg,
        20.0f,
        0.3f));

    /*
     * Supply level gravity with no gyro motion.
     * The correction must move roll toward zero.
     */
    sample.specific_force_y_g = 0.0f;
    sample.specific_force_z_g = -1.0f;

    for (index = 0UL;
         index < 1500UL;
         index++)
    {
        sample.accel_sequence++;
        sample.gyro_sequence++;
        sample.timestamp_us += 2000UL;

        assert(attitude_estimator_update(
            &sample,
            &output));
    }

    assert(fabsf(output.roll_deg) < 0.2f);
}

static void test_rejected_updates(void)
{
    imu_body_sample_t sample =
        level_sample(1UL);

    attitude_estimator_output_t output;
    float initial_w;

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        100UL);

    assert(attitude_estimator_update(
        &sample,
        &output));

    initial_w = output.quaternion_w;

    /* Invalid gyro must prevent integration. */
    sample.accel_sequence++;
    sample.gyro_sequence++;
    sample.timestamp_us += 2000UL;
    sample.gyro_flags = 0UL;

    assert(!attitude_estimator_update(
        &sample,
        &output));

    assert(
        (output.flags &
         ATTITUDE_GYRO_INVALID) != 0UL);

    assert(close_to(
        output.quaternion_w,
        initial_w,
        1.0e-6f));

    /* Sequence mismatch must be rejected. */
    sample.gyro_flags =
        GYRO_PIPELINE_VALID |
        GYRO_BIAS_READY |
        GYRO_DT_VALID;

    sample.accel_sequence++;
    sample.timestamp_us += 2000UL;

    assert(!attitude_estimator_update(
        &sample,
        &output));

    assert(
        (output.flags &
         ATTITUDE_SEQUENCE_MISMATCH) != 0UL);

    /* Unreasonable dt must be rejected. */
    sample.gyro_sequence =
        sample.accel_sequence;

    sample.dt_s = 0.1f;

    assert(!attitude_estimator_update(
        &sample,
        &output));

    assert(
        (output.flags &
         ATTITUDE_DT_INVALID) != 0UL);
}

static void test_combined_rotation_norm(void)
{
    imu_body_sample_t sample =
        level_sample(1UL);

    attitude_estimator_output_t output;
    uint32_t index;

    attitude_estimator_init(0.0f, 100UL);

    assert(attitude_estimator_update(
        &sample,
        &output));

    sample.accel_flags = 0UL;

    sample.angular_rate_x_rad_s = 0.4f;
    sample.angular_rate_y_rad_s = -0.3f;
    sample.angular_rate_z_rad_s = 0.2f;

    for (index = 0UL;
         index < 5000UL;
         index++)
    {
        sample.accel_sequence++;
        sample.gyro_sequence++;
        sample.timestamp_us += 2000UL;

        assert(attitude_estimator_update(
            &sample,
            &output));
    }

    assert(close_to(
        quaternion_norm(&output),
        1.0f,
        1.0e-4f));

    assert(fabsf(output.roll_deg) > 1.0f);
    assert(fabsf(output.pitch_deg) > 1.0f);
    assert(fabsf(output.yaw_deg) > 1.0f);
}

static void test_timestamp_wraparound(void)
{
    imu_body_sample_t sample =
        level_sample(1UL);

    attitude_estimator_output_t output;

    sample.timestamp_us = 0xFFFFFF00UL;

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        500UL);

    assert(attitude_estimator_update(
        &sample,
        &output));

    sample.accel_sequence++;
    sample.gyro_sequence++;
    sample.timestamp_us += 2000UL;

    assert(attitude_estimator_update(
        &sample,
        &output));

    assert(
        (output.flags &
         ATTITUDE_EULER_UPDATED) != 0UL);
}

int main(void)
{
    test_body_mapping();
    test_level_initialization();
    test_tilt_initialization();
    test_positive_body_rates();
    test_accel_rejection_and_recovery();
    test_accel_correction_direction();
    test_rejected_updates();
    test_combined_rotation_norm();
    test_timestamp_wraparound();

    puts("attitude_estimator_test: PASS");

    return 0;
}