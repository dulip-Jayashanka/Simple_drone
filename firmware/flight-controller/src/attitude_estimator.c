#include "attitude_estimator.h"

#include <stdbool.h>
#include <stdint.h>

#define ATTITUDE_ACCEL_MIN_G              0.85f
#define ATTITUDE_ACCEL_MAX_G              1.15f
#define ATTITUDE_DT_MIN_S                 0.0005f
#define ATTITUDE_DT_MAX_S                 0.0200f
#define ATTITUDE_VECTOR_NORM_MIN_SQUARED  1.0e-12f
#define ATTITUDE_VECTOR_NORM_MAX_SQUARED  1000000.0f
#define ATTITUDE_PI                       3.14159265358979323846f
#define ATTITUDE_DEGREES_PER_RADIAN       57.295779513082320876f
#define ATTITUDE_DEFAULT_EULER_RATE_HZ    100UL

typedef struct
{
    bool initialized;
    bool euler_timestamp_valid;

    uint32_t previous_euler_timestamp_us;
    uint32_t euler_period_us;

    uint32_t accepted_update_count;
    uint32_t rejected_update_count;
    uint32_t accel_rejection_count;

    float proportional_gain;

    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} attitude_estimator_state_t;

static attitude_estimator_state_t estimator_state;

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clamp_unit(float value)
{
    if (value > 1.0f)
    {
        return 1.0f;
    }

    if (value < -1.0f)
    {
        return -1.0f;
    }

    return value;
}

static bool inverse_square_root(float value, float *result)
{
    union
    {
        float as_float;
        uint32_t as_uint32;
    } converter;

    float estimate;
    float half_value;

    if ((result == (float *)0) ||
        !(value > ATTITUDE_VECTOR_NORM_MIN_SQUARED) ||
        !(value < ATTITUDE_VECTOR_NORM_MAX_SQUARED))
    {
        return false;
    }

    converter.as_float = value;
    converter.as_uint32 =
        0x5F375A86UL - (converter.as_uint32 >> 1U);

    estimate = converter.as_float;
    half_value = 0.5f * value;

    estimate = estimate *
        (1.5f - (half_value * estimate * estimate));
    estimate = estimate *
        (1.5f - (half_value * estimate * estimate));

    *result = estimate;
    return true;
}

static bool square_root(float value, float *result)
{
    float inverse;

    if (result == (float *)0)
    {
        return false;
    }

    if (value == 0.0f)
    {
        *result = 0.0f;
        return true;
    }

    if (!inverse_square_root(value, &inverse))
    {
        return false;
    }

    *result = value * inverse;
    return true;
}

/*
 * Fast atan2 approximation for Euler output.
 *
 * The main estimator state is the quaternion. This approximation avoids
 * pulling the standard math library into the -nostdlib firmware.
 */
static float fast_atan2(float y, float x)
{
    float absolute_y;
    float ratio;
    float angle;

    absolute_y = absolute_float(y) + 1.0e-10f;

    if (x >= 0.0f)
    {
        ratio = (x - absolute_y) / (x + absolute_y);
        angle = ATTITUDE_PI * 0.25f;
    }
    else
    {
        ratio = (x + absolute_y) / (absolute_y - x);
        angle = ATTITUDE_PI * 0.75f;
    }

    angle +=
        ((0.1963f * ratio * ratio) - 0.9817f) * ratio;

    return (y < 0.0f) ? -angle : angle;
}

static bool normalize_quaternion(
    float *w,
    float *x,
    float *y,
    float *z)
{
    float norm_squared;
    float inverse_norm;

    norm_squared =
        (*w * *w) +
        (*x * *x) +
        (*y * *y) +
        (*z * *z);

    if (!inverse_square_root(norm_squared, &inverse_norm))
    {
        return false;
    }

    *w *= inverse_norm;
    *x *= inverse_norm;
    *y *= inverse_norm;
    *z *= inverse_norm;

    return true;
}

static bool calculate_accel_magnitude(
    const imu_body_sample_t *input,
    float *magnitude)
{
    float magnitude_squared;

    magnitude_squared =
        (input->specific_force_x_g *
         input->specific_force_x_g) +
        (input->specific_force_y_g *
         input->specific_force_y_g) +
        (input->specific_force_z_g *
         input->specific_force_z_g);

    return square_root(magnitude_squared, magnitude);
}

static bool accel_magnitude_is_usable(float magnitude)
{
    return
        (magnitude >= ATTITUDE_ACCEL_MIN_G) &&
        (magnitude <= ATTITUDE_ACCEL_MAX_G);
}

static bool initialize_quaternion_from_accel(
    const imu_body_sample_t *input,
    float accel_magnitude)
{
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float horizontal_squared;
    float horizontal;

    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;

    float sin_half_roll;
    float cos_half_roll;
    float sin_half_pitch;
    float cos_half_pitch;
    float root;

    /*
     * An accelerometer measures specific force.
     * When stationary, specific force points opposite to gravity.
     */
    gravity_x =
        -input->specific_force_x_g / accel_magnitude;
    gravity_y =
        -input->specific_force_y_g / accel_magnitude;
    gravity_z =
        -input->specific_force_z_g / accel_magnitude;

    horizontal_squared =
        (gravity_y * gravity_y) +
        (gravity_z * gravity_z);

    if (!square_root(horizontal_squared, &horizontal))
    {
        return false;
    }

    if (horizontal > 1.0e-6f)
    {
        sin_roll =
            clamp_unit(gravity_y / horizontal);
        cos_roll =
            clamp_unit(gravity_z / horizontal);
    }
    else
    {
        /*
         * Roll is unobservable at exactly
         * positive or negative 90 degrees of pitch.
         */
        sin_roll = 0.0f;
        cos_roll = 1.0f;
    }

    sin_pitch = clamp_unit(-gravity_x);
    cos_pitch = clamp_unit(horizontal);

    if (!square_root(
            0.5f * (1.0f + cos_roll),
            &cos_half_roll) ||
        !square_root(
            0.5f * (1.0f - cos_roll),
            &root))
    {
        return false;
    }

    sin_half_roll =
        (sin_roll < 0.0f) ? -root : root;

    if (!square_root(
            0.5f * (1.0f + cos_pitch),
            &cos_half_pitch) ||
        !square_root(
            0.5f * (1.0f - cos_pitch),
            &root))
    {
        return false;
    }

    sin_half_pitch =
        (sin_pitch < 0.0f) ? -root : root;

    /*
     * 3-2-1 Euler quaternion with yaw initialized to zero.
     */
    estimator_state.quaternion_w =
        cos_half_roll * cos_half_pitch;

    estimator_state.quaternion_x =
        sin_half_roll * cos_half_pitch;

    estimator_state.quaternion_y =
        cos_half_roll * sin_half_pitch;

    estimator_state.quaternion_z =
        -sin_half_roll * sin_half_pitch;

    return normalize_quaternion(
        &estimator_state.quaternion_w,
        &estimator_state.quaternion_x,
        &estimator_state.quaternion_y,
        &estimator_state.quaternion_z);
}

static bool update_euler_angles(void)
{
    float w;
    float x;
    float y;
    float z;
    float sin_pitch;
    float cos_pitch_squared;
    float cos_pitch;

    w = estimator_state.quaternion_w;
    x = estimator_state.quaternion_x;
    y = estimator_state.quaternion_y;
    z = estimator_state.quaternion_z;

    estimator_state.roll_rad = fast_atan2(
        2.0f * ((w * x) + (y * z)),
        1.0f - (2.0f * ((x * x) + (y * y))));

    sin_pitch =
        2.0f * ((w * y) - (z * x));

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    cos_pitch_squared =
        1.0f - (sin_pitch * sin_pitch);

    if (cos_pitch_squared < 0.0f)
    {
        cos_pitch_squared = 0.0f;
    }

    if (!square_root(
            cos_pitch_squared,
            &cos_pitch))
    {
        return false;
    }

    estimator_state.pitch_rad =
        fast_atan2(sin_pitch, cos_pitch);

    estimator_state.yaw_rad = fast_atan2(
        2.0f * ((w * z) + (x * y)),
        1.0f - (2.0f * ((y * y) + (z * z))));

    return true;
}

static void fill_output(
    const imu_body_sample_t *input,
    float accel_magnitude,
    uint32_t flags,
    attitude_estimator_output_t *output)
{
    output->sequence = input->gyro_sequence;
    output->timestamp_us = input->timestamp_us;
    output->sample_interval_us =
        input->sample_interval_us;
    output->flags = flags;

    output->accepted_update_count =
        estimator_state.accepted_update_count;

    output->rejected_update_count =
        estimator_state.rejected_update_count;

    output->accel_rejection_count =
        estimator_state.accel_rejection_count;

    output->dt_s = input->dt_s;
    output->accel_magnitude_g = accel_magnitude;

    output->body_rate_x_rad_s =
        input->angular_rate_x_rad_s;

    output->body_rate_y_rad_s =
        input->angular_rate_y_rad_s;

    output->body_rate_z_rad_s =
        input->angular_rate_z_rad_s;

    output->quaternion_w =
        estimator_state.quaternion_w;

    output->quaternion_x =
        estimator_state.quaternion_x;

    output->quaternion_y =
        estimator_state.quaternion_y;

    output->quaternion_z =
        estimator_state.quaternion_z;

    output->roll_rad = estimator_state.roll_rad;
    output->pitch_rad = estimator_state.pitch_rad;
    output->yaw_rad = estimator_state.yaw_rad;

    output->roll_deg =
        estimator_state.roll_rad *
        ATTITUDE_DEGREES_PER_RADIAN;

    output->pitch_deg =
        estimator_state.pitch_rad *
        ATTITUDE_DEGREES_PER_RADIAN;

    output->yaw_deg =
        estimator_state.yaw_rad *
        ATTITUDE_DEGREES_PER_RADIAN;
}

void attitude_estimator_init(
    float proportional_gain,
    uint32_t euler_rate_hz)
{
    estimator_state =
        (attitude_estimator_state_t){0};

    if ((proportional_gain >= 0.0f) &&
        (proportional_gain <= 50.0f))
    {
        estimator_state.proportional_gain =
            proportional_gain;
    }
    else
    {
        estimator_state.proportional_gain =
            ATTITUDE_ESTIMATOR_DEFAULT_KP;
    }

    if ((euler_rate_hz == 0UL) ||
        (euler_rate_hz > 500UL))
    {
        euler_rate_hz =
            ATTITUDE_DEFAULT_EULER_RATE_HZ;
    }

    estimator_state.euler_period_us =
        1000000UL / euler_rate_hz;
}

bool attitude_estimator_update(
    const imu_body_sample_t *input,
    attitude_estimator_output_t *output)
{
    uint32_t flags;
    bool gyro_valid;
    bool accel_valid;
    bool accel_magnitude_valid;
    float accel_magnitude;

    float gravity_x;
    float gravity_y;
    float gravity_z;

    float predicted_gravity_x;
    float predicted_gravity_y;
    float predicted_gravity_z;

    float error_x;
    float error_y;
    float error_z;

    float rate_x;
    float rate_y;
    float rate_z;

    float w;
    float x;
    float y;
    float z;

    float next_w;
    float next_x;
    float next_y;
    float next_z;
    float half_dt;

    if ((input == (const imu_body_sample_t *)0) ||
        (output ==
         (attitude_estimator_output_t *)0))
    {
        return false;
    }

    *output = (attitude_estimator_output_t){0};

    flags = ATTITUDE_YAW_RELATIVE_ONLY;
    accel_magnitude = 0.0f;

    gyro_valid =
        ((input->gyro_flags &
          (GYRO_PIPELINE_VALID |
           GYRO_BIAS_READY |
           GYRO_DT_VALID)) ==
         (GYRO_PIPELINE_VALID |
          GYRO_BIAS_READY |
          GYRO_DT_VALID));

    accel_valid =
        ((input->accel_flags &
          ACCEL_PIPELINE_VALID) != 0UL);

    if (!gyro_valid)
    {
        flags |= ATTITUDE_GYRO_INVALID;
    }

    if (!accel_valid)
    {
        flags |= ATTITUDE_ACCEL_INVALID;
    }

    if (input->accel_sequence !=
        input->gyro_sequence)
    {
        flags |= ATTITUDE_SEQUENCE_MISMATCH;
    }

    if (!(input->dt_s >= ATTITUDE_DT_MIN_S) ||
        !(input->dt_s <= ATTITUDE_DT_MAX_S))
    {
        flags |= ATTITUDE_DT_INVALID;
    }

    accel_magnitude_valid =
        accel_valid &&
        calculate_accel_magnitude(
            input,
            &accel_magnitude);

    if (accel_valid &&
        !accel_magnitude_valid)
    {
        flags |= ATTITUDE_ACCEL_INVALID;
        accel_valid = false;
    }

    /*
     * Initial state:
     *
     * Wait for valid synchronized accel/gyro data,
     * completed gyro bias calibration, valid dt,
     * and an acceleration magnitude close to 1 g.
     */
    if (!estimator_state.initialized)
    {
        if (!gyro_valid ||
            ((flags & ATTITUDE_DT_INVALID) != 0UL) ||
            ((flags &
              ATTITUDE_SEQUENCE_MISMATCH) != 0UL) ||
            !accel_valid ||
            !accel_magnitude_is_usable(
                accel_magnitude))
        {
            if (accel_valid &&
                !accel_magnitude_is_usable(
                    accel_magnitude))
            {
                flags |= ATTITUDE_ACCEL_REJECTED;
                estimator_state.accel_rejection_count++;
            }

            estimator_state.rejected_update_count++;

            fill_output(
                input,
                accel_magnitude,
                flags,
                output);

            return false;
        }

        if (!initialize_quaternion_from_accel(
                input,
                accel_magnitude) ||
            !update_euler_angles())
        {
            flags |= ATTITUDE_NUMERIC_ERROR;
            estimator_state.rejected_update_count++;

            fill_output(
                input,
                accel_magnitude,
                flags,
                output);

            return false;
        }

        estimator_state.initialized = true;
        estimator_state.accepted_update_count++;

        estimator_state.previous_euler_timestamp_us =
            input->timestamp_us;

        estimator_state.euler_timestamp_valid = true;

        flags |=
            ATTITUDE_VALID |
            ATTITUDE_INITIALIZED |
            ATTITUDE_ACCEL_CORRECTION_USED |
            ATTITUDE_QUATERNION_NORMALIZED |
            ATTITUDE_EULER_UPDATED;

        fill_output(
            input,
            accel_magnitude,
            flags,
            output);

        return true;
    }

    flags |= ATTITUDE_INITIALIZED;

    /*
     * Gyro propagation requires valid angular rate,
     * valid measured dt and synchronized sequences.
     */
    if (!gyro_valid ||
        ((flags & ATTITUDE_DT_INVALID) != 0UL) ||
        ((flags &
          ATTITUDE_SEQUENCE_MISMATCH) != 0UL))
    {
        estimator_state.rejected_update_count++;

        fill_output(
            input,
            accel_magnitude,
            flags,
            output);

        return false;
    }

    rate_x = input->angular_rate_x_rad_s;
    rate_y = input->angular_rate_y_rad_s;
    rate_z = input->angular_rate_z_rad_s;

    /*
     * Use accelerometer correction only when its
     * magnitude is close enough to 1 g.
     */
    if (accel_valid &&
        accel_magnitude_is_usable(
            accel_magnitude))
    {
        gravity_x =
            -input->specific_force_x_g /
            accel_magnitude;

        gravity_y =
            -input->specific_force_y_g /
            accel_magnitude;

        gravity_z =
            -input->specific_force_z_g /
            accel_magnitude;

        w = estimator_state.quaternion_w;
        x = estimator_state.quaternion_x;
        y = estimator_state.quaternion_y;
        z = estimator_state.quaternion_z;

        /*
         * World-down gravity direction predicted
         * in the drone body frame.
         */
        predicted_gravity_x =
            2.0f * ((x * z) - (w * y));

        predicted_gravity_y =
            2.0f * ((w * x) + (y * z));

        predicted_gravity_z =
            1.0f -
            (2.0f * ((x * x) + (y * y)));

        /*
         * Mahony-style error:
         *
         * measured gravity cross predicted gravity.
         */
        error_x =
            (gravity_y * predicted_gravity_z) -
            (gravity_z * predicted_gravity_y);

        error_y =
            (gravity_z * predicted_gravity_x) -
            (gravity_x * predicted_gravity_z);

        error_z =
            (gravity_x * predicted_gravity_y) -
            (gravity_y * predicted_gravity_x);

        rate_x +=
            estimator_state.proportional_gain *
            error_x;

        rate_y +=
            estimator_state.proportional_gain *
            error_y;

        rate_z +=
            estimator_state.proportional_gain *
            error_z;

        flags |= ATTITUDE_ACCEL_CORRECTION_USED;
    }
    else
    {
        estimator_state.accel_rejection_count++;

        if (accel_valid)
        {
            flags |= ATTITUDE_ACCEL_REJECTED;
        }
    }

    /*
     * Quaternion propagation:
     *
     * q_dot = 0.5 * q multiplied by angular rate.
     */
    w = estimator_state.quaternion_w;
    x = estimator_state.quaternion_x;
    y = estimator_state.quaternion_y;
    z = estimator_state.quaternion_z;

    half_dt = 0.5f * input->dt_s;

    next_w =
        w +
        ((-x * rate_x) -
         ( y * rate_y) -
         ( z * rate_z)) * half_dt;

    next_x =
        x +
        (( w * rate_x) +
         ( y * rate_z) -
         ( z * rate_y)) * half_dt;

    next_y =
        y +
        (( w * rate_y) -
         ( x * rate_z) +
         ( z * rate_x)) * half_dt;

    next_z =
        z +
        (( w * rate_z) +
         ( x * rate_y) -
         ( y * rate_x)) * half_dt;

    if (!normalize_quaternion(
            &next_w,
            &next_x,
            &next_y,
            &next_z))
    {
        flags |= ATTITUDE_NUMERIC_ERROR;
        estimator_state.rejected_update_count++;

        fill_output(
            input,
            accel_magnitude,
            flags,
            output);

        return false;
    }

    estimator_state.quaternion_w = next_w;
    estimator_state.quaternion_x = next_x;
    estimator_state.quaternion_y = next_y;
    estimator_state.quaternion_z = next_z;

    estimator_state.accepted_update_count++;

    flags |=
        ATTITUDE_VALID |
        ATTITUDE_QUATERNION_NORMALIZED;

    /*
     * The quaternion is updated at approximately 500 Hz.
     * Euler angles are calculated at the selected lower rate.
     *
     * Unsigned subtraction correctly handles TIM2 wraparound.
     */
    if ((!estimator_state.euler_timestamp_valid) ||
        ((uint32_t)(
             input->timestamp_us -
             estimator_state.previous_euler_timestamp_us) >=
         estimator_state.euler_period_us))
    {
        if (!update_euler_angles())
        {
            flags &= ~ATTITUDE_VALID;
            flags |= ATTITUDE_NUMERIC_ERROR;
            estimator_state.rejected_update_count++;

            fill_output(
                input,
                accel_magnitude,
                flags,
                output);

            return false;
        }

        estimator_state.previous_euler_timestamp_us =
            input->timestamp_us;

        estimator_state.euler_timestamp_valid = true;

        flags |= ATTITUDE_EULER_UPDATED;
    }

    fill_output(
        input,
        accel_magnitude,
        flags,
        output);

    return true;
}