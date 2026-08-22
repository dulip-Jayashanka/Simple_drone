#include "fault_handlers.h"

#if ACCEL_PIPELINE_ENABLE
#include "accel_pipeline.h"
#endif

#if GYRO_PIPELINE_ENABLE
#include "gyro_pipeline.h"
#endif

#if ATTITUDE_ESTIMATOR_ENABLE
#include "attitude_estimator.h"
#include "imu_body_frame.h"
#endif

#if INNER_RATE_CONTROL_ENABLE
#include "rate_controller.h"
#endif

#if OUTER_ATTITUDE_CONTROL_ENABLE
#include "attitude_controller.h"
#endif

#if MOTOR_MIXER_ENABLE
#include "motor_mixer.h"
#endif

#include "imu_acquisition.h"
#include "micros.h"
#include "mpu9250.h"
#include "system_clock.h"
#include "system_time.h"
#include "uart_diag.h"
#include "i2c1.h"

#include <stdbool.h>
#include <stdint.h>


#define UART_BAUD_RATE   115200UL
#define DEBUG_PERIOD_MS  1000UL


#ifndef I2C_BUS_HZ
#define I2C_BUS_HZ 400000UL
#endif


#ifndef ACQ_UART_RAW_DEBUG
#define ACQ_UART_RAW_DEBUG 0
#endif


#ifndef ACCEL_CAPTURE_TEST
#define ACCEL_CAPTURE_TEST 0
#endif


#ifndef GYRO_CAPTURE_TEST
#define GYRO_CAPTURE_TEST 0
#endif


#if ACCEL_CAPTURE_TEST && GYRO_CAPTURE_TEST
#error "ACCEL_CAPTURE_TEST and GYRO_CAPTURE_TEST cannot both be enabled"
#endif


/*
 * ============================================================
 * ACCELEROMETER PIPELINE
 * ============================================================
 */

#ifndef ACCEL_PIPELINE_ENABLE
#define ACCEL_PIPELINE_ENABLE 0
#endif


/*
 * ============================================================
 * GYROSCOPE PIPELINE
 * ============================================================
 */

#ifndef GYRO_PIPELINE_ENABLE
#define GYRO_PIPELINE_ENABLE 0
#endif


#ifndef GYRO_PIPELINE_UART_DEBUG
#define GYRO_PIPELINE_UART_DEBUG 0
#endif


#ifndef GYRO_INITIAL_BIAS_CAL_ENABLE
#define GYRO_INITIAL_BIAS_CAL_ENABLE 0
#endif


#ifndef GYRO_PIPELINE_UART_RATE_HZ
#define GYRO_PIPELINE_UART_RATE_HZ 10
#endif


#if GYRO_PIPELINE_UART_DEBUG && \
    !GYRO_PIPELINE_ENABLE
#error "GYRO_PIPELINE_UART_DEBUG requires GYRO_PIPELINE_ENABLE=1"
#endif


#if GYRO_INITIAL_BIAS_CAL_ENABLE && \
    !GYRO_PIPELINE_ENABLE
#error "GYRO_INITIAL_BIAS_CAL_ENABLE requires GYRO_PIPELINE_ENABLE=1"
#endif


#if GYRO_PIPELINE_UART_DEBUG && \
    ((GYRO_PIPELINE_UART_RATE_HZ < 1) || \
     (GYRO_PIPELINE_UART_RATE_HZ > 500))
#error "GYRO_PIPELINE_UART_RATE_HZ must be from 1 to 500"
#endif


#if GYRO_PIPELINE_ENABLE && \
    (ACCEL_CAPTURE_TEST || GYRO_CAPTURE_TEST)
#error "Gyro pipeline cannot run while a capture test owns the main loop"
#endif


#if GYRO_PIPELINE_UART_DEBUG

#define GYRO_PIPELINE_UART_PERIOD_US \
    (1000000UL / \
     (uint32_t)GYRO_PIPELINE_UART_RATE_HZ)

#endif


/*
 * ============================================================
 * ATTITUDE ESTIMATOR
 * ============================================================
 */

#ifndef ATTITUDE_ESTIMATOR_ENABLE
#define ATTITUDE_ESTIMATOR_ENABLE 0
#endif


#ifndef ATTITUDE_ESTIMATOR_UART_DEBUG
#define ATTITUDE_ESTIMATOR_UART_DEBUG 0
#endif


#ifndef ATTITUDE_ESTIMATOR_UART_RATE_HZ
#define ATTITUDE_ESTIMATOR_UART_RATE_HZ 2
#endif


#ifndef ATTITUDE_EULER_RATE_HZ
#define ATTITUDE_EULER_RATE_HZ 100
#endif


#if ATTITUDE_ESTIMATOR_ENABLE && \
    (!ACCEL_PIPELINE_ENABLE || \
     !GYRO_PIPELINE_ENABLE)
#error "Attitude estimator requires accelerometer and gyro pipelines"
#endif


#if ATTITUDE_ESTIMATOR_ENABLE && \
    (ACCEL_CAPTURE_TEST || GYRO_CAPTURE_TEST)
#error "Attitude estimator cannot run while a capture test owns the main loop"
#endif


#if ATTITUDE_ESTIMATOR_UART_DEBUG && \
    !ATTITUDE_ESTIMATOR_ENABLE
#error "ATTITUDE_ESTIMATOR_UART_DEBUG requires ATTITUDE_ESTIMATOR_ENABLE=1"
#endif


#if ATTITUDE_ESTIMATOR_ENABLE && \
    ((ATTITUDE_EULER_RATE_HZ < 1) || \
     (ATTITUDE_EULER_RATE_HZ > 500))
#error "ATTITUDE_EULER_RATE_HZ must be from 1 to 500"
#endif


#if ATTITUDE_ESTIMATOR_UART_DEBUG && \
    ((ATTITUDE_ESTIMATOR_UART_RATE_HZ < 1) || \
     (ATTITUDE_ESTIMATOR_UART_RATE_HZ > 500))
#error "ATTITUDE_ESTIMATOR_UART_RATE_HZ must be from 1 to 500"
#endif


#if ATTITUDE_ESTIMATOR_UART_DEBUG

#define ATTITUDE_ESTIMATOR_UART_PERIOD_US \
    (1000000UL / \
     (uint32_t)ATTITUDE_ESTIMATOR_UART_RATE_HZ)

#endif


/*
 * ============================================================
 * INNER ANGULAR-RATE CONTROL
 * ============================================================
 *
 * Fast loop.
 *
 * approximately 500 Hz:
 *
 * desired angular rate
 *          -
 * measured gyro angular rate
 *          |
 *          v
 *      rate PID
 *          |
 *          v
 * roll / pitch / yaw correction
 */

#ifndef INNER_RATE_CONTROL_ENABLE
#define INNER_RATE_CONTROL_ENABLE 0
#endif


#ifndef INNER_RATE_CONTROL_UART_DEBUG
#define INNER_RATE_CONTROL_UART_DEBUG 0
#endif


#if INNER_RATE_CONTROL_ENABLE && \
    !ATTITUDE_ESTIMATOR_ENABLE
#error "Inner rate control currently requires ATTITUDE_ESTIMATOR_ENABLE=1"
#endif


#if INNER_RATE_CONTROL_UART_DEBUG && \
    !INNER_RATE_CONTROL_ENABLE
#error "INNER_RATE_CONTROL_UART_DEBUG requires INNER_RATE_CONTROL_ENABLE=1"
#endif


#if INNER_RATE_CONTROL_UART_DEBUG

#define INNER_RATE_CONTROL_UART_PERIOD_US \
    100000UL

#endif


/*
 * ============================================================
 * OUTER ATTITUDE CONTROL
 * ============================================================
 *
 * Slower roll/pitch loop.
 *
 * target angle
 *      -
 * estimated angle
 *      |
 *      v
 * angle error
 *      |
 *      v
 * proportional attitude gain
 *      |
 *      v
 * desired angular rate
 *      |
 *      v
 * rate limit
 *      |
 *      v
 * held rate setpoint for inner PID
 *
 * Yaw heading control is intentionally NOT implemented here.
 * The yaw inner PID continues to receive a direct yaw-rate
 * setpoint.
 */

#ifndef OUTER_ATTITUDE_CONTROL_ENABLE
#define OUTER_ATTITUDE_CONTROL_ENABLE 0
#endif


#ifndef OUTER_ATTITUDE_CONTROL_UART_DEBUG
#define OUTER_ATTITUDE_CONTROL_UART_DEBUG 0
#endif


#if OUTER_ATTITUDE_CONTROL_ENABLE && \
    !ATTITUDE_ESTIMATOR_ENABLE
#error "Outer attitude control requires ATTITUDE_ESTIMATOR_ENABLE=1"
#endif


#if OUTER_ATTITUDE_CONTROL_ENABLE && \
    !INNER_RATE_CONTROL_ENABLE
#error "Outer attitude control requires INNER_RATE_CONTROL_ENABLE=1"
#endif


#if OUTER_ATTITUDE_CONTROL_UART_DEBUG && \
    !OUTER_ATTITUDE_CONTROL_ENABLE
#error "OUTER_ATTITUDE_CONTROL_UART_DEBUG requires OUTER_ATTITUDE_CONTROL_ENABLE=1"
#endif


#if OUTER_ATTITUDE_CONTROL_UART_DEBUG

#define OUTER_ATTITUDE_CONTROL_UART_PERIOD_US \
    500000UL

#endif


/*
 * ============================================================
 * MOTOR MIXER
 * ============================================================
 *
 * The mixer belongs to the fast control path:
 *
 *     inner rate controller
 *             |
 *             v
 *       R / P / Y corrections
 *             +
 *         collective C
 *             |
 *             v
 *         motor mixer
 *             |
 *             v
 *       M1 / M2 / M3 / M4
 *
 * This phase stops at normalized M1..M4 diagnostics.
 *
 * There is NO:
 *
 *     FC -> motor-node command transmission
 *     ESC PWM generation
 *     motor arming
 *
 * in this phase.
 */

#ifndef MOTOR_MIXER_ENABLE
#define MOTOR_MIXER_ENABLE 0
#endif


/*
 * Independent mixer UART diagnostic switch.
 *
 * Enabling this does NOT enable any of the existing:
 *
 *     gyro pipeline UART
 *     attitude UART
 *     inner-rate UART
 *     outer-attitude UART
 *
 * debug streams.
 */
#ifndef MOTOR_MIXER_UART_DEBUG
#define MOTOR_MIXER_UART_DEBUG 0
#endif


#if MOTOR_MIXER_ENABLE && \
    !INNER_RATE_CONTROL_ENABLE
#error "Motor mixer requires INNER_RATE_CONTROL_ENABLE=1"
#endif


#if MOTOR_MIXER_UART_DEBUG && \
    !MOTOR_MIXER_ENABLE
#error "MOTOR_MIXER_UART_DEBUG requires MOTOR_MIXER_ENABLE=1"
#endif


/*
 * The mixer calculation itself remains on the fast inner-loop
 * path.
 *
 * UART observation is intentionally much slower: 5 Hz.
 */
#if MOTOR_MIXER_UART_DEBUG

#define MOTOR_MIXER_UART_PERIOD_US \
    200000UL

#endif


/*
 * ============================================================
 * EXECUTION-TIME MEASUREMENT
 * ============================================================
 */

#ifndef MEASURE_PIPELINE_TIMES
#define MEASURE_PIPELINE_TIMES 0
#endif


#if MEASURE_PIPELINE_TIMES && \
    !ACCEL_PIPELINE_ENABLE
#error "MEASURE_PIPELINE_TIMES requires ACCEL_PIPELINE_ENABLE=1"
#endif


#ifndef MEASURE_RAW_TO_ATTITUDE_TIMES
#define MEASURE_RAW_TO_ATTITUDE_TIMES 0
#endif


#if MEASURE_RAW_TO_ATTITUDE_TIMES && \
    !ATTITUDE_ESTIMATOR_ENABLE
#error "MEASURE_RAW_TO_ATTITUDE_TIMES requires ATTITUDE_ESTIMATOR_ENABLE=1"
#endif


#if MEASURE_RAW_TO_ATTITUDE_TIMES && \
    MEASURE_PIPELINE_TIMES
#error "Do not enable both timing measurement modes"
#endif


#if MEASURE_RAW_TO_ATTITUDE_TIMES && \
    INNER_RATE_CONTROL_ENABLE
#error "Raw-to-attitude timing must be measured with INNER_RATE_CONTROL_ENABLE=0"
#endif


#if MEASURE_RAW_TO_ATTITUDE_TIMES && \
    (ACQ_UART_RAW_DEBUG || \
     GYRO_PIPELINE_UART_DEBUG || \
     ATTITUDE_ESTIMATOR_UART_DEBUG || \
     INNER_RATE_CONTROL_UART_DEBUG || \
     OUTER_ATTITUDE_CONTROL_UART_DEBUG || \
     MOTOR_MIXER_UART_DEBUG)
#error "Raw-to-attitude timing requires continuous UART debug to be disabled"
#endif


/*
 * ============================================================
 * ACCELEROMETER PIPELINE DEBUG
 * ============================================================
 */

#ifndef ACCEL_PIPELINE_DEBUG_RAW
#define ACCEL_PIPELINE_DEBUG_RAW 0
#endif


#ifndef ACCEL_PIPELINE_DEBUG_MEDIAN
#define ACCEL_PIPELINE_DEBUG_MEDIAN 0
#endif


#ifndef ACCEL_PIPELINE_DEBUG_CALIBRATED_G
#define ACCEL_PIPELINE_DEBUG_CALIBRATED_G 0
#endif


#ifndef ACCEL_PIPELINE_DEBUG_FILTERED_G
#define ACCEL_PIPELINE_DEBUG_FILTERED_G 0
#endif


#ifndef ACCEL_PIPELINE_DEBUG_MS2
#define ACCEL_PIPELINE_DEBUG_MS2 0
#endif


#ifndef ACCEL_PIPELINE_DEBUG_DECIMATION
#define ACCEL_PIPELINE_DEBUG_DECIMATION 50
#endif


#define ACCEL_PIPELINE_DEBUG_ACTIVE ( \
    ACCEL_PIPELINE_DEBUG_RAW || \
    ACCEL_PIPELINE_DEBUG_MEDIAN || \
    ACCEL_PIPELINE_DEBUG_CALIBRATED_G || \
    ACCEL_PIPELINE_DEBUG_FILTERED_G || \
    ACCEL_PIPELINE_DEBUG_MS2)


#define ACCEL_PIPELINE_DEBUG_INT_ACTIVE ( \
    ACCEL_PIPELINE_DEBUG_RAW || \
    ACCEL_PIPELINE_DEBUG_MEDIAN)


#define ACCEL_PIPELINE_DEBUG_FLOAT_ACTIVE ( \
    ACCEL_PIPELINE_DEBUG_CALIBRATED_G || \
    ACCEL_PIPELINE_DEBUG_FILTERED_G || \
    ACCEL_PIPELINE_DEBUG_MS2)


#if ACCEL_PIPELINE_DEBUG_ACTIVE && \
    !ACCEL_PIPELINE_ENABLE
#error "Accelerometer pipeline debug requires ACCEL_PIPELINE_ENABLE=1"
#endif


#if ACCEL_PIPELINE_DEBUG_ACTIVE && \
    (ACCEL_PIPELINE_DEBUG_DECIMATION < 1)
#error "ACCEL_PIPELINE_DEBUG_DECIMATION must be at least 1"
#endif


#if MEASURE_RAW_TO_ATTITUDE_TIMES && \
    ACCEL_PIPELINE_DEBUG_ACTIVE
#error "Raw-to-attitude timing requires accelerometer UART debug to be disabled"
#endif


/*
 * ============================================================
 * MPU TEST
 * ============================================================
 */

#ifndef MPU_WHO_AM_I_TEST
#define MPU_WHO_AM_I_TEST 0
#endif


/*
 * ============================================================
 * GLOBAL PLATFORM STATUS
 * ============================================================
 */

#if IMU_INIT_DEBUG

mpu_init_diag_t
    init_diag;

i2c1_diag_t
    i2c_diag;

#endif


volatile system_clock_status_t
    g_fc_clock_status;


volatile system_time_status_t
    g_fc_time_status;


volatile micros_status_t
    g_fc_micros_status;


volatile uart_diag_status_t
    g_fc_uart_status;


volatile imu_acquisition_status_t
    g_imu_acquisition_status;


/*
 * ============================================================
 * ACCEL CAPTURE BUFFER
 * ============================================================
 */

#if ACCEL_CAPTURE_TEST

#define ACCEL_CAPTURE_DURATION_US \
    5000000UL

#define ACCEL_CAPTURE_MAX_SAMPLES \
    2600UL

#define ACCEL_CAPTURE_TOTAL_BLOCKS \
    240UL


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

} accel_capture_sample_t;


static accel_capture_sample_t
    accel_capture_buffer[
        ACCEL_CAPTURE_MAX_SAMPLES];


static uint32_t
    accel_capture_count;

#endif


/*
 * ============================================================
 * GYRO CAPTURE BUFFER
 * ============================================================
 */

#if GYRO_CAPTURE_TEST

#define GYRO_CAPTURE_DURATION_US \
    5000000UL

#define GYRO_CAPTURE_MAX_SAMPLES \
    2600UL

#define GYRO_CAPTURE_TOTAL_BLOCKS \
    240UL


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

} gyro_capture_sample_t;


static gyro_capture_sample_t
    gyro_capture_buffer[
        GYRO_CAPTURE_MAX_SAMPLES];


static uint32_t
    gyro_capture_count;

#endif


/*
 * ============================================================
 * LATEST PUBLISHED DATA
 * ============================================================
 */

volatile imu_raw_sample_t
    g_latest_raw_imu_sample;


volatile imu_acquisition_stats_t
    g_imu_acquisition_stats;


#if ACCEL_PIPELINE_ENABLE

volatile accel_pipeline_output_t
    g_latest_accel_pipeline_output;

#endif


#if GYRO_PIPELINE_ENABLE

volatile gyro_pipeline_output_t
    g_latest_gyro_pipeline_output;

#endif


#if ATTITUDE_ESTIMATOR_ENABLE

volatile attitude_estimator_output_t
    g_latest_attitude_estimator_output;

#endif


/*
 * ============================================================
 * INNER RATE CONTROLLER
 * ============================================================
 */

#if INNER_RATE_CONTROL_ENABLE

volatile rate_controller_output_t
    g_latest_rate_controller_output;


/*
 * ------------------------------------------------------------
 * TEMPORARY INNER-RATE TEST PARAMETERS
 * ------------------------------------------------------------
 *
 * NOT FLIGHT-TUNED GAINS.
 *
 * Kp = 1
 * Ki = 0
 * Kd = 0
 *
 * This remains the existing sign-test configuration.
 */

#define INNER_RATE_TEST_KP \
    1.0f

#define INNER_RATE_TEST_KI \
    0.0f

#define INNER_RATE_TEST_KD \
    0.0f

#define INNER_RATE_TEST_INTEGRAL_LIMIT \
    1.0f

#define INNER_RATE_TEST_OUTPUT_LIMIT \
    4.0f

#define INNER_RATE_TEST_D_CUTOFF_HZ \
    30.0f


static void
inner_rate_load_sign_test_config(
    rate_controller_config_t *config)
{
    pid_config_t axis;


    if (config ==
        (rate_controller_config_t *)0)
    {
        return;
    }


    axis.kp =
        INNER_RATE_TEST_KP;

    axis.ki =
        INNER_RATE_TEST_KI;

    axis.kd =
        INNER_RATE_TEST_KD;


    axis.integral_min =
        -INNER_RATE_TEST_INTEGRAL_LIMIT;

    axis.integral_max =
        INNER_RATE_TEST_INTEGRAL_LIMIT;


    axis.output_min =
        -INNER_RATE_TEST_OUTPUT_LIMIT;

    axis.output_max =
        INNER_RATE_TEST_OUTPUT_LIMIT;


    axis.derivative_cutoff_hz =
        INNER_RATE_TEST_D_CUTOFF_HZ;


    config->roll =
        axis;

    config->pitch =
        axis;

    config->yaw =
        axis;
}


static bool
inner_rate_source_is_valid(
    const imu_body_sample_t *body)
{
    uint32_t required_flags;


    if (body ==
        (const imu_body_sample_t *)0)
    {
        return false;
    }


    required_flags =
        GYRO_PIPELINE_VALID |
        GYRO_BIAS_READY |
        GYRO_DT_VALID;


    return
        (body->gyro_flags &
         required_flags) ==
        required_flags;
}

#endif


/*
 * ============================================================
 * MOTOR MIXER
 * ============================================================
 */

#if MOTOR_MIXER_ENABLE

volatile motor_mixer_output_t
    g_latest_motor_mixer_output;


/*
 * Temporary mathematical base command used only for the mixer
 * integration test.
 *
 * IMPORTANT:
 *
 *     0.50 does NOT mean known hover throttle.
 *
 *     0.50 is NOT sent to an ESC.
 *
 *     0.50 is NOT a flight-tuned value.
 *
 * It simply provides a visible center point around which the
 * roll/pitch/yaw differential corrections can be inspected.
 */
#define MOTOR_MIXER_TEST_COLLECTIVE \
    0.50f

#endif


/*
 * ============================================================
 * OUTER ATTITUDE CONTROLLER
 * ============================================================
 */

#if OUTER_ATTITUDE_CONTROL_ENABLE

volatile attitude_controller_output_t
    g_latest_attitude_controller_output;


/*
 * ------------------------------------------------------------
 * TEMPORARY LEVEL-HOLD TEST PARAMETERS
 * ------------------------------------------------------------
 */

#define OUTER_ATTITUDE_TEST_ROLL_GAIN_PER_S \
    1.0f

#define OUTER_ATTITUDE_TEST_PITCH_GAIN_PER_S \
    1.0f


#define OUTER_ATTITUDE_TEST_MAX_ROLL_RATE_RAD_S \
    1.0f

#define OUTER_ATTITUDE_TEST_MAX_PITCH_RATE_RAD_S \
    1.0f


static void
outer_attitude_load_level_test_config(
    attitude_controller_config_t *config)
{
    if (config ==
        (attitude_controller_config_t *)0)
    {
        return;
    }


    config->roll_gain_per_s =
        OUTER_ATTITUDE_TEST_ROLL_GAIN_PER_S;


    config->pitch_gain_per_s =
        OUTER_ATTITUDE_TEST_PITCH_GAIN_PER_S;


    config->max_roll_rate_rad_s =
        OUTER_ATTITUDE_TEST_MAX_ROLL_RATE_RAD_S;


    config->max_pitch_rate_rad_s =
        OUTER_ATTITUDE_TEST_MAX_PITCH_RATE_RAD_S;
}

#endif


/*
 * ============================================================
 * ACCEL PIPELINE EXECUTION-TIME PROFILER
 * ============================================================
 */

#if MEASURE_PIPELINE_TIMES

volatile uint32_t
    g_pipeline_time_last_us;

volatile uint32_t
    g_pipeline_time_min_us;

volatile uint32_t
    g_pipeline_time_max_us;

volatile uint64_t
    g_pipeline_time_total_us;

volatile uint32_t
    g_pipeline_time_sample_count;

#endif


/*
 * ============================================================
 * RAW -> ATTITUDE PROFILER
 * ============================================================
 */

#if MEASURE_RAW_TO_ATTITUDE_TIMES

#define RAW_TO_ATTITUDE_TIMING_WARMUP_SAMPLES \
    250UL


typedef struct
{
    uint32_t last_us;
    uint32_t min_us;
    uint32_t max_us;

    uint64_t total_us;

    uint32_t sample_count;

} raw_to_attitude_time_metric_t;


typedef struct
{
    raw_to_attitude_time_metric_t
        raw_processing;

    raw_to_attitude_time_metric_t
        acquisition_and_scheduling;

    raw_to_attitude_time_metric_t
        data_ready_to_output;

    raw_to_attitude_time_metric_t
        euler_output_age;


    uint32_t
        latest_sequence;

    uint32_t
        latest_sample_interval_us;

    uint32_t
        raw_sample_count;

    uint32_t
        valid_output_count;

    uint32_t
        estimator_not_ready_count;

    uint32_t
        body_frame_failure_count;


    uint32_t
        warmup_skipped_count;

    uint32_t
        warmup_remaining;


    uint32_t
        deadline_miss_count;

    uint32_t
        processing_overrun_count;


    uint32_t
        gyro_timing_warning_count;

    uint32_t
        gyro_sequence_gap_count;


    uint32_t
        euler_update_count;

    uint32_t
        euler_reuse_count;


    uint32_t
        first_data_ready_timestamp_us;

    uint32_t
        first_attitude_output_timestamp_us;

    uint32_t
        startup_to_first_attitude_us;

    uint32_t
        first_data_ready_seen;

    uint32_t
        first_attitude_seen;

} raw_to_attitude_timing_report_t;


volatile raw_to_attitude_timing_report_t
    g_raw_to_attitude_timing;


static bool
    raw_to_attitude_measurement_started;


static bool
    raw_to_attitude_euler_timestamp_valid;


static uint32_t
    raw_to_attitude_euler_timestamp_us;


static void
raw_to_attitude_metric_update(
    volatile raw_to_attitude_time_metric_t *metric,
    uint32_t elapsed_us)
{
    if ((metric->sample_count == 0UL) ||
        (elapsed_us < metric->min_us))
    {
        metric->min_us =
            elapsed_us;
    }


    if (elapsed_us >
        metric->max_us)
    {
        metric->max_us =
            elapsed_us;
    }


    metric->last_us =
        elapsed_us;


    metric->total_us +=
        (uint64_t)elapsed_us;


    metric->sample_count++;
}


static void
raw_to_attitude_timing_init(void)
{
    g_raw_to_attitude_timing =
        (raw_to_attitude_timing_report_t){0};


    raw_to_attitude_measurement_started =
        false;


    raw_to_attitude_euler_timestamp_valid =
        false;


    raw_to_attitude_euler_timestamp_us =
        0UL;
}


static void
raw_to_attitude_timing_note_raw_sample(
    uint32_t sequence,
    uint32_t data_ready_timestamp_us)
{
    g_raw_to_attitude_timing
        .latest_sequence =
        sequence;


    g_raw_to_attitude_timing
        .raw_sample_count++;


    if (g_raw_to_attitude_timing
            .first_data_ready_seen == 0UL)
    {
        g_raw_to_attitude_timing
            .first_data_ready_seen =
            1UL;


        g_raw_to_attitude_timing
            .first_data_ready_timestamp_us =
            data_ready_timestamp_us;
    }
}


static void
raw_to_attitude_timing_record(
    uint32_t sequence,
    uint32_t data_ready_timestamp_us,
    uint32_t raw_processing_start_us,
    uint32_t output_timestamp_us,
    uint32_t sample_interval_us,
    uint32_t gyro_flags,
    uint32_t attitude_flags,
    bool attitude_output_ready)
{
    uint32_t
        raw_processing_us;

    uint32_t
        acquisition_and_scheduling_us;

    uint32_t
        data_ready_to_output_us;

    uint32_t
        euler_output_age_us;


    g_raw_to_attitude_timing
        .latest_sequence =
        sequence;


    g_raw_to_attitude_timing
        .latest_sample_interval_us =
        sample_interval_us;


    if ((attitude_flags &
         ATTITUDE_EULER_UPDATED) != 0UL)
    {
        raw_to_attitude_euler_timestamp_us =
            data_ready_timestamp_us;


        raw_to_attitude_euler_timestamp_valid =
            true;
    }


    if (!attitude_output_ready)
    {
        g_raw_to_attitude_timing
            .estimator_not_ready_count++;

        return;
    }


    if (g_raw_to_attitude_timing
            .first_attitude_seen == 0UL)
    {
        g_raw_to_attitude_timing
            .first_attitude_seen =
            1UL;


        g_raw_to_attitude_timing
            .first_attitude_output_timestamp_us =
            output_timestamp_us;


        g_raw_to_attitude_timing
            .startup_to_first_attitude_us =
            output_timestamp_us -
            g_raw_to_attitude_timing
                .first_data_ready_timestamp_us;
    }


    if (!raw_to_attitude_measurement_started)
    {
        raw_to_attitude_measurement_started =
            true;


        g_raw_to_attitude_timing
            .warmup_remaining =
            RAW_TO_ATTITUDE_TIMING_WARMUP_SAMPLES;
    }


    if (g_raw_to_attitude_timing
            .warmup_remaining != 0UL)
    {
        g_raw_to_attitude_timing
            .warmup_remaining--;


        g_raw_to_attitude_timing
            .warmup_skipped_count++;

        return;
    }


    raw_processing_us =
        output_timestamp_us -
        raw_processing_start_us;


    acquisition_and_scheduling_us =
        raw_processing_start_us -
        data_ready_timestamp_us;


    data_ready_to_output_us =
        output_timestamp_us -
        data_ready_timestamp_us;


    raw_to_attitude_metric_update(
        &g_raw_to_attitude_timing
            .raw_processing,
        raw_processing_us);


    raw_to_attitude_metric_update(
        &g_raw_to_attitude_timing
            .acquisition_and_scheduling,
        acquisition_and_scheduling_us);


    raw_to_attitude_metric_update(
        &g_raw_to_attitude_timing
            .data_ready_to_output,
        data_ready_to_output_us);


    g_raw_to_attitude_timing
        .valid_output_count++;


    if (sample_interval_us != 0UL)
    {
        if (data_ready_to_output_us >
            sample_interval_us)
        {
            g_raw_to_attitude_timing
                .deadline_miss_count++;
        }


        if (raw_processing_us >
            sample_interval_us)
        {
            g_raw_to_attitude_timing
                .processing_overrun_count++;
        }
    }


    if ((gyro_flags &
         GYRO_TIMING_WARNING) != 0UL)
    {
        g_raw_to_attitude_timing
            .gyro_timing_warning_count++;
    }


    if ((gyro_flags &
         GYRO_SEQUENCE_GAP) != 0UL)
    {
        g_raw_to_attitude_timing
            .gyro_sequence_gap_count++;
    }


    if ((attitude_flags &
         ATTITUDE_EULER_UPDATED) != 0UL)
    {
        g_raw_to_attitude_timing
            .euler_update_count++;
    }
    else
    {
        g_raw_to_attitude_timing
            .euler_reuse_count++;
    }


    if (raw_to_attitude_euler_timestamp_valid)
    {
        euler_output_age_us =
            output_timestamp_us -
            raw_to_attitude_euler_timestamp_us;


        raw_to_attitude_metric_update(
            &g_raw_to_attitude_timing
                .euler_output_age,
            euler_output_age_us);
    }
}

#endif


/*
 * ============================================================
 * ACCEL UART HELPERS
 * ============================================================
 */

#if ACCEL_PIPELINE_DEBUG_ACTIVE

#if ACCEL_PIPELINE_DEBUG_INT_ACTIVE

static void
accel_debug_write_signed(
    int16_t value)
{
    int32_t wide;


    wide =
        (int32_t)value;


    if (wide < 0)
    {
        (void)uart_diag_write_char(
            '-');

        wide =
            -wide;
    }


    (void)uart_diag_write_uint32(
        (uint32_t)wide);
}

#endif


#if ACCEL_PIPELINE_DEBUG_FLOAT_ACTIVE

static void
accel_debug_write_fixed3(
    float value)
{
    int32_t scaled;

    uint32_t magnitude;
    uint32_t fraction;


    if (value < 0.0f)
    {
        scaled =
            (int32_t)(
                (value * 1000.0f) -
                0.5f);
    }
    else
    {
        scaled =
            (int32_t)(
                (value * 1000.0f) +
                0.5f);
    }


    if (scaled < 0)
    {
        (void)uart_diag_write_char(
            '-');

        magnitude =
            (uint32_t)(-scaled);
    }
    else
    {
        magnitude =
            (uint32_t)scaled;
    }


    (void)uart_diag_write_uint32(
        magnitude / 1000UL);


    (void)uart_diag_write_char(
        '.');


    fraction =
        magnitude % 1000UL;


    if (fraction < 100UL)
    {
        (void)uart_diag_write_char(
            '0');
    }


    if (fraction < 10UL)
    {
        (void)uart_diag_write_char(
            '0');
    }


    (void)uart_diag_write_uint32(
        fraction);
}

#endif


#if ACCEL_PIPELINE_DEBUG_INT_ACTIVE

static void
accel_debug_write_i16_triplet(
    int16_t x,
    int16_t y,
    int16_t z)
{
    accel_debug_write_signed(
        x);

    (void)uart_diag_write_char(
        ',');

    accel_debug_write_signed(
        y);

    (void)uart_diag_write_char(
        ',');

    accel_debug_write_signed(
        z);
}

#endif


#if ACCEL_PIPELINE_DEBUG_FLOAT_ACTIVE

static void
accel_debug_write_float_triplet(
    float x,
    float y,
    float z)
{
    accel_debug_write_fixed3(
        x);

    (void)uart_diag_write_char(
        ',');

    accel_debug_write_fixed3(
        y);

    (void)uart_diag_write_char(
        ',');

    accel_debug_write_fixed3(
        z);
}

#endif


static void
accel_debug_write_pipeline(
    const accel_pipeline_output_t *output)
{
    (void)uart_diag_write_string(
        "[ACCEL PIPE] seq=");

    (void)uart_diag_write_uint32(
        output->sequence);


    (void)uart_diag_write_string(
        " t_us=");

    (void)uart_diag_write_uint32(
        output->timestamp_us);


#if ACCEL_PIPELINE_DEBUG_RAW

    (void)uart_diag_write_string(
        " raw=");

    accel_debug_write_i16_triplet(
        output->raw_x,
        output->raw_y,
        output->raw_z);

#endif


#if ACCEL_PIPELINE_DEBUG_MEDIAN

    (void)uart_diag_write_string(
        " median=");

    accel_debug_write_i16_triplet(
        output->median_x,
        output->median_y,
        output->median_z);

#endif


#if ACCEL_PIPELINE_DEBUG_CALIBRATED_G

    (void)uart_diag_write_string(
        " calibrated_g=");

    accel_debug_write_float_triplet(
        output->calibrated_x_g,
        output->calibrated_y_g,
        output->calibrated_z_g);

#endif


#if ACCEL_PIPELINE_DEBUG_FILTERED_G

    (void)uart_diag_write_string(
        " filtered_g=");

    accel_debug_write_float_triplet(
        output->filtered_x_g,
        output->filtered_y_g,
        output->filtered_z_g);

#endif


#if ACCEL_PIPELINE_DEBUG_MS2

    (void)uart_diag_write_string(
        " filtered_ms2=");

    accel_debug_write_float_triplet(
        output->filtered_x_ms2,
        output->filtered_y_ms2,
        output->filtered_z_ms2);

#endif


    (void)uart_diag_write_string(
        " flags=");

    (void)uart_diag_write_hex32(
        output->flags);

    (void)uart_diag_write_string(
        "\r\n");
}

#endif


/*
 * ============================================================
 * GYRO UART HELPERS
 * ============================================================
 */

#if GYRO_PIPELINE_ENABLE

static void
gyro_uart_write_fixed3(
    float value)
{
    int32_t scaled;

    uint32_t magnitude;
    uint32_t fraction;


    if (value < 0.0f)
    {
        scaled =
            (int32_t)(
                (value * 1000.0f) -
                0.5f);
    }
    else
    {
        scaled =
            (int32_t)(
                (value * 1000.0f) +
                0.5f);
    }


    if (scaled < 0)
    {
        (void)uart_diag_write_char(
            '-');

        magnitude =
            (uint32_t)(-scaled);
    }
    else
    {
        magnitude =
            (uint32_t)scaled;
    }


    (void)uart_diag_write_uint32(
        magnitude / 1000UL);


    (void)uart_diag_write_char(
        '.');


    fraction =
        magnitude % 1000UL;


    if (fraction < 100UL)
    {
        (void)uart_diag_write_char(
            '0');
    }


    if (fraction < 10UL)
    {
        (void)uart_diag_write_char(
            '0');
    }


    (void)uart_diag_write_uint32(
        fraction);
}


static void
gyro_uart_write_float_triplet(
    float x,
    float y,
    float z)
{
    gyro_uart_write_fixed3(
        x);

    (void)uart_diag_write_char(
        ',');

    gyro_uart_write_fixed3(
        y);

    (void)uart_diag_write_char(
        ',');

    gyro_uart_write_fixed3(
        z);
}


static void
gyro_uart_report_calibration_state(
    const gyro_pipeline_output_t *output,
    uint32_t previous_flags)
{
    if ((output->flags &
         GYRO_BIAS_SETTLING) != 0UL)
    {
        if ((previous_flags &
             GYRO_BIAS_SETTLING) == 0UL)
        {
            if ((output->flags &
                 GYRO_BIAS_UNSTABLE) != 0UL)
            {
                (void)uart_diag_write_line(
                    "[GYRO CAL] Motion detected; restarting calibration");
            }


            (void)uart_diag_write_line(
                "[GYRO CAL] Settling for 5 seconds; keep stationary");
        }
    }
    else if ((output->flags &
              GYRO_BIAS_CALIBRATING) != 0UL)
    {
        if ((previous_flags &
             GYRO_BIAS_CALIBRATING) == 0UL)
        {
            (void)uart_diag_write_line(
                "[GYRO CAL] Collecting 5 seconds; keep stationary");
        }
    }
    else if (((output->flags &
               GYRO_BIAS_READY) != 0UL) &&
             ((previous_flags &
               GYRO_BIAS_READY) == 0UL))
    {
        if ((output->flags &
             GYRO_BIAS_FROM_STARTUP) != 0UL)
        {
            (void)uart_diag_write_string(
                "[GYRO CAL] Startup bias ready samples=");


            (void)uart_diag_write_uint32(
                output->calibration_sample_count);
        }
        else
        {
            (void)uart_diag_write_string(
                "[GYRO CAL] Using fixed 20-minute bias");
        }


        (void)uart_diag_write_string(
            " bias_counts=");


        gyro_uart_write_float_triplet(
            output->bias_x_counts,
            output->bias_y_counts,
            output->bias_z_counts);


        (void)uart_diag_write_string(
            "\r\n");
    }
}


#if GYRO_PIPELINE_UART_DEBUG

static void
gyro_uart_write_pipeline(
    const gyro_pipeline_output_t *output)
{
    (void)uart_diag_write_string(
        "[GYRO PIPE] seq=");

    (void)uart_diag_write_uint32(
        output->sequence);


    (void)uart_diag_write_string(
        " t_us=");

    (void)uart_diag_write_uint32(
        output->timestamp_us);


    (void)uart_diag_write_string(
        " dt_us=");

    (void)uart_diag_write_uint32(
        output->sample_interval_us);


    (void)uart_diag_write_string(
        " dps=");

    gyro_uart_write_float_triplet(
        output->x_dps,
        output->y_dps,
        output->z_dps);


    (void)uart_diag_write_string(
        " flags=");

    (void)uart_diag_write_hex32(
        output->flags);


    (void)uart_diag_write_string(
        "\r\n");
}

#endif

#endif


/*
 * ============================================================
 * INNER RATE UART
 * ============================================================
 */

#if INNER_RATE_CONTROL_UART_DEBUG

static void
inner_rate_uart_write_output(
    const rate_controller_output_t *output)
{
    (void)uart_diag_write_string(
        "[RATE PID] seq=");


    (void)uart_diag_write_uint32(
        output->sequence);


    (void)uart_diag_write_string(
        " out=");


    gyro_uart_write_float_triplet(
        output->roll.output,
        output->pitch.output,
        output->yaw.output);


    (void)uart_diag_write_string(
        " flags=");


    (void)uart_diag_write_hex32(
        output->flags);


    (void)uart_diag_write_string(
        "\r\n");
}

#endif


/*
 * ============================================================
 * MOTOR MIXER UART
 * ============================================================
 *
 * Completely independent from the existing controller UART
 * debug switches.
 *
 * This section is compiled only when:
 *
 *     MOTOR_MIXER_UART_DEBUG=1
 */

#if MOTOR_MIXER_UART_DEBUG

static void
motor_mixer_uart_write_quad(
    float m1,
    float m2,
    float m3,
    float m4)
{
    gyro_uart_write_fixed3(
        m1);


    (void)uart_diag_write_char(
        ',');


    gyro_uart_write_fixed3(
        m2);


    (void)uart_diag_write_char(
        ',');


    gyro_uart_write_fixed3(
        m3);


    (void)uart_diag_write_char(
        ',');


    gyro_uart_write_fixed3(
        m4);
}


static void
motor_mixer_uart_write_output(
    const motor_mixer_output_t *output)
{
    (void)uart_diag_write_string(
        "[MIXER] seq=");


    (void)uart_diag_write_uint32(
        output->sequence);


    (void)uart_diag_write_string(
        " dt_us=");


    (void)uart_diag_write_uint32(
        output->sample_interval_us);


    /*
     * Base/common collective input.
     */
    (void)uart_diag_write_string(
        " C=");


    gyro_uart_write_fixed3(
        output->collective_input);


    /*
     * Inner-rate-controller corrections:
     *
     *     roll,pitch,yaw
     */
    (void)uart_diag_write_string(
        " corr=");


    gyro_uart_write_float_triplet(
        output->roll_correction,
        output->pitch_correction,
        output->yaw_correction);


    /*
     * Requested M1..M4 before mixer desaturation.
     */
    (void)uart_diag_write_string(
        " raw=");


    motor_mixer_uart_write_quad(
        output->raw_m1,
        output->raw_m2,
        output->raw_m3,
        output->raw_m4);


    /*
     * Final normalized M1..M4.
     */
    (void)uart_diag_write_string(
        " out=");


    motor_mixer_uart_write_quad(
        output->m1,
        output->m2,
        output->m3,
        output->m4);


    /*
     * Desaturation diagnostics.
     */
    (void)uart_diag_write_string(
        " scale=");


    gyro_uart_write_fixed3(
        output->differential_scale);


    (void)uart_diag_write_string(
        " C_used=");


    gyro_uart_write_fixed3(
        output->collective_used);


    (void)uart_diag_write_string(
        " flags=");


    (void)uart_diag_write_hex32(
        output->flags);


    (void)uart_diag_write_string(
        "\r\n");
}

#endif


/*
 * ============================================================
 * OUTER ATTITUDE UART
 * ============================================================
 */

#if OUTER_ATTITUDE_CONTROL_UART_DEBUG

static void
outer_attitude_uart_write_pair(
    float first,
    float second)
{
    gyro_uart_write_fixed3(
        first);

    (void)uart_diag_write_char(
        ',');

    gyro_uart_write_fixed3(
        second);
}


static void
outer_attitude_uart_write_output(
    const attitude_controller_output_t *output)
{
    (void)uart_diag_write_string(
        "[ATT CTRL] seq=");


    (void)uart_diag_write_uint32(
        output->sequence);


    (void)uart_diag_write_string(
        " target_rad=");


    outer_attitude_uart_write_pair(
        output->desired_roll_rad,
        output->desired_pitch_rad);


    (void)uart_diag_write_string(
        " angle_rad=");


    outer_attitude_uart_write_pair(
        output->estimated_roll_rad,
        output->estimated_pitch_rad);


    (void)uart_diag_write_string(
        " error_rad=");


    outer_attitude_uart_write_pair(
        output->roll_error_rad,
        output->pitch_error_rad);


    (void)uart_diag_write_string(
        " rate_sp=");


    outer_attitude_uart_write_pair(
        output->desired_roll_rate_rad_s,
        output->desired_pitch_rate_rad_s);


    (void)uart_diag_write_string(
        " flags=");


    (void)uart_diag_write_hex32(
        output->flags);


    (void)uart_diag_write_string(
        "\r\n");
}

#endif


/*
 * ============================================================
 * ATTITUDE UART
 * ============================================================
 */

#if ATTITUDE_ESTIMATOR_UART_DEBUG

static void
attitude_uart_write_pipeline(
    const attitude_estimator_output_t *output)
{
    (void)uart_diag_write_string(
        "[ATT] seq=");


    (void)uart_diag_write_uint32(
        output->sequence);


    (void)uart_diag_write_string(
        " dt_us=");


    (void)uart_diag_write_uint32(
        output->sample_interval_us);


    (void)uart_diag_write_string(
        " angle_deg=");


    gyro_uart_write_float_triplet(
        output->roll_deg,
        output->pitch_deg,
        output->yaw_deg);


    (void)uart_diag_write_string(
        " amag_g=");


    gyro_uart_write_fixed3(
        output->accel_magnitude_g);


    (void)uart_diag_write_string(
        " flags=");


    (void)uart_diag_write_hex32(
        output->flags);


    (void)uart_diag_write_string(
        "\r\n");
}

#endif


/*
 * ============================================================
 * RAW ACQUISITION UART
 * ============================================================
 */

#if ACQ_UART_RAW_DEBUG

static void
write_signed(
    int16_t value)
{
    int32_t wide;


    wide =
        (int32_t)value;


    if (wide < 0)
    {
        (void)uart_diag_write_char(
            '-');

        wide =
            -wide;
    }


    (void)uart_diag_write_uint32(
        (uint32_t)wide);
}


static void
write_separator(void)
{
    (void)uart_diag_write_char(
        ',');
}


static void
write_raw_debug_line(
    const imu_raw_sample_t *sample)
{
    (void)uart_diag_write_string(
        "[RAW] seq=");


    (void)uart_diag_write_uint32(
        sample->sequence);


    (void)uart_diag_write_string(
        " t_us=");


    (void)uart_diag_write_uint32(
        sample->motion_timestamp_us);


    (void)uart_diag_write_string(
        " A=");


    write_signed(
        sample->motion.accel_x);

    write_separator();

    write_signed(
        sample->motion.accel_y);

    write_separator();

    write_signed(
        sample->motion.accel_z);


    (void)uart_diag_write_string(
        " G=");


    write_signed(
        sample->motion.gyro_x);

    write_separator();

    write_signed(
        sample->motion.gyro_y);

    write_separator();

    write_signed(
        sample->motion.gyro_z);


    if (mpu9250_has_magnetometer())
    {
        (void)uart_diag_write_string(
            " M=");


        write_signed(
            sample->mag.x);

        write_separator();

        write_signed(
            sample->mag.y);

        write_separator();

        write_signed(
            sample->mag.z);
    }


    (void)uart_diag_write_string(
        " flags=");


    (void)uart_diag_write_hex32(
        sample->flags);


    (void)uart_diag_write_string(
        "\r\n");
}


static void
write_i2c_health_debug_line(
    const i2c1_diag_t *diag)
{
    (void)uart_diag_write_string(
        "[I2C] tx=");

    (void)uart_diag_write_uint32(
        diag->transactions);


    (void)uart_diag_write_string(
        " ok=");

    (void)uart_diag_write_uint32(
        diag->successful_transactions);


    (void)uart_diag_write_string(
        " retry=");

    (void)uart_diag_write_uint32(
        diag->retries);


    (void)uart_diag_write_string(
        " recover=");

    (void)uart_diag_write_uint32(
        diag->recoveries);


    (void)uart_diag_write_string(
        " rec_fail=");

    (void)uart_diag_write_uint32(
        diag->recovery_failures);


    (void)uart_diag_write_string(
        " last_err=");

    (void)uart_diag_write_uint32(
        (uint32_t)
        diag->last_error_status);


    (void)uart_diag_write_string(
        " stage=");

    (void)uart_diag_write_uint32(
        (uint32_t)
        diag->last_error_stage);


    (void)uart_diag_write_string(
        " err_attempt=");

    (void)uart_diag_write_uint32(
        diag->last_error_attempt);


    (void)uart_diag_write_string(
        " SR1=");

    (void)uart_diag_write_hex32(
        diag->sr1);


    (void)uart_diag_write_string(
        " SR2=");

    (void)uart_diag_write_hex32(
        diag->sr2);


    (void)uart_diag_write_string(
        " GPIOB_IDR=");

    (void)uart_diag_write_hex32(
        diag->gpio_idr);


    (void)uart_diag_write_string(
        "\r\n");
}


static void
write_stale_debug_line(
    const imu_raw_sample_t *sample)
{
    (void)uart_diag_write_string(
        "[RAW STALE] seq=");


    (void)uart_diag_write_uint32(
        sample->sequence);


    (void)uart_diag_write_string(
        " acq_status=");


    (void)uart_diag_write_uint32(
        (uint32_t)
        g_imu_acquisition_status);


    (void)uart_diag_write_string(
        " driver_status=");


    (void)uart_diag_write_uint32(
        (uint32_t)
        imu_acquisition_last_driver_status());


    (void)uart_diag_write_string(
        "\r\n");
}

#endif


/*
 * ============================================================
 * HALT HELPER
 * ============================================================
 */

static __attribute__((noreturn))
void
stop_with_message(
    const char *message)
{
    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            message);
    }


    for (;;)
    {
        __asm volatile (
            "nop");
    }
}


/*
 * ============================================================
 * ACCEL CAPTURE
 * ============================================================
 */

#if ACCEL_CAPTURE_TEST

static void
accel_capture_write_signed(
    int16_t value)
{
    int32_t wide;


    wide =
        (int32_t)value;


    if (wide < 0)
    {
        (void)uart_diag_write_char(
            '-');

        wide =
            -wide;
    }


    (void)uart_diag_write_uint32(
        (uint32_t)wide);
}


static void
accel_capture_send_values(
    uint32_t block_number)
{
    uint32_t i;


    (void)uart_diag_write_string(
        "# ACCEL_CAPTURE_BEGIN block=");


    (void)uart_diag_write_uint32(
        block_number);


    (void)uart_diag_write_string(
        " samples=");


    (void)uart_diag_write_uint32(
        accel_capture_count);


    (void)uart_diag_write_string(
        "\r\n");


    (void)uart_diag_write_line(
        "block,index,ax_raw,ay_raw,az_raw");


    for (i = 0UL;
         i < accel_capture_count;
         i++)
    {
        (void)uart_diag_write_uint32(
            block_number);


        (void)uart_diag_write_char(
            ',');


        (void)uart_diag_write_uint32(
            i);


        (void)uart_diag_write_char(
            ',');


        accel_capture_write_signed(
            accel_capture_buffer[i].x);


        (void)uart_diag_write_char(
            ',');


        accel_capture_write_signed(
            accel_capture_buffer[i].y);


        (void)uart_diag_write_char(
            ',');


        accel_capture_write_signed(
            accel_capture_buffer[i].z);


        (void)uart_diag_write_string(
            "\r\n");
    }


    (void)uart_diag_write_string(
        "# ACCEL_CAPTURE_END block=");


    (void)uart_diag_write_uint32(
        block_number);


    (void)uart_diag_write_string(
        "\r\n");
}


static __attribute__((noreturn))
void
run_accel_capture_test(void)
{
    imu_raw_sample_t sample;

    uint32_t start_time_us;
    uint32_t elapsed_us;
    uint32_t block_number;

    bool started;


    for (block_number = 1UL;
         block_number <=
         ACCEL_CAPTURE_TOTAL_BLOCKS;
         block_number++)
    {
        accel_capture_count =
            0UL;


        start_time_us =
            0UL;

        elapsed_us =
            0UL;

        started =
            false;


        for (;;)
        {
            g_imu_acquisition_status =
                imu_acquisition_process();


            if (g_imu_acquisition_status !=
                IMU_ACQUISITION_OK)
            {
                continue;
            }


            if (!imu_acquisition_get_latest(
                    &sample))
            {
                continue;
            }


            if ((sample.flags &
                 IMU_RAW_MOTION_VALID) == 0UL)
            {
                continue;
            }


            if (!started)
            {
                start_time_us =
                    sample.motion_timestamp_us;

                started =
                    true;
            }


            elapsed_us =
                (uint32_t)(
                    sample.motion_timestamp_us -
                    start_time_us);


            if (elapsed_us >=
                ACCEL_CAPTURE_DURATION_US)
            {
                break;
            }


            if (accel_capture_count >=
                ACCEL_CAPTURE_MAX_SAMPLES)
            {
                break;
            }


            accel_capture_buffer[
                accel_capture_count].x =
                    sample.motion.accel_x;


            accel_capture_buffer[
                accel_capture_count].y =
                    sample.motion.accel_y;


            accel_capture_buffer[
                accel_capture_count].z =
                    sample.motion.accel_z;


            accel_capture_count++;
        }


        if (g_fc_uart_status ==
            UART_DIAG_OK)
        {
            (void)uart_diag_write_string(
                "[ACCEL CAPTURE] block=");


            (void)uart_diag_write_uint32(
                block_number);


            (void)uart_diag_write_string(
                "/");


            (void)uart_diag_write_uint32(
                ACCEL_CAPTURE_TOTAL_BLOCKS);


            (void)uart_diag_write_string(
                " samples=");


            (void)uart_diag_write_uint32(
                accel_capture_count);


            (void)uart_diag_write_string(
                "\r\n");


            accel_capture_send_values(
                block_number);
        }
    }


    stop_with_message(
        "[HALT] 20-minute accelerometer capture finished");
}

#endif


/*
 * ============================================================
 * GYRO CAPTURE
 * ============================================================
 */

#if GYRO_CAPTURE_TEST

static void
gyro_capture_write_signed(
    int16_t value)
{
    int32_t wide;


    wide =
        (int32_t)value;


    if (wide < 0)
    {
        (void)uart_diag_write_char(
            '-');

        wide =
            -wide;
    }


    (void)uart_diag_write_uint32(
        (uint32_t)wide);
}


static void
gyro_capture_send_values(
    uint32_t block_number,
    uint32_t block_start_us)
{
    uint32_t i;


    (void)uart_diag_write_string(
        "# GYRO_CAPTURE_BEGIN block=");


    (void)uart_diag_write_uint32(
        block_number);


    (void)uart_diag_write_string(
        " samples=");


    (void)uart_diag_write_uint32(
        gyro_capture_count);


    (void)uart_diag_write_string(
        " start_us=");


    (void)uart_diag_write_uint32(
        block_start_us);


    (void)uart_diag_write_string(
        "\r\n");


    (void)uart_diag_write_line(
        "block,index,gx_raw,gy_raw,gz_raw");


    for (i = 0UL;
         i < gyro_capture_count;
         i++)
    {
        (void)uart_diag_write_uint32(
            block_number);


        (void)uart_diag_write_char(
            ',');


        (void)uart_diag_write_uint32(
            i);


        (void)uart_diag_write_char(
            ',');


        gyro_capture_write_signed(
            gyro_capture_buffer[i].x);


        (void)uart_diag_write_char(
            ',');


        gyro_capture_write_signed(
            gyro_capture_buffer[i].y);


        (void)uart_diag_write_char(
            ',');


        gyro_capture_write_signed(
            gyro_capture_buffer[i].z);


        (void)uart_diag_write_string(
            "\r\n");
    }


    (void)uart_diag_write_string(
        "# GYRO_CAPTURE_END block=");


    (void)uart_diag_write_uint32(
        block_number);


    (void)uart_diag_write_string(
        "\r\n");
}


static __attribute__((noreturn))
void
run_gyro_capture_test(void)
{
    imu_raw_sample_t sample;

    uint32_t start_time_us;
    uint32_t elapsed_us;
    uint32_t block_number;

    bool started;


    for (block_number = 1UL;
         block_number <=
         GYRO_CAPTURE_TOTAL_BLOCKS;
         block_number++)
    {
        gyro_capture_count =
            0UL;


        start_time_us =
            0UL;

        elapsed_us =
            0UL;

        started =
            false;


        for (;;)
        {
            g_imu_acquisition_status =
                imu_acquisition_process();


            if (g_imu_acquisition_status !=
                IMU_ACQUISITION_OK)
            {
                continue;
            }


            if (!imu_acquisition_get_latest(
                    &sample))
            {
                continue;
            }


            if ((sample.flags &
                 IMU_RAW_MOTION_VALID) == 0UL)
            {
                continue;
            }


            if (!started)
            {
                start_time_us =
                    sample.motion_timestamp_us;

                started =
                    true;
            }


            elapsed_us =
                (uint32_t)(
                    sample.motion_timestamp_us -
                    start_time_us);


            if (elapsed_us >=
                GYRO_CAPTURE_DURATION_US)
            {
                break;
            }


            if (gyro_capture_count >=
                GYRO_CAPTURE_MAX_SAMPLES)
            {
                break;
            }


            gyro_capture_buffer[
                gyro_capture_count].x =
                    sample.motion.gyro_x;


            gyro_capture_buffer[
                gyro_capture_count].y =
                    sample.motion.gyro_y;


            gyro_capture_buffer[
                gyro_capture_count].z =
                    sample.motion.gyro_z;


            gyro_capture_count++;
        }


        if (g_fc_uart_status ==
            UART_DIAG_OK)
        {
            (void)uart_diag_write_string(
                "[GYRO CAPTURE] block=");


            (void)uart_diag_write_uint32(
                block_number);


            (void)uart_diag_write_string(
                "/");


            (void)uart_diag_write_uint32(
                GYRO_CAPTURE_TOTAL_BLOCKS);


            (void)uart_diag_write_string(
                " samples=");


            (void)uart_diag_write_uint32(
                gyro_capture_count);


            (void)uart_diag_write_string(
                "\r\n");


            gyro_capture_send_values(
                block_number,
                start_time_us);
        }
    }


    stop_with_message(
        "[HALT] 20-minute gyroscope capture finished");
}

#endif


/*
 * ============================================================
 * DIRECT MPU WHO_AM_I TEST
 * ============================================================
 */

#if MPU_WHO_AM_I_TEST

static void
mpu_who_am_i_test(
    uint32_t pclk1_hz)
{
    uint8_t id_68 =
        0U;

    uint8_t id_69 =
        0U;


    i2c1_status_t
        init_status;

    i2c1_status_t
        status_68;

    i2c1_status_t
        status_69;


    init_status =
        i2c1_init(
            pclk1_hz,
            I2C_BUS_HZ);


    (void)uart_diag_write_string(
        "[WHO_AM_I TEST] I2C init status=");


    (void)uart_diag_write_uint32(
        (uint32_t)init_status);


    (void)uart_diag_write_string(
        "\r\n");


    if (init_status !=
        I2C1_OK)
    {
        (void)uart_diag_write_line(
            "[WHO_AM_I TEST] I2C initialization failed");

        return;
    }


    status_68 =
        i2c1_read_registers(
            0x68U,
            0x75U,
            &id_68,
            1U);


    status_69 =
        i2c1_read_registers(
            0x69U,
            0x75U,
            &id_69,
            1U);


    (void)uart_diag_write_string(
        "[WHO_AM_I TEST] address=0x68 status=");


    (void)uart_diag_write_uint32(
        (uint32_t)status_68);


    (void)uart_diag_write_string(
        " id_decimal=");


    (void)uart_diag_write_uint32(
        (uint32_t)id_68);


    (void)uart_diag_write_string(
        "\r\n");


    (void)uart_diag_write_string(
        "[WHO_AM_I TEST] address=0x69 status=");


    (void)uart_diag_write_uint32(
        (uint32_t)status_69);


    (void)uart_diag_write_string(
        " id_decimal=");


    (void)uart_diag_write_uint32(
        (uint32_t)id_69);


    (void)uart_diag_write_string(
        "\r\n");


    if ((status_68 == I2C1_OK) ||
        (status_69 == I2C1_OK))
    {
        uint8_t detected_id;


        detected_id =
            (status_68 == I2C1_OK) ?
            id_68 :
            id_69;


        switch (detected_id)
        {
            case 0x68U:

                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Possible MPU6050");

                break;


            case 0x70U:

                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Possible MPU6500");

                break;


            case 0x71U:

                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] MPU9250 identity detected");

                break;


            case 0x73U:

                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Possible MPU9255");

                break;


            default:

                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Unknown identity value");

                break;
        }
    }
    else
    {
        (void)uart_diag_write_line(
            "[WHO_AM_I TEST] No device responded at 0x68 or 0x69");
    }
}

#endif


/*
 * ============================================================
 * MAIN
 * ============================================================
 */

int
main(void)
{
    uint32_t
        pclk1_hz;

    uint32_t
        tim2_hz;


#if ACQ_UART_RAW_DEBUG

    uint32_t
        previous_debug_ms;

    uint32_t
        previous_debug_sequence;

    uint32_t
        previous_i2c_error_count;

    uint32_t
        previous_i2c_recoveries;

    i2c1_diag_t
        runtime_i2c_diag;

#endif


    imu_raw_sample_t
        sample;


#if ACCEL_PIPELINE_ENABLE

    accel_pipeline_input_t
        accel_input;

    accel_pipeline_output_t
        accel_output;

#endif


#if GYRO_PIPELINE_ENABLE

    gyro_pipeline_input_t
        gyro_input;

    gyro_pipeline_output_t
        gyro_output;

    uint32_t
        gyro_previous_calibration_flags;

#endif


#if GYRO_PIPELINE_UART_DEBUG

    bool
        gyro_output_ready;

    uint32_t
        gyro_previous_uart_timestamp_us;

    bool
        gyro_uart_timestamp_valid;

#endif


#if ATTITUDE_ESTIMATOR_ENABLE

    imu_body_sample_t
        body_sample;

    attitude_estimator_output_t
        attitude_output;

    bool
        attitude_output_ready;

    bool
        attitude_initialization_reported;


#if ATTITUDE_ESTIMATOR_UART_DEBUG

    uint32_t
        attitude_previous_uart_timestamp_us;

    bool
        attitude_uart_timestamp_valid;

#endif

#endif


/*
 * ------------------------------------------------------------
 * INNER RATE CONTROLLER LOCAL STATE
 * ------------------------------------------------------------
 */

#if INNER_RATE_CONTROL_ENABLE

    rate_controller_config_t
        inner_rate_config;

    rate_controller_input_t
        inner_rate_input;

    rate_controller_output_t
        inner_rate_output;


#if INNER_RATE_CONTROL_UART_DEBUG || \
    MOTOR_MIXER_ENABLE

    /*
     * Mixer also needs the success/failure state of the current
     * rate-controller update.
     */
    bool
        inner_rate_output_ready;

#endif


#if INNER_RATE_CONTROL_UART_DEBUG

    uint32_t
        inner_rate_previous_uart_timestamp_us;

    bool
        inner_rate_uart_timestamp_valid;

#endif

#endif


/*
 * ------------------------------------------------------------
 * MOTOR MIXER LOCAL STATE
 * ------------------------------------------------------------
 */

#if MOTOR_MIXER_ENABLE

    motor_mixer_input_t
        motor_mixer_input;

    motor_mixer_output_t
        motor_mixer_output;


#if MOTOR_MIXER_UART_DEBUG

    /*
     * UART timing state is completely separate from mixer
     * calculation state.
     */
    uint32_t
        motor_mixer_previous_uart_timestamp_us;

    bool
        motor_mixer_uart_timestamp_valid;

#endif

#endif


/*
 * ------------------------------------------------------------
 * OUTER ATTITUDE CONTROLLER LOCAL STATE
 * ------------------------------------------------------------
 */

#if OUTER_ATTITUDE_CONTROL_ENABLE

    attitude_controller_config_t
        outer_attitude_config;

    attitude_controller_input_t
        outer_attitude_input;

    attitude_controller_output_t
        outer_attitude_output;


    bool
        outer_attitude_output_ready;


    float
        held_roll_rate_setpoint_rad_s;

    float
        held_pitch_rate_setpoint_rad_s;


#if OUTER_ATTITUDE_CONTROL_UART_DEBUG

    uint32_t
        outer_attitude_previous_uart_timestamp_us;

    bool
        outer_attitude_uart_timestamp_valid;

#endif

#endif


#if MEASURE_PIPELINE_TIMES

    uint32_t
        pipeline_start_us;

    uint32_t
        pipeline_elapsed_us;

#endif


#if MEASURE_RAW_TO_ATTITUDE_TIMES

    uint32_t
        raw_to_attitude_start_us;

    uint32_t
        raw_to_attitude_finish_us;

#endif


#if ACCEL_PIPELINE_DEBUG_ACTIVE

    uint32_t
        accel_debug_sample_count;

#endif


    /*
     * ========================================================
     * PLATFORM INITIALIZATION
     * ========================================================
     */

    fault_record_clear();


    g_fc_clock_status =
        system_clock_init();


    if (g_fc_clock_status !=
        SYSTEM_CLOCK_OK)
    {
        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    g_fc_time_status =
        system_time_init(
            system_clock_get_hclk_hz());


    if (g_fc_time_status !=
        SYSTEM_TIME_OK)
    {
        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    pclk1_hz =
        system_clock_get_pclk1_hz();


    tim2_hz =
        (pclk1_hz ==
         system_clock_get_hclk_hz()) ?
        pclk1_hz :
        (pclk1_hz * 2UL);


    g_fc_micros_status =
        micros_init(
            tim2_hz);


    if (g_fc_micros_status !=
        MICROS_OK)
    {
        for (;;)
        {
            __asm volatile (
                "nop");
        }
    }


    g_fc_uart_status =
        uart_diag_init(
            system_clock_get_pclk2_hz(),
            UART_BAUD_RATE);


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[BOOT] Flight controller starting");


#if IMU_MODEL == IMU_MODEL_MPU6500

        (void)uart_diag_write_line(
            "[ACQ] MPU6500 accel/gyro acquisition");

#else

        (void)uart_diag_write_line(
            "[ACQ] MPU9250 accel/gyro/mag acquisition");

#endif
    }


#if MPU_WHO_AM_I_TEST

    (void)uart_diag_write_line(
        "[TEST MODE] Direct MPU WHO_AM_I test enabled");


    mpu_who_am_i_test(
        pclk1_hz);


    stop_with_message(
        "[HALT] WHO_AM_I test completed");

#endif


    /*
     * ========================================================
     * IMU ACQUISITION INITIALIZATION
     * ========================================================
     */

    g_imu_acquisition_status =
        imu_acquisition_init(
            pclk1_hz);


    if (g_imu_acquisition_status !=
        IMU_ACQUISITION_OK)
    {
        if (g_fc_uart_status ==
            UART_DIAG_OK)
        {
            (void)uart_diag_write_string(
                "[ERROR] Acquisition init status=");


            (void)uart_diag_write_uint32(
                (uint32_t)
                g_imu_acquisition_status);


            (void)uart_diag_write_string(
                " driver_status=");


            (void)uart_diag_write_uint32(
                (uint32_t)
                imu_acquisition_last_driver_status());


#if IMU_INIT_DEBUG

            init_diag =
                mpu9250_get_init_diag();


            (void)uart_diag_write_string(
                "\r\n[IMU DEBUG] stage=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                init_diag.stage);


            (void)uart_diag_write_string(
                " phase=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                init_diag.phase);


            (void)uart_diag_write_string(
                " i2c_status=");

            (void)uart_diag_write_uint32(
                init_diag.i2c_status);


            (void)uart_diag_write_string(
                " attempt=");

            (void)uart_diag_write_uint32(
                init_diag.attempt);


            (void)uart_diag_write_string(
                "\r\n[IMU DEBUG] device=");

            (void)uart_diag_write_hex32(
                (uint32_t)
                init_diag.device_address);


            (void)uart_diag_write_string(
                " reg=");

            (void)uart_diag_write_hex32(
                (uint32_t)
                init_diag.register_address);


            (void)uart_diag_write_string(
                "\r\n[IMU DEBUG] written=");

            (void)uart_diag_write_hex32(
                (uint32_t)
                init_diag.written_value);


            (void)uart_diag_write_string(
                " readback=");

            (void)uart_diag_write_hex32(
                (uint32_t)
                init_diag.readback_value);


            (void)uart_diag_write_string(
                " mask=");

            (void)uart_diag_write_hex32(
                (uint32_t)
                init_diag.mask);


            i2c1_get_diag(
                &i2c_diag);


            (void)uart_diag_write_string(
                "\r\n[I2C DEBUG] stage=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                i2c_diag.stage);


            (void)uart_diag_write_string(
                " status=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                i2c_diag.last_status);


            (void)uart_diag_write_string(
                " attempt=");

            (void)uart_diag_write_uint32(
                i2c_diag.attempt);


            (void)uart_diag_write_string(
                "\r\n[I2C DEBUG] CR1=");

            (void)uart_diag_write_hex32(
                i2c_diag.cr1);


            (void)uart_diag_write_string(
                " SR1=");

            (void)uart_diag_write_hex32(
                i2c_diag.sr1);


            (void)uart_diag_write_string(
                " SR2=");

            (void)uart_diag_write_hex32(
                i2c_diag.sr2);


            (void)uart_diag_write_string(
                " DR=");

            (void)uart_diag_write_hex32(
                i2c_diag.dr);


            (void)uart_diag_write_string(
                "\r\n[I2C DEBUG] last_error_status=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                i2c_diag.last_error_status);


            (void)uart_diag_write_string(
                " last_error_stage=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                i2c_diag.last_error_stage);


            (void)uart_diag_write_string(
                " err_attempt=");

            (void)uart_diag_write_uint32(
                i2c_diag.last_error_attempt);


            (void)uart_diag_write_string(
                " last_recovery=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                i2c_diag.last_recovery_status);


            (void)uart_diag_write_string(
                "\r\n[I2C DEBUG] GPIOB_IDR=");

            (void)uart_diag_write_hex32(
                i2c_diag.gpio_idr);


            (void)uart_diag_write_string(
                " bus_hz=");

            (void)uart_diag_write_uint32(
                i2c_diag.bus_hz);


            (void)uart_diag_write_string(
                " dev=");

            (void)uart_diag_write_hex32(
                (uint32_t)
                i2c_diag.device_address);


            (void)uart_diag_write_string(
                " reg=");

            (void)uart_diag_write_hex32(
                (uint32_t)
                i2c_diag.register_address);


            (void)uart_diag_write_string(
                " len=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                i2c_diag.length);


            (void)uart_diag_write_string(
                "\r\n[I2C DEBUG] retries=");

            (void)uart_diag_write_uint32(
                i2c_diag.retries);


            (void)uart_diag_write_string(
                " recoveries=");

            (void)uart_diag_write_uint32(
                i2c_diag.recoveries);


            (void)uart_diag_write_string(
                " rec_fail=");

            (void)uart_diag_write_uint32(
                i2c_diag.recovery_failures);


            (void)uart_diag_write_string(
                " timeouts=");

            (void)uart_diag_write_uint32(
                i2c_diag.timeouts);


            (void)uart_diag_write_string(
                " nacks=");

            (void)uart_diag_write_uint32(
                i2c_diag.nacks);


            (void)uart_diag_write_string(
                " busy=");

            (void)uart_diag_write_uint32(
                i2c_diag.bus_busy_errors);


            (void)uart_diag_write_string(
                " berr=");

            (void)uart_diag_write_uint32(
                i2c_diag.bus_errors);


            (void)uart_diag_write_string(
                " arlo=");

            (void)uart_diag_write_uint32(
                i2c_diag.arbitration_losses);


            (void)uart_diag_write_string(
                " ovr=");

            (void)uart_diag_write_uint32(
                i2c_diag.overruns);


            (void)uart_diag_write_string(
                " line_low=");

            (void)uart_diag_write_uint32(
                i2c_diag.line_stuck_errors);

#endif


            (void)uart_diag_write_string(
                "\r\n");
        }


        stop_with_message(
            "[HALT] Check power, wiring, address and pull-ups");
    }


    /*
     * ========================================================
     * ACCEL PIPELINE INITIALIZATION
     * ========================================================
     */

#if ACCEL_PIPELINE_ENABLE

    accel_pipeline_init();


    accel_output =
        (accel_pipeline_output_t){0};


    g_latest_accel_pipeline_output =
        (accel_pipeline_output_t){0};


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[ACCEL PIPE] median3 -> calibration -> 20Hz LPF -> m/s2");
    }

#endif


    /*
     * ========================================================
     * GYRO PIPELINE INITIALIZATION
     * ========================================================
     */

#if GYRO_PIPELINE_ENABLE

    gyro_pipeline_init(
        GYRO_INITIAL_BIAS_CAL_ENABLE != 0);


    gyro_output =
        (gyro_pipeline_output_t){0};


    g_latest_gyro_pipeline_output =
        (gyro_pipeline_output_t){0};


    gyro_previous_calibration_flags =
        0UL;


#if GYRO_PIPELINE_UART_DEBUG

    gyro_previous_uart_timestamp_us =
        0UL;


    gyro_uart_timestamp_valid =
        false;

#endif


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[GYRO PIPE] validate -> bias -> dps -> rad/s");
    }

#endif


    /*
     * ========================================================
     * ATTITUDE ESTIMATOR INITIALIZATION
     * ========================================================
     */

#if ATTITUDE_ESTIMATOR_ENABLE

    attitude_estimator_init(
        ATTITUDE_ESTIMATOR_DEFAULT_KP,
        (uint32_t)
        ATTITUDE_EULER_RATE_HZ);


    g_latest_attitude_estimator_output =
        (attitude_estimator_output_t){0};


    attitude_output_ready =
        false;


    attitude_initialization_reported =
        false;


#if ATTITUDE_ESTIMATOR_UART_DEBUG

    attitude_previous_uart_timestamp_us =
        0UL;


    attitude_uart_timestamp_valid =
        false;

#endif


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[ATT] Waiting for gyro bias and valid gravity");
    }

#endif


    /*
     * ========================================================
     * INNER RATE CONTROLLER INITIALIZATION
     * ========================================================
     */

#if INNER_RATE_CONTROL_ENABLE

    inner_rate_load_sign_test_config(
        &inner_rate_config);


    if (!rate_controller_init(
            &inner_rate_config))
    {
        stop_with_message(
            "[HALT] Inner rate controller configuration invalid");
    }


    inner_rate_input =
        (rate_controller_input_t){0};


    inner_rate_output =
        (rate_controller_output_t){0};


    g_latest_rate_controller_output =
        (rate_controller_output_t){0};


#if INNER_RATE_CONTROL_UART_DEBUG || \
    MOTOR_MIXER_ENABLE

    inner_rate_output_ready =
        false;

#endif


#if INNER_RATE_CONTROL_UART_DEBUG

    inner_rate_previous_uart_timestamp_us =
        0UL;


    inner_rate_uart_timestamp_valid =
        false;

#endif


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[RATE] Inner roll/pitch/yaw PID enabled");


#if OUTER_ATTITUDE_CONTROL_ENABLE

        (void)uart_diag_write_line(
            "[RATE] Roll/pitch rate setpoints supplied by outer attitude controller");


        (void)uart_diag_write_line(
            "[RATE] Yaw rate setpoint remains direct 0 rad/s");

#else

        (void)uart_diag_write_line(
            "[RATE] Outer loop bypassed; setpoint_rad_s=0,0,0");

#endif


        (void)uart_diag_write_line(
            "[RATE] Sign-test gains Kp=1 Ki=0 Kd=0");


#if MOTOR_MIXER_ENABLE

        (void)uart_diag_write_line(
            "[RATE] Outputs feed diagnostic motor mixer");


        (void)uart_diag_write_line(
            "[RATE] Mixer output is NOT transmitted to motors");

#else

        (void)uart_diag_write_line(
            "[RATE] Outputs diagnostic only; no mixer or motors");

#endif
    }

#endif


    /*
     * ========================================================
     * MOTOR MIXER INITIALIZATION
     * ========================================================
     *
     * motor_mixer.c is stateless, so there is no mixer_init().
     *
     * Only local and published diagnostic structures need
     * clearing.
     */

#if MOTOR_MIXER_ENABLE

    motor_mixer_input =
        (motor_mixer_input_t){0};


    motor_mixer_output =
        (motor_mixer_output_t){0};


    g_latest_motor_mixer_output =
        (motor_mixer_output_t){0};


#if MOTOR_MIXER_UART_DEBUG

    motor_mixer_previous_uart_timestamp_us =
        0UL;


    motor_mixer_uart_timestamp_valid =
        false;

#endif


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[MIXER] X-frame mixer enabled");


        (void)uart_diag_write_line(
            "[MIXER] Test collective C=0.500; NOT hover tuned");


        (void)uart_diag_write_line(
            "[MIXER] M1..M4 normalized outputs available through GDB");


        (void)uart_diag_write_line(
            "[MIXER] No motor-node UART/PWM in this phase");


#if MOTOR_MIXER_UART_DEBUG

        (void)uart_diag_write_line(
            "[MIXER] Independent UART debug enabled at 5 Hz");

#endif
    }

#endif


    /*
     * ========================================================
     * OUTER ATTITUDE CONTROLLER INITIALIZATION
     * ========================================================
     */

#if OUTER_ATTITUDE_CONTROL_ENABLE

    outer_attitude_load_level_test_config(
        &outer_attitude_config);


    if (!attitude_controller_init(
            &outer_attitude_config))
    {
        stop_with_message(
            "[HALT] Outer attitude controller configuration invalid");
    }


    outer_attitude_input =
        (attitude_controller_input_t){0};


    outer_attitude_output =
        (attitude_controller_output_t){0};


    g_latest_attitude_controller_output =
        (attitude_controller_output_t){0};


    outer_attitude_output_ready =
        false;


    held_roll_rate_setpoint_rad_s =
        0.0f;


    held_pitch_rate_setpoint_rad_s =
        0.0f;


#if OUTER_ATTITUDE_CONTROL_UART_DEBUG

    outer_attitude_previous_uart_timestamp_us =
        0UL;


    outer_attitude_uart_timestamp_valid =
        false;

#endif


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[ATT CTRL] Outer roll/pitch P controller enabled");


        (void)uart_diag_write_line(
            "[ATT CTRL] Level-hold target_rad=0,0");


        (void)uart_diag_write_line(
            "[ATT CTRL] Diagnostic gain=1/s rate_cap=1 rad/s");


        (void)uart_diag_write_line(
            "[ATT CTRL] Fresh Euler output updates held inner rate setpoints");


        (void)uart_diag_write_line(
            "[ATT CTRL] Yaw heading loop not implemented; yaw-rate target remains 0");
    }

#endif


    /*
     * ========================================================
     * PROFILER INITIALIZATION
     * ========================================================
     */

#if MEASURE_PIPELINE_TIMES

    g_pipeline_time_last_us =
        0UL;


    g_pipeline_time_min_us =
        0UL;


    g_pipeline_time_max_us =
        0UL;


    g_pipeline_time_total_us =
        0ULL;


    g_pipeline_time_sample_count =
        0UL;

#endif


#if MEASURE_RAW_TO_ATTITUDE_TIMES

    raw_to_attitude_timing_init();


    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[TIMING] Raw MPU to attitude timing enabled; inspect with GDB");
    }

#endif


#if ACCEL_PIPELINE_DEBUG_ACTIVE

    accel_debug_sample_count =
        0UL;

#endif


    /*
     * ========================================================
     * SENSOR IDENTIFICATION
     * ========================================================
     */

    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {

#if IMU_MODEL == IMU_MODEL_MPU6500

        (void)uart_diag_write_string(
            "[ID] MPU6500=");

#else

        (void)uart_diag_write_string(
            "[ID] MPU9250=");

#endif


        (void)uart_diag_write_hex32(
            (uint32_t)
            mpu9250_who_am_i());


        if (mpu9250_has_magnetometer())
        {
            (void)uart_diag_write_string(
                " AK8963=");


            (void)uart_diag_write_hex32(
                (uint32_t)
                ak8963_who_am_i());
        }


        (void)uart_diag_write_string(
            "\r\n");
    }


#if ACQ_UART_RAW_DEBUG

    previous_debug_ms =
        millis();


    previous_debug_sequence =
        0UL;


    previous_i2c_error_count =
        0UL;


    previous_i2c_recoveries =
        0UL;

#endif


    /*
     * ========================================================
     * TEST MODE BOOT MESSAGES
     * ========================================================
     */

    if (g_fc_uart_status ==
        UART_DIAG_OK)
    {

#if ACCEL_CAPTURE_TEST

        (void)uart_diag_write_line(
            "[TEST] Repeated 5-second accelerometer capture");


        (void)uart_diag_write_line(
            "[TEST] Target = 240 blocks = 20 minutes captured data");

#endif


#if GYRO_CAPTURE_TEST

        (void)uart_diag_write_line(
            "[TEST] Repeated 5-second gyroscope capture");


        (void)uart_diag_write_line(
            "[TEST] Target = 240 blocks = 20 minutes captured data");

#endif


        (void)uart_diag_write_line(
            "[READY] Starting PA0 DATA_RDY");
    }


    /*
     * ========================================================
     * START IMU DATA READY
     * ========================================================
     */

    g_imu_acquisition_status =
        imu_acquisition_start();


    if (g_imu_acquisition_status !=
        IMU_ACQUISITION_OK)
    {
        stop_with_message(
            "[HALT] Could not start IMU DATA_RDY");
    }


#if ACCEL_CAPTURE_TEST

    run_accel_capture_test();

#endif


#if GYRO_CAPTURE_TEST

    run_gyro_capture_test();

#endif


    /*
     * ========================================================
     * MAIN FLIGHT-CONTROLLER LOOP
     * ========================================================
     */

    for (;;)
    {
        /*
         * ----------------------------------------------------
         * IMU ACQUISITION
         * ----------------------------------------------------
         */

        g_imu_acquisition_status =
            imu_acquisition_process();


        if ((g_imu_acquisition_status ==
             IMU_ACQUISITION_OK) &&
            imu_acquisition_get_latest(
                &sample))
        {

#if MEASURE_RAW_TO_ATTITUDE_TIMES

            raw_to_attitude_timing_note_raw_sample(
                sample.sequence,
                sample.motion_timestamp_us);


            raw_to_attitude_start_us =
                micros();

#endif


            g_latest_raw_imu_sample =
                sample;


            /*
             * =================================================
             * ACCELEROMETER PIPELINE
             * =================================================
             */

#if ACCEL_PIPELINE_ENABLE

            if ((sample.flags &
                 IMU_RAW_MOTION_VALID) != 0UL)
            {
                accel_input.sequence =
                    sample.sequence;


                accel_input.timestamp_us =
                    sample.motion_timestamp_us;


                accel_input.raw_x =
                    sample.motion.accel_x;


                accel_input.raw_y =
                    sample.motion.accel_y;


                accel_input.raw_z =
                    sample.motion.accel_z;


#if MEASURE_PIPELINE_TIMES

                pipeline_start_us =
                    micros();

#endif


                if (accel_pipeline_process(
                        &accel_input,
                        &accel_output))
                {

#if MEASURE_PIPELINE_TIMES

                    pipeline_elapsed_us =
                        micros() -
                        pipeline_start_us;


                    g_pipeline_time_last_us =
                        pipeline_elapsed_us;


                    if ((g_pipeline_time_sample_count ==
                         0UL) ||
                        (pipeline_elapsed_us <
                         g_pipeline_time_min_us))
                    {
                        g_pipeline_time_min_us =
                            pipeline_elapsed_us;
                    }


                    if (pipeline_elapsed_us >
                        g_pipeline_time_max_us)
                    {
                        g_pipeline_time_max_us =
                            pipeline_elapsed_us;
                    }


                    g_pipeline_time_total_us +=
                        (uint64_t)
                        pipeline_elapsed_us;


                    g_pipeline_time_sample_count++;

#endif


                    g_latest_accel_pipeline_output =
                        accel_output;


#if ACCEL_PIPELINE_DEBUG_ACTIVE

                    accel_debug_sample_count++;


                    if (accel_debug_sample_count >=
                        (uint32_t)
                        ACCEL_PIPELINE_DEBUG_DECIMATION)
                    {
                        accel_debug_sample_count =
                            0UL;


                        if (g_fc_uart_status ==
                            UART_DIAG_OK)
                        {
                            accel_debug_write_pipeline(
                                &accel_output);
                        }
                    }

#endif
                }
            }

#endif


            /*
             * =================================================
             * GYRO PIPELINE
             * =================================================
             */

#if GYRO_PIPELINE_ENABLE

            if ((sample.flags &
                 IMU_RAW_MOTION_VALID) != 0UL)
            {
                gyro_input.sequence =
                    sample.sequence;


                gyro_input.timestamp_us =
                    sample.motion_timestamp_us;


                gyro_input.raw_x =
                    sample.motion.gyro_x;


                gyro_input.raw_y =
                    sample.motion.gyro_y;


                gyro_input.raw_z =
                    sample.motion.gyro_z;


#if GYRO_PIPELINE_UART_DEBUG

                gyro_output_ready =
                    gyro_pipeline_process(
                        &gyro_input,
                        &gyro_output);

#else

                (void)gyro_pipeline_process(
                    &gyro_input,
                    &gyro_output);

#endif


                g_latest_gyro_pipeline_output =
                    gyro_output;


                if (g_fc_uart_status ==
                    UART_DIAG_OK)
                {
                    uint32_t
                        calibration_flags;


                    calibration_flags =
                        gyro_output.flags &
                        (GYRO_BIAS_READY |
                         GYRO_BIAS_SETTLING |
                         GYRO_BIAS_CALIBRATING);


                    if (calibration_flags !=
                        0UL)
                    {
                        gyro_uart_report_calibration_state(
                            &gyro_output,
                            gyro_previous_calibration_flags);


                        gyro_previous_calibration_flags =
                            gyro_output.flags;
                    }


#if GYRO_PIPELINE_UART_DEBUG

                    if (gyro_output_ready &&
                        ((!gyro_uart_timestamp_valid) ||
                         ((uint32_t)(
                              gyro_output.timestamp_us -
                              gyro_previous_uart_timestamp_us) >=
                          GYRO_PIPELINE_UART_PERIOD_US)))
                    {
                        gyro_uart_write_pipeline(
                            &gyro_output);


                        gyro_previous_uart_timestamp_us =
                            gyro_output.timestamp_us;


                        gyro_uart_timestamp_valid =
                            true;
                    }

#endif
                }
            }

#endif


            /*
             * =================================================
             * BODY FRAME + CASCADED CONTROL + ATTITUDE
             * =================================================
             */

#if ATTITUDE_ESTIMATOR_ENABLE

            if ((sample.flags &
                 IMU_RAW_MOTION_VALID) != 0UL)
            {
                if (imu_body_frame_from_pipeline(
                        &accel_output,
                        &gyro_output,
                        &body_sample))
                {
                    /*
                     * ==========================================
                     * INNER ANGULAR-RATE CONTROL
                     * ==========================================
                     */

#if INNER_RATE_CONTROL_ENABLE

                    inner_rate_input.sequence =
                        body_sample.gyro_sequence;


                    inner_rate_input.timestamp_us =
                        body_sample.timestamp_us;


                    inner_rate_input.sample_interval_us =
                        body_sample.sample_interval_us;


                    inner_rate_input.measurements_valid =
                        inner_rate_source_is_valid(
                            &body_sample);


                    inner_rate_input.dt_s =
                        body_sample.dt_s;


#if OUTER_ATTITUDE_CONTROL_ENABLE

                    inner_rate_input
                        .desired_roll_rate_rad_s =
                        held_roll_rate_setpoint_rad_s;


                    inner_rate_input
                        .desired_pitch_rate_rad_s =
                        held_pitch_rate_setpoint_rad_s;

#else

                    inner_rate_input
                        .desired_roll_rate_rad_s =
                        0.0f;


                    inner_rate_input
                        .desired_pitch_rate_rad_s =
                        0.0f;

#endif


                    /*
                     * No yaw-heading outer loop yet.
                     */
                    inner_rate_input
                        .desired_yaw_rate_rad_s =
                        0.0f;


                    inner_rate_input
                        .measured_roll_rate_rad_s =
                        body_sample
                            .angular_rate_x_rad_s;


                    inner_rate_input
                        .measured_pitch_rate_rad_s =
                        body_sample
                            .angular_rate_y_rad_s;


                    inner_rate_input
                        .measured_yaw_rate_rad_s =
                        body_sample
                            .angular_rate_z_rad_s;


#if INNER_RATE_CONTROL_UART_DEBUG || \
    MOTOR_MIXER_ENABLE

                    inner_rate_output_ready =
                        rate_controller_update(
                            &inner_rate_input,
                            &inner_rate_output);

#else

                    (void)rate_controller_update(
                        &inner_rate_input,
                        &inner_rate_output);

#endif


                    g_latest_rate_controller_output =
                        inner_rate_output;


                    /*
                     * ==========================================
                     * MOTOR MIXER
                     * ==========================================
                     *
                     * Locked X-frame allocation:
                     *
                     * M1 = C - R + P + Y
                     *
                     * M2 = C + R + P - Y
                     *
                     * M3 = C + R - P + Y
                     *
                     * M4 = C - R - P - Y
                     *
                     * M1 = front-left  / CCW
                     * M2 = front-right / CW
                     * M3 = rear-right  / CCW
                     * M4 = rear-left   / CW
                     *
                     * The output remains diagnostic only.
                     */

#if MOTOR_MIXER_ENABLE

                    motor_mixer_input.sequence =
                        inner_rate_output.sequence;


                    motor_mixer_input.timestamp_us =
                        inner_rate_output.timestamp_us;


                    motor_mixer_input.sample_interval_us =
                        inner_rate_output.sample_interval_us;


                    /*
                     * The mixer may produce a valid motor
                     * allocation only when the current inner
                     * controller update itself is valid.
                     */
                    motor_mixer_input.control_valid =
                        inner_rate_output_ready &&
                        ((inner_rate_output.flags &
                          RATE_CONTROL_VALID) != 0UL);


                    /*
                     * Temporary diagnostic collective.
                     *
                     * Future:
                     *
                     * this value will come from the appropriate
                     * flight/movement/throttle setpoint layer.
                     */
                    motor_mixer_input.collective =
                        MOTOR_MIXER_TEST_COLLECTIVE;


                    motor_mixer_input.roll_correction =
                        inner_rate_output
                            .roll.output;


                    motor_mixer_input.pitch_correction =
                        inner_rate_output
                            .pitch.output;


                    motor_mixer_input.yaw_correction =
                        inner_rate_output
                            .yaw.output;


                    /*
                     * Keep the existing mixer execution exactly
                     * independent from UART debugging.
                     */
                    (void)motor_mixer_update(
                        &motor_mixer_input,
                        &motor_mixer_output);


                    g_latest_motor_mixer_output =
                        motor_mixer_output;

#endif

#endif


                    /*
                     * ==========================================
                     * ATTITUDE ESTIMATOR
                     * ==========================================
                     */

                    attitude_output_ready =
                        attitude_estimator_update(
                            &body_sample,
                            &attitude_output);


                    g_latest_attitude_estimator_output =
                        attitude_output;


                    /*
                     * ==========================================
                     * OUTER ROLL/PITCH ATTITUDE CONTROL
                     * ==========================================
                     */

#if OUTER_ATTITUDE_CONTROL_ENABLE

                    if (attitude_output_ready &&
                        ((attitude_output.flags &
                          (ATTITUDE_VALID |
                           ATTITUDE_INITIALIZED)) ==
                         (ATTITUDE_VALID |
                          ATTITUDE_INITIALIZED)))
                    {
                        /*
                         * Run outer controller only when a fresh
                         * Euler angle calculation exists.
                         */
                        if ((attitude_output.flags &
                             ATTITUDE_EULER_UPDATED) != 0UL)
                        {
                            outer_attitude_input.sequence =
                                attitude_output.sequence;


                            outer_attitude_input.timestamp_us =
                                attitude_output.timestamp_us;


                            outer_attitude_input.attitude_valid =
                                true;


                            /*
                             * Current level-hold test.
                             *
                             * Future movement manager replaces
                             * these fixed zero targets.
                             */
                            outer_attitude_input
                                .desired_roll_rad =
                                0.0f;


                            outer_attitude_input
                                .desired_pitch_rad =
                                0.0f;


                            outer_attitude_input
                                .estimated_roll_rad =
                                attitude_output.roll_rad;


                            outer_attitude_input
                                .estimated_pitch_rad =
                                attitude_output.pitch_rad;


                            outer_attitude_output_ready =
                                attitude_controller_update(
                                    &outer_attitude_input,
                                    &outer_attitude_output);


                            g_latest_attitude_controller_output =
                                outer_attitude_output;


                            if (outer_attitude_output_ready)
                            {
                                held_roll_rate_setpoint_rad_s =
                                    outer_attitude_output
                                        .desired_roll_rate_rad_s;


                                held_pitch_rate_setpoint_rad_s =
                                    outer_attitude_output
                                        .desired_pitch_rate_rad_s;


#if OUTER_ATTITUDE_CONTROL_UART_DEBUG

                                if ((g_fc_uart_status ==
                                     UART_DIAG_OK) &&
                                    ((!outer_attitude_uart_timestamp_valid) ||
                                     ((uint32_t)(
                                          outer_attitude_output.timestamp_us -
                                          outer_attitude_previous_uart_timestamp_us) >=
                                      OUTER_ATTITUDE_CONTROL_UART_PERIOD_US)))
                                {
                                    outer_attitude_uart_write_output(
                                        &outer_attitude_output);


                                    outer_attitude_previous_uart_timestamp_us =
                                        outer_attitude_output.timestamp_us;


                                    outer_attitude_uart_timestamp_valid =
                                        true;
                                }

#endif
                            }
                            else
                            {
                                /*
                                 * Do not retain an old non-zero
                                 * outer-loop request after an
                                 * invalid update.
                                 */
                                held_roll_rate_setpoint_rad_s =
                                    0.0f;


                                held_pitch_rate_setpoint_rad_s =
                                    0.0f;
                            }
                        }
                    }
                    else
                    {
                        held_roll_rate_setpoint_rad_s =
                            0.0f;


                        held_pitch_rate_setpoint_rad_s =
                            0.0f;


                        attitude_controller_reset();


                        outer_attitude_output =
                            (attitude_controller_output_t){0};


                        outer_attitude_output.sequence =
                            attitude_output.sequence;


                        outer_attitude_output.timestamp_us =
                            attitude_output.timestamp_us;


                        outer_attitude_output.flags =
                            ATTITUDE_CONTROL_INPUT_INVALID;


                        g_latest_attitude_controller_output =
                            outer_attitude_output;
                    }

#endif


                    /*
                     * ==========================================
                     * EXISTING RAW -> ATTITUDE TIMING
                     * ==========================================
                     */

#if MEASURE_RAW_TO_ATTITUDE_TIMES

                    raw_to_attitude_finish_us =
                        micros();


                    raw_to_attitude_timing_record(
                        sample.sequence,
                        sample.motion_timestamp_us,
                        raw_to_attitude_start_us,
                        raw_to_attitude_finish_us,
                        attitude_output
                            .sample_interval_us,
                        gyro_output.flags,
                        attitude_output.flags,
                        attitude_output_ready);

#endif


                    /*
                     * ==========================================
                     * INNER RATE UART
                     * ==========================================
                     */

#if INNER_RATE_CONTROL_UART_DEBUG

                    if (inner_rate_output_ready &&
                        (g_fc_uart_status ==
                         UART_DIAG_OK) &&
                        ((!inner_rate_uart_timestamp_valid) ||
                         ((uint32_t)(
                              inner_rate_output.timestamp_us -
                              inner_rate_previous_uart_timestamp_us) >=
                          INNER_RATE_CONTROL_UART_PERIOD_US)))
                    {
                        inner_rate_uart_write_output(
                            &inner_rate_output);


                        inner_rate_previous_uart_timestamp_us =
                            inner_rate_output.timestamp_us;


                        inner_rate_uart_timestamp_valid =
                            true;
                    }

#endif


                    /*
                     * ==========================================
                     * MOTOR MIXER UART
                     * ==========================================
                     *
                     * This is only an observer of the mixer
                     * output that has already been calculated.
                     *
                     * The mixer update above is unchanged.
                     */
#if MOTOR_MIXER_UART_DEBUG

                    if (((motor_mixer_output.flags &
                          MOTOR_MIXER_VALID) != 0UL) &&
                        (g_fc_uart_status ==
                         UART_DIAG_OK) &&
                        ((!motor_mixer_uart_timestamp_valid) ||
                         ((uint32_t)(
                              motor_mixer_output.timestamp_us -
                              motor_mixer_previous_uart_timestamp_us) >=
                          MOTOR_MIXER_UART_PERIOD_US)))
                    {
                        motor_mixer_uart_write_output(
                            &motor_mixer_output);


                        motor_mixer_previous_uart_timestamp_us =
                            motor_mixer_output.timestamp_us;


                        motor_mixer_uart_timestamp_valid =
                            true;
                    }

#endif


                    /*
                     * ==========================================
                     * ATTITUDE INITIALIZATION + UART
                     * ==========================================
                     */

                    if (g_fc_uart_status ==
                        UART_DIAG_OK)
                    {
                        if (attitude_output_ready &&
                            !attitude_initialization_reported &&
                            ((attitude_output.flags &
                              ATTITUDE_INITIALIZED) != 0UL))
                        {
                            (void)uart_diag_write_string(
                                "[ATT] Initialized angle_deg=");


                            gyro_uart_write_float_triplet(
                                attitude_output.roll_deg,
                                attitude_output.pitch_deg,
                                attitude_output.yaw_deg);


                            (void)uart_diag_write_line(
                                " yaw is relative only");


                            attitude_initialization_reported =
                                true;
                        }


#if ATTITUDE_ESTIMATOR_UART_DEBUG

                        if (attitude_output_ready &&
                            ((!attitude_uart_timestamp_valid) ||
                             ((uint32_t)(
                                  attitude_output.timestamp_us -
                                  attitude_previous_uart_timestamp_us) >=
                              ATTITUDE_ESTIMATOR_UART_PERIOD_US)))
                        {
                            attitude_uart_write_pipeline(
                                &attitude_output);


                            attitude_previous_uart_timestamp_us =
                                attitude_output.timestamp_us;


                            attitude_uart_timestamp_valid =
                                true;
                        }

#endif
                    }
                }

#if MEASURE_RAW_TO_ATTITUDE_TIMES

                else
                {
                    g_raw_to_attitude_timing
                        .body_frame_failure_count++;
                }

#endif
            }

#endif
        }


        /*
         * ====================================================
         * ACQUISITION STATISTICS
         * ====================================================
         */

        g_imu_acquisition_stats =
            imu_acquisition_get_stats();


        /*
         * ====================================================
         * RAW/I2C DEBUG
         * ====================================================
         */

#if ACQ_UART_RAW_DEBUG

        if ((uint32_t)(
                millis() -
                previous_debug_ms) >=
            DEBUG_PERIOD_MS)
        {
            previous_debug_ms =
                millis();


            if (g_fc_uart_status ==
                UART_DIAG_OK)
            {
                uint32_t
                    i2c_error_count;


                i2c1_get_diag(
                    &runtime_i2c_diag);


                i2c_error_count =
                    runtime_i2c_diag.timeouts +
                    runtime_i2c_diag.nacks +
                    runtime_i2c_diag.bus_busy_errors +
                    runtime_i2c_diag.bus_errors +
                    runtime_i2c_diag.arbitration_losses +
                    runtime_i2c_diag.overruns +
                    runtime_i2c_diag.line_stuck_errors;


                /*
                 * Only print I2C health when the state changes.
                 */
                if ((i2c_error_count !=
                     previous_i2c_error_count) ||
                    (runtime_i2c_diag.recoveries !=
                     previous_i2c_recoveries))
                {
                    write_i2c_health_debug_line(
                        &runtime_i2c_diag);


                    previous_i2c_error_count =
                        i2c_error_count;


                    previous_i2c_recoveries =
                        runtime_i2c_diag.recoveries;
                }


                if (imu_acquisition_get_latest(
                        &sample))
                {
                    if (sample.sequence !=
                        previous_debug_sequence)
                    {
                        write_raw_debug_line(
                            &sample);


                        previous_debug_sequence =
                            sample.sequence;
                    }
                    else
                    {
                        write_stale_debug_line(
                            &sample);
                    }
                }
            }
        }

#endif


        __asm volatile (
            "nop");
    }
}