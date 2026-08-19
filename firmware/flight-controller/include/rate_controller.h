#ifndef RATE_CONTROLLER_H
#define RATE_CONTROLLER_H

#include "pid.h"

#include <stdbool.h>
#include <stdint.h>

/* Three-axis controller result and diagnostic flags. */
#define RATE_CONTROL_VALID                 (1UL << 0)
#define RATE_CONTROL_NOT_INITIALIZED       (1UL << 1)
#define RATE_CONTROL_INPUT_INVALID         (1UL << 2)
#define RATE_CONTROL_DT_INVALID            (1UL << 3)
#define RATE_CONTROL_DUPLICATE_SEQUENCE    (1UL << 4)
#define RATE_CONTROL_SEQUENCE_GAP          (1UL << 5)
#define RATE_CONTROL_PID_FAILURE           (1UL << 6)
#define RATE_CONTROL_ROLL_SATURATED        (1UL << 7)
#define RATE_CONTROL_PITCH_SATURATED       (1UL << 8)
#define RATE_CONTROL_YAW_SATURATED         (1UL << 9)

typedef struct
{
    pid_config_t roll;
    pid_config_t pitch;
    pid_config_t yaw;
} rate_controller_config_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t sample_interval_us;

    /*
     * Set true only when the processed gyro measurement, bias and dt are
     * ready for control.
     */
    bool measurements_valid;

    float dt_s;

    /*
     * Desired body angular rates.
     *
     * During this inner-loop-only phase, all three are set directly to zero
     * so the future outer attitude controller is bypassed.
     */
    float desired_roll_rate_rad_s;
    float desired_pitch_rate_rad_s;
    float desired_yaw_rate_rad_s;

    /*
     * Measured body rates:
     *
     * roll  = body X angular rate
     * pitch = body Y angular rate
     * yaw   = body Z angular rate
     */
    float measured_roll_rate_rad_s;
    float measured_pitch_rate_rad_s;
    float measured_yaw_rate_rad_s;
} rate_controller_input_t;

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t sample_interval_us;
    uint32_t flags;

    float dt_s;

    float desired_roll_rate_rad_s;
    float desired_pitch_rate_rad_s;
    float desired_yaw_rate_rad_s;

    float measured_roll_rate_rad_s;
    float measured_pitch_rate_rad_s;
    float measured_yaw_rate_rad_s;

    /*
     * Each axis result contains:
     *
     * error
     * P term
     * I term
     * D term
     * unsaturated output
     * final output
     * diagnostic flags
     */
    pid_output_t roll;
    pid_output_t pitch;
    pid_output_t yaw;
} rate_controller_output_t;

/*
 * Validate all three PID configurations and clear all controller state.
 */
bool rate_controller_init(
    const rate_controller_config_t *config);

/*
 * Clear integral, derivative and sequence history for all three axes.
 */
void rate_controller_reset(void);

/*
 * Run roll, pitch and yaw rate PIDs in one coherent update.
 *
 * No PID state is committed unless every axis update succeeds.
 */
bool rate_controller_update(
    const rate_controller_input_t *input,
    rate_controller_output_t *output);

#endif /* RATE_CONTROLLER_H */