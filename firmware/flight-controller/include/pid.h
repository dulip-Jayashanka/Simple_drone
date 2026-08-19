#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Broad timing guardrails for one PID update.
 *
 * The current MPU6500 pipeline normally gives approximately 0.002 s.
 * These broad limits reject clearly invalid controller timing while still
 * allowing moderate scheduling variation during development.
 */
#define PID_DT_MIN_S 0.0005f
#define PID_DT_MAX_S 0.0200f

/* Per-update result and diagnostic flags. */
#define PID_OUTPUT_VALID                   (1UL << 0)
#define PID_OUTPUT_DERIVATIVE_PRIMED       (1UL << 1)
#define PID_OUTPUT_INTEGRAL_CLAMPED        (1UL << 2)
#define PID_OUTPUT_INTEGRAL_HELD           (1UL << 3)
#define PID_OUTPUT_SATURATED_HIGH          (1UL << 4)
#define PID_OUTPUT_SATURATED_LOW           (1UL << 5)
#define PID_OUTPUT_CONFIG_INVALID          (1UL << 6)
#define PID_OUTPUT_INPUT_INVALID           (1UL << 7)
#define PID_OUTPUT_DT_INVALID              (1UL << 8)
#define PID_OUTPUT_NUMERIC_ERROR           (1UL << 9)

typedef struct
{
    float kp;
    float ki;
    float kd;

    /*
     * The stored integral value is already the complete I-term
     * contribution. Therefore:
     *
     * integral = integral + Ki * error * dt
     */
    float integral_min;
    float integral_max;

    /* Final abstract PID correction bounds. */
    float output_min;
    float output_max;

    /*
     * First-order low-pass cutoff applied to:
     *
     *     d(measurement) / dt
     *
     * A value of 0.0f selects an unfiltered derivative.
     */
    float derivative_cutoff_hz;
} pid_config_t;

typedef struct
{
    pid_config_t config;

    float integral;

    float previous_measurement;
    float filtered_measurement_derivative;

    bool config_valid;
    bool derivative_initialized;
} pid_controller_t;

typedef struct
{
    uint32_t flags;

    float setpoint;
    float measurement;
    float dt_s;
    float error;

    float p_term;
    float i_term;
    float d_term;

    float raw_measurement_derivative;
    float filtered_measurement_derivative;

    float unsaturated_output;
    float output;
} pid_output_t;

/*
 * Validate and copy the configuration, then clear all PID dynamic state.
 *
 * Returns true when the configuration is valid.
 */
bool pid_init(
    pid_controller_t *controller,
    const pid_config_t *config);

/*
 * Clear integral and derivative history while retaining the configuration.
 *
 * This function will later be used during arm/disarm or flight-state
 * transitions.
 */
void pid_reset(pid_controller_t *controller);

/*
 * Clear only derivative history while preserving the integral.
 *
 * This is useful after a missed sensor sequence because calculating a
 * derivative across a missing sample could produce an artificial spike.
 */
void pid_reset_derivative(pid_controller_t *controller);

/*
 * Execute one discrete PID update.
 *
 *     error = setpoint - measurement
 *
 *     P = Kp * error
 *
 *     I = previous_I + Ki * error * dt
 *
 *     D = -Kd * filtered(d(measurement) / dt)
 *
 *     output = clamp(P + I + D)
 *
 * The D term uses derivative-on-measurement. This avoids a derivative kick
 * when the requested rate changes suddenly.
 *
 * Conditional integration prevents the I term from driving an already
 * saturated output farther into saturation.
 *
 * The output structure is filled for both accepted and rejected updates.
 */
bool pid_update(
    pid_controller_t *controller,
    float setpoint,
    float measurement,
    float dt_s,
    pid_output_t *output);

#endif /* PID_H */