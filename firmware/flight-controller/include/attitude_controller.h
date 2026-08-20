#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * ------------------------------------------------------------
 * OUTER ATTITUDE CONTROLLER FLAGS
 * ------------------------------------------------------------
 */

#define ATTITUDE_CONTROL_VALID                 (1UL << 0)
#define ATTITUDE_CONTROL_NOT_INITIALIZED       (1UL << 1)
#define ATTITUDE_CONTROL_INPUT_INVALID         (1UL << 2)
#define ATTITUDE_CONTROL_DUPLICATE_SEQUENCE    (1UL << 3)
#define ATTITUDE_CONTROL_ROLL_RATE_LIMITED     (1UL << 4)
#define ATTITUDE_CONTROL_PITCH_RATE_LIMITED    (1UL << 5)
#define ATTITUDE_CONTROL_NUMERIC_ERROR         (1UL << 6)


/*
 * ------------------------------------------------------------
 * CONFIGURATION
 * ------------------------------------------------------------
 *
 * This first outer-loop implementation is proportional-only:
 *
 *     angle_error =
 *         desired_angle - estimated_angle
 *
 *     desired_rate =
 *         attitude_gain * angle_error
 *
 * The result is then limited before being passed to the
 * existing inner angular-rate controller.
 */

typedef struct
{
    /*
     * Angle-to-rate proportional gains.
     *
     * Units:
     *
     *     (rad/s) / rad
     *
     * which is equivalent to approximately 1/s.
     */
    float roll_gain_per_s;
    float pitch_gain_per_s;

    /*
     * Absolute desired angular-rate limits.
     *
     * The real flight values must later be determined
     * experimentally.
     */
    float max_roll_rate_rad_s;
    float max_pitch_rate_rad_s;

} attitude_controller_config_t;


/*
 * ------------------------------------------------------------
 * INPUT
 * ------------------------------------------------------------
 */

typedef struct
{
    /*
     * Sequence/timestamp of the Euler attitude sample that
     * produced this outer-loop update.
     */
    uint32_t sequence;
    uint32_t timestamp_us;

    /*
     * true only when the attitude estimator has a valid,
     * initialized roll/pitch attitude.
     */
    bool attitude_valid;

    /*
     * Requested attitude from the higher-level setpoint layer.
     *
     * During the first level-hold test:
     *
     *     desired_roll_rad  = 0
     *     desired_pitch_rad = 0
     */
    float desired_roll_rad;
    float desired_pitch_rad;

    /*
     * Current estimated attitude.
     *
     * These come from:
     *
     *     attitude_estimator_output_t.roll_rad
     *     attitude_estimator_output_t.pitch_rad
     */
    float estimated_roll_rad;
    float estimated_pitch_rad;

} attitude_controller_input_t;


/*
 * ------------------------------------------------------------
 * OUTPUT
 * ------------------------------------------------------------
 */

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_us;
    uint32_t flags;

    /*
     * Input values copied for GDB/UART diagnostics.
     */
    float desired_roll_rad;
    float desired_pitch_rad;

    float estimated_roll_rad;
    float estimated_pitch_rad;

    /*
     * Outer-loop angle errors.
     */
    float roll_error_rad;
    float pitch_error_rad;

    /*
     * P-controller outputs before rate limiting.
     */
    float raw_roll_rate_rad_s;
    float raw_pitch_rate_rad_s;

    /*
     * Final bounded rate setpoints.
     *
     * These go directly into:
     *
     *     rate_controller_input_t
     *
     * as the desired roll/pitch rates.
     */
    float desired_roll_rate_rad_s;
    float desired_pitch_rate_rad_s;

} attitude_controller_output_t;


/*
 * Validate and store the outer-controller configuration.
 */
bool attitude_controller_init(
    const attitude_controller_config_t *config);


/*
 * Clear sequence history while preserving the configured gains
 * and rate limits.
 *
 * This controller has no I or D state because the first outer
 * loop is proportional-only.
 */
void attitude_controller_reset(void);


/*
 * Process one fresh roll/pitch attitude sample.
 *
 * This function:
 *
 *     1. validates the input;
 *     2. calculates roll/pitch angle errors;
 *     3. converts angle errors to desired angular rates;
 *     4. applies rate limits;
 *     5. returns bounded rate setpoints for rate_controller.c.
 *
 * dt is intentionally not required because this controller is
 * proportional-only.
 *
 * Yaw heading control is intentionally not implemented here.
 */
bool attitude_controller_update(
    const attitude_controller_input_t *input,
    attitude_controller_output_t *output);


#endif /* ATTITUDE_CONTROLLER_H */