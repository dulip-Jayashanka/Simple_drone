#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Normalized flight-controller motor-command domain.
 *
 * These are abstract motor commands, not ESC pulse widths.
 *
 * The future motor-node command/PWM layer owns:
 *
 *     arm/disarm enforcement
 *     command timeout
 *     final actuator limits
 *     ESC pulse generation
 */
#define MOTOR_MIXER_COMMAND_MIN 0.0f
#define MOTOR_MIXER_COMMAND_MAX 1.0f


/*
 * ============================================================
 * MIXER RESULT FLAGS
 * ============================================================
 */

#define MOTOR_MIXER_VALID                    (1UL << 0)

#define MOTOR_MIXER_SOURCE_INVALID           (1UL << 1)

#define MOTOR_MIXER_INPUT_INVALID            (1UL << 2)

#define MOTOR_MIXER_COLLECTIVE_CLAMPED       (1UL << 3)

#define MOTOR_MIXER_DIFFERENTIAL_SCALED      (1UL << 4)

#define MOTOR_MIXER_COLLECTIVE_SHIFTED       (1UL << 5)

#define MOTOR_MIXER_OUTPUT_CLAMPED            (1UL << 6)

#define MOTOR_MIXER_NUMERIC_ERROR             (1UL << 7)


/*
 * ============================================================
 * MIXER INPUT
 * ============================================================
 */

typedef struct
{
    uint32_t sequence;

    uint32_t timestamp_us;

    uint32_t sample_interval_us;


    /*
     * True only when the upstream inner angular-rate
     * controller produced a valid correction set.
     */
    bool control_valid;


    /*
     * Common/base thrust command.
     *
     * This is NOT a motor-to-motor difference.
     *
     * Conceptually:
     *
     *     collective = 0.50
     *
     * with zero roll/pitch/yaw correction gives:
     *
     *     M1 = 0.50
     *     M2 = 0.50
     *     M3 = 0.50
     *     M4 = 0.50
     *
     * The final hover value is NOT known yet and must
     * later be determined experimentally.
     */
    float collective;


    /*
     * Differential aircraft-control corrections.
     *
     * These come from:
     *
     *     rate_controller_output_t.roll.output
     *     rate_controller_output_t.pitch.output
     *     rate_controller_output_t.yaw.output
     */
    float roll_correction;

    float pitch_correction;

    float yaw_correction;

} motor_mixer_input_t;


/*
 * ============================================================
 * MIXER OUTPUT
 * ============================================================
 */

typedef struct
{
    uint32_t sequence;

    uint32_t timestamp_us;

    uint32_t sample_interval_us;

    uint32_t flags;


    /*
     * Original mixer inputs copied for GDB diagnostics.
     */
    float collective_input;

    float roll_correction;

    float pitch_correction;

    float yaw_correction;


    /*
     * Raw requested motor values before desaturation.
     *
     * These values are intentionally retained even when
     * outside the normalized [0, 1] range.
     */
    float raw_m1;

    float raw_m2;

    float raw_m3;

    float raw_m4;


    /*
     * Desaturation diagnostics.
     *
     * 1.0 means differential corrections were not scaled.
     *
     * A value below 1.0 means the requested roll/pitch/yaw
     * differential span exceeded the complete available
     * motor-command range.
     */
    float differential_scale;


    /*
     * Actual collective component after:
     *
     *     input collective limiting
     *     and/or
     *     common collective shifting
     *
     * during desaturation.
     */
    float collective_used;


    /*
     * Final normalized motor commands.
     *
     * These remain flight-controller values.
     *
     * They are NOT PWM microseconds.
     */
    float m1;

    float m2;

    float m3;

    float m4;

} motor_mixer_output_t;


/*
 * ============================================================
 * LOCKED SIMPLE_DRONE MOTOR GEOMETRY AND SIGN CONVENTION
 * ============================================================
 *
 * Viewed FROM ABOVE:
 *
 *
 *                       FRONT / +X
 *
 *                           ^
 *
 *
 *                M1                    M2
 *            FRONT-LEFT            FRONT-RIGHT
 *                CCW                   CW
 *
 *
 *
 *                M4                    M3
 *             REAR-LEFT             REAR-RIGHT
 *                CW                    CCW
 *
 *
 * Finalized controller-side positive rotations:
 *
 *     +roll
 *         right side goes DOWN
 *
 *     +pitch
 *         nose/front goes UP
 *
 *     +yaw
 *         nose turns RIGHT
 *
 *
 * Corresponding positive correction effects:
 *
 *     +R
 *         M1/M4 increase, M2/M3 decrease
 *
 *     +P
 *         M1/M2 increase, M3/M4 decrease
 *
 *     +Y
 *         M1/M3 increase, M2/M4 decrease
 *
 *
 * Therefore the locked mixer equations are:
 *
 *     M1 = C + R + P + Y
 *
 *     M2 = C - R + P - Y
 *
 *     M3 = C - R - P + Y
 *
 *     M4 = C + R - P - Y
 *
 *
 * where:
 *
 *     C = collective
 *
 *     R = roll correction
 *
 *     P = pitch correction
 *
 *     Y = yaw correction
 */


/*
 * ============================================================
 * MIXER UPDATE
 * ============================================================
 *
 * This mixer is intentionally STATELESS.
 *
 * It does not contain:
 *
 *     PID state
 *     integrators
 *     derivative state
 *     sensor state
 *     dt calculations
 *     UART
 *     PWM
 *
 *
 * Desaturation policy:
 *
 * 1. Validate all inputs.
 *
 * 2. Calculate the locked X-frame differential allocation.
 *
 * 3. Preserve the original raw M1..M4 request for diagnostics.
 *
 * 4. Limit collective to the normalized [0, 1] domain.
 *
 * 5. Determine the complete differential motor span.
 *
 *    If that span is larger than the complete available
 *    motor-command range, scale all differential corrections
 *    together.
 *
 *    This preserves their relative roll/pitch/yaw allocation.
 *
 * 6. If the differential span itself fits, but collective puts
 *    the result above or below the available range, shift the
 *    common collective component.
 *
 *    A common shift preserves motor-to-motor differences.
 *
 * 7. Apply a final clamp only as a last numerical/safety guard.
 *
 *
 * Returns:
 *
 *     true
 *         valid bounded M1..M4 produced
 *
 *     false
 *         no valid motor command produced
 */
bool motor_mixer_update(
    const motor_mixer_input_t *input,
    motor_mixer_output_t *output);


#endif /* MOTOR_MIXER_H */