#include "pid.h"

#include <stdbool.h>
#include <stdint.h>

#define PID_TWO_PI 6.28318530717958647692f

typedef union
{
    float as_float;
    uint32_t as_uint32;
} pid_float_bits_t;

static bool value_is_finite(float value)
{
    pid_float_bits_t bits;

    bits.as_float = value;

    /*
     * IEEE-754 exponent 0xFF represents infinity or NaN.
     *
     * This avoids requiring libm isfinite() in the bare-metal firmware.
     */
    return
        (bits.as_uint32 & 0x7F800000UL) !=
        0x7F800000UL;
}

static float clamp_float(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static bool config_is_valid(const pid_config_t *config)
{
    if (config == (const pid_config_t *)0)
    {
        return false;
    }

    if (!value_is_finite(config->kp) ||
        !value_is_finite(config->ki) ||
        !value_is_finite(config->kd) ||
        !value_is_finite(config->integral_min) ||
        !value_is_finite(config->integral_max) ||
        !value_is_finite(config->output_min) ||
        !value_is_finite(config->output_max) ||
        !value_is_finite(config->derivative_cutoff_hz))
    {
        return false;
    }

    /*
     * Keep gain signs positive.
     *
     * D damping receives its minus sign from:
     *
     *     D = -Kd * d(measurement)/dt
     */
    if ((config->kp < 0.0f) ||
        (config->ki < 0.0f) ||
        (config->kd < 0.0f) ||
        (config->derivative_cutoff_hz < 0.0f))
    {
        return false;
    }

    if (config->integral_min > config->integral_max)
    {
        return false;
    }

    if (!(config->output_min < config->output_max))
    {
        return false;
    }

    return true;
}

bool pid_init(
    pid_controller_t *controller,
    const pid_config_t *config)
{
    if (controller == (pid_controller_t *)0)
    {
        return false;
    }

    *controller = (pid_controller_t){0};

    if (!config_is_valid(config))
    {
        return false;
    }

    controller->config = *config;
    controller->config_valid = true;

    return true;
}

void pid_reset(pid_controller_t *controller)
{
    if (controller == (pid_controller_t *)0)
    {
        return;
    }

    controller->integral = 0.0f;
    controller->previous_measurement = 0.0f;
    controller->filtered_measurement_derivative = 0.0f;
    controller->derivative_initialized = false;
}

void pid_reset_derivative(pid_controller_t *controller)
{
    if (controller == (pid_controller_t *)0)
    {
        return;
    }

    controller->previous_measurement = 0.0f;
    controller->filtered_measurement_derivative = 0.0f;
    controller->derivative_initialized = false;
}

bool pid_update(
    pid_controller_t *controller,
    float setpoint,
    float measurement,
    float dt_s,
    pid_output_t *output)
{
    uint32_t flags;

    float error;
    float p_term;
    float d_term;

    float raw_derivative;
    float filtered_derivative;
    float previous_measurement;
    bool derivative_initialized;

    float integral_increment;
    float candidate_integral_unclamped;
    float candidate_integral;
    float accepted_integral;

    float candidate_unsaturated;
    float unsaturated_output;
    float final_output;

    bool candidate_saturated_high;
    bool candidate_saturated_low;
    bool hold_integral;

    if (output == (pid_output_t *)0)
    {
        return false;
    }

    *output = (pid_output_t){0};

    output->setpoint = setpoint;
    output->measurement = measurement;
    output->dt_s = dt_s;

    if ((controller == (pid_controller_t *)0) ||
        !controller->config_valid)
    {
        output->flags = PID_OUTPUT_CONFIG_INVALID;
        return false;
    }

    if (!value_is_finite(setpoint) ||
        !value_is_finite(measurement))
    {
        output->flags = PID_OUTPUT_INPUT_INVALID;
        return false;
    }

    if (!value_is_finite(dt_s) ||
        !(dt_s >= PID_DT_MIN_S) ||
        !(dt_s <= PID_DT_MAX_S))
    {
        output->flags = PID_OUTPUT_DT_INVALID;
        return false;
    }

    flags = 0UL;

    /*
     * Present rate error:
     *
     * desired angular rate - measured angular rate
     */
    error = setpoint - measurement;

    /*
     * Immediate proportional correction.
     */
    p_term = controller->config.kp * error;

    if (!value_is_finite(error) ||
        !value_is_finite(p_term))
    {
        output->flags = PID_OUTPUT_NUMERIC_ERROR;
        return false;
    }

    /*
     * Work with local derivative-state copies.
     *
     * The real state is committed only after the complete update succeeds.
     */
    previous_measurement =
        controller->previous_measurement;

    filtered_derivative =
        controller->filtered_measurement_derivative;

    derivative_initialized =
        controller->derivative_initialized;

    raw_derivative = 0.0f;

    /*
     * The first valid measurement only initializes derivative history.
     * No derivative output is generated from an unknown previous sample.
     */
    if (!derivative_initialized)
    {
        previous_measurement = measurement;
        filtered_derivative = 0.0f;
        derivative_initialized = true;

        flags |= PID_OUTPUT_DERIVATIVE_PRIMED;
    }
    else
    {
        float measurement_change;

        measurement_change =
            measurement - previous_measurement;

        raw_derivative =
            measurement_change / dt_s;

        if (!value_is_finite(raw_derivative))
        {
            output->flags = PID_OUTPUT_NUMERIC_ERROR;
            return false;
        }

        if (controller->config.derivative_cutoff_hz > 0.0f)
        {
            float time_constant_s;
            float alpha;

            /*
             * First-order low-pass:
             *
             *     tau = 1 / (2*pi*fc)
             *
             *     alpha = dt / (tau + dt)
             *
             *     filtered =
             *         filtered +
             *         alpha * (raw - filtered)
             */
            time_constant_s =
                1.0f /
                (PID_TWO_PI *
                 controller->config.derivative_cutoff_hz);

            alpha =
                dt_s / (time_constant_s + dt_s);

            filtered_derivative +=
                alpha *
                (raw_derivative -
                 filtered_derivative);
        }
        else
        {
            filtered_derivative =
                raw_derivative;
        }

        previous_measurement = measurement;
    }

    /*
     * Derivative-on-measurement damping.
     *
     * Kd remains positive. The explicit minus sign makes the D term oppose
     * rapid growth in measured angular rate.
     */
    d_term =
        -controller->config.kd *
        filtered_derivative;

    /*
     * Candidate integral contribution.
     */
    integral_increment =
        controller->config.ki *
        error *
        dt_s;

    candidate_integral_unclamped =
        controller->integral +
        integral_increment;

    if (!value_is_finite(filtered_derivative) ||
        !value_is_finite(d_term) ||
        !value_is_finite(integral_increment) ||
        !value_is_finite(candidate_integral_unclamped))
    {
        output->flags = PID_OUTPUT_NUMERIC_ERROR;
        return false;
    }

    /*
     * First anti-windup layer: hard integral contribution bounds.
     */
    candidate_integral =
        clamp_float(
            candidate_integral_unclamped,
            controller->config.integral_min,
            controller->config.integral_max);

    if (candidate_integral !=
        candidate_integral_unclamped)
    {
        flags |= PID_OUTPUT_INTEGRAL_CLAMPED;
    }

    candidate_unsaturated =
        p_term +
        candidate_integral +
        d_term;

    if (!value_is_finite(candidate_unsaturated))
    {
        output->flags = PID_OUTPUT_NUMERIC_ERROR;
        return false;
    }

    candidate_saturated_high =
        candidate_unsaturated >
        controller->config.output_max;

    candidate_saturated_low =
        candidate_unsaturated <
        controller->config.output_min;

    /*
     * Second anti-windup layer: conditional integration.
     *
     * If the candidate output is high-saturated and positive error would
     * increase that saturation, hold the old integral.
     *
     * If the candidate output is low-saturated and negative error would
     * increase that saturation, hold the old integral.
     *
     * Integration in the opposite direction is still allowed so the
     * controller can recover from saturation.
     */
    hold_integral =
        (candidate_saturated_high &&
         (error > 0.0f)) ||
        (candidate_saturated_low &&
         (error < 0.0f));

    if (hold_integral)
    {
        accepted_integral =
            controller->integral;

        flags |= PID_OUTPUT_INTEGRAL_HELD;
    }
    else
    {
        accepted_integral =
            candidate_integral;
    }

    unsaturated_output =
        p_term +
        accepted_integral +
        d_term;

    if (!value_is_finite(unsaturated_output))
    {
        output->flags = PID_OUTPUT_NUMERIC_ERROR;
        return false;
    }

    final_output =
        clamp_float(
            unsaturated_output,
            controller->config.output_min,
            controller->config.output_max);

    if (unsaturated_output >
        controller->config.output_max)
    {
        flags |= PID_OUTPUT_SATURATED_HIGH;
    }
    else if (unsaturated_output <
             controller->config.output_min)
    {
        flags |= PID_OUTPUT_SATURATED_LOW;
    }

    /*
     * Commit dynamic state only after the complete update is valid.
     */
    controller->integral =
        accepted_integral;

    controller->previous_measurement =
        previous_measurement;

    controller->filtered_measurement_derivative =
        filtered_derivative;

    controller->derivative_initialized =
        derivative_initialized;

    output->error = error;

    output->p_term = p_term;
    output->i_term = accepted_integral;
    output->d_term = d_term;

    output->raw_measurement_derivative =
        raw_derivative;

    output->filtered_measurement_derivative =
        filtered_derivative;

    output->unsaturated_output =
        unsaturated_output;

    output->output =
        final_output;

    output->flags =
        flags |
        PID_OUTPUT_VALID;

    return true;
}