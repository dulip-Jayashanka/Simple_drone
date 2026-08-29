#include "motor_actuator.h"

#include "motor_pwm.h"

#include <stdbool.h>
#include <stdint.h>


volatile motor_actuator_status_t
    g_motor_actuator_status =
        MOTOR_ACTUATOR_STATUS_NOT_INITIALIZED;


volatile motor_actuator_diag_t
    g_motor_actuator_diag;


static bool
    motor_actuator_ready;


static motor_actuator_status_t
record_status(
    motor_actuator_status_t status)
{
    g_motor_actuator_status =
        status;


    g_motor_actuator_diag
        .last_status =
        (uint32_t)
        status;


    return status;
}


bool
motor_actuator_command_to_pulse_us(
    uint16_t command,
    uint16_t *pulse_us_out)
{
    uint32_t
        command_value;

    uint32_t
        pulse_span;

    uint32_t
        pulse_us;


    if (pulse_us_out ==
        (uint16_t *)0)
    {
        return false;
    }


    command_value =
        (uint32_t)
        command;


    if (command_value >
        MOTOR_LINK_COMMAND_MAX)
    {
        return false;
    }


    pulse_span =
        MOTOR_ESC_MAX_US -
        MOTOR_ESC_MIN_US;


    pulse_us =
        MOTOR_ESC_MIN_US +
        ((command_value *
          pulse_span) /
         MOTOR_LINK_COMMAND_MAX);


    if ((pulse_us <
         MOTOR_ESC_MIN_US) ||
        (pulse_us >
         MOTOR_ESC_MAX_US) ||
        (pulse_us >
         UINT16_MAX))
    {
        return false;
    }


    *pulse_us_out =
        (uint16_t)
        pulse_us;


    return true;
}


bool
motor_actuator_init(void)
{
    motor_actuator_ready =
        false;


    g_motor_actuator_diag =
        (motor_actuator_diag_t){0};


    g_motor_actuator_status =
        MOTOR_ACTUATOR_STATUS_NOT_INITIALIZED;


    if ((!motor_pwm_is_initialized()) ||
        (!motor_pwm_configuration_is_valid()))
    {
        (void)record_status(
            MOTOR_ACTUATOR_STATUS_PWM_NOT_READY);


        return false;
    }


    if (!motor_pwm_set_safe())
    {
        g_motor_actuator_diag
            .pwm_failure_count++;


        (void)record_status(
            MOTOR_ACTUATOR_STATUS_PWM_FAILURE);


        return false;
    }


    motor_actuator_ready =
        true;


    g_motor_actuator_diag
        .init_count =
        1UL;


    g_motor_actuator_diag
        .safe_count =
        1UL;


    g_motor_actuator_diag
        .last_m1_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_actuator_diag
        .last_m2_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_actuator_diag
        .last_m3_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_actuator_diag
        .last_m4_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;


    g_motor_actuator_diag
        .initialized =
        1UL;


    (void)record_status(
        MOTOR_ACTUATOR_STATUS_OK);


    return true;
}


bool
motor_actuator_apply(
    uint16_t m1,
    uint16_t m2,
    uint16_t m3,
    uint16_t m4)
{
    uint16_t
        m1_us;

    uint16_t
        m2_us;

    uint16_t
        m3_us;

    uint16_t
        m4_us;


    if ((!motor_actuator_ready) ||
        (!motor_pwm_configuration_is_valid()))
    {
        (void)record_status(
            MOTOR_ACTUATOR_STATUS_NOT_INITIALIZED);


        return false;
    }


    if ((!motor_actuator_command_to_pulse_us(
             m1,
             &m1_us)) ||
        (!motor_actuator_command_to_pulse_us(
             m2,
             &m2_us)) ||
        (!motor_actuator_command_to_pulse_us(
             m3,
             &m3_us)) ||
        (!motor_actuator_command_to_pulse_us(
             m4,
             &m4_us)))
    {
        g_motor_actuator_diag
            .invalid_command_count++;


        (void)record_status(
            MOTOR_ACTUATOR_STATUS_COMMAND_OUT_OF_RANGE);


        return false;
    }


    if (!motor_pwm_set_us(
            m1_us,
            m2_us,
            m3_us,
            m4_us))
    {
        g_motor_actuator_diag
            .pwm_failure_count++;


        (void)record_status(
            MOTOR_ACTUATOR_STATUS_PWM_FAILURE);


        return false;
    }


    g_motor_actuator_diag
        .apply_count++;


    g_motor_actuator_diag
        .last_m1_command =
        m1;

    g_motor_actuator_diag
        .last_m2_command =
        m2;

    g_motor_actuator_diag
        .last_m3_command =
        m3;

    g_motor_actuator_diag
        .last_m4_command =
        m4;


    g_motor_actuator_diag
        .last_m1_us =
        m1_us;

    g_motor_actuator_diag
        .last_m2_us =
        m2_us;

    g_motor_actuator_diag
        .last_m3_us =
        m3_us;

    g_motor_actuator_diag
        .last_m4_us =
        m4_us;


    (void)record_status(
        MOTOR_ACTUATOR_STATUS_OK);


    return true;
}


bool
motor_actuator_set_safe(void)
{
    if ((!motor_actuator_ready) ||
        (!motor_pwm_configuration_is_valid()))
    {
        (void)record_status(
            MOTOR_ACTUATOR_STATUS_NOT_INITIALIZED);


        return false;
    }


    if (!motor_pwm_set_safe())
    {
        g_motor_actuator_diag
            .pwm_failure_count++;


        (void)record_status(
            MOTOR_ACTUATOR_STATUS_PWM_FAILURE);


        return false;
    }


    g_motor_actuator_diag
        .safe_count++;


    g_motor_actuator_diag
        .last_m1_command =
        0U;

    g_motor_actuator_diag
        .last_m2_command =
        0U;

    g_motor_actuator_diag
        .last_m3_command =
        0U;

    g_motor_actuator_diag
        .last_m4_command =
        0U;


    g_motor_actuator_diag
        .last_m1_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_actuator_diag
        .last_m2_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_actuator_diag
        .last_m3_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;

    g_motor_actuator_diag
        .last_m4_us =
        (uint16_t)
        MOTOR_ESC_SAFE_US;


    (void)record_status(
        MOTOR_ACTUATOR_STATUS_OK);


    return true;
}


void
motor_actuator_hard_disable(void)
{
    motor_pwm_hard_disable();


    motor_actuator_ready =
        false;


    g_motor_actuator_diag
        .initialized =
        0UL;


    g_motor_actuator_diag
        .hard_disable_count++;
}


bool
motor_actuator_is_ready(void)
{
    return
        motor_actuator_ready &&
        motor_pwm_is_initialized() &&
        motor_pwm_configuration_is_valid();
}
