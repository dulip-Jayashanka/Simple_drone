#ifndef MOTOR_ACTUATOR_H
#define MOTOR_ACTUATOR_H

#include "motor_link_protocol.h"
#include "motor_pwm.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * PHASE 6.3 — MOTOR COMMAND -> ESC PULSE MAPPING
 * ============================================================
 *
 * The communication domain remains:
 *
 *     0 ... MOTOR_LINK_COMMAND_MAX (1000)
 *
 * This layer owns the final deterministic mapping into the configured
 * physical ESC pulse interval. It is intentionally separate from the
 * low-level TIM3 driver.
 */


#ifndef MOTOR_ACTUATOR_TEST_MODE
#define MOTOR_ACTUATOR_TEST_MODE 0
#endif

#if (MOTOR_ACTUATOR_TEST_MODE != 0) && \
    (MOTOR_ACTUATOR_TEST_MODE != 1)
#error "MOTOR_ACTUATOR_TEST_MODE must be 0 or 1"
#endif

#ifndef MOTOR_ACTUATOR_TEST_M1
#define MOTOR_ACTUATOR_TEST_M1 0UL
#endif

#ifndef MOTOR_ACTUATOR_TEST_M2
#define MOTOR_ACTUATOR_TEST_M2 0UL
#endif

#ifndef MOTOR_ACTUATOR_TEST_M3
#define MOTOR_ACTUATOR_TEST_M3 0UL
#endif

#ifndef MOTOR_ACTUATOR_TEST_M4
#define MOTOR_ACTUATOR_TEST_M4 0UL
#endif

#if MOTOR_ACTUATOR_TEST_MODE

#if (MOTOR_ACTUATOR_TEST_M1 > MOTOR_LINK_COMMAND_MAX) || \
    (MOTOR_ACTUATOR_TEST_M2 > MOTOR_LINK_COMMAND_MAX) || \
    (MOTOR_ACTUATOR_TEST_M3 > MOTOR_LINK_COMMAND_MAX) || \
    (MOTOR_ACTUATOR_TEST_M4 > MOTOR_LINK_COMMAND_MAX)
#error "Motor actuator test commands must be in the 0..1000 protocol range"
#endif

#endif


typedef enum
{
    MOTOR_ACTUATOR_STATUS_NOT_INITIALIZED = 0,

    MOTOR_ACTUATOR_STATUS_OK,

    MOTOR_ACTUATOR_STATUS_INVALID_ARGUMENT,

    MOTOR_ACTUATOR_STATUS_COMMAND_OUT_OF_RANGE,

    MOTOR_ACTUATOR_STATUS_PWM_NOT_READY,

    MOTOR_ACTUATOR_STATUS_PWM_FAILURE

} motor_actuator_status_t;


typedef struct
{
    uint32_t init_count;

    uint32_t apply_count;

    uint32_t safe_count;

    uint32_t invalid_command_count;

    uint32_t pwm_failure_count;

    uint32_t hard_disable_count;


    uint16_t last_m1_command;
    uint16_t last_m2_command;
    uint16_t last_m3_command;
    uint16_t last_m4_command;


    uint16_t last_m1_us;
    uint16_t last_m2_us;
    uint16_t last_m3_us;
    uint16_t last_m4_us;


    uint32_t initialized;

    uint32_t last_status;

} motor_actuator_diag_t;


extern volatile motor_actuator_status_t
    g_motor_actuator_status;


extern volatile motor_actuator_diag_t
    g_motor_actuator_diag;


/*
 * Pure mapping helper used by host tests and the runtime path.
 */
bool
motor_actuator_command_to_pulse_us(
    uint16_t command,
    uint16_t *pulse_us_out);


/*
 * Initialize the actuator boundary and immediately establish the
 * configured ESC-safe pulse on all four channels.
 */
bool
motor_actuator_init(void);


/*
 * Validate four protocol-domain motor commands, map them into the
 * configured physical ESC range, and publish them through TIM3.
 */
bool
motor_actuator_apply(
    uint16_t m1,
    uint16_t m2,
    uint16_t m3,
    uint16_t m4);


/*
 * Normal runtime safe policy while TIM3 remains active.
 */
bool
motor_actuator_set_safe(void);


/*
 * Fatal/emergency shutdown. This intentionally leaves normal PWM
 * operation and returns to the hard GPIO-LOW path.
 */
void
motor_actuator_hard_disable(void);


bool
motor_actuator_is_ready(void);


#endif /* MOTOR_ACTUATOR_H */
