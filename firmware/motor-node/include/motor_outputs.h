#ifndef MOTOR_OUTPUTS_H
#define MOTOR_OUTPUTS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Debugger-visible safety state:
 *
 * 0 = no emergency shutdown requested
 * 1 = emergency shutdown requested
 */
extern volatile uint32_t g_motor_safe_state_requested;

/*
 * Request immediate motor shutdown.
 *
 * Fault handlers call this function before recording fault information.
 */
void motor_outputs_force_safe(void);

/*
 * Check whether an emergency shutdown was requested.
 */
bool motor_outputs_safe_state_was_requested(void);

/*
 * Hardware-dependent shutdown operation.
 *
 * A weak empty implementation is provided during Phase 2.1.5 because
 * PWM has not been implemented yet.
 *
 * The PWM phase will replace it with a strong implementation.
 */
void motor_outputs_hardware_force_safe(void);

#endif /* MOTOR_OUTPUTS_H */