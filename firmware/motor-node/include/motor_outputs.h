#ifndef MOTOR_OUTPUTS_H
#define MOTOR_OUTPUTS_H

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * HARD-SAFE MOTOR OUTPUT BOUNDARY
 * ============================================================
 *
 * Phase 6.3 keeps this module deliberately separate from normal ESC
 * PWM operation.
 *
 * Normal DISARMED / FAILSAFE policy:
 *     motor_pwm / motor_actuator keep TIM3 active at the configured
 *     ESC-safe pulse.
 *
 * Fatal/emergency policy implemented here:
 *     PA6, PA7, PB0 and PB1 are disconnected from timer alternate
 *     function and configured as verified ordinary GPIO LOW.
 *
 * Fault handlers use this stronger policy before recording fault
 * information. The low-level PWM module also uses it when the PWM
 * subsystem cannot safely continue.
 */


/*
 * Debugger-visible hard-safety state.
 */
extern volatile uint32_t
    g_motor_safe_state_requested;

extern volatile uint32_t
    g_motor_outputs_safe;


/*
 * Request immediate hard motor shutdown using GPIO LOW.
 */
void
motor_outputs_force_safe(void);


/*
 * Verify PA6/PA7/PB0/PB1 are in the expected GPIO output
 * configuration with all four output latches LOW.
 */
bool
motor_outputs_are_safe(void);


/*
 * True after at least one hard-safe request has been made.
 */
bool
motor_outputs_safe_state_was_requested(void);


#endif /* MOTOR_OUTPUTS_H */
