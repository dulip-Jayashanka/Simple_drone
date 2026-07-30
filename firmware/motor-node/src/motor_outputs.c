#include "motor_outputs.h"

volatile uint32_t g_motor_safe_state_requested;

__attribute__((weak))
void motor_outputs_hardware_force_safe(void)
{
    /*
     * Phase 2.1.5 has no PWM output yet.
     *
     * A later PWM module can replace this weak function:
     *
     * void motor_outputs_hardware_force_safe(void)
     * {
     *     Disable PWM timer outputs;
     *     Set ESC command to minimum throttle;
     * }
     */
}

void motor_outputs_force_safe(void)
{
    /*
     * Record the safety request first.
     */
    g_motor_safe_state_requested = 1UL;

    /*
     * Complete the memory write before touching motor hardware.
     */
    __asm volatile ("dmb" ::: "memory");

    motor_outputs_hardware_force_safe();
}

bool motor_outputs_safe_state_was_requested(void)
{
    return (g_motor_safe_state_requested != 0UL);
}