#include "motor_node_state.h"

#include "motor_outputs.h"

/*
 * Phase 2.1.11 debugger-visible state.
 *
 * The zero-valued DISARMED state makes reset fail-safe by default.
 */
volatile motor_node_state_t g_motor_node_state =
    MOTOR_NODE_STATE_DISARMED;

volatile uint32_t g_motor_node_state_initialized;
volatile uint32_t g_disarmed_enforcement_count;
volatile uint32_t g_disarmed_safety_failure_count;
volatile uint32_t g_unexpected_state_count;

static bool disarmed_outputs_enforce(void)
{
    /*
     * State policy first becomes a physical action here. The low-level
     * motor driver remains responsible for configuring and driving PA6,
     * PA7, PB0 and PB1 LOW.
     */
    motor_outputs_force_safe();
    g_disarmed_enforcement_count++;

    if (!motor_outputs_are_safe())
    {
        g_disarmed_safety_failure_count++;
        return false;
    }

    return true;
}

bool motor_node_state_init(void)
{
    bool outputs_safe;

    g_motor_node_state_initialized = 0UL;
    g_disarmed_enforcement_count = 0UL;
    g_disarmed_safety_failure_count = 0UL;
    g_unexpected_state_count = 0UL;

    /* Every power-on and reset explicitly returns to DISARMED. */
    g_motor_node_state = MOTOR_NODE_STATE_DISARMED;

    outputs_safe = disarmed_outputs_enforce();

    if (outputs_safe)
    {
        g_motor_node_state_initialized = 1UL;
    }

    return outputs_safe;
}

bool motor_node_state_process(void)
{
    /*
     * With only one valid state in this phase, any other value is unsafe.
     * Record it, restore DISARMED, and enforce safe physical outputs.
     */
    if (g_motor_node_state != MOTOR_NODE_STATE_DISARMED)
    {
        g_unexpected_state_count++;
        g_motor_node_state = MOTOR_NODE_STATE_DISARMED;
    }

    return disarmed_outputs_enforce();
}

bool motor_node_is_disarmed(void)
{
    return (g_motor_node_state_initialized != 0UL) &&
           (g_motor_node_state == MOTOR_NODE_STATE_DISARMED);
}
