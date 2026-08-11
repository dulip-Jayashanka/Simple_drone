#ifndef MOTOR_NODE_STATE_H
#define MOTOR_NODE_STATE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Phase 2.1.11 application states.
 *
 * DISARMED is deliberately zero so the reset-time .bss value is the
 * safest application state, even before main() performs initialization.
 */
typedef enum
{
    MOTOR_NODE_STATE_DISARMED = 0
} motor_node_state_t;

/* Debugger-visible Phase 2.1.11 state and test evidence. */
extern volatile motor_node_state_t g_motor_node_state;
extern volatile uint32_t g_motor_node_state_initialized;
extern volatile uint32_t g_disarmed_enforcement_count;
extern volatile uint32_t g_disarmed_safety_failure_count;
extern volatile uint32_t g_unexpected_state_count;

/*
 * Enter the fail-safe default state and immediately verify the physical
 * motor outputs. Returns false if the outputs cannot be confirmed safe.
 */
bool motor_node_state_init(void);

/*
 * Run one application-state iteration.
 *
 * Phase 2.1.11 has only DISARMED, so every call reasserts and verifies the
 * safe motor outputs. Unknown state values are forced back to DISARMED.
 */
bool motor_node_state_process(void);

/* Return true only when initialization succeeded and the state is DISARMED. */
bool motor_node_is_disarmed(void);

#endif /* MOTOR_NODE_STATE_H */
