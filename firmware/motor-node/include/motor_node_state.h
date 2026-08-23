#ifndef MOTOR_NODE_STATE_H
#define MOTOR_NODE_STATE_H

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * MOTOR-NODE APPLICATION STATES
 * ============================================================
 *
 * DISARMED remains zero intentionally.
 *
 * The reset-time .bss value therefore naturally represents the
 * safest normal application state.
 *
 * Phase 6.2:
 *
 *     DISARMED
 *     ARMED       logical only; PWM is still not implemented
 *     FAILSAFE    latched until explicit disarm/reset
 */
typedef enum
{
    MOTOR_NODE_STATE_DISARMED = 0,

    MOTOR_NODE_STATE_ARMED,

    MOTOR_NODE_STATE_FAILSAFE

} motor_node_state_t;


/*
 * ============================================================
 * FAILSAFE REASON
 * ============================================================
 */

typedef enum
{
    MOTOR_NODE_FAILSAFE_NONE = 0,

    MOTOR_NODE_FAILSAFE_COMMAND_TIMEOUT,

    MOTOR_NODE_FAILSAFE_OUTPUT_FAILURE,

    MOTOR_NODE_FAILSAFE_UNEXPECTED_STATE

} motor_node_failsafe_reason_t;


/*
 * ============================================================
 * DEBUGGER-VISIBLE STATE
 * ============================================================
 */

extern volatile motor_node_state_t
    g_motor_node_state;


extern volatile motor_node_failsafe_reason_t
    g_motor_node_failsafe_reason;


extern volatile uint32_t
    g_motor_node_state_initialized;


extern volatile uint32_t
    g_disarmed_enforcement_count;


extern volatile uint32_t
    g_armed_safe_enforcement_count;


extern volatile uint32_t
    g_failsafe_enforcement_count;


extern volatile uint32_t
    g_disarmed_safety_failure_count;


extern volatile uint32_t
    g_armed_safety_failure_count;


extern volatile uint32_t
    g_failsafe_safety_failure_count;


extern volatile uint32_t
    g_unexpected_state_count;


extern volatile uint32_t
    g_arm_request_count;


extern volatile uint32_t
    g_arm_accept_count;


extern volatile uint32_t
    g_arm_reject_count;


extern volatile uint32_t
    g_disarm_request_count;


extern volatile uint32_t
    g_failsafe_entry_count;


/*
 * Initialize in DISARMED and immediately verify physical motor
 * outputs are safe.
 */
bool
motor_node_state_init(void);


/*
 * Execute one iteration of the current state policy.
 *
 * Phase 6.2 deliberately keeps all physical motor outputs safe LOW
 * even while logically ARMED.
 */
bool
motor_node_state_process(void);


/*
 * Request logical ARMED state.
 *
 * command_stream_healthy must be true.
 *
 * Ordinary MOTOR_COMMAND packets never call this automatically.
 */
bool
motor_node_state_request_arm(
    bool command_stream_healthy);


/*
 * Explicitly return to DISARMED.
 *
 * This is also the permitted recovery path from a latched FAILSAFE.
 */
bool
motor_node_state_request_disarm(void);


/*
 * Immediately latch FAILSAFE and enforce safe motor outputs.
 */
bool
motor_node_state_enter_failsafe(
    motor_node_failsafe_reason_t reason);


/*
 * State helpers.
 */
bool
motor_node_is_disarmed(void);


bool
motor_node_is_armed(void);


bool
motor_node_is_failsafe(void);


#endif /* MOTOR_NODE_STATE_H */