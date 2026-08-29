#ifndef MOTOR_NODE_STATE_H
#define MOTOR_NODE_STATE_H

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * MOTOR-NODE APPLICATION STATES
 * ============================================================
 *
 * DISARMED remains zero intentionally so reset-time .bss naturally
 * represents the safest normal application state.
 *
 * Phase 6.3 physical policy when MOTOR_PWM_ENABLE=1:
 *
 *     DISARMED
 *         TIM3 remains active at the configured ESC-safe pulse.
 *
 *     ARMED
 *         validated fresh M1..M4 commands may drive the actuator.
 *
 *     FAILSAFE
 *         latched until explicit DISARM; TIM3 returns immediately to
 *         the configured ESC-safe pulse.
 *
 * Fatal/output-subsystem failures are stronger than the normal state
 * policy and fall back to the existing hard GPIO-LOW shutdown path.
 *
 * With MOTOR_PWM_ENABLE=0 the previous Phase 6.2/6.2.5 behavior is
 * preserved: every state physically holds the four motor pins LOW.
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


/*
 * Historical name retained for debugger/build compatibility.
 *
 * PWM disabled:
 *     counts ARMED GPIO-safe enforcement.
 *
 * PWM enabled:
 *     counts ARMED actuator/PWM readiness checks; it does not mean
 *     the active motor command was overwritten with a safe value.
 */
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
 * Initialize in DISARMED and establish the appropriate physical safe
 * policy for the selected build.
 */
bool
motor_node_state_init(void);


/*
 * Execute one iteration of the current local state policy.
 *
 * In a Phase 6.3 PWM build ARMED deliberately does not rewrite the
 * current M1..M4 output. Fresh command application is owned by the
 * validated receive path in main.c.
 */
bool
motor_node_state_process(void);


/*
 * Request logical ARMED state.
 *
 * command_stream_healthy must be true. The version-2 command gate may
 * call this only after protocol validation and its local interlocks
 * have passed.
 */
bool
motor_node_state_request_arm(
    bool command_stream_healthy);


/*
 * Explicitly return to DISARMED after establishing normal physical
 * safe output. This remains the permitted recovery from FAILSAFE.
 */
bool
motor_node_state_request_disarm(void);


/*
 * Immediately latch FAILSAFE and enforce the normal failsafe output
 * policy. In a PWM build this is the configured ESC-safe pulse.
 */
bool
motor_node_state_enter_failsafe(
    motor_node_failsafe_reason_t reason);


bool
motor_node_is_disarmed(void);


bool
motor_node_is_armed(void);


bool
motor_node_is_failsafe(void);


#endif /* MOTOR_NODE_STATE_H */
