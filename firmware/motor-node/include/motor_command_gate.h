#ifndef MOTOR_COMMAND_GATE_H
#define MOTOR_COMMAND_GATE_H

#include "command_receiver.h"

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    MOTOR_COMMAND_GATE_OK = 0,

    MOTOR_COMMAND_GATE_NOT_INITIALIZED,

    MOTOR_COMMAND_GATE_INVALID_ARGUMENT,

    MOTOR_COMMAND_GATE_INVALID_REQUESTED_STATE,

    MOTOR_COMMAND_GATE_ARM_REJECT_NO_DISARM_SEEN,

    MOTOR_COMMAND_GATE_ARM_REJECT_WATCHDOG,

    MOTOR_COMMAND_GATE_ARM_REJECT_NONZERO_COMMAND,

    MOTOR_COMMAND_GATE_ARM_REJECT_FAILSAFE,

    MOTOR_COMMAND_GATE_STATE_TRANSITION_FAILED

} motor_command_gate_status_t;


typedef struct
{
    uint32_t init_count;

    uint32_t apply_count;

    uint32_t session_reset_count;

    uint32_t disarm_seen_since_session_start;


    uint32_t disarm_request_count;

    uint32_t disarm_accept_count;


    uint32_t arm_request_count;

    uint32_t arm_accept_count;

    uint32_t arm_already_armed_count;

    uint32_t arm_reject_no_disarm_seen_count;

    uint32_t arm_reject_watchdog_count;

    uint32_t arm_reject_nonzero_command_count;

    uint32_t arm_reject_failsafe_count;


    uint32_t state_transition_failure_count;


    uint32_t last_requested_state;

    uint32_t last_status;

} motor_command_gate_diag_t;


extern volatile motor_command_gate_status_t
    g_motor_command_gate_status;


extern volatile motor_command_gate_diag_t
    g_motor_command_gate_diag;


/*
 * Initialize the requested-state safety gate.
 *
 * A fresh communication session begins with ARM inhibited until at
 * least one valid DISARMED command has been observed and accepted.
 */
bool
motor_command_gate_init(void);


/*
 * Mark a real FC <-> motor-node communication-session break.
 *
 * This does not itself change the motor-node application state.
 * It clears only the "DISARM seen" permission so a restarted FC
 * cannot cause an immediate re-arm by continuing to transmit ARMED.
 */
void
motor_command_gate_reset_session(void);


/*
 * Apply the requested state carried by one already validated, fresh
 * MOTOR_COMMAND.
 *
 * DISARM is always honored when the local state layer can enforce it.
 *
 * DISARMED -> ARMED additionally requires:
 *
 *     a DISARMED command observed in this communication session
 *     healthy/current command stream
 *     all M1..M4 equal to zero on the transition command
 *     local state not FAILSAFE
 *
 * Once already ARMED, repeated ARMED commands may contain non-zero
 * M1..M4 values. Those values are consumed later by the PWM layer.
 */
motor_command_gate_status_t
motor_command_gate_apply(
    const command_receiver_output_t *command,
    bool command_stream_healthy);


bool
motor_command_gate_disarm_seen(void);


#endif /* MOTOR_COMMAND_GATE_H */
