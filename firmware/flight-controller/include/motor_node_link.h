#ifndef MOTOR_NODE_LINK_H
#define MOTOR_NODE_LINK_H

#include "motor_link_protocol.h"
#include "motor_mixer.h"

#include <stdbool.h>
#include <stdint.h>


#ifndef MOTOR_NODE_LINK_ARM_GUARD_FRAMES
#define MOTOR_NODE_LINK_ARM_GUARD_FRAMES  50UL
#endif


#if (MOTOR_NODE_LINK_ARM_GUARD_FRAMES < 1) || \
    (MOTOR_NODE_LINK_ARM_GUARD_FRAMES > 1000)
#error "MOTOR_NODE_LINK_ARM_GUARD_FRAMES must be from 1 to 1000"
#endif


/*
 * Temporary FC-side bench ARM source.
 *
 * Normal firmware keeps this disabled. When enabled, the link first
 * transmits a controlled number of successful DISARMED zero-command
 * frames, then requests ARMED exactly once through the same normal
 * state-request path that the future pilot/LoRa layer will use.
 *
 * This deliberately does NOT bypass the motor-node state gate,
 * watchdog, DISARM-seen interlock or ARM guard.
 */
#ifndef MOTOR_ARM_BENCH_TEST
#define MOTOR_ARM_BENCH_TEST  0
#endif


#if (MOTOR_ARM_BENCH_TEST != 0) && \
    (MOTOR_ARM_BENCH_TEST != 1)
#error "MOTOR_ARM_BENCH_TEST must be 0 or 1"
#endif


#ifndef MOTOR_ARM_BENCH_DISARMED_FRAMES
#define MOTOR_ARM_BENCH_DISARMED_FRAMES  500UL
#endif


#if MOTOR_ARM_BENCH_TEST && \
    ((MOTOR_ARM_BENCH_DISARMED_FRAMES < 1) || \
     (MOTOR_ARM_BENCH_DISARMED_FRAMES > 100000))
#error "MOTOR_ARM_BENCH_DISARMED_FRAMES must be from 1 to 100000"
#endif


typedef enum
{
    MOTOR_NODE_LINK_OK = 0,

    MOTOR_NODE_LINK_INVALID_ARGUMENT,

    MOTOR_NODE_LINK_UART_INIT_FAILED,

    MOTOR_NODE_LINK_NOT_READY,

    MOTOR_NODE_LINK_MIXER_INVALID,

    MOTOR_NODE_LINK_COMMAND_INVALID,

    MOTOR_NODE_LINK_STATE_INVALID,

    MOTOR_NODE_LINK_ENCODE_FAILED,

    MOTOR_NODE_LINK_TX_BUSY,

    MOTOR_NODE_LINK_TX_FAILED

} motor_node_link_status_t;


typedef struct
{
    uint32_t init_count;

    uint32_t send_attempt_count;

    uint32_t sent_frame_count;

    uint32_t mixer_invalid_count;

    uint32_t command_invalid_count;

    uint32_t state_invalid_count;

    uint32_t encode_error_count;

    uint32_t tx_busy_drop_count;

    uint32_t tx_failure_count;


    uint32_t state_change_count;

    uint32_t disarmed_zero_frame_count;

    uint32_t arm_guard_zero_frame_count;


    uint32_t bench_arm_request_count;

    uint32_t bench_arm_request_issued;

    uint32_t bench_arm_trigger_disarmed_frame_count;


    uint32_t last_attempted_sequence;

    uint32_t last_sent_sequence;


    uint16_t last_m1;
    uint16_t last_m2;
    uint16_t last_m3;
    uint16_t last_m4;


    uint32_t requested_state;

    uint32_t last_sent_requested_state;

    uint32_t arm_guard_frames_remaining;


    uint32_t last_status;

} motor_node_link_diag_t;


extern volatile motor_node_link_status_t
    g_motor_node_link_status;


extern volatile motor_node_link_diag_t
    g_motor_node_link_diag;


/*
 * Initialize dedicated USART2 TX communication to the motor-node.
 *
 * The requested state always resets to DISARMED at link init.
 */
motor_node_link_status_t
motor_node_link_init(
    uint32_t pclk1_hz,
    uint32_t baud_rate);


/*
 * Set the state request embedded in every subsequent MOTOR_COMMAND.
 *
 * Only DISARMED and ARMED are legal. FAILSAFE remains local to the
 * motor-node.
 *
 * A DISARMED -> ARMED request starts an ARM guard. During that guard
 * the link transmits ARMED with M1..M4 forced to zero for
 * MOTOR_NODE_LINK_ARM_GUARD_FRAMES successfully transmitted frames.
 */
motor_node_link_status_t
motor_node_link_set_requested_state(
    motor_link_requested_state_t requested_state);


motor_link_requested_state_t
motor_node_link_get_requested_state(void);


/*
 * Convert a valid normalized motor-mixer output into one fixed
 * binary MOTOR_COMMAND packet and request a non-blocking USART2
 * DMA transmission.
 *
 * The packet contains final M1..M4 plus the FC-requested state.
 *
 * DISARMED always transmits M1..M4 = 0 regardless of the mixer
 * values. During the ARM guard, ARMED also transmits zeros. Actual
 * mixer values are released only after that guard completes.
 *
 * When MOTOR_ARM_BENCH_TEST=1, the link automatically issues one
 * normal ARMED request only after MOTOR_ARM_BENCH_DISARMED_FRAMES
 * successful DISARMED frames have been transmitted. An explicit
 * later DISARM never causes the bench mode to auto-arm a second time.
 *
 * There is deliberately no backlog of old motor commands.
 *
 * If the previous frame has not finished, the current frame is
 * dropped and the next fresh mixer update gets another opportunity.
 * Dropped frames do not consume ARM-guard or bench-delay progress.
 */
motor_node_link_status_t
motor_node_link_send(
    const motor_mixer_output_t *mixer_output);


bool
motor_node_link_is_ready(void);


#endif /* MOTOR_NODE_LINK_H */