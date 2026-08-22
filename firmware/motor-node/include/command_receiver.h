#ifndef COMMAND_RECEIVER_H
#define COMMAND_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>


/*
 * Validated motor command published by the motor-node receive layer.
 *
 * M1..M4 remain in protocol units:
 *
 *      0 ... 1000
 *
 * They are NOT PWM pulse widths.
 */
typedef struct
{
    uint32_t sequence;


    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;


    uint32_t received_timestamp_ms;


    bool valid;

} command_receiver_output_t;


typedef struct
{
    uint32_t bytes_processed;

    uint32_t sync_discarded_bytes;


    uint32_t completed_frames;

    uint32_t valid_frames;


    uint32_t protocol_error_count;

    uint32_t sync_error_count;

    uint32_t version_error_count;

    uint32_t type_error_count;

    uint32_t length_error_count;

    uint32_t crc_error_count;

    uint32_t range_error_count;


    uint32_t duplicate_sequence_count;

    uint32_t stale_sequence_count;

    uint32_t sequence_gap_event_count;

    uint32_t missing_sequence_count;


    uint32_t parser_resync_count;

    uint32_t sequence_history_reset_count;


    uint32_t last_valid_sequence;

    uint32_t last_valid_timestamp_ms;

    uint32_t last_rejected_sequence;

} command_receiver_stats_t;


extern volatile command_receiver_output_t
    g_latest_received_motor_command;


extern volatile command_receiver_stats_t
    g_command_receiver_stats;


/*
 * Reset parser, latest output, sequence history and diagnostics.
 */
void
command_receiver_init(void);


/*
 * Drain every currently available USART2 RX byte.
 *
 * Returns number of newly accepted complete motor commands.
 */
uint32_t
command_receiver_process(
    uint32_t now_ms);


/*
 * Host-testable byte parser.
 *
 * Returns true only when this byte completes a frame that passes:
 *
 *     synchronization
 *     version
 *     type
 *     length
 *     CRC
 *     motor ranges
 *     sequence freshness
 */
bool
command_receiver_process_byte(
    uint8_t byte,
    uint32_t now_ms);


/*
 * Forget sequence ordering history.
 *
 * This exists for the later command-watchdog/failsafe phase.
 *
 * Example:
 *
 *     FC was running at sequence 50000
 *     FC loses power
 *     motor node remains powered
 *     watchdog identifies a real communication-session break
 *     FC restarts sequence from 0
 *
 * The watchdog may then explicitly reset sequence history before
 * accepting commands from the new communication session.
 *
 * Phase 6.1 does NOT call this automatically because the UART
 * timeout policy has intentionally not been selected yet.
 */
void
command_receiver_reset_sequence_history(void);


/*
 * Copy latest validated motor command.
 */
bool
command_receiver_get_latest(
    command_receiver_output_t *output);


#endif /* COMMAND_RECEIVER_H */