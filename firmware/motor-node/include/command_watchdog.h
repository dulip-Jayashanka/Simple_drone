#ifndef COMMAND_WATCHDOG_H
#define COMMAND_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * COMMAND WATCHDOG STATUS
 * ============================================================
 *
 * WAITING:
 *     No valid motor command has been accepted yet.
 *
 * HEALTHY:
 *     At least one valid command has been accepted and the most
 *     recent accepted command is younger than the timeout.
 *
 * TIMED_OUT:
 *     A valid command was previously received, but no new valid
 *     command has arrived within the configured timeout.
 */
typedef enum
{
    COMMAND_WATCHDOG_WAITING = 0,

    COMMAND_WATCHDOG_HEALTHY,

    COMMAND_WATCHDOG_TIMED_OUT

} command_watchdog_status_t;


/*
 * ============================================================
 * DEBUGGER-VISIBLE WATCHDOG STATE
 * ============================================================
 */

extern volatile command_watchdog_status_t
    g_command_watchdog_status;


extern volatile uint32_t
    g_command_watchdog_initialized;


extern volatile uint32_t
    g_command_watchdog_has_valid_command;


extern volatile uint32_t
    g_command_watchdog_timeout_ms;


extern volatile uint32_t
    g_command_watchdog_last_valid_sequence;


extern volatile uint32_t
    g_command_watchdog_last_valid_timestamp_ms;


extern volatile uint32_t
    g_command_watchdog_last_age_ms;


extern volatile uint32_t
    g_command_watchdog_refresh_count;


extern volatile uint32_t
    g_command_watchdog_timeout_event_count;


extern volatile uint32_t
    g_command_watchdog_recovery_count;


/*
 * Initialize the watchdog.
 *
 * timeout_ms must be non-zero.
 *
 * The watchdog begins in WAITING and does NOT time out until at
 * least one valid command has been explicitly reported.
 */
bool
command_watchdog_init(
    uint32_t timeout_ms);


/*
 * Report one command that has already passed:
 *
 *     UART framing
 *     protocol validation
 *     CRC validation
 *     motor range validation
 *     sequence freshness validation
 *
 * This function must never be called for rejected packets.
 */
bool
command_watchdog_note_valid_command(
    uint32_t sequence,
    uint32_t received_timestamp_ms);


/*
 * Evaluate the age of the latest accepted command.
 *
 * Uses unsigned subtraction so millis() wraparound is handled
 * naturally.
 */
command_watchdog_status_t
command_watchdog_process(
    uint32_t now_ms);


/*
 * Return current watchdog state.
 */
command_watchdog_status_t
command_watchdog_get_status(void);


/*
 * Convenience status helpers.
 */
bool
command_watchdog_is_healthy(void);


bool
command_watchdog_has_timed_out(void);


#endif /* COMMAND_WATCHDOG_H */