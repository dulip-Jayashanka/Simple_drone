#include "command_watchdog.h"


volatile command_watchdog_status_t
    g_command_watchdog_status =
        COMMAND_WATCHDOG_WAITING;


volatile uint32_t
    g_command_watchdog_initialized;


volatile uint32_t
    g_command_watchdog_has_valid_command;


volatile uint32_t
    g_command_watchdog_timeout_ms;


volatile uint32_t
    g_command_watchdog_last_valid_sequence;


volatile uint32_t
    g_command_watchdog_last_valid_timestamp_ms;


volatile uint32_t
    g_command_watchdog_last_age_ms;


volatile uint32_t
    g_command_watchdog_refresh_count;


volatile uint32_t
    g_command_watchdog_timeout_event_count;


volatile uint32_t
    g_command_watchdog_recovery_count;


bool
command_watchdog_init(
    uint32_t timeout_ms)
{
    g_command_watchdog_status =
        COMMAND_WATCHDOG_WAITING;


    g_command_watchdog_initialized =
        0UL;


    g_command_watchdog_has_valid_command =
        0UL;


    g_command_watchdog_timeout_ms =
        0UL;


    g_command_watchdog_last_valid_sequence =
        0UL;


    g_command_watchdog_last_valid_timestamp_ms =
        0UL;


    g_command_watchdog_last_age_ms =
        0UL;


    g_command_watchdog_refresh_count =
        0UL;


    g_command_watchdog_timeout_event_count =
        0UL;


    g_command_watchdog_recovery_count =
        0UL;


    if (timeout_ms ==
        0UL)
    {
        return false;
    }


    g_command_watchdog_timeout_ms =
        timeout_ms;


    g_command_watchdog_initialized =
        1UL;


    return true;
}


bool
command_watchdog_note_valid_command(
    uint32_t sequence,
    uint32_t received_timestamp_ms)
{
    if (g_command_watchdog_initialized ==
        0UL)
    {
        return false;
    }


    /*
     * A fresh valid command after TIMED_OUT means the communication
     * stream itself has recovered.
     *
     * This does NOT automatically recover the motor-node from its
     * latched FAILSAFE state.
     */
    if (g_command_watchdog_status ==
        COMMAND_WATCHDOG_TIMED_OUT)
    {
        g_command_watchdog_recovery_count++;
    }


    g_command_watchdog_last_valid_sequence =
        sequence;


    g_command_watchdog_last_valid_timestamp_ms =
        received_timestamp_ms;


    g_command_watchdog_last_age_ms =
        0UL;


    g_command_watchdog_has_valid_command =
        1UL;


    g_command_watchdog_refresh_count++;


    g_command_watchdog_status =
        COMMAND_WATCHDOG_HEALTHY;


    return true;
}


command_watchdog_status_t
command_watchdog_process(
    uint32_t now_ms)
{
    uint32_t
        age_ms;


    /*
     * An uninitialized watchdog is never treated as healthy.
     */
    if (g_command_watchdog_initialized ==
        0UL)
    {
        g_command_watchdog_status =
            COMMAND_WATCHDOG_WAITING;


        return
            g_command_watchdog_status;
    }


    /*
     * Before the first valid command, remain WAITING indefinitely.
     *
     * This prevents boot time itself from being interpreted as a
     * communication failure.
     */
    if (g_command_watchdog_has_valid_command ==
        0UL)
    {
        g_command_watchdog_last_age_ms =
            0UL;


        g_command_watchdog_status =
            COMMAND_WATCHDOG_WAITING;


        return
            g_command_watchdog_status;
    }


    /*
     * Unsigned subtraction makes this safe across uint32_t millis()
     * wraparound.
     */
    age_ms =
        (uint32_t)(
            now_ms -
            g_command_watchdog_last_valid_timestamp_ms);


    g_command_watchdog_last_age_ms =
        age_ms;


    if (age_ms >=
        g_command_watchdog_timeout_ms)
    {
        /*
         * Count only the transition into TIMED_OUT.
         *
         * Staying timed out for many main-loop iterations must not
         * continually increment the event counter.
         */
        if (g_command_watchdog_status !=
            COMMAND_WATCHDOG_TIMED_OUT)
        {
            g_command_watchdog_timeout_event_count++;
        }


        g_command_watchdog_status =
            COMMAND_WATCHDOG_TIMED_OUT;
    }
    else
    {
        g_command_watchdog_status =
            COMMAND_WATCHDOG_HEALTHY;
    }


    return
        g_command_watchdog_status;
}


command_watchdog_status_t
command_watchdog_get_status(void)
{
    return
        g_command_watchdog_status;
}


bool
command_watchdog_is_healthy(void)
{
    return
        (g_command_watchdog_initialized !=
         0UL) &&
        (g_command_watchdog_status ==
         COMMAND_WATCHDOG_HEALTHY);
}


bool
command_watchdog_has_timed_out(void)
{
    return
        (g_command_watchdog_initialized !=
         0UL) &&
        (g_command_watchdog_status ==
         COMMAND_WATCHDOG_TIMED_OUT);
}