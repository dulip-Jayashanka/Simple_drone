#include "command_watchdog.h"

#include <assert.h>
#include <stdint.h>


static void
test_zero_timeout_is_rejected(void)
{
    assert(
        !command_watchdog_init(
            0UL));


    assert(
        g_command_watchdog_initialized ==
        0UL);


    assert(
        command_watchdog_get_status() ==
        COMMAND_WATCHDOG_WAITING);
}


static void
test_waiting_does_not_timeout_before_first_command(void)
{
    assert(
        command_watchdog_init(
            20UL));


    assert(
        command_watchdog_process(
            0UL) ==
        COMMAND_WATCHDOG_WAITING);


    /*
     * Even a very long boot/idle period does not become a timeout
     * until at least one valid command has first been accepted.
     */
    assert(
        command_watchdog_process(
            100000UL) ==
        COMMAND_WATCHDOG_WAITING);


    assert(
        g_command_watchdog_timeout_event_count ==
        0UL);
}


static void
test_valid_command_makes_watchdog_healthy(void)
{
    assert(
        command_watchdog_init(
            20UL));


    assert(
        command_watchdog_note_valid_command(
            123UL,
            1000UL));


    assert(
        command_watchdog_is_healthy());


    assert(
        g_command_watchdog_last_valid_sequence ==
        123UL);


    assert(
        g_command_watchdog_last_valid_timestamp_ms ==
        1000UL);


    assert(
        g_command_watchdog_refresh_count ==
        1UL);


    assert(
        command_watchdog_process(
            1019UL) ==
        COMMAND_WATCHDOG_HEALTHY);


    assert(
        g_command_watchdog_last_age_ms ==
        19UL);
}


static void
test_timeout_occurs_at_configured_age_once(void)
{
    assert(
        command_watchdog_init(
            20UL));


    assert(
        command_watchdog_note_valid_command(
            10UL,
            500UL));


    assert(
        command_watchdog_process(
            519UL) ==
        COMMAND_WATCHDOG_HEALTHY);


    /*
     * The timeout becomes active when age >= timeout.
     */
    assert(
        command_watchdog_process(
            520UL) ==
        COMMAND_WATCHDOG_TIMED_OUT);


    assert(
        command_watchdog_has_timed_out());


    assert(
        g_command_watchdog_timeout_event_count ==
        1UL);


    /*
     * Remaining timed out must not create another timeout event.
     */
    assert(
        command_watchdog_process(
            600UL) ==
        COMMAND_WATCHDOG_TIMED_OUT);


    assert(
        g_command_watchdog_timeout_event_count ==
        1UL);
}


static void
test_valid_command_recovers_watchdog_after_timeout(void)
{
    assert(
        command_watchdog_init(
            20UL));


    assert(
        command_watchdog_note_valid_command(
            50UL,
            100UL));


    assert(
        command_watchdog_process(
            120UL) ==
        COMMAND_WATCHDOG_TIMED_OUT);


    assert(
        command_watchdog_note_valid_command(
            51UL,
            121UL));


    assert(
        command_watchdog_is_healthy());


    assert(
        g_command_watchdog_recovery_count ==
        1UL);


    assert(
        g_command_watchdog_refresh_count ==
        2UL);
}


static void
test_millis_wraparound_is_handled(void)
{
    assert(
        command_watchdog_init(
            20UL));


    assert(
        command_watchdog_note_valid_command(
            0xFFFFFFFFUL,
            0xFFFFFFF8UL));


    /*
     * 0xFFFFFFF8 -> 5 is an elapsed time of 13 ms.
     */
    assert(
        command_watchdog_process(
            5UL) ==
        COMMAND_WATCHDOG_HEALTHY);


    assert(
        g_command_watchdog_last_age_ms ==
        13UL);


    /*
     * 0xFFFFFFF8 -> 12 is exactly 20 ms.
     */
    assert(
        command_watchdog_process(
            12UL) ==
        COMMAND_WATCHDOG_TIMED_OUT);


    assert(
        g_command_watchdog_last_age_ms ==
        20UL);
}


int
main(void)
{
    test_zero_timeout_is_rejected();

    test_waiting_does_not_timeout_before_first_command();

    test_valid_command_makes_watchdog_healthy();

    test_timeout_occurs_at_configured_age_once();

    test_valid_command_recovers_watchdog_after_timeout();

    test_millis_wraparound_is_handled();


    return 0;
}