#include "command_receiver.h"

#include "motor_link_protocol.h"
#include "uart2_link.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


/*
 * STM32 USART2 stubs.
 *
 * command_receiver_process_byte() allows the actual parser to be
 * host-tested without hardware.
 */

volatile uart2_link_diag_t
    g_uart2_link_diag;


uart2_link_status_t
uart2_link_init(
    uint32_t pclk1_hz,
    uint32_t baud_rate,
    bool enable_tx,
    bool enable_rx)
{
    (void)pclk1_hz;
    (void)baud_rate;
    (void)enable_tx;
    (void)enable_rx;


    return
        UART2_LINK_OK;
}


bool
uart2_link_start_tx(
    const uint8_t *data,
    uint32_t length)
{
    (void)data;
    (void)length;


    return false;
}


bool
uart2_link_tx_busy(void)
{
    return false;
}


bool
uart2_link_read_byte(
    uint8_t *byte)
{
    (void)byte;


    return false;
}


void
uart2_link_flush_rx(void)
{
}


void
uart2_link_get_diag(
    uart2_link_diag_t *diag)
{
    if (diag !=
        (uart2_link_diag_t *)0)
    {
        *diag =
            (uart2_link_diag_t){0};
    }
}


static void
make_frame(
    uint32_t sequence,
    uint16_t m1,
    uint16_t m2,
    uint16_t m3,
    uint16_t m4,
    uint8_t *frame)
{
    motor_link_motor_command_t
        command;


    command.sequence =
        sequence;


    command.m1 =
        m1;

    command.m2 =
        m2;

    command.m3 =
        m3;

    command.m4 =
        m4;


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE) ==
        MOTOR_LINK_PROTOCOL_OK);
}


static uint32_t
feed_frame(
    const uint8_t *frame,
    uint32_t now_ms)
{
    uint32_t i;

    uint32_t accepted;


    accepted =
        0UL;


    for (i = 0UL;
         i <
         MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE;
         i++)
    {
        if (command_receiver_process_byte(
                frame[i],
                now_ms))
        {
            accepted++;
        }
    }


    return accepted;
}


static void
test_valid_frame(void)
{
    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_output_t
        output;


    command_receiver_init();


    make_frame(
        10UL,
        100U,
        200U,
        300U,
        400U,
        frame);


    assert(
        feed_frame(
            frame,
            1234UL) ==
        1UL);


    assert(
        command_receiver_get_latest(
            &output));


    assert(
        output.sequence ==
        10UL);


    assert(
        output.m1 ==
        100U);

    assert(
        output.m2 ==
        200U);

    assert(
        output.m3 ==
        300U);

    assert(
        output.m4 ==
        400U);


    assert(
        output.received_timestamp_ms ==
        1234UL);


    assert(
        output.valid);


    assert(
        g_command_receiver_stats
            .valid_frames ==
        1UL);
}


static void
test_noise_before_sync_is_discarded(void)
{
    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_init();


    assert(
        !command_receiver_process_byte(
            0x00U,
            1UL));


    assert(
        !command_receiver_process_byte(
            0x11U,
            1UL));


    assert(
        !command_receiver_process_byte(
            0xA5U,
            1UL));


    assert(
        !command_receiver_process_byte(
            0x22U,
            1UL));


    make_frame(
        20UL,
        1U,
        2U,
        3U,
        4U,
        frame);


    assert(
        feed_frame(
            frame,
            2UL) ==
        1UL);


    assert(
        g_command_receiver_stats
            .sync_discarded_bytes >=
        3UL);
}


static void
test_crc_error_is_rejected(void)
{
    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_output_t
        output;


    command_receiver_init();


    make_frame(
        30UL,
        10U,
        20U,
        30U,
        40U,
        frame);


    frame[12] ^=
        0x01U;


    assert(
        feed_frame(
            frame,
            3UL) ==
        0UL);


    assert(
        !command_receiver_get_latest(
            &output));


    assert(
        g_command_receiver_stats
            .crc_error_count ==
        1UL);
}


static void
test_duplicate_gap_and_stale_sequence_policy(void)
{
    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_output_t
        output;


    command_receiver_init();


    make_frame(
        100UL,
        1U,
        1U,
        1U,
        1U,
        frame);


    assert(
        feed_frame(
            frame,
            10UL) ==
        1UL);


    /*
     * Duplicate.
     */
    make_frame(
        100UL,
        2U,
        2U,
        2U,
        2U,
        frame);


    assert(
        feed_frame(
            frame,
            11UL) ==
        0UL);


    assert(
        g_command_receiver_stats
            .duplicate_sequence_count ==
        1UL);


    /*
     * Sequence 101 was lost.
     *
     * 102 is still the newest valid command and must be accepted.
     */
    make_frame(
        102UL,
        3U,
        3U,
        3U,
        3U,
        frame);


    assert(
        feed_frame(
            frame,
            12UL) ==
        1UL);


    assert(
        g_command_receiver_stats
            .sequence_gap_event_count ==
        1UL);


    assert(
        g_command_receiver_stats
            .missing_sequence_count ==
        1UL);


    /*
     * Late/stale 101.
     */
    make_frame(
        101UL,
        4U,
        4U,
        4U,
        4U,
        frame);


    assert(
        feed_frame(
            frame,
            13UL) ==
        0UL);


    assert(
        g_command_receiver_stats
            .stale_sequence_count ==
        1UL);


    assert(
        command_receiver_get_latest(
            &output));


    assert(
        output.sequence ==
        102UL);
}


static void
test_sequence_wrap(void)
{
    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_output_t
        output;


    command_receiver_init();


    make_frame(
        0xFFFFFFFFUL,
        1U,
        2U,
        3U,
        4U,
        frame);


    assert(
        feed_frame(
            frame,
            20UL) ==
        1UL);


    /*
     * Natural uint32_t wrap.
     */
    make_frame(
        0UL,
        5U,
        6U,
        7U,
        8U,
        frame);


    assert(
        feed_frame(
            frame,
            21UL) ==
        1UL);


    assert(
        command_receiver_get_latest(
            &output));


    assert(
        output.sequence ==
        0UL);


    assert(
        g_command_receiver_stats
            .sequence_gap_event_count ==
        0UL);
}


static void
test_dropped_byte_recovers_on_next_frame(void)
{
    uint8_t
        bad_frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    uint8_t
        good_frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_output_t
        output;


    uint32_t i;

    uint32_t accepted;


    command_receiver_init();


    make_frame(
        200UL,
        100U,
        200U,
        300U,
        400U,
        bad_frame);


    make_frame(
        201UL,
        500U,
        600U,
        700U,
        800U,
        good_frame);


    accepted =
        0UL;


    /*
     * Drop one byte in first frame.
     */
    for (i = 0UL;
         i <
         MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE;
         i++)
    {
        if (i ==
            8UL)
        {
            continue;
        }


        if (command_receiver_process_byte(
                bad_frame[i],
                30UL))
        {
            accepted++;
        }
    }


    /*
     * Immediately send the next complete frame.
     *
     * Parser should resynchronize and recover.
     */
    for (i = 0UL;
         i <
         MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE;
         i++)
    {
        if (command_receiver_process_byte(
                good_frame[i],
                31UL))
        {
            accepted++;
        }
    }


    assert(
        accepted ==
        1UL);


    assert(
        command_receiver_get_latest(
            &output));


    assert(
        output.sequence ==
        201UL);


    assert(
        g_command_receiver_stats
            .crc_error_count >=
        1UL);


    assert(
        g_command_receiver_stats
            .parser_resync_count >=
        1UL);
}


static void
test_back_to_back_frames(void)
{
    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_output_t
        output;


    command_receiver_init();


    make_frame(
        300UL,
        10U,
        20U,
        30U,
        40U,
        frame);


    assert(
        feed_frame(
            frame,
            40UL) ==
        1UL);


    make_frame(
        301UL,
        50U,
        60U,
        70U,
        80U,
        frame);


    assert(
        feed_frame(
            frame,
            41UL) ==
        1UL);


    assert(
        command_receiver_get_latest(
            &output));


    assert(
        output.sequence ==
        301UL);


    assert(
        output.m4 ==
        80U);


    assert(
        g_command_receiver_stats
            .valid_frames ==
        2UL);
}


static void
test_explicit_sequence_history_reset_accepts_new_session(void)
{
    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command_receiver_output_t
        output;


    command_receiver_init();


    /*
     * Existing FC session.
     */
    make_frame(
        5000UL,
        10U,
        20U,
        30U,
        40U,
        frame);


    assert(
        feed_frame(
            frame,
            50UL) ==
        1UL);


    /*
     * Imagine FC rebooted and restarted its sequence.
     *
     * Without an identified session break this should be stale.
     */
    make_frame(
        1UL,
        50U,
        60U,
        70U,
        80U,
        frame);


    assert(
        feed_frame(
            frame,
            51UL) ==
        0UL);


    assert(
        g_command_receiver_stats
            .stale_sequence_count ==
        1UL);


    /*
     * Later watchdog/failsafe code can declare the previous
     * communication session ended.
     */
    command_receiver_reset_sequence_history();


    assert(
        feed_frame(
            frame,
            52UL) ==
        1UL);


    assert(
        command_receiver_get_latest(
            &output));


    assert(
        output.sequence ==
        1UL);


    assert(
        g_command_receiver_stats
            .sequence_history_reset_count ==
        1UL);
}


int
main(void)
{
    test_valid_frame();

    test_noise_before_sync_is_discarded();

    test_crc_error_is_rejected();

    test_duplicate_gap_and_stale_sequence_policy();

    test_sequence_wrap();

    test_dropped_byte_recovers_on_next_frame();

    test_back_to_back_frames();

    test_explicit_sequence_history_reset_accepts_new_session();


    puts(
        "command_receiver_test: PASS");


    return 0;
}