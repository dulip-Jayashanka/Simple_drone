#include "command_receiver.h"

#include "motor_link_protocol.h"
#include "uart2_link.h"

#include <stdbool.h>
#include <stdint.h>


volatile command_receiver_output_t
    g_latest_received_motor_command;


volatile command_receiver_stats_t
    g_command_receiver_stats;


static uint8_t
    frame_buffer[
        MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


static uint32_t
    frame_index;


static bool
    sequence_history_valid;


static uint32_t
    last_accepted_sequence;


/*
 * ============================================================
 * Parser helpers
 * ============================================================
 */

static void
parser_clear(void)
{
    frame_index =
        0UL;
}


/*
 * Try to recover synchronization after a complete but invalid
 * fixed-length frame.
 */
static void
parser_resynchronize_after_invalid_frame(void)
{
    uint32_t i;

    uint32_t j;

    uint32_t remaining;


    /*
     * Look for another complete sync sequence inside the rejected
     * frame.
     *
     * This is useful after a byte loss because part of the next
     * packet may already have entered the current 19-byte candidate.
     */
    for (i = 1UL;
         (i + 1UL) <
         frame_index;
         i++)
    {
        if ((frame_buffer[i] ==
             MOTOR_LINK_SYNC_1) &&
            (frame_buffer[
                 i + 1UL] ==
             MOTOR_LINK_SYNC_2))
        {
            remaining =
                frame_index -
                i;


            for (j = 0UL;
                 j < remaining;
                 j++)
            {
                frame_buffer[j] =
                    frame_buffer[
                        i + j];
            }


            frame_index =
                remaining;


            g_command_receiver_stats
                .parser_resync_count++;


            return;
        }
    }


    /*
     * A trailing 0xA5 could be the first byte of the next packet.
     */
    if ((frame_index !=
         0UL) &&
        (frame_buffer[
             frame_index -
             1UL] ==
         MOTOR_LINK_SYNC_1))
    {
        frame_buffer[0] =
            MOTOR_LINK_SYNC_1;


        frame_index =
            1UL;


        g_command_receiver_stats
            .parser_resync_count++;


        return;
    }


    parser_clear();
}


static void
record_protocol_error(
    motor_link_protocol_status_t status)
{
    g_command_receiver_stats
        .protocol_error_count++;


    switch (status)
    {
        case MOTOR_LINK_PROTOCOL_SYNC_ERROR:

            g_command_receiver_stats
                .sync_error_count++;

            break;


        case MOTOR_LINK_PROTOCOL_VERSION_ERROR:

            g_command_receiver_stats
                .version_error_count++;

            break;


        case MOTOR_LINK_PROTOCOL_TYPE_ERROR:

            g_command_receiver_stats
                .type_error_count++;

            break;


        case MOTOR_LINK_PROTOCOL_LENGTH_ERROR:

            g_command_receiver_stats
                .length_error_count++;

            break;


        case MOTOR_LINK_PROTOCOL_CRC_ERROR:

            g_command_receiver_stats
                .crc_error_count++;

            break;


        case MOTOR_LINK_PROTOCOL_RANGE_ERROR:

            g_command_receiver_stats
                .range_error_count++;

            break;


        default:

            break;
    }
}


/*
 * Compare uint32_t sequence numbers in modulo-2^32 space.
 *
 * delta:
 *
 *     0                       duplicate
 *
 *     1 .. 0x7FFFFFFF         newer
 *
 *     0x80000000 .. FFFFFFFF  older/stale
 *
 * This naturally accepts:
 *
 *     0xFFFFFFFF -> 0
 *
 * because the unsigned delta is 1.
 */
static bool
sequence_is_fresh(
    uint32_t candidate,
    uint32_t *delta_out)
{
    uint32_t delta;


    if (!sequence_history_valid)
    {
        if (delta_out !=
            (uint32_t *)0)
        {
            *delta_out =
                1UL;
        }


        return true;
    }


    delta =
        candidate -
        last_accepted_sequence;


    if (delta_out !=
        (uint32_t *)0)
    {
        *delta_out =
            delta;
    }


    return
        (delta != 0UL) &&
        (delta <
         0x80000000UL);
}


/*
 * Publish only a fully validated and fresh command.
 */
static void
publish_command(
    const motor_link_motor_command_t *command,
    uint32_t now_ms)
{
    command_receiver_output_t
        output;


    output.sequence =
        command->sequence;


    output.m1 =
        command->m1;

    output.m2 =
        command->m2;

    output.m3 =
        command->m3;

    output.m4 =
        command->m4;


    output.received_timestamp_ms =
        now_ms;


    output.valid =
        true;


    g_latest_received_motor_command =
        output;


    g_command_receiver_stats
        .valid_frames++;


    g_command_receiver_stats
        .last_valid_sequence =
        command->sequence;


    g_command_receiver_stats
        .last_valid_timestamp_ms =
        now_ms;


    last_accepted_sequence =
        command->sequence;


    sequence_history_valid =
        true;
}


/*
 * ============================================================
 * Public interface
 * ============================================================
 */

void
command_receiver_init(void)
{
    frame_index =
        0UL;


    sequence_history_valid =
        false;


    last_accepted_sequence =
        0UL;


    g_latest_received_motor_command =
        (command_receiver_output_t){0};


    g_command_receiver_stats =
        (command_receiver_stats_t){0};
}


void
command_receiver_reset_sequence_history(void)
{
    sequence_history_valid =
        false;


    last_accepted_sequence =
        0UL;


    g_command_receiver_stats
        .sequence_history_reset_count++;
}


bool
command_receiver_process_byte(
    uint8_t byte,
    uint32_t now_ms)
{
    motor_link_motor_command_t
        command;


    motor_link_protocol_status_t
        protocol_status;


    uint32_t
        sequence_delta;


    g_command_receiver_stats
        .bytes_processed++;


    /*
     * --------------------------------------------------------
     * Waiting for first sync byte.
     * --------------------------------------------------------
     */
    if (frame_index ==
        0UL)
    {
        if (byte !=
            MOTOR_LINK_SYNC_1)
        {
            g_command_receiver_stats
                .sync_discarded_bytes++;


            return false;
        }


        frame_buffer[0] =
            byte;


        frame_index =
            1UL;


        return false;
    }


    /*
     * --------------------------------------------------------
     * Waiting for second sync byte.
     * --------------------------------------------------------
     */
    if (frame_index ==
        1UL)
    {
        if (byte ==
            MOTOR_LINK_SYNC_2)
        {
            frame_buffer[1] =
                byte;


            frame_index =
                2UL;


            return false;
        }


        if (byte ==
            MOTOR_LINK_SYNC_1)
        {
            /*
             * Keep newest possible first-sync byte.
             */
            frame_buffer[0] =
                byte;


            g_command_receiver_stats
                .sync_discarded_bytes++;


            return false;
        }


        /*
         * Original A5 and this non-5A byte are both discarded.
         */
        g_command_receiver_stats
            .sync_discarded_bytes +=
            2UL;


        parser_clear();


        return false;
    }


    /*
     * --------------------------------------------------------
     * Collect remaining fixed-length frame bytes.
     * --------------------------------------------------------
     */
    frame_buffer[
        frame_index] =
        byte;


    frame_index++;


    if (frame_index <
        MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE)
    {
        return false;
    }


    g_command_receiver_stats
        .completed_frames++;


    protocol_status =
        motor_link_decode_motor_command(
            frame_buffer,
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE,
            &command);


    /*
     * Any malformed/corrupt packet is rejected.
     *
     * Most importantly:
     *
     * it never changes g_latest_received_motor_command.
     */
    if (protocol_status !=
        MOTOR_LINK_PROTOCOL_OK)
    {
        record_protocol_error(
            protocol_status);


        parser_resynchronize_after_invalid_frame();


        return false;
    }


    /*
     * --------------------------------------------------------
     * Sequence validation.
     * --------------------------------------------------------
     */
    if (!sequence_is_fresh(
            command.sequence,
            &sequence_delta))
    {
        g_command_receiver_stats
            .last_rejected_sequence =
            command.sequence;


        if (sequence_delta ==
            0UL)
        {
            g_command_receiver_stats
                .duplicate_sequence_count++;
        }
        else
        {
            g_command_receiver_stats
                .stale_sequence_count++;
        }


        parser_clear();


        return false;
    }


    /*
     * A missing command is diagnostic information.
     *
     * The newest correctly validated command remains more useful
     * than an older lost command, so a forward sequence gap is
     * accepted.
     */
    if (sequence_history_valid &&
        (sequence_delta >
         1UL))
    {
        g_command_receiver_stats
            .sequence_gap_event_count++;


        g_command_receiver_stats
            .missing_sequence_count +=
            sequence_delta -
            1UL;
    }


    publish_command(
        &command,
        now_ms);


    parser_clear();


    return true;
}


uint32_t
command_receiver_process(
    uint32_t now_ms)
{
    uint8_t byte;


    uint32_t
        accepted_count;


    accepted_count =
        0UL;


    /*
     * Consume every byte currently buffered by USART2 ISR.
     *
     * This code executes in normal main-loop context.
     */
    while (uart2_link_read_byte(
               &byte))
    {
        if (command_receiver_process_byte(
                byte,
                now_ms))
        {
            accepted_count++;
        }
    }


    return
        accepted_count;
}


bool
command_receiver_get_latest(
    command_receiver_output_t *output)
{
    if (output ==
        (command_receiver_output_t *)0)
    {
        return false;
    }


    if (!g_latest_received_motor_command
             .valid)
    {
        return false;
    }


    output->sequence =
        g_latest_received_motor_command
            .sequence;


    output->m1 =
        g_latest_received_motor_command
            .m1;

    output->m2 =
        g_latest_received_motor_command
            .m2;

    output->m3 =
        g_latest_received_motor_command
            .m3;

    output->m4 =
        g_latest_received_motor_command
            .m4;


    output->received_timestamp_ms =
        g_latest_received_motor_command
            .received_timestamp_ms;


    output->valid =
        g_latest_received_motor_command
            .valid;


    return true;
}