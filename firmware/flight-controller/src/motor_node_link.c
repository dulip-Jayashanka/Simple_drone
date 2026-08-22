#include "motor_node_link.h"

#include "motor_link_protocol.h"
#include "uart2_link.h"

#include <stdbool.h>
#include <stdint.h>


_Static_assert(
    UART2_LINK_TX_BUFFER_CAPACITY >=
    MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE,
    "USART2 TX buffer must hold one complete motor-command frame");


volatile motor_node_link_status_t
    g_motor_node_link_status;


volatile motor_node_link_diag_t
    g_motor_node_link_diag;


static bool
    motor_node_link_ready;


/*
 * Mixer values sent over this link must already be finite and
 * inside the mixer's normalized output interval.
 */
static bool
normalized_motor_value_is_valid(
    float value)
{
    /*
     * NaN is the only floating-point value unequal to itself.
     */
    if (value != value)
    {
        return false;
    }


    return
        (value >=
         MOTOR_MIXER_COMMAND_MIN) &&
        (value <=
         MOTOR_MIXER_COMMAND_MAX);
}


/*
 * Convert:
 *
 *     0.000 -> 0
 *     0.500 -> 500
 *     1.000 -> 1000
 *
 * with nearest-integer rounding.
 */
static uint16_t
normalized_to_wire_command(
    float value)
{
    uint32_t scaled;


    scaled =
        (uint32_t)(
            (value *
             (float)
             MOTOR_LINK_COMMAND_MAX) +
            0.5f);


    /*
     * Numerical guard.
     */
    if (scaled >
        MOTOR_LINK_COMMAND_MAX)
    {
        scaled =
            MOTOR_LINK_COMMAND_MAX;
    }


    return
        (uint16_t)
        scaled;
}


static motor_node_link_status_t
record_status(
    motor_node_link_status_t status)
{
    g_motor_node_link_status =
        status;


    g_motor_node_link_diag
        .last_status =
        (uint32_t)
        status;


    return status;
}


motor_node_link_status_t
motor_node_link_init(
    uint32_t pclk1_hz,
    uint32_t baud_rate)
{
    uart2_link_status_t
        uart_status;


    motor_node_link_ready =
        false;


    g_motor_node_link_diag =
        (motor_node_link_diag_t){0};


    if ((pclk1_hz == 0UL) ||
        (baud_rate == 0UL))
    {
        return
            record_status(
                MOTOR_NODE_LINK_INVALID_ARGUMENT);
    }


    /*
     * Phase 6.1 is one-way:
     *
     * flight-controller TX -> motor-node RX.
     *
     * PA2 is therefore enabled here.
     *
     * PA3 remains free for the future reverse status channel.
     */
    uart_status =
        uart2_link_init(
            pclk1_hz,
            baud_rate,
            true,
            false);


    if (uart_status !=
        UART2_LINK_OK)
    {
        return
            record_status(
                MOTOR_NODE_LINK_UART_INIT_FAILED);
    }


    motor_node_link_ready =
        true;


    g_motor_node_link_diag
        .init_count =
        1UL;


    return
        record_status(
            MOTOR_NODE_LINK_OK);
}


motor_node_link_status_t
motor_node_link_send(
    const motor_mixer_output_t *mixer_output)
{
    motor_link_motor_command_t
        command;

    motor_link_protocol_status_t
        protocol_status;

    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    g_motor_node_link_diag
        .send_attempt_count++;


    if (!motor_node_link_ready)
    {
        return
            record_status(
                MOTOR_NODE_LINK_NOT_READY);
    }


    if (mixer_output ==
        (const motor_mixer_output_t *)0)
    {
        g_motor_node_link_diag
            .command_invalid_count++;


        return
            record_status(
                MOTOR_NODE_LINK_INVALID_ARGUMENT);
    }


    g_motor_node_link_diag
        .last_attempted_sequence =
        mixer_output->sequence;


    if ((mixer_output->flags &
         MOTOR_MIXER_VALID) ==
        0UL)
    {
        g_motor_node_link_diag
            .mixer_invalid_count++;


        return
            record_status(
                MOTOR_NODE_LINK_MIXER_INVALID);
    }


    if ((!normalized_motor_value_is_valid(
             mixer_output->m1)) ||
        (!normalized_motor_value_is_valid(
             mixer_output->m2)) ||
        (!normalized_motor_value_is_valid(
             mixer_output->m3)) ||
        (!normalized_motor_value_is_valid(
             mixer_output->m4)))
    {
        g_motor_node_link_diag
            .command_invalid_count++;


        return
            record_status(
                MOTOR_NODE_LINK_COMMAND_INVALID);
    }


    /*
     * Do not block the stabilization path waiting for UART.
     *
     * Also do not create a queue of old motor commands.
     */
    if (uart2_link_tx_busy())
    {
        g_motor_node_link_diag
            .tx_busy_drop_count++;


        return
            record_status(
                MOTOR_NODE_LINK_TX_BUSY);
    }


    command.sequence =
        mixer_output->sequence;


    command.m1 =
        normalized_to_wire_command(
            mixer_output->m1);

    command.m2 =
        normalized_to_wire_command(
            mixer_output->m2);

    command.m3 =
        normalized_to_wire_command(
            mixer_output->m3);

    command.m4 =
        normalized_to_wire_command(
            mixer_output->m4);


    protocol_status =
        motor_link_encode_motor_command(
            &command,
            frame,
            (uint32_t)
            sizeof(frame));


    if (protocol_status !=
        MOTOR_LINK_PROTOCOL_OK)
    {
        g_motor_node_link_diag
            .encode_error_count++;


        return
            record_status(
                MOTOR_NODE_LINK_ENCODE_FAILED);
    }


    if (!uart2_link_start_tx(
            frame,
            (uint32_t)
            sizeof(frame)))
    {
        g_motor_node_link_diag
            .tx_failure_count++;


        return
            record_status(
                MOTOR_NODE_LINK_TX_FAILED);
    }


    g_motor_node_link_diag
        .sent_frame_count++;


    g_motor_node_link_diag
        .last_sent_sequence =
        command.sequence;


    g_motor_node_link_diag
        .last_m1 =
        command.m1;

    g_motor_node_link_diag
        .last_m2 =
        command.m2;

    g_motor_node_link_diag
        .last_m3 =
        command.m3;

    g_motor_node_link_diag
        .last_m4 =
        command.m4;


    return
        record_status(
            MOTOR_NODE_LINK_OK);
}


bool
motor_node_link_is_ready(void)
{
    return
        motor_node_link_ready;
}