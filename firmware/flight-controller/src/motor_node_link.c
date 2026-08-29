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


static motor_link_requested_state_t
    motor_node_link_requested_state;


static uint32_t
    arm_guard_frames_remaining;


#if MOTOR_ARM_BENCH_TEST

static bool
    motor_arm_bench_request_issued;

#endif


static bool
normalized_motor_value_is_valid(
    float value)
{
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


static bool
requested_state_is_valid(
    motor_link_requested_state_t state)
{
    return
        (state == MOTOR_LINK_REQUEST_DISARMED) ||
        (state == MOTOR_LINK_REQUEST_ARMED);
}


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


    motor_node_link_requested_state =
        MOTOR_LINK_REQUEST_DISARMED;


    arm_guard_frames_remaining =
        0UL;


#if MOTOR_ARM_BENCH_TEST

    motor_arm_bench_request_issued =
        false;

#endif


    g_motor_node_link_diag =
        (motor_node_link_diag_t){0};


    g_motor_node_link_diag
        .requested_state =
        (uint32_t)
        MOTOR_LINK_REQUEST_DISARMED;


    if ((pclk1_hz == 0UL) ||
        (baud_rate == 0UL))
    {
        return
            record_status(
                MOTOR_NODE_LINK_INVALID_ARGUMENT);
    }


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
motor_node_link_set_requested_state(
    motor_link_requested_state_t requested_state)
{
    if (!motor_node_link_ready)
    {
        return
            record_status(
                MOTOR_NODE_LINK_NOT_READY);
    }


    if (!requested_state_is_valid(
            requested_state))
    {
        g_motor_node_link_diag
            .state_invalid_count++;


        return
            record_status(
                MOTOR_NODE_LINK_STATE_INVALID);
    }


#if MOTOR_ARM_BENCH_TEST

    /*
     * Any ARM request consumes the one-shot bench ARM source.
     *
     * Therefore a later explicit DISARM can never be followed by an
     * unexpected automatic re-arm from this temporary test mode.
     */
    if (requested_state ==
        MOTOR_LINK_REQUEST_ARMED)
    {
        motor_arm_bench_request_issued =
            true;


        g_motor_node_link_diag
            .bench_arm_request_issued =
            1UL;
    }

#endif


    if (requested_state ==
        motor_node_link_requested_state)
    {
        return
            record_status(
                MOTOR_NODE_LINK_OK);
    }


    motor_node_link_requested_state =
        requested_state;


    g_motor_node_link_diag
        .requested_state =
        (uint32_t)
        requested_state;


    g_motor_node_link_diag
        .state_change_count++;


    if (requested_state ==
        MOTOR_LINK_REQUEST_ARMED)
    {
        arm_guard_frames_remaining =
            (uint32_t)
            MOTOR_NODE_LINK_ARM_GUARD_FRAMES;
    }
    else
    {
        arm_guard_frames_remaining =
            0UL;
    }


    g_motor_node_link_diag
        .arm_guard_frames_remaining =
        arm_guard_frames_remaining;


    return
        record_status(
            MOTOR_NODE_LINK_OK);
}


motor_link_requested_state_t
motor_node_link_get_requested_state(void)
{
    return
        motor_node_link_requested_state;
}


motor_node_link_status_t
motor_node_link_send(
    const motor_mixer_output_t *mixer_output)
{
    motor_link_motor_command_t
        command;

    motor_link_protocol_status_t
        protocol_status;

    bool
        zero_motor_command;

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


    if (!requested_state_is_valid(
            motor_node_link_requested_state))
    {
        g_motor_node_link_diag
            .state_invalid_count++;


        return
            record_status(
                MOTOR_NODE_LINK_STATE_INVALID);
    }


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


    command.requested_state =
        motor_node_link_requested_state;


    zero_motor_command =
        (motor_node_link_requested_state ==
         MOTOR_LINK_REQUEST_DISARMED) ||
        ((motor_node_link_requested_state ==
          MOTOR_LINK_REQUEST_ARMED) &&
         (arm_guard_frames_remaining !=
          0UL));


    if (zero_motor_command)
    {
        command.m1 = 0U;
        command.m2 = 0U;
        command.m3 = 0U;
        command.m4 = 0U;
    }
    else
    {
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
    }


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


    g_motor_node_link_diag
        .last_sent_requested_state =
        (uint32_t)
        command.requested_state;


    if (command.requested_state ==
        MOTOR_LINK_REQUEST_DISARMED)
    {
        g_motor_node_link_diag
            .disarmed_zero_frame_count++;
    }
    else if (arm_guard_frames_remaining !=
             0UL)
    {
        g_motor_node_link_diag
            .arm_guard_zero_frame_count++;


        arm_guard_frames_remaining--;
    }


    g_motor_node_link_diag
        .arm_guard_frames_remaining =
        arm_guard_frames_remaining;


#if MOTOR_ARM_BENCH_TEST

    /*
     * Temporary one-shot pilot substitute for Phase 6.3 bench work.
     *
     * Count only successful DISARMED transmissions. A busy DMA or TX
     * failure therefore cannot shorten the safe DISARMED interval.
     *
     * The transition is requested only after the current DISARMED
     * frame has started transmitting, so the next packet is the first
     * ARMED + zero-command guard packet.
     */
    if ((!motor_arm_bench_request_issued) &&
        (command.requested_state ==
         MOTOR_LINK_REQUEST_DISARMED) &&
        (g_motor_node_link_diag
             .disarmed_zero_frame_count >=
         (uint32_t)
         MOTOR_ARM_BENCH_DISARMED_FRAMES))
    {
        uint32_t
            trigger_count;


        trigger_count =
            g_motor_node_link_diag
                .disarmed_zero_frame_count;


        if (motor_node_link_set_requested_state(
                MOTOR_LINK_REQUEST_ARMED) ==
            MOTOR_NODE_LINK_OK)
        {
            g_motor_node_link_diag
                .bench_arm_request_count++;


            g_motor_node_link_diag
                .bench_arm_trigger_disarmed_frame_count =
                trigger_count;
        }
    }

#endif


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
