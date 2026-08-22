#include "motor_link_protocol.h"

#include <stdint.h>


#define MOTOR_LINK_OFFSET_SYNC_1       0U
#define MOTOR_LINK_OFFSET_SYNC_2       1U

#define MOTOR_LINK_OFFSET_VERSION      2U
#define MOTOR_LINK_OFFSET_TYPE         3U
#define MOTOR_LINK_OFFSET_LENGTH       4U

#define MOTOR_LINK_OFFSET_SEQUENCE     5U

#define MOTOR_LINK_OFFSET_M1           9U
#define MOTOR_LINK_OFFSET_M2           11U
#define MOTOR_LINK_OFFSET_M3           13U
#define MOTOR_LINK_OFFSET_M4           15U

#define MOTOR_LINK_OFFSET_CRC          17U


#define MOTOR_LINK_CRC_START_OFFSET    \
    MOTOR_LINK_OFFSET_VERSION

/*
 * Bytes:
 *
 * 2..16 inclusive = 15 bytes.
 */
#define MOTOR_LINK_CRC_INPUT_LENGTH    15U


static void
write_u16_le(
    uint8_t *destination,
    uint16_t value)
{
    destination[0] =
        (uint8_t)(
            value &
            0x00FFU);

    destination[1] =
        (uint8_t)(
            (value >> 8U) &
            0x00FFU);
}


static void
write_u32_le(
    uint8_t *destination,
    uint32_t value)
{
    destination[0] =
        (uint8_t)(
            value &
            0x000000FFUL);

    destination[1] =
        (uint8_t)(
            (value >> 8U) &
            0x000000FFUL);

    destination[2] =
        (uint8_t)(
            (value >> 16U) &
            0x000000FFUL);

    destination[3] =
        (uint8_t)(
            (value >> 24U) &
            0x000000FFUL);
}


static uint16_t
read_u16_le(
    const uint8_t *source)
{
    return
        (uint16_t)(
            (uint16_t)source[0] |
            ((uint16_t)source[1] << 8U));
}


static uint32_t
read_u32_le(
    const uint8_t *source)
{
    return
        (uint32_t)source[0] |
        ((uint32_t)source[1] << 8U) |
        ((uint32_t)source[2] << 16U) |
        ((uint32_t)source[3] << 24U);
}


static int
command_values_in_range(
    const motor_link_motor_command_t *command)
{
    if (command ==
        (const motor_link_motor_command_t *)0)
    {
        return 0;
    }


    return
        (command->m1 <= MOTOR_LINK_COMMAND_MAX) &&
        (command->m2 <= MOTOR_LINK_COMMAND_MAX) &&
        (command->m3 <= MOTOR_LINK_COMMAND_MAX) &&
        (command->m4 <= MOTOR_LINK_COMMAND_MAX);
}


uint16_t
motor_link_crc16_ccitt_false(
    const uint8_t *data,
    uint32_t length)
{
    uint16_t crc;

    uint32_t byte_index;
    uint32_t bit_index;


    crc =
        MOTOR_LINK_CRC16_INITIAL_VALUE;


    if ((data ==
         (const uint8_t *)0) &&
        (length != 0UL))
    {
        return 0U;
    }


    for (byte_index = 0UL;
         byte_index < length;
         byte_index++)
    {
        crc ^=
            (uint16_t)(
                (uint16_t)data[byte_index]
                << 8U);


        for (bit_index = 0UL;
             bit_index < 8UL;
             bit_index++)
        {
            if ((crc &
                 0x8000U) != 0U)
            {
                crc =
                    (uint16_t)(
                        (crc << 1U) ^
                        MOTOR_LINK_CRC16_POLYNOMIAL);
            }
            else
            {
                crc =
                    (uint16_t)(
                        crc << 1U);
            }
        }
    }


    return crc;
}


motor_link_protocol_status_t
motor_link_encode_motor_command(
    const motor_link_motor_command_t *command,
    uint8_t *frame,
    uint32_t frame_capacity)
{
    uint16_t crc;


    if ((command ==
         (const motor_link_motor_command_t *)0) ||
        (frame ==
         (uint8_t *)0))
    {
        return
            MOTOR_LINK_PROTOCOL_INVALID_ARGUMENT;
    }


    if (frame_capacity <
        MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE)
    {
        return
            MOTOR_LINK_PROTOCOL_FRAME_SIZE_ERROR;
    }


    if (!command_values_in_range(
            command))
    {
        return
            MOTOR_LINK_PROTOCOL_RANGE_ERROR;
    }


    frame[MOTOR_LINK_OFFSET_SYNC_1] =
        MOTOR_LINK_SYNC_1;

    frame[MOTOR_LINK_OFFSET_SYNC_2] =
        MOTOR_LINK_SYNC_2;

    frame[MOTOR_LINK_OFFSET_VERSION] =
        MOTOR_LINK_PROTOCOL_VERSION;

    frame[MOTOR_LINK_OFFSET_TYPE] =
        MOTOR_LINK_MESSAGE_MOTOR_COMMAND;

    frame[MOTOR_LINK_OFFSET_LENGTH] =
        MOTOR_LINK_MOTOR_COMMAND_PAYLOAD_LENGTH;


    write_u32_le(
        &frame[
            MOTOR_LINK_OFFSET_SEQUENCE],
        command->sequence);


    write_u16_le(
        &frame[
            MOTOR_LINK_OFFSET_M1],
        command->m1);

    write_u16_le(
        &frame[
            MOTOR_LINK_OFFSET_M2],
        command->m2);

    write_u16_le(
        &frame[
            MOTOR_LINK_OFFSET_M3],
        command->m3);

    write_u16_le(
        &frame[
            MOTOR_LINK_OFFSET_M4],
        command->m4);


    crc =
        motor_link_crc16_ccitt_false(
            &frame[
                MOTOR_LINK_CRC_START_OFFSET],
            MOTOR_LINK_CRC_INPUT_LENGTH);


    write_u16_le(
        &frame[
            MOTOR_LINK_OFFSET_CRC],
        crc);


    return
        MOTOR_LINK_PROTOCOL_OK;
}


motor_link_protocol_status_t
motor_link_decode_motor_command(
    const uint8_t *frame,
    uint32_t frame_length,
    motor_link_motor_command_t *command)
{
    motor_link_motor_command_t
        candidate;

    uint16_t
        received_crc;

    uint16_t
        calculated_crc;


    if ((frame ==
         (const uint8_t *)0) ||
        (command ==
         (motor_link_motor_command_t *)0))
    {
        return
            MOTOR_LINK_PROTOCOL_INVALID_ARGUMENT;
    }


    if (frame_length !=
        MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE)
    {
        return
            MOTOR_LINK_PROTOCOL_FRAME_SIZE_ERROR;
    }


    if ((frame[
            MOTOR_LINK_OFFSET_SYNC_1] !=
         MOTOR_LINK_SYNC_1) ||
        (frame[
            MOTOR_LINK_OFFSET_SYNC_2] !=
         MOTOR_LINK_SYNC_2))
    {
        return
            MOTOR_LINK_PROTOCOL_SYNC_ERROR;
    }


    if (frame[
            MOTOR_LINK_OFFSET_VERSION] !=
        MOTOR_LINK_PROTOCOL_VERSION)
    {
        return
            MOTOR_LINK_PROTOCOL_VERSION_ERROR;
    }


    if (frame[
            MOTOR_LINK_OFFSET_TYPE] !=
        MOTOR_LINK_MESSAGE_MOTOR_COMMAND)
    {
        return
            MOTOR_LINK_PROTOCOL_TYPE_ERROR;
    }


    if (frame[
            MOTOR_LINK_OFFSET_LENGTH] !=
        MOTOR_LINK_MOTOR_COMMAND_PAYLOAD_LENGTH)
    {
        return
            MOTOR_LINK_PROTOCOL_LENGTH_ERROR;
    }


    received_crc =
        read_u16_le(
            &frame[
                MOTOR_LINK_OFFSET_CRC]);


    calculated_crc =
        motor_link_crc16_ccitt_false(
            &frame[
                MOTOR_LINK_CRC_START_OFFSET],
            MOTOR_LINK_CRC_INPUT_LENGTH);


    if (received_crc !=
        calculated_crc)
    {
        return
            MOTOR_LINK_PROTOCOL_CRC_ERROR;
    }


    candidate.sequence =
        read_u32_le(
            &frame[
                MOTOR_LINK_OFFSET_SEQUENCE]);


    candidate.m1 =
        read_u16_le(
            &frame[
                MOTOR_LINK_OFFSET_M1]);

    candidate.m2 =
        read_u16_le(
            &frame[
                MOTOR_LINK_OFFSET_M2]);

    candidate.m3 =
        read_u16_le(
            &frame[
                MOTOR_LINK_OFFSET_M3]);

    candidate.m4 =
        read_u16_le(
            &frame[
                MOTOR_LINK_OFFSET_M4]);


    if (!command_values_in_range(
            &candidate))
    {
        return
            MOTOR_LINK_PROTOCOL_RANGE_ERROR;
    }


    *command =
        candidate;


    return
        MOTOR_LINK_PROTOCOL_OK;
}