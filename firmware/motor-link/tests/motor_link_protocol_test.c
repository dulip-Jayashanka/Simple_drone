#include "motor_link_protocol.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>


static void
test_crc_known_vector(void)
{
    static const uint8_t vector[] =
    {
        '1', '2', '3',
        '4', '5', '6',
        '7', '8', '9'
    };


    assert(
        motor_link_crc16_ccitt_false(
            vector,
            (uint32_t)sizeof(vector)) ==
        0x29B1U);
}


static motor_link_motor_command_t
make_command(void)
{
    motor_link_motor_command_t
        command;


    command.sequence =
        0x12345678UL;

    command.m1 =
        844U;

    command.m2 =
        280U;

    command.m3 =
        154U;

    command.m4 =
        723U;


    return command;
}


static void
test_round_trip_and_endian_layout(void)
{
    motor_link_motor_command_t
        input;

    motor_link_motor_command_t
        output;

    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    input =
        make_command();


    assert(
        motor_link_encode_motor_command(
            &input,
            frame,
            (uint32_t)sizeof(frame)) ==
        MOTOR_LINK_PROTOCOL_OK);


    assert(
        frame[0] ==
        0xA5U);

    assert(
        frame[1] ==
        0x5AU);

    assert(
        frame[2] ==
        0x01U);

    assert(
        frame[3] ==
        0x01U);

    assert(
        frame[4] ==
        0x0CU);


    assert(
        frame[5] ==
        0x78U);

    assert(
        frame[6] ==
        0x56U);

    assert(
        frame[7] ==
        0x34U);

    assert(
        frame[8] ==
        0x12U);


    assert(
        frame[9] ==
        (uint8_t)(
            844U &
            0xFFU));

    assert(
        frame[10] ==
        (uint8_t)(
            844U >>
            8U));


    assert(
        motor_link_decode_motor_command(
            frame,
            (uint32_t)sizeof(frame),
            &output) ==
        MOTOR_LINK_PROTOCOL_OK);


    assert(
        output.sequence ==
        input.sequence);

    assert(
        output.m1 ==
        input.m1);

    assert(
        output.m2 ==
        input.m2);

    assert(
        output.m3 ==
        input.m3);

    assert(
        output.m4 ==
        input.m4);
}


static void
test_crc_detects_corruption(void)
{
    motor_link_motor_command_t
        command;

    motor_link_motor_command_t
        decoded;

    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command =
        make_command();


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            (uint32_t)sizeof(frame)) ==
        MOTOR_LINK_PROTOCOL_OK);


    frame[12] ^=
        0x01U;


    assert(
        motor_link_decode_motor_command(
            frame,
            (uint32_t)sizeof(frame),
            &decoded) ==
        MOTOR_LINK_PROTOCOL_CRC_ERROR);
}


static void
test_header_validation(void)
{
    motor_link_motor_command_t
        command;

    motor_link_motor_command_t
        decoded;

    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command =
        make_command();


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            (uint32_t)sizeof(frame)) ==
        MOTOR_LINK_PROTOCOL_OK);


    frame[0] =
        0U;


    assert(
        motor_link_decode_motor_command(
            frame,
            (uint32_t)sizeof(frame),
            &decoded) ==
        MOTOR_LINK_PROTOCOL_SYNC_ERROR);


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            (uint32_t)sizeof(frame)) ==
        MOTOR_LINK_PROTOCOL_OK);


    frame[2] =
        2U;


    assert(
        motor_link_decode_motor_command(
            frame,
            (uint32_t)sizeof(frame),
            &decoded) ==
        MOTOR_LINK_PROTOCOL_VERSION_ERROR);


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            (uint32_t)sizeof(frame)) ==
        MOTOR_LINK_PROTOCOL_OK);


    frame[3] =
        2U;


    assert(
        motor_link_decode_motor_command(
            frame,
            (uint32_t)sizeof(frame),
            &decoded) ==
        MOTOR_LINK_PROTOCOL_TYPE_ERROR);


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            (uint32_t)sizeof(frame)) ==
        MOTOR_LINK_PROTOCOL_OK);


    frame[4] =
        11U;


    assert(
        motor_link_decode_motor_command(
            frame,
            (uint32_t)sizeof(frame),
            &decoded) ==
        MOTOR_LINK_PROTOCOL_LENGTH_ERROR);
}


static void
test_range_and_size_validation(void)
{
    motor_link_motor_command_t
        command;

    uint8_t
        frame[
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE];


    command =
        make_command();


    command.m3 =
        1001U;


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            (uint32_t)sizeof(frame)) ==
        MOTOR_LINK_PROTOCOL_RANGE_ERROR);


    command =
        make_command();


    assert(
        motor_link_encode_motor_command(
            &command,
            frame,
            MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE -
            1U) ==
        MOTOR_LINK_PROTOCOL_FRAME_SIZE_ERROR);
}


int
main(void)
{
    test_crc_known_vector();

    test_round_trip_and_endian_layout();

    test_crc_detects_corruption();

    test_header_validation();

    test_range_and_size_validation();


    puts(
        "motor_link_protocol_test: PASS");


    return 0;
}