#ifndef MOTOR_LINK_PROTOCOL_H
#define MOTOR_LINK_PROTOCOL_H

#include <stdint.h>

/*
 * Simple_drone flight-controller -> motor-node wire protocol.
 *
 * Version 1 uses one fixed-length binary MOTOR_COMMAND frame:
 *
 *   byte 0      sync 1                 0xA5
 *   byte 1      sync 2                 0x5A
 *   byte 2      protocol version       0x01
 *   byte 3      message type           0x01 = MOTOR_COMMAND
 *   byte 4      payload length         0x0C
 *   bytes 5-8   sequence               uint32_t, little-endian
 *   bytes 9-10  M1                     uint16_t, little-endian
 *   bytes 11-12 M2                     uint16_t, little-endian
 *   bytes 13-14 M3                     uint16_t, little-endian
 *   bytes 15-16 M4                     uint16_t, little-endian
 *   bytes 17-18 CRC-16/CCITT-FALSE     uint16_t, little-endian
 *
 * CRC covers bytes 2 through 16 inclusive.
 *
 * The sync bytes and the CRC field itself are not included in CRC.
 *
 * Motor command representation:
 *
 *      0    -> normalized command 0.000
 *      1000 -> normalized command 1.000
 *
 * These values are NOT ESC PWM pulse widths.
 *
 * ESC electrical mapping belongs to the later motor-node PWM layer.
 */

#define MOTOR_LINK_SYNC_1                         0xA5U
#define MOTOR_LINK_SYNC_2                         0x5AU

#define MOTOR_LINK_PROTOCOL_VERSION               0x01U

#define MOTOR_LINK_MESSAGE_MOTOR_COMMAND          0x01U

#define MOTOR_LINK_MOTOR_COMMAND_PAYLOAD_LENGTH   12U
#define MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE       19U

#define MOTOR_LINK_COMMAND_MIN                    0U
#define MOTOR_LINK_COMMAND_MAX                    1000U

#define MOTOR_LINK_CRC16_POLYNOMIAL               0x1021U
#define MOTOR_LINK_CRC16_INITIAL_VALUE            0xFFFFU


typedef enum
{
    MOTOR_LINK_PROTOCOL_OK = 0,

    MOTOR_LINK_PROTOCOL_INVALID_ARGUMENT,

    MOTOR_LINK_PROTOCOL_FRAME_SIZE_ERROR,

    MOTOR_LINK_PROTOCOL_SYNC_ERROR,

    MOTOR_LINK_PROTOCOL_VERSION_ERROR,

    MOTOR_LINK_PROTOCOL_TYPE_ERROR,

    MOTOR_LINK_PROTOCOL_LENGTH_ERROR,

    MOTOR_LINK_PROTOCOL_CRC_ERROR,

    MOTOR_LINK_PROTOCOL_RANGE_ERROR

} motor_link_protocol_status_t;


typedef struct
{
    uint32_t sequence;

    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;

} motor_link_motor_command_t;


/*
 * CRC-16/CCITT-FALSE:
 *
 * polynomial = 0x1021
 * initial    = 0xFFFF
 * refin      = false
 * refout     = false
 * xorout     = 0x0000
 */
uint16_t
motor_link_crc16_ccitt_false(
    const uint8_t *data,
    uint32_t length);


/*
 * Encode one complete fixed-length MOTOR_COMMAND frame.
 */
motor_link_protocol_status_t
motor_link_encode_motor_command(
    const motor_link_motor_command_t *command,
    uint8_t *frame,
    uint32_t frame_capacity);


/*
 * Decode and validate one complete fixed-length MOTOR_COMMAND frame.
 */
motor_link_protocol_status_t
motor_link_decode_motor_command(
    const uint8_t *frame,
    uint32_t frame_length,
    motor_link_motor_command_t *command);


#endif /* MOTOR_LINK_PROTOCOL_H */