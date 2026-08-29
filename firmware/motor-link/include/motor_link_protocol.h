#ifndef MOTOR_LINK_PROTOCOL_H
#define MOTOR_LINK_PROTOCOL_H

#include <stdint.h>

/*
 * Simple_drone flight-controller -> motor-node wire protocol.
 *
 * Version 2 keeps one fixed-length binary MOTOR_COMMAND frame and
 * adds the flight-controller's requested motor-node state beside
 * the final M1..M4 values:
 *
 *   byte 0      sync 1                 0xA5
 *   byte 1      sync 2                 0x5A
 *   byte 2      protocol version       0x02
 *   byte 3      message type           0x01 = MOTOR_COMMAND
 *   byte 4      payload length         0x0D
 *   bytes 5-8   sequence               uint32_t, little-endian
 *   bytes 9-10  M1                     uint16_t, little-endian
 *   bytes 11-12 M2                     uint16_t, little-endian
 *   bytes 13-14 M3                     uint16_t, little-endian
 *   bytes 15-16 M4                     uint16_t, little-endian
 *   byte 17     requested state        0 = DISARMED, 1 = ARMED
 *   bytes 18-19 CRC-16/CCITT-FALSE     uint16_t, little-endian
 *
 * CRC covers bytes 2 through 17 inclusive, therefore the requested
 * state is protected by the same CRC as the motor commands.
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
 * ESC electrical mapping belongs to the motor-node PWM layer.
 *
 * Only DISARMED and ARMED are transmitted. FAILSAFE is intentionally
 * local to the motor-node and can never be requested by the FC.
 */

#define MOTOR_LINK_SYNC_1                         0xA5U
#define MOTOR_LINK_SYNC_2                         0x5AU

#define MOTOR_LINK_PROTOCOL_VERSION               0x02U

#define MOTOR_LINK_MESSAGE_MOTOR_COMMAND          0x01U

#define MOTOR_LINK_MOTOR_COMMAND_PAYLOAD_LENGTH   13U
#define MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE       20U

#define MOTOR_LINK_COMMAND_MIN                    0U
#define MOTOR_LINK_COMMAND_MAX                    1000U

#define MOTOR_LINK_CRC16_POLYNOMIAL               0x1021U
#define MOTOR_LINK_CRC16_INITIAL_VALUE            0xFFFFU


typedef enum
{
    MOTOR_LINK_REQUEST_DISARMED = 0,

    MOTOR_LINK_REQUEST_ARMED = 1

} motor_link_requested_state_t;


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

    MOTOR_LINK_PROTOCOL_RANGE_ERROR,

    MOTOR_LINK_PROTOCOL_STATE_ERROR

} motor_link_protocol_status_t;


typedef struct
{
    uint32_t sequence;

    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;

    motor_link_requested_state_t
        requested_state;

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
