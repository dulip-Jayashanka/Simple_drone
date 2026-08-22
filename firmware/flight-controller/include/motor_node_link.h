#ifndef MOTOR_NODE_LINK_H
#define MOTOR_NODE_LINK_H

#include "motor_mixer.h"

#include <stdbool.h>
#include <stdint.h>


typedef enum
{
    MOTOR_NODE_LINK_OK = 0,

    MOTOR_NODE_LINK_INVALID_ARGUMENT,

    MOTOR_NODE_LINK_UART_INIT_FAILED,

    MOTOR_NODE_LINK_NOT_READY,

    MOTOR_NODE_LINK_MIXER_INVALID,

    MOTOR_NODE_LINK_COMMAND_INVALID,

    MOTOR_NODE_LINK_ENCODE_FAILED,

    MOTOR_NODE_LINK_TX_BUSY,

    MOTOR_NODE_LINK_TX_FAILED

} motor_node_link_status_t;


typedef struct
{
    uint32_t init_count;

    uint32_t send_attempt_count;

    uint32_t sent_frame_count;

    uint32_t mixer_invalid_count;

    uint32_t command_invalid_count;

    uint32_t encode_error_count;

    uint32_t tx_busy_drop_count;

    uint32_t tx_failure_count;


    uint32_t last_attempted_sequence;

    uint32_t last_sent_sequence;


    uint16_t last_m1;
    uint16_t last_m2;
    uint16_t last_m3;
    uint16_t last_m4;


    uint32_t last_status;

} motor_node_link_diag_t;


extern volatile motor_node_link_status_t
    g_motor_node_link_status;


extern volatile motor_node_link_diag_t
    g_motor_node_link_diag;


/*
 * Initialize dedicated USART2 TX communication to the motor-node.
 */
motor_node_link_status_t
motor_node_link_init(
    uint32_t pclk1_hz,
    uint32_t baud_rate);


/*
 * Convert a valid normalized motor-mixer output into one fixed
 * binary MOTOR_COMMAND packet and request a non-blocking USART2
 * DMA transmission.
 *
 * There is deliberately no backlog of old motor commands.
 *
 * If the previous frame has not finished, the current frame is
 * dropped and the next fresh mixer update gets another opportunity.
 */
motor_node_link_status_t
motor_node_link_send(
    const motor_mixer_output_t *mixer_output);


bool
motor_node_link_is_ready(void);


#endif /* MOTOR_NODE_LINK_H */