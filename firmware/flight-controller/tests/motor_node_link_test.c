#include "motor_node_link.h"

#include "motor_link_protocol.h"
#include "uart2_link.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


/*
 * Host-test stubs for the STM32-specific USART2 transport.
 */

static bool
    stub_tx_busy;

static bool
    stub_start_tx_success;


static uint8_t
    captured_frame[
        UART2_LINK_TX_BUFFER_CAPACITY];


static uint32_t
    captured_length;


static uint32_t
    init_pclk1;

static uint32_t
    init_baud;

static bool
    init_tx;

static bool
    init_rx;


volatile uart2_link_diag_t
    g_uart2_link_diag;


uart2_link_status_t
uart2_link_init(
    uint32_t pclk1_hz,
    uint32_t baud_rate,
    bool enable_tx,
    bool enable_rx)
{
    init_pclk1 =
        pclk1_hz;

    init_baud =
        baud_rate;

    init_tx =
        enable_tx;

    init_rx =
        enable_rx;


    return
        UART2_LINK_OK;
}


bool
uart2_link_start_tx(
    const uint8_t *data,
    uint32_t length)
{
    uint32_t i;


    if ((!stub_start_tx_success) ||
        (data ==
         (const uint8_t *)0) ||
        (length >
         UART2_LINK_TX_BUFFER_CAPACITY))
    {
        return false;
    }


    captured_length =
        length;


    for (i = 0UL;
         i < length;
         i++)
    {
        captured_frame[i] =
            data[i];
    }


    return true;
}


bool
uart2_link_tx_busy(void)
{
    return
        stub_tx_busy;
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


static motor_mixer_output_t
make_valid_output(void)
{
    motor_mixer_output_t
        output;


    output =
        (motor_mixer_output_t){0};


    output.sequence =
        77UL;


    output.flags =
        MOTOR_MIXER_VALID;


    output.m1 =
        0.844f;

    output.m2 =
        0.280f;

    output.m3 =
        0.154f;

    output.m4 =
        0.723f;


    return output;
}


static void
reset_stubs(void)
{
    uint32_t i;


    stub_tx_busy =
        false;

    stub_start_tx_success =
        true;


    captured_length =
        0UL;


    init_pclk1 =
        0UL;

    init_baud =
        0UL;

    init_tx =
        false;

    init_rx =
        false;


    for (i = 0UL;
         i <
         UART2_LINK_TX_BUFFER_CAPACITY;
         i++)
    {
        captured_frame[i] =
            0U;
    }
}


static void
test_initialization(void)
{
    reset_stubs();


    assert(
        motor_node_link_init(
            36000000UL,
            230400UL) ==
        MOTOR_NODE_LINK_OK);


    assert(
        motor_node_link_is_ready());


    assert(
        init_pclk1 ==
        36000000UL);


    assert(
        init_baud ==
        230400UL);


    assert(
        init_tx);


    assert(
        !init_rx);
}


static void
test_valid_mixer_output_is_encoded(void)
{
    motor_mixer_output_t
        output;

    motor_link_motor_command_t
        decoded;


    reset_stubs();


    assert(
        motor_node_link_init(
            36000000UL,
            230400UL) ==
        MOTOR_NODE_LINK_OK);


    output =
        make_valid_output();


    assert(
        motor_node_link_send(
            &output) ==
        MOTOR_NODE_LINK_OK);


    assert(
        captured_length ==
        MOTOR_LINK_MOTOR_COMMAND_FRAME_SIZE);


    assert(
        motor_link_decode_motor_command(
            captured_frame,
            captured_length,
            &decoded) ==
        MOTOR_LINK_PROTOCOL_OK);


    assert(
        decoded.sequence ==
        77UL);


    assert(
        decoded.m1 ==
        844U);

    assert(
        decoded.m2 ==
        280U);

    assert(
        decoded.m3 ==
        154U);

    assert(
        decoded.m4 ==
        723U);


    assert(
        g_motor_node_link_diag
            .sent_frame_count ==
        1UL);


    assert(
        g_motor_node_link_diag
            .last_sent_sequence ==
        77UL);
}


static void
test_invalid_mixer_output_is_rejected(void)
{
    motor_mixer_output_t
        output;


    reset_stubs();


    assert(
        motor_node_link_init(
            36000000UL,
            230400UL) ==
        MOTOR_NODE_LINK_OK);


    output =
        make_valid_output();


    output.flags =
        0UL;


    assert(
        motor_node_link_send(
            &output) ==
        MOTOR_NODE_LINK_MIXER_INVALID);


    assert(
        captured_length ==
        0UL);


    output =
        make_valid_output();


    output.m4 =
        1.1f;


    assert(
        motor_node_link_send(
            &output) ==
        MOTOR_NODE_LINK_COMMAND_INVALID);


    assert(
        captured_length ==
        0UL);
}


static void
test_busy_transfer_drops_fresh_command_without_blocking(void)
{
    motor_mixer_output_t
        output;


    reset_stubs();


    assert(
        motor_node_link_init(
            36000000UL,
            230400UL) ==
        MOTOR_NODE_LINK_OK);


    output =
        make_valid_output();


    stub_tx_busy =
        true;


    assert(
        motor_node_link_send(
            &output) ==
        MOTOR_NODE_LINK_TX_BUSY);


    assert(
        g_motor_node_link_diag
            .tx_busy_drop_count ==
        1UL);


    assert(
        captured_length ==
        0UL);
}


static void
test_uart_start_failure_is_reported(void)
{
    motor_mixer_output_t
        output;


    reset_stubs();


    assert(
        motor_node_link_init(
            36000000UL,
            230400UL) ==
        MOTOR_NODE_LINK_OK);


    output =
        make_valid_output();


    stub_start_tx_success =
        false;


    assert(
        motor_node_link_send(
            &output) ==
        MOTOR_NODE_LINK_TX_FAILED);


    assert(
        g_motor_node_link_diag
            .tx_failure_count ==
        1UL);
}


int
main(void)
{
    test_initialization();

    test_valid_mixer_output_is_encoded();

    test_invalid_mixer_output_is_rejected();

    test_busy_transfer_drops_fresh_command_without_blocking();

    test_uart_start_failure_is_reported();


    puts(
        "motor_node_link_test: PASS");


    return 0;
}