#ifndef UART2_LINK_H
#define UART2_LINK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Dedicated USART2 transport for the flight-controller <-> motor-node
 * binary communication link.
 *
 * STM32F103C8 default USART2 pins:
 *
 *     PA2 = USART2_TX
 *     PA3 = USART2_RX
 *
 * USART1 remains owned by the existing uart_diag module.
 */

#define UART2_LINK_TX_BUFFER_CAPACITY  32U
#define UART2_LINK_RX_BUFFER_CAPACITY  128U


typedef enum
{
    UART2_LINK_OK = 0,

    UART2_LINK_INVALID_ARGUMENT,

    UART2_LINK_BAUD_OUT_OF_RANGE,

    UART2_LINK_CONFIGURATION_ERROR

} uart2_link_status_t;


typedef struct
{
    uint32_t init_count;

    uint32_t tx_start_count;

    uint32_t tx_complete_count;

    uint32_t tx_busy_reject_count;

    uint32_t tx_invalid_reject_count;

    uint32_t tx_dma_error_count;

    uint32_t last_tx_length;


    uint32_t rx_byte_count;

    uint32_t rx_buffer_overflow_count;

    uint32_t rx_parity_error_count;

    uint32_t rx_framing_error_count;

    uint32_t rx_noise_error_count;

    uint32_t rx_overrun_error_count;

    uint32_t last_rx_byte;

} uart2_link_diag_t;


extern volatile uart2_link_diag_t
    g_uart2_link_diag;


/*
 * Initialize USART2 for the requested direction(s).
 *
 * At least one of enable_tx or enable_rx must be true.
 *
 * TX:
 *     USART2 + DMA1 Channel 7.
 *
 * RX:
 *     USART2 RXNE interrupt +
 *     small circular software buffer.
 */
uart2_link_status_t
uart2_link_init(
    uint32_t pclk1_hz,
    uint32_t baud_rate,
    bool enable_tx,
    bool enable_rx);


/*
 * Start one non-blocking DMA transmission.
 *
 * The source is copied to the driver's internal DMA buffer before
 * this function returns.
 */
bool
uart2_link_start_tx(
    const uint8_t *data,
    uint32_t length);


/*
 * True while a USART2 TX DMA request is active.
 */
bool
uart2_link_tx_busy(void);


/*
 * Remove one byte from the RX circular buffer.
 *
 * Returns false when no byte is available.
 */
bool
uart2_link_read_byte(
    uint8_t *byte);


/*
 * Discard every currently buffered receive byte.
 */
void
uart2_link_flush_rx(void);


/*
 * Copy transport diagnostics.
 */
void
uart2_link_get_diag(
    uart2_link_diag_t *diag);


#endif /* UART2_LINK_H */