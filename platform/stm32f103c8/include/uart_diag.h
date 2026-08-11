#ifndef UART_DIAG_H
#define UART_DIAG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    UART_DIAG_OK = 0,
    UART_DIAG_INVALID_ARGUMENT,
    UART_DIAG_BAUD_OUT_OF_RANGE,
    UART_DIAG_CONFIGURATION_ERROR
} uart_diag_status_t;

/*
 * Initialize USART1 TX on PA9.
 *
 * pclk2_hz:
 *     USART1 peripheral clock frequency.
 *
 * baud_rate:
 *     Required UART baud rate.
 */
uart_diag_status_t uart_diag_init(uint32_t pclk2_hz,
                                  uint32_t baud_rate);

/*
 * Polling diagnostic-output functions.
 *
 * false means transmission failed or timed out.
 */
bool uart_diag_write_char(char character);
bool uart_diag_write_string(const char *text);
bool uart_diag_write_line(const char *text);
bool uart_diag_write_uint32(uint32_t value);
bool uart_diag_write_hex32(uint32_t value);

#endif /* UART_DIAG_H */