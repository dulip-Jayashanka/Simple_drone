#include "uart_diag.h"

#include <stdint.h>

/* Peripheral base addresses from STM32F103 RM0008. */
#define RCC_BASE_ADDRESS       0x40021000UL
#define GPIOA_BASE_ADDRESS     0x40010800UL
#define USART1_BASE_ADDRESS    0x40013800UL

/* Register offsets. */
#define RCC_APB2ENR_OFFSET     0x18UL
#define GPIO_CRH_OFFSET        0x04UL

#define USART_SR_OFFSET        0x00UL
#define USART_DR_OFFSET        0x04UL
#define USART_BRR_OFFSET       0x08UL
#define USART_CR1_OFFSET       0x0CUL
#define USART_CR2_OFFSET       0x10UL
#define USART_CR3_OFFSET       0x14UL

#define REG32(address) \
    (*(volatile uint32_t *)(address))

#define RCC_APB2ENR \
    REG32(RCC_BASE_ADDRESS + RCC_APB2ENR_OFFSET)

#define GPIOA_CRH \
    REG32(GPIOA_BASE_ADDRESS + GPIO_CRH_OFFSET)

#define USART1_SR \
    REG32(USART1_BASE_ADDRESS + USART_SR_OFFSET)

#define USART1_DR \
    REG32(USART1_BASE_ADDRESS + USART_DR_OFFSET)

#define USART1_BRR \
    REG32(USART1_BASE_ADDRESS + USART_BRR_OFFSET)

#define USART1_CR1 \
    REG32(USART1_BASE_ADDRESS + USART_CR1_OFFSET)

#define USART1_CR2 \
    REG32(USART1_BASE_ADDRESS + USART_CR2_OFFSET)

#define USART1_CR3 \
    REG32(USART1_BASE_ADDRESS + USART_CR3_OFFSET)

/* RCC_APB2ENR bits. */
#define RCC_APB2ENR_IOPAEN     (1UL << 2)
#define RCC_APB2ENR_USART1EN   (1UL << 14)

/*
 * PA9 is configured through GPIOA_CRH bits 7:4.
 *
 * CNF  = 10: alternate-function push-pull
 * MODE = 10: maximum output speed 2 MHz
 *
 * Four-bit configuration = 0b1010 = 0xA
 */
#define GPIO_PA9_CRH_SHIFT       4UL
#define GPIO_PA9_CRH_MASK        (0xFUL << GPIO_PA9_CRH_SHIFT)
#define GPIO_AF_2MHZ_PUSH_PULL   0xAUL

#define GPIO_PA9_CRH_VALUE \
    (GPIO_AF_2MHZ_PUSH_PULL << GPIO_PA9_CRH_SHIFT)

/* USART status and control bits. */
#define USART_SR_TXE           (1UL << 7)
#define USART_CR1_TE           (1UL << 3)
#define USART_CR1_UE           (1UL << 13)

/*
 * The encoded USARTDIV value in BRR must include a nonzero
 * mantissa. For oversampling by 16, the smallest valid value is 16.
 */
#define USART_BRR_MIN_VALUE    16UL
#define USART_BRR_MAX_VALUE    0xFFFFUL

/*
 * Prevent a failed UART peripheral from trapping the motor firmware
 * permanently inside the diagnostic driver.
 */
#define UART_DIAG_TX_TIMEOUT_ITERATIONS  100000UL

static bool uart_diag_ready;

static bool uart_diag_wait_for_txe(void)
{
    uint32_t iteration;

    for (iteration = 0UL;
         iteration < UART_DIAG_TX_TIMEOUT_ITERATIONS;
         iteration++)
    {
        if ((USART1_SR & USART_SR_TXE) != 0UL)
        {
            return true;
        }
    }

    /*
     * Check once more at the timeout boundary.
     */
    return ((USART1_SR & USART_SR_TXE) != 0UL);
}

uart_diag_status_t uart_diag_init(uint32_t pclk2_hz,
                                  uint32_t baud_rate)
{
    uint32_t baud_register_value;
    uint32_t gpio_crh;

    uart_diag_ready = false;

    if ((pclk2_hz == 0UL) || (baud_rate == 0UL))
    {
        return UART_DIAG_INVALID_ARGUMENT;
    }

    /*
     * For oversampling by 16:
     *
     *     baud = PCLK2 / BRR
     *
     * Adding baud_rate / 2 performs integer rounding.
     *
     * At 72 MHz and 115200 baud:
     *
     *     BRR = 72000000 / 115200
     *         = 625
     *         = 0x0271
     */
    baud_register_value =
        (pclk2_hz + (baud_rate / 2UL)) / baud_rate;

    if ((baud_register_value < USART_BRR_MIN_VALUE) ||
        (baud_register_value > USART_BRR_MAX_VALUE))
    {
        return UART_DIAG_BAUD_OUT_OF_RANGE;
    }

    /*
     * Enable GPIOA and USART1 peripheral clocks.
     */
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN |
                   RCC_APB2ENR_USART1EN;

    /*
     * Read back the clock-enable register before accessing GPIOA
     * and USART1.
     */
    (void)RCC_APB2ENR;

    /*
     * Configure PA9 as alternate-function push-pull.
     *
     * Preserve the configurations of PA8 and PA10-PA15.
     */
    gpio_crh = GPIOA_CRH;
    gpio_crh &= (uint32_t)(~GPIO_PA9_CRH_MASK);
    gpio_crh |= GPIO_PA9_CRH_VALUE;
    GPIOA_CRH = gpio_crh;

    /*
     * Configure:
     *
     * 8 data bits
     * No parity
     * One stop bit
     * No flow control
     * Transmit only
     * Polling mode
     */
    USART1_CR1 = 0UL;
    USART1_CR2 = 0UL;
    USART1_CR3 = 0UL;

    USART1_CR1 = USART_CR1_UE;
    USART1_BRR = baud_register_value;
    USART1_CR1 = USART_CR1_TE |
                 USART_CR1_UE;

    /*
     * Verify important register values and confirm that the
     * transmit data register becomes available.
     */
    if (((GPIOA_CRH & GPIO_PA9_CRH_MASK) !=
         GPIO_PA9_CRH_VALUE) ||
        (USART1_BRR != baud_register_value) ||
        ((USART1_CR1 &
          (USART_CR1_TE | USART_CR1_UE)) !=
         (USART_CR1_TE | USART_CR1_UE)) ||
        (!uart_diag_wait_for_txe()))
    {
        USART1_CR1 = 0UL;

        return UART_DIAG_CONFIGURATION_ERROR;
    }

    uart_diag_ready = true;

    return UART_DIAG_OK;
}

bool uart_diag_write_char(char character)
{
    if ((!uart_diag_ready) ||
        (!uart_diag_wait_for_txe()))
    {
        return false;
    }

    /*
     * Writing USART_DR clears TXE. Hardware sets TXE again after
     * moving the byte into the transmit shift register.
     */
    USART1_DR = (uint32_t)(uint8_t)character;

    return true;
}

bool uart_diag_write_string(const char *text)
{
    if (text == (const char *)0)
    {
        return false;
    }

    while (*text != '\0')
    {
        if (!uart_diag_write_char(*text))
        {
            return false;
        }

        text++;
    }

    return true;
}

bool uart_diag_write_line(const char *text)
{
    if (!uart_diag_write_string(text))
    {
        return false;
    }

    /*
     * CRLF works correctly with common serial terminals.
     */
    return uart_diag_write_string("\r\n");
}

bool uart_diag_write_uint32(uint32_t value)
{
    char digits[10];
    uint32_t digit_count;

    if (value == 0UL)
    {
        return uart_diag_write_char('0');
    }

    digit_count = 0UL;

    /*
     * Store digits in reverse order.
     */
    while (value != 0UL)
    {
        digits[digit_count] =
            (char)('0' + (value % 10UL));

        digit_count++;
        value /= 10UL;
    }

    /*
     * Send digits in the correct order.
     */
    while (digit_count != 0UL)
    {
        digit_count--;

        if (!uart_diag_write_char(digits[digit_count]))
        {
            return false;
        }
    }

    return true;
}

bool uart_diag_write_hex32(uint32_t value)
{
    static const char hex_digits[] =
        "0123456789ABCDEF";

    uint32_t shift;

    if (!uart_diag_write_string("0x"))
    {
        return false;
    }

    for (shift = 28UL; ; shift -= 4UL)
    {
        if (!uart_diag_write_char(
                hex_digits[(value >> shift) & 0xFUL]))
        {
            return false;
        }

        if (shift == 0UL)
        {
            break;
        }
    }

    return true;
}