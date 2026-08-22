#include "uart2_link.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * ============================================================
 * Peripheral base addresses
 * ============================================================
 */

#define RCC_BASE_ADDRESS       0x40021000UL

#define GPIOA_BASE_ADDRESS     0x40010800UL

#define USART2_BASE_ADDRESS    0x40004400UL

#define DMA1_BASE_ADDRESS      0x40020000UL


/*
 * ============================================================
 * Register offsets
 * ============================================================
 */

#define RCC_AHBENR_OFFSET      0x14UL
#define RCC_APB2ENR_OFFSET     0x18UL
#define RCC_APB1ENR_OFFSET     0x1CUL


#define GPIO_CRL_OFFSET        0x00UL


#define USART_SR_OFFSET        0x00UL
#define USART_DR_OFFSET        0x04UL
#define USART_BRR_OFFSET       0x08UL
#define USART_CR1_OFFSET       0x0CUL
#define USART_CR2_OFFSET       0x10UL
#define USART_CR3_OFFSET       0x14UL


#define DMA_ISR_OFFSET         0x00UL
#define DMA_IFCR_OFFSET        0x04UL

#define DMA_CH7_CCR_OFFSET     0x80UL
#define DMA_CH7_CNDTR_OFFSET   0x84UL
#define DMA_CH7_CPAR_OFFSET    0x88UL
#define DMA_CH7_CMAR_OFFSET    0x8CUL


/*
 * ============================================================
 * Register access
 * ============================================================
 */

#define REG32(address) \
    (*(volatile uint32_t *)(address))

#define REG8(address) \
    (*(volatile uint8_t *)(address))


#define RCC_AHBENR \
    REG32( \
        RCC_BASE_ADDRESS + \
        RCC_AHBENR_OFFSET)

#define RCC_APB2ENR \
    REG32( \
        RCC_BASE_ADDRESS + \
        RCC_APB2ENR_OFFSET)

#define RCC_APB1ENR \
    REG32( \
        RCC_BASE_ADDRESS + \
        RCC_APB1ENR_OFFSET)


#define GPIOA_CRL \
    REG32( \
        GPIOA_BASE_ADDRESS + \
        GPIO_CRL_OFFSET)


#define USART2_SR \
    REG32( \
        USART2_BASE_ADDRESS + \
        USART_SR_OFFSET)

#define USART2_DR \
    REG32( \
        USART2_BASE_ADDRESS + \
        USART_DR_OFFSET)

#define USART2_BRR \
    REG32( \
        USART2_BASE_ADDRESS + \
        USART_BRR_OFFSET)

#define USART2_CR1 \
    REG32( \
        USART2_BASE_ADDRESS + \
        USART_CR1_OFFSET)

#define USART2_CR2 \
    REG32( \
        USART2_BASE_ADDRESS + \
        USART_CR2_OFFSET)

#define USART2_CR3 \
    REG32( \
        USART2_BASE_ADDRESS + \
        USART_CR3_OFFSET)


#define DMA1_ISR \
    REG32( \
        DMA1_BASE_ADDRESS + \
        DMA_ISR_OFFSET)

#define DMA1_IFCR \
    REG32( \
        DMA1_BASE_ADDRESS + \
        DMA_IFCR_OFFSET)

#define DMA1_CH7_CCR \
    REG32( \
        DMA1_BASE_ADDRESS + \
        DMA_CH7_CCR_OFFSET)

#define DMA1_CH7_CNDTR \
    REG32( \
        DMA1_BASE_ADDRESS + \
        DMA_CH7_CNDTR_OFFSET)

#define DMA1_CH7_CPAR \
    REG32( \
        DMA1_BASE_ADDRESS + \
        DMA_CH7_CPAR_OFFSET)

#define DMA1_CH7_CMAR \
    REG32( \
        DMA1_BASE_ADDRESS + \
        DMA_CH7_CMAR_OFFSET)


#define NVIC_ISER0 \
    REG32(0xE000E100UL)

#define NVIC_ISER1 \
    REG32(0xE000E104UL)

#define NVIC_IPR_BASE \
    0xE000E400UL


/*
 * ============================================================
 * RCC bits
 * ============================================================
 */

#define RCC_AHBENR_DMA1EN \
    (1UL << 0)

#define RCC_APB2ENR_IOPAEN \
    (1UL << 2)

#define RCC_APB1ENR_USART2EN \
    (1UL << 17)


/*
 * ============================================================
 * GPIO
 * ============================================================
 *
 * PA2 = USART2_TX
 *
 * Alternate-function push-pull,
 * 2 MHz:
 *
 *     CNF  = 10
 *     MODE = 10
 *
 *     0b1010 = 0xA
 *
 *
 * PA3 = USART2_RX
 *
 * Floating input:
 *
 *     CNF  = 01
 *     MODE = 00
 *
 *     0b0100 = 0x4
 */

#define GPIO_PA2_SHIFT \
    8UL

#define GPIO_PA3_SHIFT \
    12UL

#define GPIO_CRL_FIELD_MASK \
    0xFUL

#define GPIO_AF_2MHZ_PUSH_PULL \
    0xAUL

#define GPIO_FLOATING_INPUT \
    0x4UL


#define GPIO_PA2_MASK \
    (GPIO_CRL_FIELD_MASK << \
     GPIO_PA2_SHIFT)

#define GPIO_PA3_MASK \
    (GPIO_CRL_FIELD_MASK << \
     GPIO_PA3_SHIFT)


#define GPIO_PA2_TX_VALUE \
    (GPIO_AF_2MHZ_PUSH_PULL << \
     GPIO_PA2_SHIFT)

#define GPIO_PA3_RX_VALUE \
    (GPIO_FLOATING_INPUT << \
     GPIO_PA3_SHIFT)


/*
 * ============================================================
 * USART status bits
 * ============================================================
 */

#define USART_SR_PE \
    (1UL << 0)

#define USART_SR_FE \
    (1UL << 1)

#define USART_SR_NE \
    (1UL << 2)

#define USART_SR_ORE \
    (1UL << 3)

#define USART_SR_RXNE \
    (1UL << 5)


#define USART_SR_ERROR_MASK \
    (USART_SR_PE | \
     USART_SR_FE | \
     USART_SR_NE | \
     USART_SR_ORE)


/*
 * ============================================================
 * USART control bits
 * ============================================================
 */

#define USART_CR1_RE \
    (1UL << 2)

#define USART_CR1_TE \
    (1UL << 3)

#define USART_CR1_RXNEIE \
    (1UL << 5)

#define USART_CR1_UE \
    (1UL << 13)


#define USART_CR3_EIE \
    (1UL << 0)

#define USART_CR3_DMAT \
    (1UL << 7)


/*
 * ============================================================
 * DMA Channel 7 control bits
 * ============================================================
 */

#define DMA_CCR_EN \
    (1UL << 0)

#define DMA_CCR_TCIE \
    (1UL << 1)

#define DMA_CCR_TEIE \
    (1UL << 3)

#define DMA_CCR_DIR \
    (1UL << 4)

#define DMA_CCR_MINC \
    (1UL << 7)

#define DMA_CCR_PL_HIGH \
    (2UL << 12)


/*
 * DMA1 Channel 7 interrupt flags.
 */
#define DMA_ISR_TCIF7 \
    (1UL << 25)

#define DMA_ISR_TEIF7 \
    (1UL << 27)


#define DMA_IFCR_CH7_ALL \
    (0xFUL << 24)


/*
 * ============================================================
 * NVIC
 * ============================================================
 */

#define DMA1_CHANNEL7_IRQ \
    17UL

#define USART2_IRQ \
    38UL


/*
 * Current MPU DATA_RDY interrupt uses a higher priority.
 *
 * Communication must not preempt the most time-critical sensor
 * acquisition path unnecessarily.
 */
#define USART2_IRQ_PRIORITY \
    0x80U

#define DMA1_CH7_IRQ_PRIORITY \
    0xA0U


/*
 * Same BRR validity policy already used by uart_diag.c.
 */
#define USART_BRR_MIN_VALUE \
    16UL

#define USART_BRR_MAX_VALUE \
    0xFFFFUL


/*
 * Circular-buffer indexing uses a mask.
 *
 * Therefore capacity must remain a power of two.
 */
_Static_assert(
    (UART2_LINK_RX_BUFFER_CAPACITY &
     (UART2_LINK_RX_BUFFER_CAPACITY -
      1U)) ==
    0U,
    "UART2_LINK_RX_BUFFER_CAPACITY must be a power of two");


volatile uart2_link_diag_t
    g_uart2_link_diag;


static volatile bool
    uart2_link_ready;

static volatile bool
    uart2_tx_enabled;

static volatile bool
    uart2_rx_enabled;

static volatile bool
    uart2_tx_dma_busy;


static uint8_t
    tx_dma_buffer[
        UART2_LINK_TX_BUFFER_CAPACITY];


static volatile uint16_t
    rx_head;

static volatile uint16_t
    rx_tail;


static uint8_t
    rx_buffer[
        UART2_LINK_RX_BUFFER_CAPACITY];


/*
 * ============================================================
 * Critical section
 * ============================================================
 */

static uint32_t
enter_critical(void)
{
    uint32_t primask;


    __asm volatile (
        "mrs %0, primask\n"
        "cpsid i\n"
        : "=r" (primask)
        :
        : "memory");


    return primask;
}


static void
exit_critical(
    uint32_t primask)
{
    if ((primask &
         1UL) ==
        0UL)
    {
        __asm volatile (
            "cpsie i"
            :
            :
            : "memory");
    }
}


/*
 * ============================================================
 * GPIO
 * ============================================================
 */

static void
configure_gpio(
    bool enable_tx,
    bool enable_rx)
{
    uint32_t gpio_crl;


    RCC_APB2ENR |=
        RCC_APB2ENR_IOPAEN;


    /*
     * Ensure peripheral clock write completes.
     */
    (void)RCC_APB2ENR;


    gpio_crl =
        GPIOA_CRL;


    if (enable_tx)
    {
        gpio_crl &=
            (uint32_t)(
                ~GPIO_PA2_MASK);


        gpio_crl |=
            GPIO_PA2_TX_VALUE;
    }


    if (enable_rx)
    {
        gpio_crl &=
            (uint32_t)(
                ~GPIO_PA3_MASK);


        gpio_crl |=
            GPIO_PA3_RX_VALUE;
    }


    GPIOA_CRL =
        gpio_crl;
}


static bool
gpio_configuration_is_valid(
    bool enable_tx,
    bool enable_rx)
{
    if (enable_tx &&
        ((GPIOA_CRL &
          GPIO_PA2_MASK) !=
         GPIO_PA2_TX_VALUE))
    {
        return false;
    }


    if (enable_rx &&
        ((GPIOA_CRL &
          GPIO_PA3_MASK) !=
         GPIO_PA3_RX_VALUE))
    {
        return false;
    }


    return true;
}


/*
 * ============================================================
 * TX DMA configuration
 * ============================================================
 */

static void
configure_tx_dma(void)
{
    RCC_AHBENR |=
        RCC_AHBENR_DMA1EN;


    (void)RCC_AHBENR;


    /*
     * DMA channel must be disabled while configuration registers
     * are changed.
     */
    DMA1_CH7_CCR &=
        (uint32_t)(
            ~DMA_CCR_EN);


    DMA1_IFCR =
        DMA_IFCR_CH7_ALL;


    /*
     * USART2_TX DMA request uses DMA1 Channel 7.
     *
     * Peripheral address is USART2_DR.
     */
    DMA1_CH7_CPAR =
        (uint32_t)(
            USART2_BASE_ADDRESS +
            USART_DR_OFFSET);


    /*
     * Fixed internal TX staging buffer.
     */
    DMA1_CH7_CMAR =
        (uint32_t)(
            uintptr_t)
            tx_dma_buffer;


    DMA1_CH7_CNDTR =
        0UL;


    /*
     * Peripheral size = 8-bit, default
     * Memory size     = 8-bit, default
     * Peripheral increment disabled
     * Memory increment enabled
     * Memory -> peripheral
     * High priority
     * Transfer complete IRQ
     * Transfer error IRQ
     */
    DMA1_CH7_CCR =
        DMA_CCR_TCIE |
        DMA_CCR_TEIE |
        DMA_CCR_DIR |
        DMA_CCR_MINC |
        DMA_CCR_PL_HIGH;


    REG8(
        NVIC_IPR_BASE +
        DMA1_CHANNEL7_IRQ) =
        DMA1_CH7_IRQ_PRIORITY;


    NVIC_ISER0 =
        (1UL <<
         DMA1_CHANNEL7_IRQ);
}


/*
 * ============================================================
 * RX interrupt configuration
 * ============================================================
 */

static void
configure_rx_interrupt(void)
{
    REG8(
        NVIC_IPR_BASE +
        USART2_IRQ) =
        USART2_IRQ_PRIORITY;


    /*
     * USART2 = IRQ 38.
     *
     * IRQ 38 is NVIC_ISER1 bit:
     *
     * 38 - 32 = 6
     */
    NVIC_ISER1 =
        (1UL <<
         (USART2_IRQ -
          32UL));
}


/*
 * ============================================================
 * Initialization
 * ============================================================
 */

uart2_link_status_t
uart2_link_init(
    uint32_t pclk1_hz,
    uint32_t baud_rate,
    bool enable_tx,
    bool enable_rx)
{
    uint32_t baud_register_value;

    uint32_t cr1;
    uint32_t cr3;

    uint32_t primask;


    uart2_link_ready =
        false;

    uart2_tx_enabled =
        false;

    uart2_rx_enabled =
        false;

    uart2_tx_dma_busy =
        false;


    if ((pclk1_hz == 0UL) ||
        (baud_rate == 0UL) ||
        ((!enable_tx) &&
         (!enable_rx)))
    {
        return
            UART2_LINK_INVALID_ARGUMENT;
    }


    /*
     * STM32F1 USART BRR representation:
     *
     * with OVER8 unavailable,
     *
     * baud ≈ peripheral_clock / BRR.
     *
     * Add baud / 2 for integer rounding.
     */
    baud_register_value =
        (pclk1_hz +
         (baud_rate /
          2UL)) /
        baud_rate;


    if ((baud_register_value <
         USART_BRR_MIN_VALUE) ||
        (baud_register_value >
         USART_BRR_MAX_VALUE))
    {
        return
            UART2_LINK_BAUD_OUT_OF_RANGE;
    }


    primask =
        enter_critical();


    g_uart2_link_diag =
        (uart2_link_diag_t){0};


    g_uart2_link_diag.init_count =
        1UL;


    rx_head =
        0U;

    rx_tail =
        0U;


    exit_critical(
        primask);


    configure_gpio(
        enable_tx,
        enable_rx);


    /*
     * USART2 belongs to APB1.
     */
    RCC_APB1ENR |=
        RCC_APB1ENR_USART2EN;


    (void)RCC_APB1ENR;


    /*
     * Disable/reset USART configuration before programming it.
     */
    USART2_CR1 =
        0UL;

    USART2_CR2 =
        0UL;

    USART2_CR3 =
        0UL;


    USART2_BRR =
        baud_register_value;


    if (enable_tx)
    {
        configure_tx_dma();
    }


    if (enable_rx)
    {
        configure_rx_interrupt();
    }


    cr3 =
        0UL;


    if (enable_tx)
    {
        cr3 |=
            USART_CR3_DMAT;
    }


    if (enable_rx)
    {
        /*
         * Enables framing/noise/overrun error interrupt path.
         */
        cr3 |=
            USART_CR3_EIE;
    }


    USART2_CR3 =
        cr3;


    cr1 =
        USART_CR1_UE;


    if (enable_tx)
    {
        cr1 |=
            USART_CR1_TE;
    }


    if (enable_rx)
    {
        cr1 |=
            USART_CR1_RE |
            USART_CR1_RXNEIE;
    }


    USART2_CR1 =
        cr1;


    /*
     * Basic post-write verification.
     */
    if (!gpio_configuration_is_valid(
            enable_tx,
            enable_rx) ||
        (USART2_BRR !=
         baud_register_value) ||
        ((USART2_CR1 &
          cr1) !=
         cr1) ||
        ((USART2_CR3 &
          cr3) !=
         cr3))
    {
        USART2_CR1 =
            0UL;

        USART2_CR2 =
            0UL;

        USART2_CR3 =
            0UL;


        if (enable_tx)
        {
            DMA1_CH7_CCR &=
                (uint32_t)(
                    ~DMA_CCR_EN);
        }


        return
            UART2_LINK_CONFIGURATION_ERROR;
    }


    uart2_tx_enabled =
        enable_tx;

    uart2_rx_enabled =
        enable_rx;

    uart2_link_ready =
        true;


    return
        UART2_LINK_OK;
}


/*
 * ============================================================
 * Non-blocking TX
 * ============================================================
 */

bool
uart2_link_start_tx(
    const uint8_t *data,
    uint32_t length)
{
    uint32_t i;

    uint32_t primask;


    if ((!uart2_link_ready) ||
        (!uart2_tx_enabled) ||
        (data ==
         (const uint8_t *)0) ||
        (length == 0UL) ||
        (length >
         UART2_LINK_TX_BUFFER_CAPACITY))
    {
        g_uart2_link_diag
            .tx_invalid_reject_count++;


        return false;
    }


    primask =
        enter_critical();


    if (uart2_tx_dma_busy)
    {
        g_uart2_link_diag
            .tx_busy_reject_count++;


        exit_critical(
            primask);


        return false;
    }


    /*
     * Reserve the DMA staging buffer before interrupts are
     * re-enabled.
     */
    uart2_tx_dma_busy =
        true;


    exit_critical(
        primask);


    /*
     * Caller may use a temporary stack frame buffer.
     *
     * Copy it to driver-owned memory before starting DMA.
     */
    for (i = 0UL;
         i < length;
         i++)
    {
        tx_dma_buffer[i] =
            data[i];
    }


    DMA1_CH7_CCR &=
        (uint32_t)(
            ~DMA_CCR_EN);


    DMA1_IFCR =
        DMA_IFCR_CH7_ALL;


    DMA1_CH7_CMAR =
        (uint32_t)(
            uintptr_t)
            tx_dma_buffer;


    DMA1_CH7_CNDTR =
        length;


    g_uart2_link_diag
        .last_tx_length =
        length;


    g_uart2_link_diag
        .tx_start_count++;


    /*
     * USART2 DMAT is already enabled.
     *
     * Enabling Channel 7 allows USART2 TXE requests to move the
     * bytes without blocking the CPU.
     */
    DMA1_CH7_CCR |=
        DMA_CCR_EN;


    return true;
}


bool
uart2_link_tx_busy(void)
{
    return
        uart2_tx_dma_busy;
}


/*
 * ============================================================
 * RX circular buffer
 * ============================================================
 */

bool
uart2_link_read_byte(
    uint8_t *byte)
{
    uint16_t tail;

    uint32_t primask;


    if ((!uart2_link_ready) ||
        (!uart2_rx_enabled) ||
        (byte ==
         (uint8_t *)0))
    {
        return false;
    }


    primask =
        enter_critical();


    tail =
        rx_tail;


    if (tail ==
        rx_head)
    {
        exit_critical(
            primask);


        return false;
    }


    *byte =
        rx_buffer[tail];


    rx_tail =
        (uint16_t)(
            (tail +
             1U) &
            (UART2_LINK_RX_BUFFER_CAPACITY -
             1U));


    exit_critical(
        primask);


    return true;
}


void
uart2_link_flush_rx(void)
{
    uint32_t primask;


    primask =
        enter_critical();


    rx_tail =
        rx_head;


    exit_critical(
        primask);
}


void
uart2_link_get_diag(
    uart2_link_diag_t *diag)
{
    uint32_t primask;


    if (diag ==
        (uart2_link_diag_t *)0)
    {
        return;
    }


    primask =
        enter_critical();


    *diag =
        g_uart2_link_diag;


    exit_critical(
        primask);
}


/*
 * ============================================================
 * DMA1 Channel 7 interrupt
 * ============================================================
 *
 * The startup file already provides a weak handler.
 *
 * This strong implementation automatically replaces it when this
 * source file is linked.
 */

void
DMA1_Channel7_IRQHandler(void)
{
    uint32_t
        dma_status;


    dma_status =
        DMA1_ISR;


    if ((dma_status &
         DMA_ISR_TEIF7) !=
        0UL)
    {
        DMA1_CH7_CCR &=
            (uint32_t)(
                ~DMA_CCR_EN);


        DMA1_IFCR =
            DMA_IFCR_CH7_ALL;


        uart2_tx_dma_busy =
            false;


        g_uart2_link_diag
            .tx_dma_error_count++;


        return;
    }


    if ((dma_status &
         DMA_ISR_TCIF7) !=
        0UL)
    {
        DMA1_CH7_CCR &=
            (uint32_t)(
                ~DMA_CCR_EN);


        DMA1_IFCR =
            DMA_IFCR_CH7_ALL;


        uart2_tx_dma_busy =
            false;


        g_uart2_link_diag
            .tx_complete_count++;
    }
}


/*
 * ============================================================
 * USART2 interrupt
 * ============================================================
 *
 * The interrupt performs only the minimum receive work:
 *
 *     read status
 *     read byte
 *     record hardware errors
 *     push good byte into circular buffer
 *     return
 *
 * Packet parsing is intentionally NOT performed in the interrupt.
 */

void
USART2_IRQHandler(void)
{
    uint32_t status;

    uint8_t received_byte;

    uint16_t head;
    uint16_t next_head;


    status =
        USART2_SR;


    if ((status &
         (USART_SR_RXNE |
          USART_SR_ERROR_MASK)) ==
        0UL)
    {
        return;
    }


    /*
     * STM32F1 receive/error clear sequence:
     *
     * read SR
     * then read DR
     */
    received_byte =
        (uint8_t)(
            USART2_DR &
            0xFFUL);


    if ((status &
         USART_SR_PE) !=
        0UL)
    {
        g_uart2_link_diag
            .rx_parity_error_count++;
    }


    if ((status &
         USART_SR_FE) !=
        0UL)
    {
        g_uart2_link_diag
            .rx_framing_error_count++;
    }


    if ((status &
         USART_SR_NE) !=
        0UL)
    {
        g_uart2_link_diag
            .rx_noise_error_count++;
    }


    if ((status &
         USART_SR_ORE) !=
        0UL)
    {
        g_uart2_link_diag
            .rx_overrun_error_count++;
    }


    /*
     * A byte associated with a detected UART hardware error does
     * not enter the protocol parser.
     */
    if ((status &
         USART_SR_ERROR_MASK) !=
        0UL)
    {
        return;
    }


    if ((!uart2_link_ready) ||
        (!uart2_rx_enabled))
    {
        return;
    }


    head =
        rx_head;


    next_head =
        (uint16_t)(
            (head +
             1U) &
            (UART2_LINK_RX_BUFFER_CAPACITY -
             1U));


    /*
     * One slot remains unused to distinguish full from empty.
     */
    if (next_head ==
        rx_tail)
    {
        g_uart2_link_diag
            .rx_buffer_overflow_count++;


        return;
    }


    rx_buffer[head] =
        received_byte;


    rx_head =
        next_head;


    g_uart2_link_diag
        .rx_byte_count++;


    g_uart2_link_diag
        .last_rx_byte =
        (uint32_t)
        received_byte;
}