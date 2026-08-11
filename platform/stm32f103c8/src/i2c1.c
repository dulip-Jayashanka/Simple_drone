#include "i2c1.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REG32(address) (*(volatile uint32_t *)(address))

#define RCC_BASE       0x40021000UL
#define GPIOB_BASE     0x40010C00UL
#define I2C1_BASE      0x40005400UL

#define RCC_APB2ENR    REG32(RCC_BASE + 0x18UL)
#define RCC_APB1ENR    REG32(RCC_BASE + 0x1CUL)
#define RCC_APB1RSTR   REG32(RCC_BASE + 0x10UL)

#define GPIOB_CRL      REG32(GPIOB_BASE + 0x00UL)
#define GPIOB_IDR      REG32(GPIOB_BASE + 0x08UL)
#define GPIOB_ODR      REG32(GPIOB_BASE + 0x0CUL)

#define I2C1_CR1       REG32(I2C1_BASE + 0x00UL)
#define I2C1_CR2       REG32(I2C1_BASE + 0x04UL)
#define I2C1_OAR1      REG32(I2C1_BASE + 0x08UL)
#define I2C1_OAR2      REG32(I2C1_BASE + 0x0CUL)
#define I2C1_DR        REG32(I2C1_BASE + 0x10UL)
#define I2C1_SR1       REG32(I2C1_BASE + 0x14UL)
#define I2C1_SR2       REG32(I2C1_BASE + 0x18UL)
#define I2C1_CCR       REG32(I2C1_BASE + 0x1CUL)
#define I2C1_TRISE     REG32(I2C1_BASE + 0x20UL)

#define RCC_APB2ENR_IOPBEN    (1UL << 3)
#define RCC_APB1ENR_I2C1EN    (1UL << 21)
#define RCC_APB1RSTR_I2C1RST  (1UL << 21)

#define GPIO_SCL              (1UL << 6)
#define GPIO_SDA              (1UL << 7)
#define GPIO_BUS_LINES        (GPIO_SCL | GPIO_SDA)

#define I2C_CR1_PE            (1UL << 0)
#define I2C_CR1_START         (1UL << 8)
#define I2C_CR1_STOP          (1UL << 9)
#define I2C_CR1_ACK           (1UL << 10)
#define I2C_CR1_POS           (1UL << 11)
#define I2C_CR1_SWRST         (1UL << 15)

#define I2C_SR1_SB            (1UL << 0)
#define I2C_SR1_ADDR          (1UL << 1)
#define I2C_SR1_BTF           (1UL << 2)
#define I2C_SR1_RXNE          (1UL << 6)
#define I2C_SR1_TXE           (1UL << 7)
#define I2C_SR1_BERR          (1UL << 8)
#define I2C_SR1_ARLO          (1UL << 9)
#define I2C_SR1_AF            (1UL << 10)
#define I2C_SR1_OVR           (1UL << 11)

#define I2C_SR1_ERROR_MASK    (I2C_SR1_BERR | \
                               I2C_SR1_ARLO | \
                               I2C_SR1_AF | \
                               I2C_SR1_OVR)

#define I2C_SR2_MSL           (1UL << 0)
#define I2C_SR2_BUSY          (1UL << 1)

#define I2C_CCR_FAST          (1UL << 15)
#define I2C_OAR1_BIT14        (1UL << 14)

/*
 * The transaction timeout is intentionally bounded and independent of
 * TIM2/micros(). At -O0 this is comfortably longer than a normal I2C event,
 * while still allowing a failed transaction to recover instead of hanging.
 */
#define I2C_EVENT_TIMEOUT_ITERATIONS 200000UL
#define I2C_STOP_TIMEOUT_ITERATIONS   50000UL
#define I2C_LINE_TIMEOUT_ITERATIONS   50000UL
#define I2C_TRANSACTION_ATTEMPTS      3UL
#define I2C_BUS_CLEAR_PULSES          9UL
#define I2C_BUS_CLEAR_DELAY_LOOPS     128UL

volatile i2c1_diag_t g_i2c1_diag;

static uint32_t configured_pclk1_hz;
static uint32_t configured_bus_hz;
static bool configuration_valid;

static uint32_t enter_critical(void)
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

static void exit_critical(uint32_t primask)
{
    if ((primask & 1UL) == 0UL)
    {
        __asm volatile ("cpsie i" ::: "memory");
    }
}

static void short_delay(void)
{
    volatile uint32_t iteration;

    for (iteration = 0UL;
         iteration < I2C_BUS_CLEAR_DELAY_LOOPS;
         iteration++)
    {
        __asm volatile ("nop");
    }
}

static void set_stage(i2c1_stage_t stage)
{
    g_i2c1_diag.stage = stage;
}

static void snapshot_registers(i2c1_status_t status,
                               i2c1_stage_t stage)
{
    g_i2c1_diag.last_status = status;
    g_i2c1_diag.stage = stage;
    g_i2c1_diag.last_error_status = status;
    g_i2c1_diag.last_error_stage = stage;
    g_i2c1_diag.last_error_attempt = g_i2c1_diag.attempt;
    g_i2c1_diag.cr1 = I2C1_CR1;
    g_i2c1_diag.sr1 = I2C1_SR1;
    g_i2c1_diag.sr2 = I2C1_SR2;
    g_i2c1_diag.dr = I2C1_DR;
    g_i2c1_diag.gpio_idr = GPIOB_IDR;
}

static void count_failure(i2c1_status_t status)
{
    switch (status)
    {
        case I2C1_TIMEOUT:
            g_i2c1_diag.timeouts++;
            break;

        case I2C1_NACK:
            g_i2c1_diag.nacks++;
            break;

        case I2C1_BUS_BUSY:
            g_i2c1_diag.bus_busy_errors++;
            break;

        case I2C1_BUS_ERROR:
            g_i2c1_diag.bus_errors++;
            break;

        case I2C1_ARBITRATION_LOST:
            g_i2c1_diag.arbitration_losses++;
            break;

        case I2C1_OVERRUN:
            g_i2c1_diag.overruns++;
            break;

        case I2C1_LINE_STUCK_LOW:
            g_i2c1_diag.line_stuck_errors++;
            break;

        default:
            break;
    }
}

static i2c1_status_t status_from_sr1(uint32_t sr1)
{
    if ((sr1 & I2C_SR1_BERR) != 0UL)
    {
        return I2C1_BUS_ERROR;
    }

    if ((sr1 & I2C_SR1_ARLO) != 0UL)
    {
        return I2C1_ARBITRATION_LOST;
    }

    if ((sr1 & I2C_SR1_AF) != 0UL)
    {
        return I2C1_NACK;
    }

    if ((sr1 & I2C_SR1_OVR) != 0UL)
    {
        return I2C1_OVERRUN;
    }

    return I2C1_OK;
}

static void clear_error_flags(void)
{
    uint32_t sr1;

    /*
     * RM0008 marks BERR/ARLO/AF/OVR as rc_w0: software clears them by
     * writing 0. Reading SR1 alone does not clear ADDR; that additionally
     * requires the SR2 read sequence.
     */
    sr1 = I2C1_SR1;
    sr1 &= ~I2C_SR1_ERROR_MASK;
    I2C1_SR1 = sr1;
}

static i2c1_status_t wait_sr1(uint32_t mask,
                              i2c1_stage_t stage)
{
    uint32_t iteration;

    set_stage(stage);

    for (iteration = 0UL;
         iteration < I2C_EVENT_TIMEOUT_ITERATIONS;
         iteration++)
    {
        uint32_t sr1 = I2C1_SR1;
        i2c1_status_t error = status_from_sr1(sr1);

        if (error != I2C1_OK)
        {
            snapshot_registers(error, stage);
            count_failure(error);
            return error;
        }

        if ((sr1 & mask) == mask)
        {
            return I2C1_OK;
        }
    }

    snapshot_registers(I2C1_TIMEOUT, stage);
    count_failure(I2C1_TIMEOUT);
    return I2C1_TIMEOUT;
}

static i2c1_status_t wait_bus_idle_raw(void)
{
    uint32_t iteration;
    i2c1_status_t final_status;

    set_stage(I2C1_STAGE_WAIT_BUS_IDLE);

    for (iteration = 0UL;
         iteration < I2C_EVENT_TIMEOUT_ITERATIONS;
         iteration++)
    {
        i2c1_status_t error = status_from_sr1(I2C1_SR1);

        if (error != I2C1_OK)
        {
            snapshot_registers(error,
                               I2C1_STAGE_WAIT_BUS_IDLE);
            count_failure(error);
            return error;
        }

        /*
         * Check both the peripheral state and the physical pins. This catches
         * a bus line held low even if the peripheral BUSY state has already
         * become inconsistent after a glitch/debug reset.
         */
        if (((I2C1_SR2 & I2C_SR2_BUSY) == 0UL) &&
            ((I2C1_CR1 & I2C_CR1_STOP) == 0UL) &&
            ((GPIOB_IDR & GPIO_BUS_LINES) == GPIO_BUS_LINES))
        {
            return I2C1_OK;
        }
    }

    final_status =
        ((GPIOB_IDR & GPIO_BUS_LINES) == GPIO_BUS_LINES) ?
        I2C1_BUS_BUSY : I2C1_LINE_STUCK_LOW;

    snapshot_registers(final_status,
                       I2C1_STAGE_WAIT_BUS_IDLE);
    count_failure(final_status);
    return final_status;
}

static i2c1_status_t wait_stop_complete(void)
{
    uint32_t iteration;

    set_stage(I2C1_STAGE_STOP);

    for (iteration = 0UL;
         iteration < I2C_STOP_TIMEOUT_ITERATIONS;
         iteration++)
    {
        if (((I2C1_CR1 & I2C_CR1_STOP) == 0UL) &&
            ((I2C1_SR2 & I2C_SR2_BUSY) == 0UL))
        {
            return I2C1_OK;
        }
    }

    snapshot_registers(I2C1_TIMEOUT,
                       I2C1_STAGE_STOP);
    count_failure(I2C1_TIMEOUT);
    return I2C1_TIMEOUT;
}

static void clear_addr(void)
{
    volatile uint32_t discard;

    /*
     * RM0008 EV6: ADDR is cleared by reading SR1 followed by SR2.
     * Call this only after ADDR has been observed set.
     */
    set_stage(I2C1_STAGE_CLEAR_ADDR);
    discard = I2C1_SR1;
    discard = I2C1_SR2;
    (void)discard;
}

static void gpio_configure_af_open_drain(void)
{
    uint32_t gpio;

    /* Release both open-drain outputs before changing the pin mode. */
    GPIOB_ODR |= GPIO_BUS_LINES;

    /*
     * PB6/PB7: MODE=11 (50 MHz), CNF=11 (alternate-function open-drain).
     */
    gpio = GPIOB_CRL;
    gpio &= ~((0xFUL << 24) |
              (0xFUL << 28));
    gpio |= (0xFUL << 24) |
            (0xFUL << 28);
    GPIOB_CRL = gpio;

    /* Open-drain release state. */
    GPIOB_ODR |= GPIO_BUS_LINES;
}

static void gpio_configure_bus_clear(void)
{
    uint32_t gpio;

    /* Release both lines before the alternate-function pins become GPIO. */
    GPIOB_ODR |= GPIO_BUS_LINES;

    /*
     * PB6/PB7: MODE=10 (2 MHz), CNF=01 (GPIO open-drain).
     * Driving ODR=1 releases a line so the external pull-up can raise it.
     */
    gpio = GPIOB_CRL;
    gpio &= ~((0xFUL << 24) |
              (0xFUL << 28));
    gpio |= (0x6UL << 24) |
            (0x6UL << 28);
    GPIOB_CRL = gpio;

    GPIOB_ODR |= GPIO_BUS_LINES;
}

static bool wait_line_high(uint32_t line)
{
    uint32_t iteration;

    for (iteration = 0UL;
         iteration < I2C_LINE_TIMEOUT_ITERATIONS;
         iteration++)
    {
        if ((GPIOB_IDR & line) != 0UL)
        {
            return true;
        }
    }

    return false;
}

static i2c1_status_t gpio_bus_clear(void)
{
    uint32_t pulse;

    set_stage(I2C1_STAGE_RECOVERY_GPIO);

    gpio_configure_bus_clear();
    short_delay();

    /*
     * A slave holding SCL low cannot be clocked free by the master.
     */
    if (!wait_line_high(GPIO_SCL))
    {
        snapshot_registers(I2C1_LINE_STUCK_LOW,
                           I2C1_STAGE_RECOVERY_GPIO);
        count_failure(I2C1_LINE_STUCK_LOW);
        return I2C1_LINE_STUCK_LOW;
    }

    /*
     * If SDA is low, clock the slave up to nine times so an interrupted
     * byte can finish and the slave can release SDA.
     */
    for (pulse = 0UL;
         (pulse < I2C_BUS_CLEAR_PULSES) &&
         ((GPIOB_IDR & GPIO_SDA) == 0UL);
         pulse++)
    {
        GPIOB_ODR &= ~GPIO_SCL;
        short_delay();

        GPIOB_ODR |= GPIO_SCL;

        if (!wait_line_high(GPIO_SCL))
        {
            snapshot_registers(I2C1_LINE_STUCK_LOW,
                               I2C1_STAGE_RECOVERY_GPIO);
            count_failure(I2C1_LINE_STUCK_LOW);
            return I2C1_LINE_STUCK_LOW;
        }

        short_delay();
    }

    /*
     * Synthesize a STOP on the physical bus:
     * SDA low while SCL high, then release SDA high.
     */
    GPIOB_ODR &= ~GPIO_SDA;
    short_delay();

    GPIOB_ODR |= GPIO_SCL;
    if (!wait_line_high(GPIO_SCL))
    {
        snapshot_registers(I2C1_LINE_STUCK_LOW,
                           I2C1_STAGE_RECOVERY_GPIO);
        count_failure(I2C1_LINE_STUCK_LOW);
        return I2C1_LINE_STUCK_LOW;
    }

    short_delay();
    GPIOB_ODR |= GPIO_SDA;
    short_delay();

    if ((GPIOB_IDR & GPIO_BUS_LINES) != GPIO_BUS_LINES)
    {
        snapshot_registers(I2C1_LINE_STUCK_LOW,
                           I2C1_STAGE_RECOVERY_GPIO);
        count_failure(I2C1_LINE_STUCK_LOW);
        return I2C1_LINE_STUCK_LOW;
    }

    return I2C1_OK;
}

static i2c1_status_t calculate_timing(uint32_t pclk1_hz,
                                      uint32_t bus_hz,
                                      uint32_t *cr2,
                                      uint32_t *ccr,
                                      uint32_t *trise)
{
    uint32_t pclk_mhz;
    uint32_t ccr_value;

    if ((pclk1_hz == 0UL) ||
        (bus_hz == 0UL) ||
        (cr2 == NULL) ||
        (ccr == NULL) ||
        (trise == NULL))
    {
        return I2C1_INVALID_ARGUMENT;
    }

    pclk_mhz = pclk1_hz / 1000000UL;

    if ((pclk_mhz < 2UL) ||
        (pclk_mhz > 36UL) ||
        ((pclk1_hz % 1000000UL) != 0UL) ||
        (bus_hz > 400000UL))
    {
        return I2C1_UNSUPPORTED_CLOCK;
    }

    *cr2 = pclk_mhz;

    if (bus_hz <= 100000UL)
    {
        /*
         * Standard mode: fSCL = PCLK1 / (2 * CCR). Use ceiling division so
         * the realized bus clock never exceeds the requested value.
         */
        {
            uint32_t denominator = bus_hz * 2UL;
            ccr_value = (pclk1_hz + denominator - 1UL) / denominator;
        }

        if (ccr_value < 4UL)
        {
            ccr_value = 4UL;
        }

        if (ccr_value > 0x0FFFUL)
        {
            return I2C1_UNSUPPORTED_CLOCK;
        }

        *ccr = ccr_value;

        /* Standard-mode maximum rise time = 1000 ns. */
        *trise = pclk_mhz + 1UL;
    }
    else
    {
        if (pclk_mhz < 4UL)
        {
            return I2C1_UNSUPPORTED_CLOCK;
        }

        /*
         * Fast mode, duty cycle 2: fSCL = PCLK1 / (3 * CCR). Again use
         * ceiling division so wiring margin is not reduced by rounding up
         * the actual SCL frequency.
         */
        {
            uint32_t denominator = bus_hz * 3UL;
            ccr_value = (pclk1_hz + denominator - 1UL) / denominator;
        }

        if ((ccr_value < 1UL) ||
            (ccr_value > 0x0FFFUL))
        {
            return I2C1_UNSUPPORTED_CLOCK;
        }

        *ccr = I2C_CCR_FAST | ccr_value;

        /* Fast-mode maximum rise time = 300 ns. */
        *trise = ((pclk_mhz * 300UL) / 1000UL) + 1UL;
    }

    return I2C1_OK;
}

static i2c1_status_t configure_peripheral(uint32_t pclk1_hz,
                                          uint32_t bus_hz,
                                          bool use_rcc_reset)
{
    uint32_t cr2;
    uint32_t ccr;
    uint32_t trise;
    i2c1_status_t status;

    status = calculate_timing(pclk1_hz,
                              bus_hz,
                              &cr2,
                              &ccr,
                              &trise);
    if (status != I2C1_OK)
    {
        return status;
    }

    RCC_APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC_APB1ENR;

    /*
     * This project uses the reset/default I2C1 mapping PB6/PB7. Do not do a
     * read-modify-write of AFIO_MAPR here: RM0008 marks SWJ_CFG as write-only
     * (read value undefined), so an unrelated MAPR RMW can disturb debug pins.
     */

    gpio_configure_af_open_drain();

    if (use_rcc_reset)
    {
        RCC_APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC_APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    }

    /* Timing registers must be programmed while PE=0. */
    I2C1_CR1 = 0UL;
    I2C1_CR2 = cr2;

    /* RM0008: OAR1 bit 14 must always be kept at 1 by software. */
    I2C1_OAR1 = I2C_OAR1_BIT14;
    I2C1_OAR2 = 0UL;

    I2C1_CCR = ccr;
    I2C1_TRISE = trise;

    clear_error_flags();

    I2C1_CR1 = I2C_CR1_PE | I2C_CR1_ACK;

    return wait_bus_idle_raw();
}

static i2c1_status_t request_stop_if_master(void)
{
    uint32_t sr2;

    set_stage(I2C1_STAGE_RECOVERY_STOP);

    sr2 = I2C1_SR2;

    if (((sr2 & I2C_SR2_MSL) != 0UL) &&
        ((I2C1_CR1 & I2C_CR1_STOP) == 0UL))
    {
        /*
         * RM0008: after setting STOP, do not write CR1 again until hardware
         * clears STOP. This function therefore performs no further CR1 write.
         */
        I2C1_CR1 |= I2C_CR1_STOP;
    }

    return wait_stop_complete();
}

static i2c1_status_t recover_internal(void)
{
    i2c1_status_t status;

    if (!configuration_valid)
    {
        g_i2c1_diag.last_recovery_status = I2C1_RECOVERY_FAILED;
        return I2C1_RECOVERY_FAILED;
    }

    g_i2c1_diag.recoveries++;
    g_i2c1_diag.last_recovery_status = I2C1_OK;

    /*
     * First try to terminate a master transfer cleanly. Failure here is not
     * fatal; the following GPIO/SWRST path is specifically for locked states.
     */
    (void)request_stop_if_master();

    clear_error_flags();

    /*
     * Take direct control of SCL/SDA first. If STOP is still stuck, avoid
     * ordinary CR1 writes; once the physical lines are released we use the
     * RM0008 SWRST mechanism to reset the peripheral state machine.
     */
    status = gpio_bus_clear();
    if (status != I2C1_OK)
    {
        g_i2c1_diag.recovery_failures++;
        g_i2c1_diag.last_recovery_status = status;
        gpio_configure_af_open_drain();
        return status;
    }

    gpio_configure_af_open_drain();

    /*
     * RM0008 26.6.1 explicitly permits SWRST to recover from an error or a
     * BUSY bit locked by a bus glitch. The lines have been released first.
     */
    set_stage(I2C1_STAGE_RECOVERY_SWRST);
    I2C1_CR1 = I2C_CR1_SWRST;
    short_delay();
    I2C1_CR1 = 0UL;

    set_stage(I2C1_STAGE_RECOVERY_REINIT);
    status = configure_peripheral(configured_pclk1_hz,
                                  configured_bus_hz,
                                  false);

    if (status != I2C1_OK)
    {
        g_i2c1_diag.recovery_failures++;
        g_i2c1_diag.last_recovery_status = I2C1_RECOVERY_FAILED;
        snapshot_registers(I2C1_RECOVERY_FAILED,
                           I2C1_STAGE_RECOVERY_REINIT);
        return I2C1_RECOVERY_FAILED;
    }

    g_i2c1_diag.last_recovery_status = I2C1_OK;
    g_i2c1_diag.last_status = I2C1_OK;
    set_stage(I2C1_STAGE_IDLE);
    return I2C1_OK;
}

static i2c1_status_t start_address(uint8_t device_address,
                                   bool read,
                                   bool repeated_start)
{
    i2c1_status_t status;

    set_stage(repeated_start ?
              I2C1_STAGE_REPEATED_START :
              I2C1_STAGE_START);

    I2C1_CR1 |= I2C_CR1_START;

    status = wait_sr1(I2C_SR1_SB,
                      I2C1_STAGE_WAIT_SB);
    if (status != I2C1_OK)
    {
        return status;
    }

    set_stage(I2C1_STAGE_ADDRESS);
    I2C1_DR = ((uint32_t)device_address << 1) |
              (read ? 1UL : 0UL);

    return wait_sr1(I2C_SR1_ADDR,
                    I2C1_STAGE_WAIT_ADDR);
}

static i2c1_status_t write_register_once(uint8_t device_address,
                                         uint8_t register_address,
                                         uint8_t value)
{
    i2c1_status_t status;

    status = wait_bus_idle_raw();
    if (status != I2C1_OK)
    {
        return status;
    }

    status = start_address(device_address,
                           false,
                           false);
    if (status != I2C1_OK)
    {
        return status;
    }

    clear_addr();

    status = wait_sr1(I2C_SR1_TXE,
                      I2C1_STAGE_WAIT_TXE);
    if (status != I2C1_OK)
    {
        return status;
    }

    set_stage(I2C1_STAGE_WRITE_REGISTER);
    I2C1_DR = register_address;

    status = wait_sr1(I2C_SR1_TXE,
                      I2C1_STAGE_WAIT_TXE);
    if (status != I2C1_OK)
    {
        return status;
    }

    set_stage(I2C1_STAGE_WRITE_DATA);
    I2C1_DR = value;

    status = wait_sr1(I2C_SR1_BTF,
                      I2C1_STAGE_WAIT_BTF);
    if (status != I2C1_OK)
    {
        return status;
    }

    /* RM0008 EV8_2: STOP is programmed when TxE/BTF is set. */
    set_stage(I2C1_STAGE_STOP);
    I2C1_CR1 |= I2C_CR1_STOP;

    /* Do not write CR1 again until STOP has been cleared by hardware. */
    return wait_stop_complete();
}

static i2c1_status_t read_registers_once(uint8_t device_address,
                                         uint8_t first_register,
                                         uint8_t *data,
                                         size_t length)
{
    i2c1_status_t status;
    size_t remaining;
    uint32_t primask;

    status = wait_bus_idle_raw();
    if (status != I2C1_OK)
    {
        return status;
    }

    /* Select the first register using a master-transmitter phase. */
    status = start_address(device_address,
                           false,
                           false);
    if (status != I2C1_OK)
    {
        return status;
    }

    clear_addr();

    status = wait_sr1(I2C_SR1_TXE,
                      I2C1_STAGE_WAIT_TXE);
    if (status != I2C1_OK)
    {
        return status;
    }

    set_stage(I2C1_STAGE_WRITE_REGISTER);
    I2C1_DR = first_register;

    /* BTF ensures the register-address byte has fully left the peripheral. */
    status = wait_sr1(I2C_SR1_BTF,
                      I2C1_STAGE_WAIT_BTF);
    if (status != I2C1_OK)
    {
        return status;
    }

    /* Prepare the receiver state before the repeated START. */
    I2C1_CR1 &= ~I2C_CR1_POS;
    I2C1_CR1 |= I2C_CR1_ACK;

    status = start_address(device_address,
                           true,
                           true);
    if (status != I2C1_OK)
    {
        return status;
    }

    if (length == 1U)
    {
        /*
         * RM0008 method 2, N=1:
         * clear ACK in the ADDR event, clear ADDR, then program STOP.
         * Keep this short sequence atomic so another ISR cannot delay it.
         */
        primask = enter_critical();

        I2C1_CR1 &= ~I2C_CR1_ACK;
        clear_addr();
        I2C1_CR1 |= I2C_CR1_STOP;

        exit_critical(primask);

        status = wait_sr1(I2C_SR1_RXNE,
                          I2C1_STAGE_WAIT_RXNE);

        if (status == I2C1_OK)
        {
            set_stage(I2C1_STAGE_READ_DATA);
            data[0] = (uint8_t)I2C1_DR;
            status = wait_stop_complete();
        }
    }
    else if (length == 2U)
    {
        /*
         * RM0008 method 2, N=2:
         * POS=1 and ACK=1 before clearing ADDR, then clear ACK immediately
         * after ADDR is cleared. Wait BTF, request STOP, read DR twice.
         */
        I2C1_CR1 |= I2C_CR1_POS;
        I2C1_CR1 |= I2C_CR1_ACK;

        primask = enter_critical();
        clear_addr();
        I2C1_CR1 &= ~I2C_CR1_ACK;
        exit_critical(primask);

        status = wait_sr1(I2C_SR1_BTF,
                          I2C1_STAGE_WAIT_BTF);

        if (status == I2C1_OK)
        {
            primask = enter_critical();

            I2C1_CR1 |= I2C_CR1_STOP;
            set_stage(I2C1_STAGE_READ_DATA);
            data[0] = (uint8_t)I2C1_DR;
            data[1] = (uint8_t)I2C1_DR;

            exit_critical(primask);

            status = wait_stop_complete();
        }
    }
    else
    {
        /*
         * RM0008 method 2, N>2. Read normally until three bytes remain.
         * At three remaining bytes, BTF stretches SCL so the ACK/STOP
         * sequence can be completed safely in polling mode.
         */
        clear_addr();
        remaining = length;

        while ((remaining > 3U) &&
               (status == I2C1_OK))
        {
            status = wait_sr1(I2C_SR1_RXNE,
                              I2C1_STAGE_WAIT_RXNE);

            if (status == I2C1_OK)
            {
                set_stage(I2C1_STAGE_READ_DATA);
                *data++ = (uint8_t)I2C1_DR;
                remaining--;
            }
        }

        if (status == I2C1_OK)
        {
            status = wait_sr1(I2C_SR1_BTF,
                              I2C1_STAGE_WAIT_BTF);
        }

        if (status == I2C1_OK)
        {
            primask = enter_critical();

            /* EV7_2: ACK=0, read N-2, STOP, read N-1. */
            I2C1_CR1 &= ~I2C_CR1_ACK;

            set_stage(I2C1_STAGE_READ_DATA);
            *data++ = (uint8_t)I2C1_DR;
            remaining--;

            I2C1_CR1 |= I2C_CR1_STOP;

            *data++ = (uint8_t)I2C1_DR;
            remaining--;

            exit_critical(primask);

            status = wait_sr1(I2C_SR1_RXNE,
                              I2C1_STAGE_WAIT_RXNE);
        }

        if (status == I2C1_OK)
        {
            set_stage(I2C1_STAGE_READ_DATA);
            *data = (uint8_t)I2C1_DR;
            remaining--;
            status = wait_stop_complete();
        }

        (void)remaining;
    }

    /*
     * RM0008 forbids CR1 writes while STOP is still set. Only restore the
     * default receiver state after STOP has completed successfully.
     */
    if (status == I2C1_OK)
    {
        I2C1_CR1 &= ~I2C_CR1_POS;
        I2C1_CR1 |= I2C_CR1_ACK;
    }

    return status;
}

static i2c1_status_t recover_and_retry(i2c1_status_t original_status)
{
    i2c1_status_t recovery_status;

    (void)original_status;

    recovery_status = recover_internal();
    g_i2c1_diag.last_recovery_status = recovery_status;

    if (recovery_status != I2C1_OK)
    {
        snapshot_registers(I2C1_RECOVERY_FAILED,
                           I2C1_STAGE_RECOVERY_REINIT);
        return I2C1_RECOVERY_FAILED;
    }

    return I2C1_OK;
}

i2c1_status_t i2c1_init(uint32_t pclk1_hz,
                        uint32_t bus_hz)
{
    uint32_t cr2;
    uint32_t ccr;
    uint32_t trise;
    i2c1_status_t status;

    g_i2c1_diag = (i2c1_diag_t){0};
    configuration_valid = false;
    set_stage(I2C1_STAGE_INIT);

    /* Validate timing before remembering the configuration. */
    status = calculate_timing(pclk1_hz,
                              bus_hz,
                              &cr2,
                              &ccr,
                              &trise);
    if (status != I2C1_OK)
    {
        snapshot_registers(status, I2C1_STAGE_INIT);
        return status;
    }

    configured_pclk1_hz = pclk1_hz;
    configured_bus_hz = bus_hz;
    configuration_valid = true;
    g_i2c1_diag.bus_hz = bus_hz;

    status = configure_peripheral(pclk1_hz,
                                  bus_hz,
                                  true);

    if (status == I2C1_OK)
    {
        g_i2c1_diag.last_status = I2C1_OK;
        set_stage(I2C1_STAGE_IDLE);
        return I2C1_OK;
    }

    /* A debugger reset or interrupted slave transfer may leave BUSY locked. */
    status = recover_internal();

    if (status != I2C1_OK)
    {
        snapshot_registers(status,
                           I2C1_STAGE_RECOVERY_REINIT);
        return status;
    }

    return I2C1_OK;
}

i2c1_status_t i2c1_recover(void)
{
    return recover_internal();
}

i2c1_status_t i2c1_write_register(uint8_t device_address,
                                  uint8_t register_address,
                                  uint8_t value)
{
    uint32_t attempt;
    i2c1_status_t status = I2C1_RECOVERY_FAILED;

    if ((device_address > 0x7FU) ||
        !configuration_valid)
    {
        return I2C1_INVALID_ARGUMENT;
    }

    g_i2c1_diag.transactions++;
    g_i2c1_diag.device_address = device_address;
    g_i2c1_diag.register_address = register_address;
    g_i2c1_diag.length = 1U;

    for (attempt = 1UL;
         attempt <= I2C_TRANSACTION_ATTEMPTS;
         attempt++)
    {
        g_i2c1_diag.attempt = attempt;

        status = write_register_once(device_address,
                                     register_address,
                                     value);
        if (status == I2C1_OK)
        {
            g_i2c1_diag.successful_transactions++;
            g_i2c1_diag.last_status = I2C1_OK;
            set_stage(I2C1_STAGE_IDLE);
            return I2C1_OK;
        }

        if (attempt == I2C_TRANSACTION_ATTEMPTS)
        {
            break;
        }

        g_i2c1_diag.retries++;

        if (recover_and_retry(status) != I2C1_OK)
        {
            return I2C1_RECOVERY_FAILED;
        }
    }

    /*
     * Leave the peripheral/bus in a known state even when this API call must
     * report failure. This makes the next 500 Hz sample independent of the
     * failed transaction rather than inheriting a half-finished state.
     */
    if (recover_internal() != I2C1_OK)
    {
        g_i2c1_diag.last_status = I2C1_RECOVERY_FAILED;
        return I2C1_RECOVERY_FAILED;
    }

    g_i2c1_diag.last_status = status;
    return status;
}

i2c1_status_t i2c1_read_registers(uint8_t device_address,
                                  uint8_t first_register,
                                  uint8_t *data,
                                  size_t length)
{
    uint32_t attempt;
    i2c1_status_t status = I2C1_RECOVERY_FAILED;

    if ((device_address > 0x7FU) ||
        (data == NULL) ||
        (length == 0U) ||
        (length > 0xFFFFU) ||
        !configuration_valid)
    {
        return I2C1_INVALID_ARGUMENT;
    }

    g_i2c1_diag.transactions++;
    g_i2c1_diag.device_address = device_address;
    g_i2c1_diag.register_address = first_register;
    g_i2c1_diag.length = (uint16_t)length;

    for (attempt = 1UL;
         attempt <= I2C_TRANSACTION_ATTEMPTS;
         attempt++)
    {
        g_i2c1_diag.attempt = attempt;

        status = read_registers_once(device_address,
                                     first_register,
                                     data,
                                     length);
        if (status == I2C1_OK)
        {
            g_i2c1_diag.successful_transactions++;
            g_i2c1_diag.last_status = I2C1_OK;
            set_stage(I2C1_STAGE_IDLE);
            return I2C1_OK;
        }

        if (attempt == I2C_TRANSACTION_ATTEMPTS)
        {
            break;
        }

        g_i2c1_diag.retries++;

        if (recover_and_retry(status) != I2C1_OK)
        {
            return I2C1_RECOVERY_FAILED;
        }
    }

    /* Same cleanup policy as writes: report the sample as failed, but recover
     * the bus so the following transaction starts from a defined idle state. */
    if (recover_internal() != I2C1_OK)
    {
        g_i2c1_diag.last_status = I2C1_RECOVERY_FAILED;
        return I2C1_RECOVERY_FAILED;
    }

    g_i2c1_diag.last_status = status;
    return status;
}

void i2c1_get_diag(i2c1_diag_t *result)
{
    uint32_t primask;

    if (result == NULL)
    {
        return;
    }

    /*
     * Copy fields explicitly.
     *
     * Do not return this large structure by value because
     * GCC may generate a memcpy() call in a freestanding
     * bare-metal build.
     */
    primask = enter_critical();

    result->transactions =
        g_i2c1_diag.transactions;

    result->successful_transactions =
        g_i2c1_diag.successful_transactions;

    result->retries =
        g_i2c1_diag.retries;

    result->recoveries =
        g_i2c1_diag.recoveries;

    result->recovery_failures =
        g_i2c1_diag.recovery_failures;


    result->timeouts =
        g_i2c1_diag.timeouts;

    result->nacks =
        g_i2c1_diag.nacks;

    result->bus_busy_errors =
        g_i2c1_diag.bus_busy_errors;

    result->bus_errors =
        g_i2c1_diag.bus_errors;

    result->arbitration_losses =
        g_i2c1_diag.arbitration_losses;

    result->overruns =
        g_i2c1_diag.overruns;

    result->line_stuck_errors =
        g_i2c1_diag.line_stuck_errors;


    result->last_status =
        g_i2c1_diag.last_status;

    result->stage =
        g_i2c1_diag.stage;

    result->attempt =
        g_i2c1_diag.attempt;


    result->last_error_status =
        g_i2c1_diag.last_error_status;

    result->last_error_stage =
        g_i2c1_diag.last_error_stage;

    result->last_error_attempt =
        g_i2c1_diag.last_error_attempt;

    result->last_recovery_status =
        g_i2c1_diag.last_recovery_status;


    result->cr1 =
        g_i2c1_diag.cr1;

    result->sr1 =
        g_i2c1_diag.sr1;

    result->sr2 =
        g_i2c1_diag.sr2;

    result->dr =
        g_i2c1_diag.dr;

    result->gpio_idr =
        g_i2c1_diag.gpio_idr;

    result->bus_hz =
        g_i2c1_diag.bus_hz;


    result->device_address =
        g_i2c1_diag.device_address;

    result->register_address =
        g_i2c1_diag.register_address;

    result->length =
        g_i2c1_diag.length;

    exit_critical(primask);
}
