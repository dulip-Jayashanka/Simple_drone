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
#define GPIOB_ODR      REG32(GPIOB_BASE + 0x0CUL)

#define I2C1_CR1       REG32(I2C1_BASE + 0x00UL)
#define I2C1_CR2       REG32(I2C1_BASE + 0x04UL)
#define I2C1_DR        REG32(I2C1_BASE + 0x10UL)
#define I2C1_SR1       REG32(I2C1_BASE + 0x14UL)
#define I2C1_SR2       REG32(I2C1_BASE + 0x18UL)
#define I2C1_CCR       REG32(I2C1_BASE + 0x1CUL)
#define I2C1_TRISE     REG32(I2C1_BASE + 0x20UL)

#define RCC_APB2ENR_IOPBEN    (1UL << 3)
#define RCC_APB1ENR_I2C1EN    (1UL << 21)
#define RCC_APB1RSTR_I2C1RST  (1UL << 21)

#define I2C_CR1_PE     (1UL << 0)
#define I2C_CR1_START  (1UL << 8)
#define I2C_CR1_STOP   (1UL << 9)
#define I2C_CR1_ACK    (1UL << 10)
#define I2C_CR1_POS    (1UL << 11)

#define I2C_SR1_SB     (1UL << 0)
#define I2C_SR1_ADDR   (1UL << 1)
#define I2C_SR1_BTF    (1UL << 2)
#define I2C_SR1_RXNE   (1UL << 6)
#define I2C_SR1_TXE    (1UL << 7)
#define I2C_SR1_BERR   (1UL << 8)
#define I2C_SR1_ARLO   (1UL << 9)
#define I2C_SR1_AF     (1UL << 10)
#define I2C_SR1_OVR    (1UL << 11)

#define I2C_SR2_BUSY   (1UL << 1)

#define I2C_CCR_FAST   (1UL << 15)
#define I2C_TIMEOUT_ITERATIONS 200000UL

static i2c1_status_t status_from_sr1(uint32_t sr1)
{
    if ((sr1 & I2C_SR1_AF) != 0UL)
    {
        return I2C1_NACK;
    }

    if ((sr1 & I2C_SR1_BERR) != 0UL)
    {
        return I2C1_BUS_ERROR;
    }

    if ((sr1 & I2C_SR1_ARLO) != 0UL)
    {
        return I2C1_ARBITRATION_LOST;
    }

    if ((sr1 & I2C_SR1_OVR) != 0UL)
    {
        return I2C1_OVERRUN;
    }

    return I2C1_OK;
}

static void abort_transfer(void)
{
    I2C1_CR1 |= I2C_CR1_STOP;

    I2C1_SR1 &= ~(I2C_SR1_BERR |
                  I2C_SR1_ARLO |
                  I2C_SR1_AF |
                  I2C_SR1_OVR);

    I2C1_CR1 &= ~I2C_CR1_POS;
    I2C1_CR1 |= I2C_CR1_ACK;
}

static i2c1_status_t wait_sr1(uint32_t mask)
{
    uint32_t iteration;

    for (iteration = 0UL;
         iteration < I2C_TIMEOUT_ITERATIONS;
         iteration++)
    {
        uint32_t sr1 = I2C1_SR1;
        i2c1_status_t error = status_from_sr1(sr1);

        if (error != I2C1_OK)
        {
            abort_transfer();
            return error;
        }

        if ((sr1 & mask) == mask)
        {
            return I2C1_OK;
        }
    }



    abort_transfer();
    return I2C1_TIMEOUT;
}

static i2c1_status_t wait_bus_idle(void)
{
    uint32_t iteration;

    for (iteration = 0UL;
         iteration < I2C_TIMEOUT_ITERATIONS;
         iteration++)
    {
        if ((I2C1_SR2 & I2C_SR2_BUSY) == 0UL)
        {
            return I2C1_OK;
        }
    }

    return I2C1_BUS_BUSY;
}

static void clear_addr(void)
{
    volatile uint32_t discard;

    /*
     * STM32F1 requires SR1 followed by SR2
     * to clear the ADDR flag.
     */
    discard = I2C1_SR1;
    discard = I2C1_SR2;
    (void)discard;
}

static i2c1_status_t start_address(uint8_t device_address,
                                   bool read)
{
    i2c1_status_t status;

    I2C1_CR1 |= I2C_CR1_START;

    status = wait_sr1(I2C_SR1_SB);
    if (status != I2C1_OK)
    {
        return status;
    }

    I2C1_DR = ((uint32_t)device_address << 1) |
              (read ? 1UL : 0UL);

    return wait_sr1(I2C_SR1_ADDR);
}

i2c1_status_t i2c1_init(uint32_t pclk1_hz,
                        uint32_t bus_hz)
{
    uint32_t pclk_mhz;
    uint32_t ccr;
    uint32_t gpio;

    if ((pclk1_hz == 0UL) || (bus_hz == 0UL))
    {
        return I2C1_INVALID_ARGUMENT;
    }

    pclk_mhz = pclk1_hz / 1000000UL;

    if ((pclk_mhz < 4UL) ||
        (pclk_mhz > 36UL) ||
        ((pclk1_hz % 1000000UL) != 0UL) ||
        (bus_hz > 400000UL))
    {
        return I2C1_UNSUPPORTED_CLOCK;
    }

    /*
     * Fast mode, duty cycle 2:
     *
     * fSCL = PCLK1 / (3 × CCR)
     */
    ccr = pclk1_hz / (bus_hz * 3UL);

    if ((ccr < 4UL) || (ccr > 0x0FFFUL))
    {
        return I2C1_UNSUPPORTED_CLOCK;
    }

    RCC_APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC_APB1ENR;

    /*
     * Reset I2C1 so a debugger restart cannot
     * retain a partial transfer.
     */
    RCC_APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC_APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    /*
     * PB6 and PB7:
     * MODE = 11: output mode, 50 MHz
     * CNF  = 11: alternate-function open-drain
     */
    gpio = GPIOB_CRL;

    gpio &= ~((0xFUL << 24) |
              (0xFUL << 28));

    gpio |= (0xFUL << 24) |
            (0xFUL << 28);

    GPIOB_CRL = gpio;

    GPIOB_ODR |= (1UL << 6) |
                 (1UL << 7);

    I2C1_CR1 = 0UL;
    I2C1_CR2 = pclk_mhz;
    I2C1_CCR = I2C_CCR_FAST | ccr;

    /*
     * Fast-mode maximum rise time:
     *
     * TRISE = 300 ns × PCLK1 + 1
     */
    I2C1_TRISE =
        ((pclk_mhz * 300UL) / 1000UL) + 1UL;

    I2C1_CR1 = I2C_CR1_PE | I2C_CR1_ACK;

    return wait_bus_idle();
}

i2c1_status_t i2c1_write_register(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t value)
{
    i2c1_status_t status;

    if (device_address > 0x7FU)
    {
        return I2C1_INVALID_ARGUMENT;
    }

    if (wait_bus_idle() != I2C1_OK)
    {
        return I2C1_BUS_BUSY;
    }

    status = start_address(device_address, false);
    if (status != I2C1_OK)
    {
        return status;
    }

    clear_addr();

    status = wait_sr1(I2C_SR1_TXE);
    if (status != I2C1_OK)
    {
        return status;
    }

    I2C1_DR = register_address;

    status = wait_sr1(I2C_SR1_TXE);
    if (status != I2C1_OK)
    {
        return status;
    }

    I2C1_DR = value;

    status = wait_sr1(I2C_SR1_BTF);
    if (status != I2C1_OK)
    {
        return status;
    }

    I2C1_CR1 |= I2C_CR1_STOP;

    return I2C1_OK;
}

i2c1_status_t i2c1_read_registers(
    uint8_t device_address,
    uint8_t first_register,
    uint8_t *data,
    size_t length)
{
    i2c1_status_t status;
    size_t remaining;

    if ((device_address > 0x7FU) ||
        (data == NULL) ||
        (length == 0U))
    {
        return I2C1_INVALID_ARGUMENT;
    }

    if (wait_bus_idle() != I2C1_OK)
    {
        return I2C1_BUS_BUSY;
    }

    /*
     * First transaction:
     * send device write address and register address.
     */
    status = start_address(device_address, false);
    if (status != I2C1_OK)
    {
        return status;
    }

    clear_addr();

    status = wait_sr1(I2C_SR1_TXE);
    if (status != I2C1_OK)
    {
        return status;
    }

    I2C1_DR = first_register;

    status = wait_sr1(I2C_SR1_BTF);
    if (status != I2C1_OK)
    {
        return status;
    }

    /*
     * Repeated START:
     * change from register selection to reading.
     */
    I2C1_CR1 |= I2C_CR1_ACK;
    I2C1_CR1 &= ~I2C_CR1_POS;

    status = start_address(device_address, true);
    if (status != I2C1_OK)
    {
        return status;
    }

    if (length == 1U)
    {
        I2C1_CR1 &= ~I2C_CR1_ACK;

        clear_addr();

        I2C1_CR1 |= I2C_CR1_STOP;

        status = wait_sr1(I2C_SR1_RXNE);

        if (status == I2C1_OK)
        {
            data[0] = (uint8_t)I2C1_DR;
        }
    }
    else if (length == 2U)
    {
        I2C1_CR1 |= I2C_CR1_POS;
        I2C1_CR1 &= ~I2C_CR1_ACK;

        clear_addr();

        status = wait_sr1(I2C_SR1_BTF);

        if (status == I2C1_OK)
        {
            I2C1_CR1 |= I2C_CR1_STOP;

            data[0] = (uint8_t)I2C1_DR;
            data[1] = (uint8_t)I2C1_DR;
        }
    }
    else
    {
        clear_addr();
        remaining = length;

        while ((remaining > 3U) &&
               (status == I2C1_OK))
        {
            status = wait_sr1(I2C_SR1_RXNE);

            if (status == I2C1_OK)
            {
                *data++ = (uint8_t)I2C1_DR;
                remaining--;
            }
        }

        if (status == I2C1_OK)
        {
            status = wait_sr1(I2C_SR1_BTF);
        }

        if (status == I2C1_OK)
        {
            I2C1_CR1 &= ~I2C_CR1_ACK;

            *data++ = (uint8_t)I2C1_DR;
            remaining--;

            I2C1_CR1 |= I2C_CR1_STOP;

            *data++ = (uint8_t)I2C1_DR;
            remaining--;

            status = wait_sr1(I2C_SR1_RXNE);
        }

        if (status == I2C1_OK)
        {
            *data = (uint8_t)I2C1_DR;
            remaining--;
        }

        (void)remaining;
    }

    I2C1_CR1 &= ~I2C_CR1_POS;
    I2C1_CR1 |= I2C_CR1_ACK;

    return status;
}