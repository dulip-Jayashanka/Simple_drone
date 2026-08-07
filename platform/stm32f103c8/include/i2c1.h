#ifndef I2C1_H
#define I2C1_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    I2C1_OK = 0,
    I2C1_INVALID_ARGUMENT,
    I2C1_UNSUPPORTED_CLOCK,
    I2C1_BUS_BUSY,
    I2C1_TIMEOUT,
    I2C1_NACK,
    I2C1_BUS_ERROR,
    I2C1_ARBITRATION_LOST,
    I2C1_OVERRUN
} i2c1_status_t;

/* Configure PB6=SCL and PB7=SDA for 400 kHz I2C1 operation. */
i2c1_status_t i2c1_init(uint32_t pclk1_hz, uint32_t bus_hz);

i2c1_status_t i2c1_write_register(uint8_t device_address,
                                  uint8_t register_address,
                                  uint8_t value);

i2c1_status_t i2c1_read_registers(uint8_t device_address,
                                  uint8_t first_register,
                                  uint8_t *data,
                                  size_t length);

#endif /* I2C1_H */