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
    I2C1_OVERRUN,
    I2C1_LINE_STUCK_LOW,
    I2C1_RECOVERY_FAILED
} i2c1_status_t;

typedef enum
{
    I2C1_STAGE_IDLE = 0,
    I2C1_STAGE_INIT,
    I2C1_STAGE_WAIT_BUS_IDLE,
    I2C1_STAGE_START,
    I2C1_STAGE_WAIT_SB,
    I2C1_STAGE_ADDRESS,
    I2C1_STAGE_WAIT_ADDR,
    I2C1_STAGE_CLEAR_ADDR,
    I2C1_STAGE_WRITE_REGISTER,
    I2C1_STAGE_WRITE_DATA,
    I2C1_STAGE_WAIT_TXE,
    I2C1_STAGE_WAIT_BTF,
    I2C1_STAGE_REPEATED_START,
    I2C1_STAGE_READ_DATA,
    I2C1_STAGE_WAIT_RXNE,
    I2C1_STAGE_STOP,
    I2C1_STAGE_RECOVERY_STOP,
    I2C1_STAGE_RECOVERY_GPIO,
    I2C1_STAGE_RECOVERY_SWRST,
    I2C1_STAGE_RECOVERY_REINIT
} i2c1_stage_t;

typedef struct
{
    uint32_t transactions;
    uint32_t successful_transactions;
    uint32_t retries;
    uint32_t recoveries;
    uint32_t recovery_failures;

    uint32_t timeouts;
    uint32_t nacks;
    uint32_t bus_busy_errors;
    uint32_t bus_errors;
    uint32_t arbitration_losses;
    uint32_t overruns;
    uint32_t line_stuck_errors;

    /* Final result/stage of the latest public I2C operation. */
    i2c1_status_t last_status;
    i2c1_stage_t stage;
    uint32_t attempt;

    /* Last low-level failure, preserved even if recovery+retry succeeds. */
    i2c1_status_t last_error_status;
    i2c1_stage_t last_error_stage;
    uint32_t last_error_attempt;
    i2c1_status_t last_recovery_status;

    /* Register snapshot captured at the last low-level failure. */
    uint32_t cr1;
    uint32_t sr1;
    uint32_t sr2;
    uint32_t dr;
    uint32_t gpio_idr;
    uint32_t bus_hz;

    uint8_t device_address;
    uint8_t register_address;
    uint16_t length;
} i2c1_diag_t;

/*
 * Live diagnostic state. This is intentionally public so it can be
 * inspected directly in GDB:
 *
 *     p g_i2c1_diag
 */
extern volatile i2c1_diag_t g_i2c1_diag;

/* Configure PB6=SCL and PB7=SDA for I2C1 master operation. */
i2c1_status_t i2c1_init(uint32_t pclk1_hz, uint32_t bus_hz);

/*
 * Explicitly recover and reinitialize the already-configured I2C1 bus.
 * Normal read/write functions also perform bounded automatic recovery.
 */
i2c1_status_t i2c1_recover(void);

i2c1_status_t i2c1_write_register(uint8_t device_address,
                                  uint8_t register_address,
                                  uint8_t value);

i2c1_status_t i2c1_read_registers(uint8_t device_address,
                                  uint8_t first_register,
                                  uint8_t *data,
                                  size_t length);

void i2c1_get_diag(i2c1_diag_t *result);

#endif /* I2C1_H */
