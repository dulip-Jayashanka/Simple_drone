#include "i2c1.h"
#include "mpu9250.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static uint8_t magnetometer_status = 0x01U;
static uint8_t magnetometer_status2;

i2c1_status_t i2c1_write_register(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t value)
{
    (void)device_address;
    (void)register_address;
    (void)value;

    return I2C1_OK;
}

i2c1_status_t i2c1_read_registers(
    uint8_t device_address,
    uint8_t first_register,
    uint8_t *data,
    size_t length)
{
    static const uint8_t motion[14] =
    {
        0x12U, 0x34U,
        0xFEU, 0xDCU,
        0x80U, 0x00U,
        0x00U, 0x01U,
        0x7FU, 0xFFU,
        0xFFU, 0xFFU,
        0x01U, 0x02U
    };

    static const uint8_t magnetic[6] =
    {
        0x34U, 0x12U,
        0xDCU, 0xFEU,
        0x00U, 0x80U
    };

    if ((device_address == 0x68U) &&
        (first_register == 0x3BU) &&
        (length == sizeof(motion)))
    {
        memcpy(data, motion, sizeof(motion));
        return I2C1_OK;
    }

    if ((device_address == 0x0CU) &&
        (first_register == 0x02U) &&
        (length == 1U))
    {
        data[0] = magnetometer_status;
        return I2C1_OK;
    }

    if ((device_address == 0x0CU) &&
        (first_register == 0x03U) &&
        (length == 7U))
    {
        memcpy(data, magnetic, sizeof(magnetic));
        data[6] = magnetometer_status2;

        return I2C1_OK;
    }

    return I2C1_NACK;
}

void delay_ms(uint32_t milliseconds)
{
    (void)milliseconds;
}

int main(void)
{
    mpu9250_motion_raw_t motion;
    ak8963_raw_t magnetic;

    /*
     * Test MPU9250 big-endian decoding.
     */
    assert(
        mpu9250_read_motion_raw(&motion) ==
        MPU9250_OK);

    assert(motion.accel_x == 4660);
    assert(motion.accel_y == -292);
    assert(motion.accel_z == -32768);
    assert(motion.temperature == 1);
    assert(motion.gyro_x == 32767);
    assert(motion.gyro_y == -1);
    assert(motion.gyro_z == 258);

    /*
     * Test AK8963 little-endian decoding.
     */
    assert(
        mpu9250_read_mag_raw(&magnetic) ==
        MPU9250_OK);

    assert(magnetic.x == 4660);
    assert(magnetic.y == -292);
    assert(magnetic.z == -32768);

    /*
     * Test magnetometer not-ready result.
     */
    magnetometer_status = 0x00U;

    assert(
        mpu9250_read_mag_raw(&magnetic) ==
        MPU9250_MAG_NOT_READY);

    /*
     * Test magnetometer overflow rejection.
     */
    magnetometer_status = 0x01U;
    magnetometer_status2 = 0x08U;

    assert(
        mpu9250_read_mag_raw(&magnetic) ==
        MPU9250_MAG_OVERFLOW);

    return 0;
}