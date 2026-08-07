#include "mpu9250.h"

#include "i2c1.h"
#include "micros.h"

#include <stddef.h>
#include <stdint.h>

#define MPU9250_DEFAULT_ADDRESS 0x68U
#define AK8963_ADDRESS          0x0CU

#define MPU_REG_SMPLRT_DIV      0x19U
#define MPU_REG_CONFIG          0x1AU
#define MPU_REG_GYRO_CONFIG     0x1BU
#define MPU_REG_ACCEL_CONFIG    0x1CU
#define MPU_REG_ACCEL_CONFIG2   0x1DU
#define MPU_REG_INT_PIN_CFG     0x37U
#define MPU_REG_INT_ENABLE      0x38U
#define MPU_REG_ACCEL_XOUT_H    0x3BU
#define MPU_REG_PWR_MGMT_1      0x6BU
#define MPU_REG_PWR_MGMT_2      0x6CU
#define MPU_REG_WHO_AM_I        0x75U

#define AK_REG_WIA              0x00U
#define AK_REG_ST1              0x02U
#define AK_REG_HXL              0x03U
#define AK_REG_CNTL1            0x0AU
#define AK_REG_ASAX             0x10U

#define MPU6500_EXPECTED_ID     0x70U
#define MPU9250_EXPECTED_ID     0x71U
#define AK8963_EXPECTED_ID      0x48U

#define MPU_RESET               0x80U
#define MPU_CLOCK_PLL_XGYRO     0x01U
#define MPU_DLPF_41HZ           0x03U
#define MPU_GYRO_500DPS         0x08U
#define MPU_ACCEL_4G            0x08U
#define MPU_BYPASS_ENABLE       0x02U
#define MPU_DATA_READY_ENABLE   0x01U

#define AK_MODE_POWER_DOWN      0x00U
#define AK_MODE_FUSE_ROM        0x0FU
#define AK_MODE_100HZ_16BIT     0x16U
#define AK_ST1_DRDY             0x01U
#define AK_ST2_HOFL             0x08U

#if IMU_MODEL == IMU_MODEL_MPU6500

#define SELECTED_EXPECTED_ID MPU6500_EXPECTED_ID
#define SELECTED_DEVICE      MPU_DEVICE_MPU6500

#else

#define SELECTED_EXPECTED_ID MPU9250_EXPECTED_ID
#define SELECTED_DEVICE      MPU_DEVICE_MPU9250

#endif

static uint8_t mpu_address =
    MPU9250_DEFAULT_ADDRESS;

static uint8_t mpu_id;
static uint8_t mag_id;

static mpu_device_t detected_device =
    MPU_DEVICE_UNKNOWN;

static ak8963_factory_adjustment_t
    mag_adjustment;
static mpu_init_diag_t init_diag;
static mpu9250_status_t read_bytes(
    uint8_t device,
    uint8_t reg,
    uint8_t *data,
    size_t length)
{
    i2c1_status_t bus_status;

    init_diag.phase =
        MPU_DIAG_PHASE_READ;

    init_diag.device_address =
        device;

    init_diag.register_address =
        reg;

    bus_status =
        i2c1_read_registers(
            device,
            reg,
            data,
            length);

    init_diag.i2c_status =
        (uint32_t)bus_status;

    if (bus_status != I2C1_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    return MPU9250_OK;
}

static mpu9250_status_t write_byte(
    uint8_t device,
    uint8_t reg,
    uint8_t value)
{
    i2c1_status_t bus_status;

    init_diag.phase =
        MPU_DIAG_PHASE_WRITE;

    init_diag.device_address =
        device;

    init_diag.register_address =
        reg;

    init_diag.written_value =
        value;

    bus_status =
        i2c1_write_register(
            device,
            reg,
            value);

    init_diag.i2c_status =
        (uint32_t)bus_status;

    if (bus_status != I2C1_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    return MPU9250_OK;
}

static mpu9250_status_t write_and_verify(
    mpu_init_stage_t stage,
    uint8_t reg,
    uint8_t value,
    uint8_t mask)
{
    uint8_t readback;
    mpu9250_status_t status;

    /*
     * Remember what we are currently configuring.
     */
    init_diag.stage =
        stage;

    init_diag.register_address =
        reg;

    init_diag.written_value =
        value;

    init_diag.readback_value =
        0U;

    init_diag.mask =
        mask;

    /*
     * Write configuration register.
     */
    status =
        write_byte(
            mpu_address,
            reg,
            value);

    if (status != MPU9250_OK)
    {
        return status;
    }

    /*
     * Read the same register back.
     */
    status =
        read_bytes(
            mpu_address,
            reg,
            &readback,
            1U);

    if (status != MPU9250_OK)
    {
        return status;
    }

    init_diag.readback_value =
        readback;

    /*
     * I2C succeeded, but now verify that the
     * register actually contains what we requested.
     */
    init_diag.phase =
        MPU_DIAG_PHASE_VERIFY;

    if ((readback & mask) !=
        (value & mask))
    {
        return MPU9250_CONFIGURATION_ERROR;
    }

    return MPU9250_OK;
}

static int16_t decode_be(
    const uint8_t *bytes)
{
    return (int16_t)(
        ((uint16_t)bytes[0] << 8) |
        bytes[1]);
}

#if IMU_MODEL == IMU_MODEL_MPU9250

static int16_t decode_le(
    const uint8_t *bytes)
{
    return (int16_t)(
        ((uint16_t)bytes[1] << 8) |
        bytes[0]);
}

static mpu9250_status_t ak8963_init(void)
{
    uint8_t asa[3];
    mpu9250_status_t status;

    /*
     * Enable bypass mode so STM32 can communicate
     * directly with AK8963 at address 0x0C.
     */
    status = write_and_verify(
    MPU_INIT_STAGE_INT_PIN_CFG,
    MPU_REG_INT_PIN_CFG,
    MPU_BYPASS_ENABLE,
    0xF2U);

    if (status != MPU9250_OK)
    {
        return status;
    }

    delay_ms(10UL);

    delay_ms(10UL);

    status = read_bytes(
        AK8963_ADDRESS,
        AK_REG_WIA,
        &mag_id,
        1U);

    if (status != MPU9250_OK)
    {
        return status;
    }

    if (mag_id != AK8963_EXPECTED_ID)
    {
        return MPU9250_ID_MISMATCH;
    }

    if (write_byte(
            AK8963_ADDRESS,
            AK_REG_CNTL1,
            AK_MODE_POWER_DOWN) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    delay_ms(10UL);

    if (write_byte(
            AK8963_ADDRESS,
            AK_REG_CNTL1,
            AK_MODE_FUSE_ROM) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    delay_ms(10UL);

    if (read_bytes(
            AK8963_ADDRESS,
            AK_REG_ASAX,
            asa,
            3U) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    mag_adjustment.x = asa[0];
    mag_adjustment.y = asa[1];
    mag_adjustment.z = asa[2];

    if (write_byte(
            AK8963_ADDRESS,
            AK_REG_CNTL1,
            AK_MODE_POWER_DOWN) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    delay_ms(10UL);

    if (write_byte(
            AK8963_ADDRESS,
            AK_REG_CNTL1,
            AK_MODE_100HZ_16BIT) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    delay_ms(10UL);

    return MPU9250_OK;
}

#endif

mpu9250_status_t mpu9250_init(
    uint8_t address)
{
    mpu9250_status_t status;

    if (address > 0x7FU)
    {
        return MPU9250_INVALID_ARGUMENT;
    }

    mpu_address = address;

    mpu_id = 0U;
    mag_id = 0U;

    detected_device =
        MPU_DEVICE_UNKNOWN;

    mag_adjustment =
        (ak8963_factory_adjustment_t){0};

    init_diag =
    (mpu_init_diag_t){0};

    init_diag.i2c_status =
        (uint32_t)I2C1_OK;

    /*
     * Reset the main IMU.
     */
     init_diag.stage = MPU_INIT_STAGE_RESET;
    if (write_byte(
            mpu_address,
            MPU_REG_PWR_MGMT_1,
            MPU_RESET) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    delay_ms(100UL);

    /*
     * Read main device identity.
     */
     init_diag.stage = MPU_INIT_STAGE_WHO_AM_I;
    status = read_bytes(
        mpu_address,
        MPU_REG_WHO_AM_I,
        &mpu_id,
        1U);

    if (status != MPU9250_OK)
    {
        return status;
    }

    /*
     * Build selects the expected identity:
     *
     * MPU6500 → 0x70
     * MPU9250 → 0x71
     */
    if (mpu_id != SELECTED_EXPECTED_ID)
    {
        return MPU9250_ID_MISMATCH;
    }

    detected_device = SELECTED_DEVICE;

    /*
     * Use gyro PLL as sensor clock source.
     */
   status = write_and_verify(
    MPU_INIT_STAGE_PWR_MGMT_1,
    MPU_REG_PWR_MGMT_1,
    MPU_CLOCK_PLL_XGYRO,
    0x7FU);

    if (status != MPU9250_OK)
    {
        return status;
    }

    delay_ms(10UL);

    /*
     * Common MPU6500/MPU9250 motion configuration.
     */
    status = write_and_verify(
    MPU_INIT_STAGE_PWR_MGMT_2,
    MPU_REG_PWR_MGMT_2,
    0x00U,
    0x3FU);

    if (status != MPU9250_OK)
    {
        return status;
    }


    status = write_and_verify(
    MPU_INIT_STAGE_CONFIG,
    MPU_REG_CONFIG,
    MPU_DLPF_41HZ,
    0x07U);

    if (status != MPU9250_OK)
    {
        return status;
    }


    status = write_and_verify(
    MPU_INIT_STAGE_SMPLRT_DIV,
    MPU_REG_SMPLRT_DIV,
    1U,
    0xFFU);

    if (status != MPU9250_OK)
    {
        return status;
    }

    status = write_and_verify(
    MPU_INIT_STAGE_GYRO_CONFIG,
    MPU_REG_GYRO_CONFIG,
    MPU_GYRO_500DPS,
    0x1BU);

    if (status != MPU9250_OK)
    {
        return status;
    }

    status = write_and_verify(
    MPU_INIT_STAGE_ACCEL_CONFIG,
    MPU_REG_ACCEL_CONFIG,
    MPU_ACCEL_4G,
    0x18U);

    if (status != MPU9250_OK)
    {
        return status;
    }

    status = write_and_verify(
    MPU_INIT_STAGE_ACCEL_CONFIG2,
    MPU_REG_ACCEL_CONFIG2,
    MPU_DLPF_41HZ,
    0x0FU);

    if (status != MPU9250_OK)
    {
        return status;
    }


    status = write_and_verify(
    MPU_INIT_STAGE_INT_ENABLE,
    MPU_REG_INT_ENABLE,
    0x00U,
    0x01U);

    if (status != MPU9250_OK)
    {
        return status;
    }

    

#if IMU_MODEL == IMU_MODEL_MPU9250

    /*
     * Only MPU9250 has the internal AK8963.
     */
    return ak8963_init();

#else

    /*
     * MPU6500 has no internal magnetometer.
     * Keep bypass disabled and finish initialization.
     */
    status = write_and_verify(
    MPU_INIT_STAGE_INT_PIN_CFG,
    MPU_REG_INT_PIN_CFG,
    0x00U,
    0xF2U);

    if (status != MPU9250_OK)
    {
        return status;
    }

    return MPU9250_OK;

#endif
}

mpu9250_status_t
mpu9250_enable_data_ready_interrupt(void)
{
    return write_and_verify(
    MPU_INIT_STAGE_INT_ENABLE,
    MPU_REG_INT_ENABLE,
    MPU_DATA_READY_ENABLE,
    0x01U);
}

mpu9250_status_t mpu9250_read_motion_raw(
    mpu9250_motion_raw_t *sample)
{
    uint8_t data[14];

    if (sample == NULL)
    {
        return MPU9250_INVALID_ARGUMENT;
    }

    if (read_bytes(
            mpu_address,
            MPU_REG_ACCEL_XOUT_H,
            data,
            sizeof(data)) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    sample->accel_x =
        decode_be(&data[0]);

    sample->accel_y =
        decode_be(&data[2]);

    sample->accel_z =
        decode_be(&data[4]);

    sample->temperature =
        decode_be(&data[6]);

    sample->gyro_x =
        decode_be(&data[8]);

    sample->gyro_y =
        decode_be(&data[10]);

    sample->gyro_z =
        decode_be(&data[12]);

    return MPU9250_OK;
}

mpu9250_status_t mpu9250_read_mag_raw(
    ak8963_raw_t *sample)
{
#if IMU_MODEL == IMU_MODEL_MPU6500

    /*
     * MPU6500 does not contain an AK8963.
     */
    (void)sample;

    return MPU9250_MAG_UNAVAILABLE;

#else

    uint8_t status1;
    uint8_t data[7];

    if (sample == NULL)
    {
        return MPU9250_INVALID_ARGUMENT;
    }

    if (read_bytes(
            AK8963_ADDRESS,
            AK_REG_ST1,
            &status1,
            1U) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    if ((status1 & AK_ST1_DRDY) == 0U)
    {
        return MPU9250_MAG_NOT_READY;
    }

    if (read_bytes(
            AK8963_ADDRESS,
            AK_REG_HXL,
            data,
            sizeof(data)) != MPU9250_OK)
    {
        return MPU9250_COMMUNICATION_ERROR;
    }

    /*
     * Reading ST2 releases the AK8963 data latch.
     */
    if ((data[6] & AK_ST2_HOFL) != 0U)
    {
        return MPU9250_MAG_OVERFLOW;
    }

    sample->x = decode_le(&data[0]);
    sample->y = decode_le(&data[2]);
    sample->z = decode_le(&data[4]);

    return MPU9250_OK;

#endif
}

uint8_t mpu9250_who_am_i(void)
{
    return mpu_id;
}

uint8_t ak8963_who_am_i(void)
{
    return mag_id;
}

ak8963_factory_adjustment_t
ak8963_factory_adjustment(void)
{
    return mag_adjustment;
}

mpu_device_t mpu9250_device(void)
{
    return detected_device;
}

bool mpu9250_has_magnetometer(void)
{
    return detected_device ==
           MPU_DEVICE_MPU9250;
}

mpu_init_diag_t mpu9250_get_init_diag(void)
{
    return init_diag;
}