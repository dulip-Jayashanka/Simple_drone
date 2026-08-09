#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Compile-time IMU selection.
 *
 * MPU6500:
 *   accelerometer + gyroscope + temperature
 *
 * MPU9250:
 *   accelerometer + gyroscope + temperature + AK8963
 */
#define IMU_MODEL_MPU6500 6500
#define IMU_MODEL_MPU9250 9250

#ifndef IMU_MODEL
#define IMU_MODEL IMU_MODEL_MPU9250
#endif

#if ((IMU_MODEL != IMU_MODEL_MPU6500) && \
     (IMU_MODEL != IMU_MODEL_MPU9250))
#error "IMU_MODEL must be IMU_MODEL_MPU6500 or IMU_MODEL_MPU9250"
#endif

typedef enum
{
    MPU9250_OK = 0,
    MPU9250_INVALID_ARGUMENT,
    MPU9250_COMMUNICATION_ERROR,
    MPU9250_ID_MISMATCH,
    MPU9250_CONFIGURATION_ERROR,
    MPU9250_MAG_NOT_READY,
    MPU9250_MAG_OVERFLOW,
    MPU9250_MAG_UNAVAILABLE
} mpu9250_status_t;

typedef enum
{
    MPU_DEVICE_UNKNOWN = 0,
    MPU_DEVICE_MPU6500,
    MPU_DEVICE_MPU9250
} mpu_device_t;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu9250_motion_raw_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ak8963_raw_t;

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t z;
} ak8963_factory_adjustment_t;

/*
 * Common configuration:
 *
 * Gyroscope:
 *   range = +/-500 degrees/second
 *   DLPF  = approximately 41 Hz
 *
 * Accelerometer:
 *   range = +/-4 g
 *   DLPF  = approximately 41 Hz
 *
 * Motion sample rate:
 *   500 Hz
 *
 * MPU9250 build only:
 *   AK8963 = 100 Hz continuous mode, 16-bit output
 */
mpu9250_status_t mpu9250_init(uint8_t address);

mpu9250_status_t
mpu9250_enable_data_ready_interrupt(void);

mpu9250_status_t mpu9250_read_motion_raw(
    mpu9250_motion_raw_t *sample);

mpu9250_status_t mpu9250_read_mag_raw(
    ak8963_raw_t *sample);

uint8_t mpu9250_who_am_i(void);
uint8_t ak8963_who_am_i(void);

ak8963_factory_adjustment_t
ak8963_factory_adjustment(void);

mpu_device_t mpu9250_device(void);

bool mpu9250_has_magnetometer(void);

/*
 * Initialization stage.
 *
 * This tells us exactly which sensor configuration
 * operation was being performed when an error occurred.
 */
typedef enum
{
    MPU_INIT_STAGE_NONE = 0,

    MPU_INIT_STAGE_RESET,
    MPU_INIT_STAGE_WHO_AM_I,

    MPU_INIT_STAGE_PWR_MGMT_1,
    MPU_INIT_STAGE_PWR_MGMT_2,

    MPU_INIT_STAGE_CONFIG,
    MPU_INIT_STAGE_SMPLRT_DIV,

    MPU_INIT_STAGE_GYRO_CONFIG,
    MPU_INIT_STAGE_ACCEL_CONFIG,
    MPU_INIT_STAGE_ACCEL_CONFIG2,

    MPU_INIT_STAGE_INT_ENABLE,
    MPU_INIT_STAGE_INT_PIN_CFG,

    MPU_INIT_STAGE_AK_WHO_AM_I,
    MPU_INIT_STAGE_AK_MODE,
    MPU_INIT_STAGE_AK_ASA,

    MPU_INIT_STAGE_DATA_READY_ENABLE,
    MPU_INIT_STAGE_RUNTIME_MOTION_READ,
    MPU_INIT_STAGE_RUNTIME_MAG_READ

} mpu_init_stage_t;


/*
 * Operation that failed inside a stage.
 */
typedef enum
{
    MPU_DIAG_PHASE_NONE = 0,

    MPU_DIAG_PHASE_WRITE,
    MPU_DIAG_PHASE_READ,
    MPU_DIAG_PHASE_VERIFY

} mpu_diag_phase_t;


/*
 * Complete initialization diagnostic record.
 */
typedef struct
{
    mpu_init_stage_t stage;

    mpu_diag_phase_t phase;

    uint32_t i2c_status;

    uint8_t device_address;
    uint8_t register_address;

    uint8_t written_value;
    uint8_t readback_value;

    uint8_t mask;

    /* High-level retry number for the current MPU operation. */
    uint32_t attempt;

} mpu_init_diag_t;


/*
 * Return diagnostic information from the latest
 * MPU initialization/configuration operation.
 */
mpu_init_diag_t mpu9250_get_init_diag(void);

#endif /* MPU9250_H */