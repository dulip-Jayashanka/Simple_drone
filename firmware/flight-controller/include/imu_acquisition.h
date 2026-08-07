#ifndef IMU_ACQUISITION_H
#define IMU_ACQUISITION_H

#include "mpu9250.h"

#include <stdbool.h>
#include <stdint.h>

#define IMU_RAW_MOTION_VALID (1UL << 0)
#define IMU_RAW_MAG_VALID    (1UL << 1)
#define IMU_RAW_MAG_NEW      (1UL << 2)

typedef enum
{
    IMU_ACQUISITION_OK = 0,
    IMU_ACQUISITION_NO_DATA,
    IMU_ACQUISITION_INVALID_ARGUMENT,
    IMU_ACQUISITION_I2C_INIT_FAILED,
    IMU_ACQUISITION_MPU_INIT_FAILED,
    IMU_ACQUISITION_EXTI_INIT_FAILED,
    IMU_ACQUISITION_READ_FAILED,
    IMU_ACQUISITION_START_FAILED
} imu_acquisition_status_t;

typedef struct
{
    uint32_t sequence;
    uint32_t motion_timestamp_us;
    uint32_t mag_timestamp_us;
    uint32_t flags;

    mpu9250_motion_raw_t motion;
    ak8963_raw_t mag;
} imu_raw_sample_t;

typedef struct
{
    uint32_t data_ready_interrupts;
    uint32_t samples_published;
    uint32_t overwritten_events;
    uint32_t communication_errors;

    /*
     * Temporary I2C1-only recovery experiment.
     */
    uint32_t i2c_recovery_attempts;
    uint32_t i2c_recovery_successes;
    uint32_t i2c_recovery_failures;

    uint32_t magnetometer_not_ready;
    uint32_t magnetometer_overflows;
} imu_acquisition_stats_t;

imu_acquisition_status_t imu_acquisition_init(
    uint32_t pclk1_hz);

imu_acquisition_status_t imu_acquisition_start(void);

/*
 * Non-blocking:
 * returns NO_DATA unless EXTI0 recorded a DATA_RDY edge.
 */
imu_acquisition_status_t imu_acquisition_process(void);

bool imu_acquisition_get_latest(imu_raw_sample_t *sample);

imu_acquisition_stats_t imu_acquisition_get_stats(void);

mpu9250_status_t imu_acquisition_last_driver_status(void);

/* Strong handler replacing the weak startup implementation. */
void EXTI0_IRQHandler(void);

#endif /* IMU_ACQUISITION_H */