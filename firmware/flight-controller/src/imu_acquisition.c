#include "imu_acquisition.h"

#include "i2c1.h"
#include "micros.h"
#include "mpu9250.h"

#include <stdbool.h>
#include <stdint.h>

#define REG32(address) (*(volatile uint32_t *)(address))
#define REG8(address)  (*(volatile uint8_t *)(address))

#define RCC_BASE       0x40021000UL
#define GPIOA_BASE     0x40010800UL
#define AFIO_BASE      0x40010000UL
#define EXTI_BASE      0x40010400UL

#define RCC_APB2ENR    REG32(RCC_BASE + 0x18UL)
#define GPIOA_CRL      REG32(GPIOA_BASE + 0x00UL)
#define AFIO_EXTICR1   REG32(AFIO_BASE + 0x08UL)

#define EXTI_IMR       REG32(EXTI_BASE + 0x00UL)
#define EXTI_RTSR      REG32(EXTI_BASE + 0x08UL)
#define EXTI_FTSR      REG32(EXTI_BASE + 0x0CUL)
#define EXTI_PR        REG32(EXTI_BASE + 0x14UL)

#define NVIC_ISER0     REG32(0xE000E100UL)
#define NVIC_IPR_BASE  0xE000E400UL

#define RCC_APB2ENR_AFIOEN (1UL << 0)
#define RCC_APB2ENR_IOPAEN (1UL << 2)

#define EXTI_LINE0          (1UL << 0)
#define NVIC_EXTI0_IRQ      6UL
#define EXTI0_PRIORITY      0x50U

#define MPU9250_I2C_ADDRESS 0x68U
#define I2C_BUS_HZ          400000UL

#ifndef I2C_RECOVERY_TEST
#define I2C_RECOVERY_TEST 0
#endif


static volatile uint32_t pending_events;
static volatile uint32_t last_irq_timestamp_us;
static volatile uint32_t total_interrupts;

static imu_raw_sample_t latest_sample;
static bool latest_available;
static imu_acquisition_stats_t stats;
static mpu9250_status_t last_driver_status;

/*
 * Remember the APB1 clock so I2C1 can be
 * reinitialized later during the runtime test.
 */
static uint32_t acquisition_pclk1_hz;

#if I2C_RECOVERY_TEST

/*
 * This test deliberately performs only ONE
 * automatic I2C1 recovery.
 *
 * That gives us a clean diagnostic result.
 */
static bool i2c_recovery_test_used;

#endif

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

static bool data_ready_exti_init(void)
{
    uint32_t gpio;

    RCC_APB2ENR |= RCC_APB2ENR_AFIOEN |
                   RCC_APB2ENR_IOPAEN;

    (void)RCC_APB2ENR;

    /*
     * PA0:
     * MODE = 00: input
     * CNF  = 01: floating input
     *
     * MPU9250 INT is configured as push-pull.
     */
    gpio = GPIOA_CRL;
    gpio &= ~0xFUL;
    gpio |= 0x4UL;
    GPIOA_CRL = gpio;

    /*
     * EXTI0 source value 0000 selects GPIO port A.
     */
    AFIO_EXTICR1 &= ~0xFUL;

    /*
     * Configure rising-edge triggering.
     */
    EXTI_IMR &= ~EXTI_LINE0;
    EXTI_FTSR &= ~EXTI_LINE0;
    EXTI_RTSR |= EXTI_LINE0;

    /*
     * Clear any pending interrupt before enabling it.
     */
    EXTI_PR = EXTI_LINE0;

    /*
     * Configure EXTI0 interrupt priority and enable IRQ 6.
     */
    REG8(NVIC_IPR_BASE + NVIC_EXTI0_IRQ) =
        EXTI0_PRIORITY;

    NVIC_ISER0 = (1UL << NVIC_EXTI0_IRQ);

    return ((GPIOA_CRL & 0xFUL) == 0x4UL) &&
           ((AFIO_EXTICR1 & 0xFUL) == 0UL) &&
           ((EXTI_RTSR & EXTI_LINE0) != 0UL);
}

imu_acquisition_status_t imu_acquisition_init(
    uint32_t pclk1_hz)
{
    uint32_t primask;

    primask = enter_critical();

    pending_events = 0UL;
    last_irq_timestamp_us = 0UL;
    total_interrupts = 0UL;
    latest_available = false;

    stats = (imu_acquisition_stats_t){0};
    latest_sample = (imu_raw_sample_t){0};
    last_driver_status = MPU9250_OK;

    /*
    * Save PCLK1 for the runtime I2C1 recovery test.
    */
    acquisition_pclk1_hz = pclk1_hz;
    exit_critical(primask);

    #if I2C_RECOVERY_TEST

    /*
    * No recovery has been attempted yet.
    */
    i2c_recovery_test_used = false;

    #endif

    

    if (i2c1_init(pclk1_hz, I2C_BUS_HZ) != I2C1_OK)
    {
        return IMU_ACQUISITION_I2C_INIT_FAILED;
    }

    last_driver_status =
        mpu9250_init(MPU9250_I2C_ADDRESS);

    if (last_driver_status != MPU9250_OK)
    {
        return IMU_ACQUISITION_MPU_INIT_FAILED;
    }

    if (!data_ready_exti_init())
    {
        return IMU_ACQUISITION_EXTI_INIT_FAILED;
    }

    return IMU_ACQUISITION_OK;
}

imu_acquisition_status_t imu_acquisition_start(void)
{
    EXTI_PR = EXTI_LINE0;
    EXTI_IMR |= EXTI_LINE0;

    last_driver_status =
        mpu9250_enable_data_ready_interrupt();

    if (last_driver_status != MPU9250_OK)
    {
        EXTI_IMR &= ~EXTI_LINE0;
        return IMU_ACQUISITION_START_FAILED;
    }

    return IMU_ACQUISITION_OK;
}

imu_acquisition_status_t imu_acquisition_process(void)
{
    uint32_t event_count;
    uint32_t event_timestamp;
    uint32_t primask;

    imu_raw_sample_t candidate;
    mpu9250_status_t mag_status;

    /*
     * Atomically take all pending DATA_RDY events.
     */
    primask = enter_critical();

    event_count = pending_events;
    event_timestamp = last_irq_timestamp_us;
    pending_events = 0UL;

    exit_critical(primask);

    if (event_count == 0UL)
    {
        return IMU_ACQUISITION_NO_DATA;
    }

    /*
     * If several interrupts arrived before processing,
     * only the newest sensor register state can be read.
     */
    if (event_count > 1UL)
    {
        stats.overwritten_events += event_count - 1UL;
    }

    /*
     * Start from the previous sample so the latest valid
     * magnetometer measurement can be retained.
     */
    candidate = latest_sample;

    candidate.flags &=
        ~(IMU_RAW_MOTION_VALID | IMU_RAW_MAG_NEW);

    /*
     * Read accel, temperature and gyro coherently.
     */
    last_driver_status =
        mpu9250_read_motion_raw(&candidate.motion);

   if (last_driver_status != MPU9250_OK)
{
    stats.communication_errors++;

#if I2C_RECOVERY_TEST

    /*
     * =================================================
     * ONE-SHOT I2C1 RECOVERY EXPERIMENT
     * =================================================
     *
     * A runtime MPU motion read failed.
     *
     * We now:
     *
     * 1. Reset/reinitialize ONLY STM32 I2C1.
     * 2. Do NOT reset the MCU.
     * 3. Do NOT call mpu9250_init().
     * 4. Do NOT reset the MPU6500.
     * 5. Retry exactly the same motion read once.
     */

    if (!i2c_recovery_test_used)
    {
        i2c1_status_t recovery_status;

        /*
         * Prevent a second automatic recovery.
         * This keeps the experiment controlled.
         */
        i2c_recovery_test_used = true;

        stats.i2c_recovery_attempts++;

        /*
         * Reset and configure ONLY STM32 I2C1.
         */
        recovery_status =
            i2c1_init(
                acquisition_pclk1_hz,
                I2C_BUS_HZ);

        if (recovery_status == I2C1_OK)
        {
            /*
             * Retry the exact same MPU6500
             * 14-byte motion read once.
             */
            last_driver_status =
                mpu9250_read_motion_raw(
                    &candidate.motion);

            if (last_driver_status == MPU9250_OK)
            {
                /*
                 * Very important result:
                 *
                 * I2C1-only reset recovered
                 * communication.
                 */
                stats.i2c_recovery_successes++;

                /*
                 * Do NOT return.
                 *
                 * Continue normal processing below
                 * using the successfully read sample.
                 */
            }
            else
            {
                /*
                 * I2C1 reset succeeded, but the
                 * MPU read still failed.
                 */
                stats.i2c_recovery_failures++;

                return IMU_ACQUISITION_READ_FAILED;
            }
        }
        else
        {
            /*
             * I2C1 itself could not be
             * reinitialized successfully.
             */
            stats.i2c_recovery_failures++;

            return IMU_ACQUISITION_READ_FAILED;
        }
    }
    else
    {
        /*
         * The one-shot recovery test was already used.
         *
         * Do not reset anything again.
         */
        return IMU_ACQUISITION_READ_FAILED;
    }

#else

    /*
     * Normal production behavior.
     */
    return IMU_ACQUISITION_READ_FAILED;

#endif
}

    candidate.motion_timestamp_us = event_timestamp;
    candidate.flags |= IMU_RAW_MOTION_VALID;

    /*
     * MPU9250 mode has an AK8963 magnetometer.
     * MPU6500 mode has no internal magnetometer, so no I2C
     * transaction to address 0x0C is attempted.
     */
    if (mpu9250_has_magnetometer())
    {
        mag_status = mpu9250_read_mag_raw(&candidate.mag);

        if (mag_status == MPU9250_OK)
        {
            candidate.mag_timestamp_us = micros();

            candidate.flags |=
                IMU_RAW_MAG_VALID |
                IMU_RAW_MAG_NEW;
        }
        else if (mag_status == MPU9250_MAG_NOT_READY)
        {
            stats.magnetometer_not_ready++;
        }
        else if (mag_status == MPU9250_MAG_OVERFLOW)
        {
            stats.magnetometer_overflows++;
            candidate.flags &= ~IMU_RAW_MAG_VALID;
        }
        else
        {
            stats.communication_errors++;
            last_driver_status = mag_status;
        }
    }
    else
    {
        candidate.mag = (ak8963_raw_t){0};
        candidate.mag_timestamp_us = 0UL;
        candidate.flags &=
            ~(IMU_RAW_MAG_VALID | IMU_RAW_MAG_NEW);
    }

    candidate.sequence = latest_sample.sequence + 1UL;

    /*
     * Publish the completed candidate atomically.
     */
    primask = enter_critical();

    latest_sample = candidate;
    latest_available = true;

    stats.samples_published++;
    stats.data_ready_interrupts = total_interrupts;

    exit_critical(primask);

    return IMU_ACQUISITION_OK;
}

bool imu_acquisition_get_latest(imu_raw_sample_t *sample)
{
    uint32_t primask;
    bool available;

    if (sample == NULL)
    {
        return false;
    }

    primask = enter_critical();

    available = latest_available;

    if (available)
    {
        *sample = latest_sample;
    }

    exit_critical(primask);

    return available;
}

imu_acquisition_stats_t imu_acquisition_get_stats(void)
{
    uint32_t primask;
    imu_acquisition_stats_t result;

    primask = enter_critical();

    result = stats;
    result.data_ready_interrupts = total_interrupts;

    exit_critical(primask);

    return result;
}

mpu9250_status_t imu_acquisition_last_driver_status(void)
{
    return last_driver_status;
}

void EXTI0_IRQHandler(void)
{
    if ((EXTI_PR & EXTI_LINE0) != 0UL)
    {
        /*
         * Writing 1 clears the pending interrupt.
         */
        EXTI_PR = EXTI_LINE0;

        /*
         * Timestamp the DATA_RDY edge.
         * Do not perform I2C inside this interrupt.
         */
        last_irq_timestamp_us = micros();

        if (pending_events != 0xFFFFFFFFUL)
        {
            pending_events++;
        }

        total_interrupts++;
    }
}