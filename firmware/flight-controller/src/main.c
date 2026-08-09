#include "fault_handlers.h"
#include "imu_acquisition.h"
#include "micros.h"
#include "mpu9250.h"
#include "system_clock.h"
#include "system_time.h"
#include "uart_diag.h"
#include "i2c1.h"
#include <stdbool.h>
#include <stdint.h>

#define UART_BAUD_RATE   115200UL
#define DEBUG_PERIOD_MS  1000UL

#ifndef I2C_BUS_HZ
#define I2C_BUS_HZ 400000UL
#endif

#ifndef ACQ_UART_RAW_DEBUG
#define ACQ_UART_RAW_DEBUG 0
#endif

#ifndef ACCEL_CAPTURE_TEST
#define ACCEL_CAPTURE_TEST 0
#endif


#ifndef MPU_WHO_AM_I_TEST
#define MPU_WHO_AM_I_TEST 0
#endif

#if IMU_INIT_DEBUG
    mpu_init_diag_t init_diag;
    i2c1_diag_t i2c_diag;
#endif
volatile system_clock_status_t g_fc_clock_status;
volatile system_time_status_t g_fc_time_status;
volatile micros_status_t g_fc_micros_status;
volatile uart_diag_status_t g_fc_uart_status;

volatile imu_acquisition_status_t
    g_imu_acquisition_status;

#if ACCEL_CAPTURE_TEST

/*
 * Five-second raw accelerometer capture.
 *
 * Expected sample rate is about 500 Hz.
 *
 * 5 seconds × 500 Hz ≈ 2500 samples.
 *
 * A little extra space is reserved so timing variation
 * does not overflow the buffer.
 */
#define ACCEL_CAPTURE_DURATION_US  5000000UL
#define ACCEL_CAPTURE_MAX_SAMPLES 2600UL

/*
 * 20 minutes of actual recorded accelerometer data.
 *
 * 20 min = 1200 seconds
 *
 * 1200 / 5 = 240 capture blocks.
 */
#define ACCEL_CAPTURE_TOTAL_BLOCKS 240UL

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} accel_capture_sample_t;

static accel_capture_sample_t
    accel_capture_buffer[ACCEL_CAPTURE_MAX_SAMPLES];

static uint32_t accel_capture_count;

#endif


volatile imu_raw_sample_t
    g_latest_raw_imu_sample;

volatile imu_acquisition_stats_t
    g_imu_acquisition_stats;

#if ACQ_UART_RAW_DEBUG

static void write_signed(int16_t value)
{
    int32_t wide = value;

    if (wide < 0)
    {
        (void)uart_diag_write_char('-');
        wide = -wide;
    }

    (void)uart_diag_write_uint32((uint32_t)wide);
}

static void write_separator(void)
{
    (void)uart_diag_write_char(',');
}

static void write_raw_debug_line(
    const imu_raw_sample_t *sample)
{
    (void)uart_diag_write_string("[RAW] seq=");
    (void)uart_diag_write_uint32(sample->sequence);

    (void)uart_diag_write_string(" t_us=");
    (void)uart_diag_write_uint32(
        sample->motion_timestamp_us);

    (void)uart_diag_write_string(" A=");

    write_signed(sample->motion.accel_x);
    write_separator();
    write_signed(sample->motion.accel_y);
    write_separator();
    write_signed(sample->motion.accel_z);

    (void)uart_diag_write_string(" G=");

    write_signed(sample->motion.gyro_x);
    write_separator();
    write_signed(sample->motion.gyro_y);
    write_separator();
    write_signed(sample->motion.gyro_z);

   if (mpu9250_has_magnetometer())
    {
        (void)uart_diag_write_string(" M=");

        write_signed(sample->mag.x);
        write_separator();

        write_signed(sample->mag.y);
        write_separator();

        write_signed(sample->mag.z);
    }

    (void)uart_diag_write_string(" flags=");
    (void)uart_diag_write_hex32(sample->flags);

    (void)uart_diag_write_string("\r\n");
}

static void write_i2c_health_debug_line(
    const i2c1_diag_t *diag)
{
    (void)uart_diag_write_string("[I2C] tx=");
    (void)uart_diag_write_uint32(diag->transactions);

    (void)uart_diag_write_string(" ok=");
    (void)uart_diag_write_uint32(diag->successful_transactions);

    (void)uart_diag_write_string(" retry=");
    (void)uart_diag_write_uint32(diag->retries);

    (void)uart_diag_write_string(" recover=");
    (void)uart_diag_write_uint32(diag->recoveries);

    (void)uart_diag_write_string(" rec_fail=");
    (void)uart_diag_write_uint32(diag->recovery_failures);

    (void)uart_diag_write_string(" last_err=");
    (void)uart_diag_write_uint32(
        (uint32_t)diag->last_error_status);

    (void)uart_diag_write_string(" stage=");
    (void)uart_diag_write_uint32(
        (uint32_t)diag->last_error_stage);

    (void)uart_diag_write_string(" err_attempt=");
    (void)uart_diag_write_uint32(diag->last_error_attempt);

    (void)uart_diag_write_string(" SR1=");
    (void)uart_diag_write_hex32(diag->sr1);

    (void)uart_diag_write_string(" SR2=");
    (void)uart_diag_write_hex32(diag->sr2);

    (void)uart_diag_write_string(" GPIOB_IDR=");
    (void)uart_diag_write_hex32(diag->gpio_idr);

    (void)uart_diag_write_string("\r\n");
}

static void write_stale_debug_line(
    const imu_raw_sample_t *sample)
{
    (void)uart_diag_write_string("[RAW STALE] seq=");
    (void)uart_diag_write_uint32(sample->sequence);

    (void)uart_diag_write_string(" acq_status=");
    (void)uart_diag_write_uint32(
        (uint32_t)g_imu_acquisition_status);

    (void)uart_diag_write_string(" driver_status=");
    (void)uart_diag_write_uint32(
        (uint32_t)imu_acquisition_last_driver_status());

    (void)uart_diag_write_string("\r\n");
}

#endif

static __attribute__((noreturn))
void stop_with_message(const char *message)
{
    if (g_fc_uart_status == UART_DIAG_OK)
    {
        (void)uart_diag_write_line(message);
    }

    for (;;)
    {
        __asm volatile ("nop");
    }
}

#if ACCEL_CAPTURE_TEST

static void accel_capture_write_signed(
    int16_t value)
{
    int32_t wide;

    wide = (int32_t)value;

    if (wide < 0)
    {
        (void)uart_diag_write_char('-');
        wide = -wide;
    }

    (void)uart_diag_write_uint32(
        (uint32_t)wide);
}

#endif

#if ACCEL_CAPTURE_TEST

static void accel_capture_send_values(
    uint32_t block_number)
{
    uint32_t i;

    /*
     * Mark the beginning of one 5-second block.
     */
    (void)uart_diag_write_string(
        "# ACCEL_CAPTURE_BEGIN block=");

    (void)uart_diag_write_uint32(
        block_number);

    (void)uart_diag_write_string(
        " samples=");

    (void)uart_diag_write_uint32(
        accel_capture_count);

    (void)uart_diag_write_string(
        "\r\n");

    /*
     * CSV header.
     */
    (void)uart_diag_write_line(
        "block,index,ax_raw,ay_raw,az_raw");

    for (i = 0UL;
         i < accel_capture_count;
         i++)
    {
        /*
         * Block number.
         */
        (void)uart_diag_write_uint32(
            block_number);

        (void)uart_diag_write_char(',');

        /*
         * Sample index inside this block.
         */
        (void)uart_diag_write_uint32(i);

        (void)uart_diag_write_char(',');

        /*
         * Raw accelerometer X.
         */
        accel_capture_write_signed(
            accel_capture_buffer[i].x);

        (void)uart_diag_write_char(',');

        /*
         * Raw accelerometer Y.
         */
        accel_capture_write_signed(
            accel_capture_buffer[i].y);

        (void)uart_diag_write_char(',');

        /*
         * Raw accelerometer Z.
         */
        accel_capture_write_signed(
            accel_capture_buffer[i].z);

        (void)uart_diag_write_string(
            "\r\n");
    }

    /*
     * Mark the end of this block.
     */
    (void)uart_diag_write_string(
        "# ACCEL_CAPTURE_END block=");

    (void)uart_diag_write_uint32(
        block_number);

    (void)uart_diag_write_string(
        "\r\n");
}

#endif


#if ACCEL_CAPTURE_TEST

static __attribute__((noreturn))
void run_accel_capture_test(void)
{
    imu_raw_sample_t sample;

    uint32_t start_time_us;
    uint32_t elapsed_us;

    uint32_t block_number;

    bool started;

    /*
     * -------------------------------------------------
     * Repeat 5-second captures.
     *
     * 240 blocks × 5 seconds =
     * 1200 seconds =
     * 20 minutes of recorded accelerometer data.
     * -------------------------------------------------
     */
    for (block_number = 1UL;
         block_number <= ACCEL_CAPTURE_TOTAL_BLOCKS;
         block_number++)
    {
        /*
         * -------------------------------------------------
         * Start a NEW capture block.
         * -------------------------------------------------
         *
         * Setting count to zero logically discards the
         * previous block.
         *
         * We do NOT need to erase the RAM physically.
         * New samples simply overwrite the old samples.
         */
        accel_capture_count = 0UL;

        start_time_us = 0UL;
        elapsed_us = 0UL;
        started = false;

        /*
         * -------------------------------------------------
         * CAPTURE PHASE
         *
         * No UART transmission occurs inside this loop.
         * -------------------------------------------------
         */
        for (;;)
        {
            g_imu_acquisition_status =
                imu_acquisition_process();

            if (g_imu_acquisition_status !=
                IMU_ACQUISITION_OK)
            {
                continue;
            }

            if (!imu_acquisition_get_latest(
                    &sample))
            {
                continue;
            }

            /*
             * Only accept valid motion samples.
             */
            if ((sample.flags &
                 IMU_RAW_MOTION_VALID) == 0UL)
            {
                continue;
            }

            /*
             * The first valid sample defines the
             * beginning of this 5-second block.
             */
            if (!started)
            {
                start_time_us =
                    sample.motion_timestamp_us;

                started = true;
            }

            elapsed_us =
                (uint32_t)(
                    sample.motion_timestamp_us -
                    start_time_us);

            /*
             * Five seconds completed.
             */
            if (elapsed_us >=
                ACCEL_CAPTURE_DURATION_US)
            {
                break;
            }

            /*
             * SRAM protection.
             */
            if (accel_capture_count >=
                ACCEL_CAPTURE_MAX_SAMPLES)
            {
                break;
            }

            /*
             * Store RAW accelerometer values only.
             *
             * No calibration.
             * No scaling.
             * No filtering.
             */
            accel_capture_buffer[
                accel_capture_count].x =
                    sample.motion.accel_x;

            accel_capture_buffer[
                accel_capture_count].y =
                    sample.motion.accel_y;

            accel_capture_buffer[
                accel_capture_count].z =
                    sample.motion.accel_z;

            accel_capture_count++;
        }

        /*
         * -------------------------------------------------
         * TRANSMISSION PHASE
         *
         * Capture has finished.
         * Now send this complete block through UART.
         * -------------------------------------------------
         */
        if (g_fc_uart_status == UART_DIAG_OK)
        {
            (void)uart_diag_write_string(
                "[ACCEL CAPTURE] block=");

            (void)uart_diag_write_uint32(
                block_number);

            (void)uart_diag_write_string(
                "/");

            (void)uart_diag_write_uint32(
                ACCEL_CAPTURE_TOTAL_BLOCKS);

            (void)uart_diag_write_string(
                " samples=");

            (void)uart_diag_write_uint32(
                accel_capture_count);

            (void)uart_diag_write_string(
                "\r\n");

            /*
             * Transmit every sample from this block.
             */
            accel_capture_send_values(
                block_number);
        }

        /*
         * -------------------------------------------------
         * OLD BLOCK IS NOW FINISHED.
         * -------------------------------------------------
         *
         * There is no reason to memset()/erase the RAM.
         *
         * On the next outer-loop iteration:
         *
         *     accel_capture_count = 0
         *
         * and new samples overwrite the old buffer.
         */
    }

    /*
     * All 240 five-second capture blocks are complete.
     */
    stop_with_message(
        "[HALT] 20-minute accelerometer capture finished");
}

#endif


//test function

#if MPU_WHO_AM_I_TEST

static void mpu_who_am_i_test(uint32_t pclk1_hz)
{
    uint8_t id_68 = 0U;
    uint8_t id_69 = 0U;

    i2c1_status_t init_status;
    i2c1_status_t status_68;
    i2c1_status_t status_69;

    /*
     * Initialize I2C1:
     * PB6 = SCL
     * PB7 = SDA
     * Bus speed = I2C_BUS_HZ build setting
     */
    init_status = i2c1_init(pclk1_hz, I2C_BUS_HZ);

    (void)uart_diag_write_string(
        "[WHO_AM_I TEST] I2C init status=");

    (void)uart_diag_write_uint32(
        (uint32_t)init_status);

    (void)uart_diag_write_string("\r\n");

    if (init_status != I2C1_OK)
    {
        (void)uart_diag_write_line(
            "[WHO_AM_I TEST] I2C initialization failed");

        return;
    }

    /*
     * Read WHO_AM_I register 0x75 at address 0x68.
     */
    status_68 = i2c1_read_registers(
        0x68U,
        0x75U,
        &id_68,
        1U
    );

    /*
     * Read WHO_AM_I register 0x75 at address 0x69.
     */
    status_69 = i2c1_read_registers(
        0x69U,
        0x75U,
        &id_69,
        1U
    );

    /*
     * Print decimal values because uart_diag_write_uint32()
     * already exists in the current project.
     */
    (void)uart_diag_write_string(
        "[WHO_AM_I TEST] address=0x68 status=");

    (void)uart_diag_write_uint32(
        (uint32_t)status_68);

    (void)uart_diag_write_string(
        " id_decimal=");

    (void)uart_diag_write_uint32(
        (uint32_t)id_68);

    (void)uart_diag_write_string("\r\n");

    (void)uart_diag_write_string(
        "[WHO_AM_I TEST] address=0x69 status=");

    (void)uart_diag_write_uint32(
        (uint32_t)status_69);

    (void)uart_diag_write_string(
        " id_decimal=");

    (void)uart_diag_write_uint32(
        (uint32_t)id_69);

    (void)uart_diag_write_string("\r\n");

    /*
     * Give a simple interpretation.
     */
    if ((status_68 == I2C1_OK) ||
        (status_69 == I2C1_OK))
    {
        uint8_t detected_id;

        detected_id =
            (status_68 == I2C1_OK) ? id_68 : id_69;

        switch (detected_id)
        {
            case 0x68U:
                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Possible MPU6050");
                break;

            case 0x70U:
                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Possible MPU6500");
                break;

            case 0x71U:
                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] MPU9250 identity detected");
                break;

            case 0x73U:
                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Possible MPU9255");
                break;

            default:
                (void)uart_diag_write_line(
                    "[WHO_AM_I TEST] Unknown identity value");
                break;
        }
    }
    else
    {
        (void)uart_diag_write_line(
            "[WHO_AM_I TEST] No device responded at 0x68 or 0x69");
    }
}

#endif


int main(void)
{
    uint32_t pclk1_hz;
    uint32_t tim2_hz;

#if ACQ_UART_RAW_DEBUG
    uint32_t previous_debug_ms;
    uint32_t previous_debug_sequence;
    uint32_t previous_i2c_error_count;
    uint32_t previous_i2c_recoveries;
    i2c1_diag_t runtime_i2c_diag;
#endif

    imu_raw_sample_t sample;

    fault_record_clear();

    g_fc_clock_status = system_clock_init();

    if (g_fc_clock_status != SYSTEM_CLOCK_OK)
    {
        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    g_fc_time_status =
        system_time_init(system_clock_get_hclk_hz());

    if (g_fc_time_status != SYSTEM_TIME_OK)
    {
        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    /*
     * When the APB1 prescaler is not 1,
     * STM32F103 timer clock = 2 × PCLK1.
     */
    pclk1_hz = system_clock_get_pclk1_hz();

    tim2_hz =
        (pclk1_hz == system_clock_get_hclk_hz()) ?
        pclk1_hz :
        (pclk1_hz * 2UL);

    g_fc_micros_status = micros_init(tim2_hz);

    if (g_fc_micros_status != MICROS_OK)
    {
        for (;;)
        {
            __asm volatile ("nop");
        }
    }

    g_fc_uart_status =
        uart_diag_init(system_clock_get_pclk2_hz(),
                       UART_BAUD_RATE);

    if (g_fc_uart_status == UART_DIAG_OK)
    {
        (void)uart_diag_write_line(
            "[BOOT] Flight controller starting");

        #if IMU_MODEL == IMU_MODEL_MPU6500

            (void)uart_diag_write_line(
                "[ACQ] MPU6500 accel/gyro acquisition");

        #else

            (void)uart_diag_write_line(
                "[ACQ] MPU9250 accel/gyro/mag acquisition");

        #endif
    }
    #if MPU_WHO_AM_I_TEST

        (void)uart_diag_write_line(
            "[TEST MODE] Direct MPU WHO_AM_I test enabled");

        mpu_who_am_i_test(pclk1_hz);

        stop_with_message(
            "[HALT] WHO_AM_I test completed");

    #endif
    g_imu_acquisition_status =
        imu_acquisition_init(pclk1_hz);

    if (g_imu_acquisition_status != IMU_ACQUISITION_OK)
    {
        if (g_fc_uart_status == UART_DIAG_OK)
        {
            (void)uart_diag_write_string(
                "[ERROR] Acquisition init status=");

            (void)uart_diag_write_uint32(
                (uint32_t)g_imu_acquisition_status);

            (void)uart_diag_write_string(
                " driver_status=");

            (void)uart_diag_write_uint32(
                (uint32_t)
                imu_acquisition_last_driver_status());
            #if IMU_INIT_DEBUG

    init_diag =
        mpu9250_get_init_diag();

    (void)uart_diag_write_string(
        "\r\n[IMU DEBUG] stage=");

    (void)uart_diag_write_uint32(
        (uint32_t)init_diag.stage);

    (void)uart_diag_write_string(
        " phase=");

    (void)uart_diag_write_uint32(
        (uint32_t)init_diag.phase);

    (void)uart_diag_write_string(
        " i2c_status=");

    (void)uart_diag_write_uint32(
        init_diag.i2c_status);

    (void)uart_diag_write_string(
        " attempt=");

    (void)uart_diag_write_uint32(
        init_diag.attempt);

    (void)uart_diag_write_string(
        "\r\n[IMU DEBUG] device=");

    (void)uart_diag_write_hex32(
        (uint32_t)
        init_diag.device_address);

    (void)uart_diag_write_string(
        " reg=");

    (void)uart_diag_write_hex32(
        (uint32_t)
        init_diag.register_address);

    (void)uart_diag_write_string(
        "\r\n[IMU DEBUG] written=");

    (void)uart_diag_write_hex32(
        (uint32_t)
        init_diag.written_value);

    (void)uart_diag_write_string(
        " readback=");

    (void)uart_diag_write_hex32(
        (uint32_t)
        init_diag.readback_value);

    (void)uart_diag_write_string(
        " mask=");

    (void)uart_diag_write_hex32(
        (uint32_t)
        init_diag.mask);

    i2c1_get_diag(&i2c_diag);

    (void)uart_diag_write_string(
        "\r\n[I2C DEBUG] stage=");
    (void)uart_diag_write_uint32(
        (uint32_t)i2c_diag.stage);

    (void)uart_diag_write_string(
        " status=");
    (void)uart_diag_write_uint32(
        (uint32_t)i2c_diag.last_status);

    (void)uart_diag_write_string(
        " attempt=");
    (void)uart_diag_write_uint32(
        i2c_diag.attempt);

    (void)uart_diag_write_string(
        "\r\n[I2C DEBUG] CR1=");
    (void)uart_diag_write_hex32(i2c_diag.cr1);

    (void)uart_diag_write_string(" SR1=");
    (void)uart_diag_write_hex32(i2c_diag.sr1);

    (void)uart_diag_write_string(" SR2=");
    (void)uart_diag_write_hex32(i2c_diag.sr2);

    (void)uart_diag_write_string(" DR=");
    (void)uart_diag_write_hex32(i2c_diag.dr);

    (void)uart_diag_write_string(
        "\r\n[I2C DEBUG] last_error_status=");
    (void)uart_diag_write_uint32(
        (uint32_t)i2c_diag.last_error_status);

    (void)uart_diag_write_string(" last_error_stage=");
    (void)uart_diag_write_uint32(
        (uint32_t)i2c_diag.last_error_stage);

    (void)uart_diag_write_string(" err_attempt=");
    (void)uart_diag_write_uint32(
        i2c_diag.last_error_attempt);

    (void)uart_diag_write_string(" last_recovery=");
    (void)uart_diag_write_uint32(
        (uint32_t)i2c_diag.last_recovery_status);

    (void)uart_diag_write_string(
        "\r\n[I2C DEBUG] GPIOB_IDR=");
    (void)uart_diag_write_hex32(i2c_diag.gpio_idr);

    (void)uart_diag_write_string(" bus_hz=");
    (void)uart_diag_write_uint32(i2c_diag.bus_hz);

    (void)uart_diag_write_string(" dev=");
    (void)uart_diag_write_hex32(
        (uint32_t)i2c_diag.device_address);

    (void)uart_diag_write_string(" reg=");
    (void)uart_diag_write_hex32(
        (uint32_t)i2c_diag.register_address);

    (void)uart_diag_write_string(" len=");
    (void)uart_diag_write_uint32(
        (uint32_t)i2c_diag.length);

    (void)uart_diag_write_string(
        "\r\n[I2C DEBUG] retries=");
    (void)uart_diag_write_uint32(i2c_diag.retries);

    (void)uart_diag_write_string(" recoveries=");
    (void)uart_diag_write_uint32(i2c_diag.recoveries);

    (void)uart_diag_write_string(" rec_fail=");
    (void)uart_diag_write_uint32(i2c_diag.recovery_failures);

    (void)uart_diag_write_string(" timeouts=");
    (void)uart_diag_write_uint32(i2c_diag.timeouts);

    (void)uart_diag_write_string(" nacks=");
    (void)uart_diag_write_uint32(i2c_diag.nacks);

    (void)uart_diag_write_string(" busy=");
    (void)uart_diag_write_uint32(i2c_diag.bus_busy_errors);

    (void)uart_diag_write_string(" berr=");
    (void)uart_diag_write_uint32(i2c_diag.bus_errors);

    (void)uart_diag_write_string(" arlo=");
    (void)uart_diag_write_uint32(i2c_diag.arbitration_losses);

    (void)uart_diag_write_string(" ovr=");
    (void)uart_diag_write_uint32(i2c_diag.overruns);

    (void)uart_diag_write_string(" line_low=");
    (void)uart_diag_write_uint32(i2c_diag.line_stuck_errors);

#endif
            (void)uart_diag_write_string("\r\n");
        }

        stop_with_message(
            "[HALT] Check power, wiring, "
            "address and pull-ups");
    }

   if (g_fc_uart_status == UART_DIAG_OK)
    {
    #if IMU_MODEL == IMU_MODEL_MPU6500

        (void)uart_diag_write_string(
            "[ID] MPU6500=");

    #else

        (void)uart_diag_write_string(
            "[ID] MPU9250=");

    #endif

        (void)uart_diag_write_hex32(
            (uint32_t)mpu9250_who_am_i());

        if (mpu9250_has_magnetometer())
        {
            (void)uart_diag_write_string(
                " AK8963=");

            (void)uart_diag_write_hex32(
                (uint32_t)ak8963_who_am_i());
        }

        (void)uart_diag_write_string("\r\n");
    }

#if ACQ_UART_RAW_DEBUG
    previous_debug_ms = millis();
    previous_debug_sequence = 0UL;
    previous_i2c_error_count = 0UL;
    previous_i2c_recoveries = 0UL;
#endif

if (g_fc_uart_status == UART_DIAG_OK)
{
#if ACCEL_CAPTURE_TEST

    (void)uart_diag_write_line(
        "[TEST] Repeated 5-second accelerometer capture");

    (void)uart_diag_write_line(
        "[TEST] Target = 240 blocks = 20 minutes captured data");

#endif

    (void)uart_diag_write_line(
        "[READY] Starting PA0 DATA_RDY");
}

g_imu_acquisition_status = imu_acquisition_start();

if (g_imu_acquisition_status != IMU_ACQUISITION_OK)
{
    stop_with_message(
        "[HALT] Could not start IMU DATA_RDY");
}

#if ACCEL_CAPTURE_TEST

/*
 * Repeatedly:
 *
 * 1. Capture 5 seconds to RAM.
 * 2. Send that block through UART.
 * 3. Reuse the same RAM buffer.
 *
 * 240 capture blocks = 20 minutes
 * of actual accelerometer measurements.
 *
 * Function halts after all blocks complete.
 */
run_accel_capture_test();

#endif

for (;;)
    {
        g_imu_acquisition_status =
            imu_acquisition_process();

        if ((g_imu_acquisition_status ==
             IMU_ACQUISITION_OK) &&
            imu_acquisition_get_latest(&sample))
        {
            g_latest_raw_imu_sample = sample;
        }

        g_imu_acquisition_stats =
            imu_acquisition_get_stats();

#if ACQ_UART_RAW_DEBUG

        if ((uint32_t)(millis() - previous_debug_ms) >=
            DEBUG_PERIOD_MS)
        {
            previous_debug_ms = millis();

            if (g_fc_uart_status == UART_DIAG_OK)
            {
                uint32_t i2c_error_count;

                i2c1_get_diag(&runtime_i2c_diag);

                i2c_error_count =
                    runtime_i2c_diag.timeouts +
                    runtime_i2c_diag.nacks +
                    runtime_i2c_diag.bus_busy_errors +
                    runtime_i2c_diag.bus_errors +
                    runtime_i2c_diag.arbitration_losses +
                    runtime_i2c_diag.overruns +
                    runtime_i2c_diag.line_stuck_errors;

                /* Print I2C health only when something changed. */
                if ((i2c_error_count != previous_i2c_error_count) ||
                    (runtime_i2c_diag.recoveries !=
                     previous_i2c_recoveries))
                {
                    write_i2c_health_debug_line(
                        &runtime_i2c_diag);

                    previous_i2c_error_count =
                        i2c_error_count;
                    previous_i2c_recoveries =
                        runtime_i2c_diag.recoveries;
                }

                if (imu_acquisition_get_latest(&sample))
                {
                    if (sample.sequence !=
                        previous_debug_sequence)
                    {
                        write_raw_debug_line(&sample);
                        previous_debug_sequence =
                            sample.sequence;
                    }
                    else
                    {
                        /* Explicitly expose a frozen/stale latest sample. */
                        write_stale_debug_line(&sample);
                    }
                }
            }
        }

#endif

        __asm volatile ("nop");
    }
}