# Stationary raw gyroscope capture

This mode records raw MPU6500 gyroscope X, Y and Z values before a gyro
calibration or filtering pipeline is selected. It is compile-time isolated from
normal flight-controller operation.

## Capture design

- Motion data rate: 500 Hz.
- One RAM block: five seconds, approximately 2500 samples.
- Buffer allowance: 2600 samples x 6 bytes = 15,600 bytes.
- Total: 240 blocks x five seconds = 20 minutes of captured gyro data.
- After each block, the MCU sends it over UART and reuses the same buffer.
- No UART transmission occurs while a five-second block is being sampled.
- Only raw `gyro_x`, `gyro_y` and `gyro_z` are stored. No calibration, scaling,
  filtering, or accelerometer pipeline operation changes the samples.

The UART transfer gaps mean the whole experiment takes longer than 20 minutes
of wall-clock time, normally about 40 to 45 minutes at 115200 baud. The CSV
contains both a continuous `captured_time_s` axis and `sensor_elapsed_s`, which
includes the between-block gaps using the MCU's TIM2 microsecond clock.

## Build and flash

From `firmware/flight-controller`:

```bash
make clean

make \
    IMU_MODEL=6500 \
    GYRO_CAPTURE_TEST=1 \
    ACCEL_CAPTURE_TEST=0 \
    ACCEL_PIPELINE_ENABLE=0 \
    ACQ_UART_RAW_DEBUG=0 \
    IMU_INIT_DEBUG=0
```

Flash the generated ELF using the normal project method. For example:

```bash
openocd \
    -f interface/stlink.cfg \
    -f target/stm32f1x.cfg \
    -c "program build/flight-controller.elf verify reset exit"
```

`ACCEL_CAPTURE_TEST=1` and `GYRO_CAPTURE_TEST=1` cannot be enabled together.
With `GYRO_CAPTURE_TEST=0`, the gyro buffer, capture functions and UART rows are
not compiled into the firmware.

## Prepare the stationary test

1. Fix the drone frame or sensor board on a firm, vibration-free surface.
2. Keep motors and other vibration sources off.
3. Do not touch or move the frame during the complete test.
4. If warm-up drift is important, start the PC logger first and then power or
   reset the MCU while the sensor is still cold.

## Save UART data

Install the PC packages:

```bash
python3 -m pip install pyserial pandas matplotlib
```

From `py_scrypts`, start the logger before resetting the STM32:

```bash
python3 gyro_uart_capture.py \
    --port /dev/ttyUSB0 \
    --baud 115200 \
    --output gyro_raw_20min.csv
```

The logger writes and flushes accepted rows continuously. `Ctrl+C` closes the
CSV safely if the test must be stopped early.

## Plot and summarize

```bash
python3 plot_gyro.py gyro_raw_20min.csv
```

This prints each axis mean, standard deviation, range and nominal deg/s values.
The configured +/-500 deg/s range uses 65.5 raw counts per deg/s. It creates:

- `gyro_x_raw.png`
- `gyro_y_raw.png`
- `gyro_z_raw.png`
- `gyro_xyz_raw_dps.png`
- `gyro_bias_drift_dps.png`

Use the raw plots to inspect noise, spikes and vibration. Use the five-second
mean bias-drift plot to see warm-up or slow offset changes before selecting the
calibration and filter stages.
