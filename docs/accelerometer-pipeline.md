# Accelerometer processing pipeline

The flight-controller pipeline is compile-time isolated from the existing raw
capture mode.

## Runtime order

For each newly published motion sample:

1. Require a successful acquisition and `IMU_RAW_MOTION_VALID`.
2. Reject a duplicate sequence or a raw value near the signed 16-bit limit.
3. Run an independent causal median-of-3 on X, Y and Z.
4. Apply the measured six-face bias and counts-per-g constants.
5. Run an independent 20 Hz second-order Butterworth filter on each axis.
6. Convert the filtered result from g to m/s2 using 9.80665 m/s2 per g.
7. Publish the completed structure in the main loop. No processing or UART
   transmission occurs in `EXTI0_IRQHandler`.

Rejected inputs do not update median or Butterworth history.

## Build flags

Main feature:

```text
ACCEL_PIPELINE_ENABLE=1
```

The calibration constants in this revision belong to the measured MPU6500, so
the Makefile requires `IMU_MODEL=6500` when the pipeline is enabled.

Optional UART columns:

```text
ACCEL_PIPELINE_DEBUG_RAW=1
ACCEL_PIPELINE_DEBUG_MEDIAN=1
ACCEL_PIPELINE_DEBUG_CALIBRATED_G=1
ACCEL_PIPELINE_DEBUG_FILTERED_G=1
ACCEL_PIPELINE_DEBUG_MS2=1
```

`ACCEL_PIPELINE_DEBUG_DECIMATION=50` prints one selected debug line for every
50 processed samples. At 500 Hz this is approximately 10 lines/s. Do not print
every multi-stage sample at 115200 baud; the polling UART would block the main
loop long enough to lose DATA_RDY events.

Recommended complete test build:

```bash
make \
    IMU_MODEL=6500 \
    ACCEL_CAPTURE_TEST=0 \
    ACQ_UART_RAW_DEBUG=0 \
    ACCEL_PIPELINE_ENABLE=1 \
    ACCEL_PIPELINE_DEBUG_RAW=1 \
    ACCEL_PIPELINE_DEBUG_MEDIAN=1 \
    ACCEL_PIPELINE_DEBUG_CALIBRATED_G=1 \
    ACCEL_PIPELINE_DEBUG_FILTERED_G=1 \
    ACCEL_PIPELINE_DEBUG_MS2=1 \
    ACCEL_PIPELINE_DEBUG_DECIMATION=50
```

The Makefile tracks these flag values in `build/.build-config`, so changing a
flag causes the affected firmware objects to rebuild. A manual `make clean` is
still safe but is no longer required just because a build flag changed.

### Measure pipeline execution time

Enable GDB-visible timing statistics without adding UART output:

```bash
make \
    IMU_MODEL=6500 \
    ACCEL_PIPELINE_ENABLE=1 \
    MEASURE_PIPELINE_TIMES=1
```

The firmware updates these file-scope variables after each successful pipeline
output:

```text
g_pipeline_time_last_us
g_pipeline_time_min_us
g_pipeline_time_max_us
g_pipeline_time_total_us
g_pipeline_time_sample_count
```

Calculate the average while the MCU is halted in GDB:

```gdb
p (double)g_pipeline_time_total_us / g_pipeline_time_sample_count
```

The measurement uses TIM2 through `micros()` and includes the two timestamp
reads. It excludes I2C acquisition and UART output. With
`MEASURE_PIPELINE_TIMES=0`, all timing code and storage are removed during
compilation.

## RAM handling

The pipeline never stores a long time series. It retains only:

- three raw values per axis for median-of-3;
- four float histories per axis for the three Butterworth filters;
- the previous sequence number;
- one latest processed output structure.

The private pipeline state is 88 bytes and the published output is 60 bytes
with the current compiler data layout: 148 bytes of persistent SRAM in total.
The input and working output in `main()` use approximately 76 bytes of stack;
exact stack allocation depends on compiler optimization.

The old five-second raw-capture buffer is 2600 x 6 = 15,600 bytes, but it only
exists when `ACCEL_CAPTURE_TEST=1`. Use `ACCEL_CAPTURE_TEST=0` for normal
pipeline operation, so this large buffer is removed at compile time.

With `ACCEL_PIPELINE_ENABLE=0`, the source file is omitted and all pipeline
state/output code is removed at compile time.

`MEASURE_PIPELINE_TIMES=1` adds 24 bytes of file-scope timing statistics and
approximately 8 bytes of temporary stack storage. With the flag set to `0`,
these variables and the two `micros()` calls are not compiled.

## Host verification

```bash
make test-accel-host
```

This checks the measured calibration centres, median spike rejection, g to
m/s2 conversion, duplicate rejection, saturation rejection, and preservation
of filter history after rejected inputs.
