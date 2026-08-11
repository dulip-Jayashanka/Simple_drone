# GY-87 Sensor Calibration TODO and Practical Workflow

## 1. Purpose

This document defines the order in which the GY-87 sensors must be calibrated, how each calibration is performed practically, what data must be calculated, and when each calibration must be repeated.

The GY-87 normally contains:

- MPU6050 three-axis accelerometer
- MPU6050 three-axis gyroscope
- HMC5883L-compatible three-axis magnetometer
- BMP180 pressure and temperature sensor

> Calibration removes repeatable measurement errors. Filtering reduces random noise. Sensor fusion combines different sensor measurements. Complete calibration before PID tuning and before judging filter performance.

## 2. Calibration order at a glance

Perform the development calibrations in this order:

1. Finalize the mechanical and electrical installation.
2. Verify sensor identities, communication, axes, and signs.
3. Select the sensor ranges, sample rates, and digital filtering configuration.
4. Calibrate the MPU6050 accelerometer using six positions.
5. Confirm the sensor-to-drone frame alignment.
6. Calibrate the magnetometer on the completely assembled drone.
7. Validate the accelerometer and magnetometer with new data.
8. Characterize gyro bias versus temperature, if required.
9. Test vibration and motor magnetic interference.
10. Implement and validate the automatic pre-flight calibration sequence.
11. Tune attitude filters and PID controllers only after the previous tasks pass.

Before every flight, perform only the short runtime sequence:

1. Sensor communication and stored-calibration checks
2. Sensor settling period
3. Stationary gyro-bias calibration
4. Accelerometer sanity and stationary checks
5. BMP180 ground-pressure zero
6. Magnetometer field sanity check
7. Attitude-estimate and sensor-health checks
8. Arm only if every check passes

## 3. Calibration categories

| Category | Examples | Frequency |
|---|---|---|
| Permanent or one-time setup | Axis mapping, sensor-to-frame orientation | Once; repeat if the board is moved |
| Stored calibration | Accelerometer offset/scale, compass hard/soft-iron correction | Occasionally; repeat after relevant hardware changes |
| Development characterization | Temperature drift, vibration spectrum, motor magnetic interference | During development or troubleshooting |
| Pre-flight runtime calibration | Gyro zero-rate bias, BMP180 ground reference | Before every flight |
| Pre-flight validation | Accelerometer magnitude, compass field, communications, calibration CRC | Before every flight |

## 4. Required equipment and safety

### TODO 4.1 — Prepare the equipment

- [ ] A stable, level work surface
- [ ] A non-magnetic support for placing the drone in six orientations
- [ ] A USB/UART connection for recording sensor data
- [ ] Firmware capable of outputting timestamped raw and calibrated readings
- [ ] A computer script or spreadsheet for calculating statistics
- [ ] A safe restraint rig for later motor tests
- [ ] Open-cell breathable foam for the BMP180 pressure port
- [ ] A method of measuring battery voltage during motor tests

### TODO 4.2 — Apply safety rules

- [ ] Remove the propellers during initial sensor, axis, and motor-interference tests.
- [ ] Keep the motors disarmed during all static sensor calibrations.
- [ ] Perform propeller-on tests only in a proper restraint rig and test area.
- [ ] Do not calibrate the compass on a steel table or near tools, vehicles, loudspeakers, reinforced concrete, or large current-carrying equipment.
- [ ] Do not seal the BMP180 pressure opening; use only breathable foam.

## 5. Stage 1 — Final installation before calibration

Calibration parameters depend on the final physical installation. Do not perform the final compass calibration on a loose GY-87 module and then move it onto the drone.

### TODO 5.1 — Install the GY-87

- [ ] Mount the module firmly.
- [ ] Record which direction the module's `+X`, `+Y`, and `+Z` axes point relative to the drone.
- [ ] Keep the module away from motors, ESCs, high-current battery cables, magnets, and magnetic screws.
- [ ] Route high-current supply and return wires together; twist the pair where practical.
- [ ] Fix the battery, power-distribution wiring, radio, and payload in their intended positions.
- [ ] Place breathable foam over the BMP180 pressure opening without blocking it.
- [ ] Keep the pressure sensor away from direct propeller airflow and major heat sources.

### Completion condition

The sensor position, orientation, power wiring, battery location, motors, ESCs, fasteners, and major payloads match the intended flight configuration.

## 6. Stage 2 — Identify sensors and verify communication

Some inexpensive GY-87 boards may contain a magnetometer that is not fully compatible with the expected HMC5883L driver. Confirm the actual device before calibration.

### TODO 6.1 — Scan and identify the I2C devices

- [ ] Detect the MPU6050 address and read its identity register.
- [ ] Detect the BMP180 and read its chip identity.
- [ ] Detect the magnetometer and confirm which driver produces valid data.
- [ ] Verify that each sensor responds repeatedly without I2C errors.
- [ ] Confirm that readings change logically when the board is moved.

### TODO 6.2 — Read and validate BMP180 factory coefficients

- [ ] Read the complete factory coefficient table from BMP180 EEPROM.
- [ ] Reject obviously invalid sets such as all `0x0000` or all `0xFFFF`.
- [ ] Use the coefficients in the compensated temperature and pressure algorithms.
- [ ] Never copy coefficients from another BMP180.
- [ ] Never overwrite or manually "calibrate" these factory coefficients.

### Completion condition

All sensors are identified, the correct drivers are selected, no persistent I2C errors occur, and raw values are physically reasonable.

## 7. Stage 3 — Verify axes and signs

Perform axis mapping before numerical calibration. Otherwise, correct sensor values may be applied to the wrong drone axes.

Use one body-axis convention consistently. A common choice is:

- Body `+X`: forward
- Body `+Y`: right
- Body `+Z`: downward
- Positive roll: right wing moves downward
- Positive pitch: nose moves upward or downward according to the chosen right-hand convention; document it explicitly
- Positive yaw: right-hand rotation around body `+Z`

### TODO 7.1 — Accelerometer axis test

- [ ] Point each sensor axis upward and downward.
- [ ] Confirm that the expected axis becomes approximately `+1 g` or `-1 g`.
- [ ] Confirm that the other two axes remain approximately near zero.

### TODO 7.2 — Gyroscope sign test

- [ ] Roll the drone slowly and confirm the expected roll-rate axis and sign.
- [ ] Pitch the drone slowly and confirm the expected pitch-rate axis and sign.
- [ ] Rotate the drone in yaw and confirm the expected yaw-rate axis and sign.

### TODO 7.3 — Magnetometer axis test

- [ ] Rotate the drone horizontally and verify that the two horizontal magnetic components change smoothly.
- [ ] Tilt the drone and confirm that the vertical magnetic component changes logically.
- [ ] Define and implement the magnetometer-to-body axis mapping.

### TODO 7.4 — Implement frame transformation

- [ ] Implement an axis permutation/sign mapping for 90-degree mounting rotations, or a rotation matrix for arbitrary mounting angles.
- [ ] Apply the transformation consistently before sensor fusion.
- [ ] Record the mapping in the source code and project documentation.

### Completion condition

Every physical movement changes the correct body-axis variable with the documented sign.

## 8. Stage 4 — Select the measurement configuration

Choose and freeze the configuration before collecting calibration data. A sensitivity-range change changes the raw-to-physical-unit conversion.

### TODO 8.1 — Choose initial ranges

Reasonable starting points for a small quadcopter are:

- [ ] Accelerometer: `±4 g` or `±8 g`
- [ ] Gyroscope: `±500 °/s` or `±1000 °/s`
- [ ] Sample rate compatible with the planned control loop
- [ ] MPU6050 digital low-pass setting appropriate for the chosen sample rate

These are starting values, not universal final values. Use the smallest range that does not clip during real operation.

### TODO 8.2 — Verify conversion and saturation flags

- [ ] Convert raw counts using the sensitivity corresponding to the selected range.
- [ ] Add checks for values close to the signed 16-bit limits.
- [ ] Log saturation events.
- [ ] Prevent arming if a sensor is stuck or clearly invalid.

### Completion condition

The ranges and sample rate are documented, unit conversions are verified, and normal movements do not clip.

## 9. Stage 5 — Accelerometer six-position calibration

### Purpose

Correct the main accelerometer zero-offset and per-axis scale-factor errors.

For one axis, let the mean measurements with that axis upward and downward be (r_+) and (r_-), expressed in `g`.

Offset:

\[
b=\frac{r_+ + r_-}{2}
\]

Scale correction:

\[
s=\frac{2}{r_+-r_-}
\]

Corrected acceleration:

\[
a_{cal}=s(a_{raw}-b)
\]

### TODO 9.1 — Prepare collection firmware

- [ ] Output timestamp, raw acceleration X/Y/Z, converted acceleration X/Y/Z, and MPU6050 temperature.
- [ ] Add a command to start each orientation capture.
- [ ] Discard an initial settling interval after the drone is placed.
- [ ] Calculate or log enough samples to calculate mean and standard deviation.

### TODO 9.2 — Collect six stable datasets

For each position, keep the drone completely still and collect approximately 1,000–3,000 samples.

- [ ] `+X` upward: expected `[+1, 0, 0] g`
- [ ] `-X` upward: expected `[-1, 0, 0] g`
- [ ] `+Y` upward: expected `[0, +1, 0] g`
- [ ] `-Y` upward: expected `[0, -1, 0] g`
- [ ] `+Z` upward: expected `[0, 0, +1] g`
- [ ] `-Z` upward: expected `[0, 0, -1] g`

For each dataset:

- [ ] Calculate the mean of X, Y, and Z.
- [ ] Calculate standard deviation for each axis.
- [ ] Calculate acceleration magnitude.
- [ ] Reject and repeat the position if the sensor moved or a large outlier occurred.

### TODO 9.3 — Calculate parameters

- [ ] Calculate `bias_x`, `bias_y`, and `bias_z`.
- [ ] Calculate `scale_x`, `scale_y`, and `scale_z`.
- [ ] Store the parameters in physical units or document the raw-count convention clearly.
- [ ] Apply offset removal before scale correction.

### TODO 9.4 — Save safely

- [ ] Store the parameters in non-volatile memory.
- [ ] Add a calibration format version.
- [ ] Add a CRC/checksum.
- [ ] Keep safe default behaviour when stored data is absent or corrupt.
- [ ] Prevent arming if required calibration is invalid.

### TODO 9.5 — Validate with independent data

Do not validate using the same samples used to calculate the parameters.

- [ ] Place the drone in all six positions again.
- [ ] Collect new datasets.
- [ ] Confirm the gravity-axis result is close to `±1 g`.
- [ ] Confirm the other axes are close to `0 g`.
- [ ] Confirm acceleration magnitude is close to `1 g`.
- [ ] Calculate six-position RMSE.
- [ ] Save the results in the calibration report.

Validation metric:

\[
RMSE=\sqrt{\frac{1}{N}\sum_{i=1}^{N}(a_{cal,i}-a_{expected,i})^2}
\]

### Completion condition

Independent six-position results have low bias, reasonable noise, magnitude close to `1 g`, and no incorrect axis mapping.

## 10. Stage 6 — Sensor-to-frame level reference

Accelerometer sensor calibration does not automatically remove an installation tilt between the GY-87 board and the drone frame.

### TODO 10.1 — Establish the level reference

- [ ] Place the completed drone frame on a verified level fixture.
- [ ] Collect stationary calibrated acceleration samples.
- [ ] Calculate the mean roll and pitch produced by the accelerometer.
- [ ] If a small repeatable mounting angle remains, store a frame-alignment correction.
- [ ] Do not use a large software correction to hide a badly mounted or loose sensor.

### Completion condition

When the frame is physically level, the stationary roll and pitch estimates are close to the defined zero angles.

## 11. Stage 7 — Magnetometer hard-iron and soft-iron calibration

Perform this only after the drone is fully assembled.

The correction model is:

\[
\mathbf{m}_{cal}=\mathbf{S}(\mathbf{m}_{raw}-\mathbf{b}_{hard})
\]

where `b_hard` removes constant magnetic offset and matrix `S` corrects soft-iron scaling and axis distortion.

### TODO 11.1 — Prepare the test location

- [ ] Move outdoors or to a magnetically clean open area.
- [ ] Stay away from steel structures, vehicles, tools, speakers, high-current cables, and reinforced concrete where possible.
- [ ] Install the battery and all flight hardware in their normal positions.
- [ ] Keep the motors disarmed.

### TODO 11.2 — Collect full three-dimensional data

- [ ] Log timestamp and raw magnetometer X/Y/Z.
- [ ] Slowly rotate the entire drone through roll, pitch, and yaw.
- [ ] Cover many orientations rather than drawing only a flat horizontal circle.
- [ ] Collect hundreds or thousands of samples distributed over the full orientation range.
- [ ] Repeat missing orientations if the point cloud has large empty regions.

### TODO 11.3 — Calculate correction

Preferred method:

- [ ] Fit an ellipsoid to the three-dimensional sample cloud.
- [ ] Calculate the hard-iron offset vector.
- [ ] Calculate the soft-iron correction matrix.

Initial simplified method, if ellipsoid fitting is not yet implemented:

\[
b_x=\frac{m_{x,max}+m_{x,min}}{2}
\]

\[
r_x=\frac{m_{x,max}-m_{x,min}}{2}
\]

Repeat for Y and Z, then calculate:

\[
r_{avg}=\frac{r_x+r_y+r_z}{3}, \qquad s_x=\frac{r_{avg}}{r_x}
\]

- [ ] Clearly mark a min/max solution as the basic version.
- [ ] Upgrade to ellipsoid fitting before relying heavily on compass heading.

### TODO 11.4 — Validate using new movements

- [ ] Collect a new independent 3D dataset.
- [ ] Apply the stored correction.
- [ ] Calculate corrected field magnitude:

\[
B_i=\sqrt{m_{x,i}^2+m_{y,i}^2+m_{z,i}^2}
\]

- [ ] Calculate mean, standard deviation, minimum, maximum, and range of `B`.
- [ ] Check that magnitude remains reasonably constant as orientation changes.
- [ ] Return to several known headings and check heading repeatability.
- [ ] Check for sudden jumps or axes with very small variation.

### Completion condition

The corrected 3D cloud is approximately centred and spherical, field magnitude is reasonably consistent, and repeated headings give similar results.

## 12. Stage 8 — Motor magnetic-interference test

Static compass calibration cannot remove magnetic fields that change with motor current.

### TODO 12.1 — Test without propellers first

- [ ] Secure the drone and remove the propellers.
- [ ] Record the calibrated magnetic vector with motors off.
- [ ] Repeat at low, medium, and higher motor commands.
- [ ] Record battery current or at least battery voltage and motor command.
- [ ] Calculate the change in each magnetic axis and field magnitude from the motors-off baseline.

### TODO 12.2 — Correct physical causes

If interference is significant:

- [ ] Increase the distance between the compass and power system.
- [ ] Reroute and twist high-current supply/return pairs.
- [ ] Replace magnetic screws or brackets near the sensor.
- [ ] Keep the battery and wiring in a repeatable position.
- [ ] Repeat the full compass calibration after changing the installation.
- [ ] Add an in-flight magnetic-consistency check and reject disturbed compass data.

### Completion condition

Motor operation does not cause an unacceptable heading or field-magnitude change, or the estimator can safely detect and reject disturbed measurements.

## 13. Stage 9 — Optional gyro temperature characterization

This is a development calibration, not a pre-flight task. The relevant temperature is the MPU6050 internal chip temperature, not the BMP180 air-temperature estimate.

For each gyro axis, a simple model is:

\[
b_g(T)=b_0+k(T-T_0)
\]

### TODO 13.1 — Collect temperature data

- [ ] Start with the drone stationary from a cold power-up.
- [ ] Keep it stationary while the MPU6050 warms naturally.
- [ ] Divide data into temperature intervals.
- [ ] Calculate the mean stationary gyro bias for each axis in each interval.
- [ ] Repeat the experiment on multiple days.

### TODO 13.2 — Fit and evaluate the model

- [ ] Fit a line or lookup table for each axis.
- [ ] Calculate residual error and coefficient of determination `R²`.
- [ ] Validate the model using a different warm-up dataset.
- [ ] Use the model only if it improves independent-data error consistently.

### Completion condition

Temperature compensation reduces independent validation bias without increasing noise or causing discontinuities. Even after implementation, keep the short pre-flight gyro-bias routine.

## 14. Stage 10 — Vibration and noise characterization

This is not ordinary calibration. It identifies mechanical and frequency-dependent disturbances before filter and PID tuning.

### TODO 14.1 — Record four conditions

- [ ] Drone stationary, motors off
- [ ] Motors running, propellers removed
- [ ] Drone securely restrained, propellers fitted
- [ ] Several throttle levels under the safe restrained setup

For each condition, record raw accelerometer and gyro data with accurate timestamps.

### TODO 14.2 — Calculate statistics

- [ ] Mean
- [ ] Standard deviation
- [ ] RMS
- [ ] Peak-to-peak range
- [ ] Minimum and maximum
- [ ] Axis covariance
- [ ] Outlier count
- [ ] Loop-period mean, standard deviation, minimum, and maximum

### TODO 14.3 — Examine frequency content

- [ ] Calculate an FFT or power spectral density on a computer.
- [ ] Identify motor and frame-resonance peaks at each throttle level.
- [ ] Correct damaged/unbalanced propellers, motor problems, frame looseness, and poor mounting first.
- [ ] Select low-pass or notch filtering only after identifying the disturbance.
- [ ] Compare noise reduction with filter response delay.

### Completion condition

Mechanical causes have been corrected, dominant vibration frequencies are understood, and the selected filtering reduces disturbance without unacceptable delay.

## 15. Stage 11 — Automatic pre-flight calibration

The following sequence must run with motors disarmed before every flight.

### Step 15.1 — Boot and communication checks

- [ ] Initialize I2C and all sensor drivers.
- [ ] Verify expected sensor identities.
- [ ] Read and validate BMP180 factory coefficients.
- [ ] Load stored accelerometer, magnetometer, and alignment parameters.
- [ ] Verify stored calibration version and CRC.
- [ ] Check for stuck, impossible, or saturated readings.

**Failure action:** remain disarmed and report the failed sensor or calibration block.

### Step 15.2 — Sensor settling

- [ ] Wait a short defined interval after sensor initialization.
- [ ] Continue reading sensors during the interval.
- [ ] Do not allow arming yet.

### Step 15.3 — Detect a stationary drone

Use a window of IMU samples and calculate:

- Gyro mean and standard deviation on each axis
- Acceleration magnitude mean and standard deviation
- Maximum gyro magnitude
- Outlier count

Acceleration magnitude:

\[
|\mathbf{a}|=\sqrt{a_x^2+a_y^2+a_z^2}
\]

- [ ] Confirm mean acceleration magnitude is close to `1 g`.
- [ ] Confirm gyro standard deviation is below the experimentally selected limit.
- [ ] Confirm acceleration variation is below the selected limit.
- [ ] Restart the stationary window if the drone is touched or moved.

### Step 15.4 — Calculate gyro zero-rate bias

With the drone stationary, collect approximately 1,000–3,000 samples over roughly 2–5 seconds.

For each axis:

\[
b_g=\frac{1}{N}\sum_{i=1}^{N}\omega_i
\]

Apply:

\[
\omega_{cal}=\omega_{raw}-b_g
\]

- [ ] Calculate X/Y/Z gyro bias.
- [ ] Calculate X/Y/Z standard deviation.
- [ ] Reject calibration if motion or excessive noise is detected.
- [ ] Store the current bias in RAM for the flight.
- [ ] Collect a short second window and confirm corrected means are close to zero.

Initial engineering target only: a stationary standard deviation around `0.1–0.3 °/s` or lower may be reasonable, but determine final limits using repeated measurements from the actual drone.

### Step 15.5 — Establish BMP180 ground pressure

- [ ] Keep the drone stationary and protect the sensor from wind.
- [ ] Collect pressure samples for approximately 3–5 seconds.
- [ ] Calculate median and median absolute deviation (MAD).
- [ ] Reject large spikes using a documented robust rule.
- [ ] Calculate mean, standard deviation, minimum, maximum, and valid-sample count.
- [ ] Store the cleaned mean as ground reference `P0`.
- [ ] Define current relative altitude as zero.

Relative altitude:

\[
h=44330\left[1-\left(\frac{P}{P_0}\right)^{1/5.255}\right]
\]

This is altitude relative to the take-off pressure, not guaranteed altitude above sea level.

### Step 15.6 — Magnetometer sanity check

- [ ] Apply stored hard-iron, soft-iron, and axis corrections.
- [ ] Calculate magnetic field magnitude.
- [ ] Compare it with the acceptable range learned during clean calibration.
- [ ] Check for a stuck axis, sudden jumps, or strong local interference.
- [ ] Mark compass data invalid if consistency checks fail.
- [ ] Decide whether the flight controller may arm without compass aiding; document this safety policy.

### Step 15.7 — Final estimator checks

- [ ] Confirm stationary roll and pitch are reasonable.
- [ ] Confirm yaw/heading is stable if the compass is valid.
- [ ] Confirm relative altitude is close to zero.
- [ ] Confirm sensor timestamps and sample intervals are valid.
- [ ] Confirm no queue overrun, missed sensor deadline, or I2C fault is active.
- [ ] Confirm all required calibration flags are valid.

### Step 15.8 — Arming decision

Allow arming only if every mandatory condition passes.

Suggested state flow:

```text
BOOT
  -> SENSOR_CHECK
  -> LOAD_CALIBRATION
  -> SETTLING
  -> WAIT_STATIONARY
  -> GYRO_BIAS
  -> PRESSURE_ZERO
  -> FINAL_HEALTH_CHECK
  -> READY_TO_ARM
```

Any required failure must transition to `CALIBRATION_FAILED` or back to `WAIT_STATIONARY`, while motor outputs remain at the safe disarmed value.

## 16. Statistical method for every calibration

Use the same general procedure for each sensor:

```text
Collect controlled data
  -> check timestamps and sample count
  -> calculate mean, standard deviation, range and outliers
  -> reject moving or corrupted data
  -> calculate calibration parameters
  -> save parameters with version and CRC
  -> collect a new independent dataset
  -> apply correction
  -> calculate residual bias and RMSE
  -> accept or reject
```

### Required statistics

Mean:

\[
\bar{x}=\frac{1}{N}\sum_{i=1}^{N}x_i
\]

Sample standard deviation:

\[
s=\sqrt{\frac{1}{N-1}\sum_{i=1}^{N}(x_i-\bar{x})^2}
\]

Standard error of the mean:

\[
SE=\frac{s}{\sqrt{N}}
\]

Approximate 95% confidence interval:

\[
\bar{x}\pm1.96\frac{s}{\sqrt{N}}
\]

Median absolute deviation:

\[
MAD=\operatorname{median}(|x_i-\operatorname{median}(x)|)
\]

Use the mean to estimate a stable bias, standard deviation to measure random variation, MAD to detect robustly the scale of noisy/outlier-contaminated data, and RMSE on new data to judge final accuracy.

## 17. Required calibration record

Create a record for every full calibration containing:

- [ ] Date and time
- [ ] Firmware version and calibration-format version
- [ ] GY-87/module identifier
- [ ] Sensor ranges, sample rate, and filter settings
- [ ] Sensor mounting orientation
- [ ] Frame, motor, ESC, wiring, battery, and payload configuration
- [ ] Number of collected and rejected samples
- [ ] Temperature range during collection
- [ ] Calculated calibration parameters
- [ ] Calibration dataset statistics
- [ ] Independent validation statistics
- [ ] Final pass/fail result
- [ ] Reason for any rejected calibration

Minimum UART/CSV fields during development:

```text
timestamp_us
accel_raw_x, accel_raw_y, accel_raw_z
accel_cal_x, accel_cal_y, accel_cal_z
gyro_raw_x, gyro_raw_y, gyro_raw_z
gyro_cal_x, gyro_cal_y, gyro_cal_z
mag_raw_x, mag_raw_y, mag_raw_z
mag_cal_x, mag_cal_y, mag_cal_z
mpu_temperature
bmp_temperature
pressure_raw_or_uncompensated
pressure_compensated
relative_altitude
roll, pitch, yaw_or_heading
sample_validity_flags
i2c_error_flags
loop_period_us
```

## 18. Recalibration rules

### Before every flight

- [ ] Gyro zero-rate bias
- [ ] BMP180 ground-pressure reference `P0`
- [ ] Accelerometer `1 g` sanity check
- [ ] Gyro stationary/noise check
- [ ] Magnetometer field sanity check
- [ ] Sensor communication, timing, stored-calibration CRC, and estimator checks

### Do not perform before every flight

- Full six-position accelerometer calibration
- Full 3D magnetometer calibration
- Sensor-to-frame alignment calibration
- Gyro temperature-model creation
- FFT/vibration characterization
- Motor magnetic-interference characterization

### Repeat accelerometer and alignment calibration after

- Replacing or remounting the GY-87
- Changing its orientation
- A strong crash or impact
- Persistent stationary roll/pitch error
- Failure of independent validation

### Repeat full magnetometer calibration after

- Moving the sensor, battery, ESC, or power wiring
- Changing motors or major electrical hardware
- Adding metal, magnetic fasteners, or a nearby payload
- Changing the frame significantly
- A crash
- Persistent or inconsistent heading error

### Repeat vibration and interference tests after

- Changing motors, propellers, frame, mounting material, ESCs, or wiring
- Repairing the drone after a crash
- Observing new oscillation, vibration, or compass disturbance

## 19. Firmware implementation TODO

Implement in the following code order:

- [ ] Sensor identity and communication diagnostics
- [ ] Raw MPU6050, BMP180, and magnetometer drivers
- [ ] Timestamped UART/CSV logging
- [ ] Axis/sign transformation layer
- [ ] Fixed sensor configuration and physical-unit conversion
- [ ] Running statistics module using a numerically stable method such as Welford's algorithm
- [ ] Six-position accelerometer calibration mode
- [ ] Calibration parameter structure with version and CRC
- [ ] Non-volatile save/load logic
- [ ] Magnetometer calibration-data recording mode
- [ ] Magnetometer correction application
- [ ] Pre-flight stationary detector
- [ ] Pre-flight gyro-bias state
- [ ] BMP180 pressure-zero state
- [ ] Sensor-health and magnetic-consistency checks
- [ ] Arming interlock and clear failure reporting
- [ ] Independent validation/test commands
- [ ] Vibration and timing logging modes

## 20. Final readiness checklist before PID tuning

- [ ] Correct sensor models and I2C addresses confirmed
- [ ] Correct body-axis mapping and signs confirmed
- [ ] Sensor ranges and sample timing verified
- [ ] Accelerometer six-position calibration passed independent validation
- [ ] Frame-level reference verified
- [ ] Magnetometer full calibration passed independent validation
- [ ] Motor magnetic interference evaluated
- [ ] BMP180 factory compensation verified
- [ ] Automatic gyro-bias calibration tested repeatedly
- [ ] Automatic ground-pressure zero tested repeatedly
- [ ] Motion during calibration causes safe rejection and restart
- [ ] Corrupt or missing stored calibration prevents unsafe arming
- [ ] Vibration and loop timing characterized
- [ ] Pre-flight state machine reaches `READY_TO_ARM` only when all mandatory checks pass

Only after this checklist is complete should attitude-filter tuning, restrained PID tuning, and flight testing begin.
