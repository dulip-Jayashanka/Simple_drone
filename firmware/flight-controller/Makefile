PROJECT := flight-controller
BUILD_DIR := build

ACQ_UART_RAW_DEBUG ?= 0
MPU_WHO_AM_I_TEST ?= 0
IMU_INIT_DEBUG ?= 0
I2C_RECOVERY_TEST ?= 0

I2C_BUS_HZ ?= 400000
IMU_MODEL ?= 9250

ACCEL_CAPTURE_TEST ?= 0
GYRO_CAPTURE_TEST ?= 0

ACCEL_PIPELINE_ENABLE ?= 0

GYRO_PIPELINE_ENABLE ?= 0
GYRO_PIPELINE_UART_DEBUG ?= 0
GYRO_INITIAL_BIAS_CAL_ENABLE ?= 0
GYRO_PIPELINE_UART_RATE_HZ ?= 10

ATTITUDE_ESTIMATOR_ENABLE ?= 0
ATTITUDE_ESTIMATOR_UART_DEBUG ?= 0
ATTITUDE_ESTIMATOR_UART_RATE_HZ ?= 2
ATTITUDE_EULER_RATE_HZ ?= 100

# Inner angular-rate control.
INNER_RATE_CONTROL_ENABLE ?= 0
INNER_RATE_CONTROL_UART_DEBUG ?= 0

# Outer roll/pitch attitude control.
OUTER_ATTITUDE_CONTROL_ENABLE ?= 0
OUTER_ATTITUDE_CONTROL_UART_DEBUG ?= 0

# Flight-controller motor mixer.
MOTOR_MIXER_ENABLE ?= 0

# Independent motor-mixer UART diagnostics.
MOTOR_MIXER_UART_DEBUG ?= 0

MEASURE_PIPELINE_TIMES ?= 0
MEASURE_RAW_TO_ATTITUDE_TIMES ?= 0

ACCEL_PIPELINE_DEBUG_RAW ?= 0
ACCEL_PIPELINE_DEBUG_MEDIAN ?= 0
ACCEL_PIPELINE_DEBUG_CALIBRATED_G ?= 0
ACCEL_PIPELINE_DEBUG_FILTERED_G ?= 0
ACCEL_PIPELINE_DEBUG_MS2 ?= 0
ACCEL_PIPELINE_DEBUG_DECIMATION ?= 50


BOOLEAN_BUILD_VALUES := \
	$(ACQ_UART_RAW_DEBUG) \
	$(MPU_WHO_AM_I_TEST) \
	$(IMU_INIT_DEBUG) \
	$(I2C_RECOVERY_TEST) \
	$(ACCEL_CAPTURE_TEST) \
	$(GYRO_CAPTURE_TEST) \
	$(ACCEL_PIPELINE_ENABLE) \
	$(GYRO_PIPELINE_ENABLE) \
	$(GYRO_PIPELINE_UART_DEBUG) \
	$(GYRO_INITIAL_BIAS_CAL_ENABLE) \
	$(ATTITUDE_ESTIMATOR_ENABLE) \
	$(ATTITUDE_ESTIMATOR_UART_DEBUG) \
	$(INNER_RATE_CONTROL_ENABLE) \
	$(INNER_RATE_CONTROL_UART_DEBUG) \
	$(OUTER_ATTITUDE_CONTROL_ENABLE) \
	$(OUTER_ATTITUDE_CONTROL_UART_DEBUG) \
	$(MOTOR_MIXER_ENABLE) \
	$(MOTOR_MIXER_UART_DEBUG) \
	$(MEASURE_PIPELINE_TIMES) \
	$(MEASURE_RAW_TO_ATTITUDE_TIMES) \
	$(ACCEL_PIPELINE_DEBUG_RAW) \
	$(ACCEL_PIPELINE_DEBUG_MEDIAN) \
	$(ACCEL_PIPELINE_DEBUG_CALIBRATED_G) \
	$(ACCEL_PIPELINE_DEBUG_FILTERED_G) \
	$(ACCEL_PIPELINE_DEBUG_MS2)


ifneq ($(filter-out 0 1,$(BOOLEAN_BUILD_VALUES)),)
$(error All enable/debug build flags must be 0 or 1)
endif


ACCEL_PIPELINE_DEBUG_VALUES := \
	$(ACCEL_PIPELINE_DEBUG_RAW) \
	$(ACCEL_PIPELINE_DEBUG_MEDIAN) \
	$(ACCEL_PIPELINE_DEBUG_CALIBRATED_G) \
	$(ACCEL_PIPELINE_DEBUG_FILTERED_G) \
	$(ACCEL_PIPELINE_DEBUG_MS2)


ifneq ($(filter 1,$(ACCEL_PIPELINE_DEBUG_VALUES)),)
ifneq ($(ACCEL_PIPELINE_ENABLE),1)
$(error Accelerometer pipeline debug requires ACCEL_PIPELINE_ENABLE=1)
endif
endif


ifeq ($(ACCEL_CAPTURE_TEST)$(GYRO_CAPTURE_TEST),11)
$(error ACCEL_CAPTURE_TEST and GYRO_CAPTURE_TEST cannot both be enabled)
endif


ifeq ($(GYRO_PIPELINE_UART_DEBUG),1)
ifneq ($(GYRO_PIPELINE_ENABLE),1)
$(error GYRO_PIPELINE_UART_DEBUG requires GYRO_PIPELINE_ENABLE=1)
endif
endif


ifeq ($(GYRO_INITIAL_BIAS_CAL_ENABLE),1)
ifneq ($(GYRO_PIPELINE_ENABLE),1)
$(error GYRO_INITIAL_BIAS_CAL_ENABLE requires GYRO_PIPELINE_ENABLE=1)
endif
endif


ifeq ($(ATTITUDE_ESTIMATOR_ENABLE),1)

ifneq ($(ACCEL_PIPELINE_ENABLE),1)
$(error ATTITUDE_ESTIMATOR_ENABLE requires ACCEL_PIPELINE_ENABLE=1)
endif

ifneq ($(GYRO_PIPELINE_ENABLE),1)
$(error ATTITUDE_ESTIMATOR_ENABLE requires GYRO_PIPELINE_ENABLE=1)
endif

endif


ifeq ($(ATTITUDE_ESTIMATOR_UART_DEBUG),1)

ifneq ($(ATTITUDE_ESTIMATOR_ENABLE),1)
$(error ATTITUDE_ESTIMATOR_UART_DEBUG requires ATTITUDE_ESTIMATOR_ENABLE=1)
endif

endif


#
# The current body-frame integration path is created inside the
# attitude-estimator integration block.
#
ifeq ($(INNER_RATE_CONTROL_ENABLE),1)

ifneq ($(ATTITUDE_ESTIMATOR_ENABLE),1)
$(error INNER_RATE_CONTROL_ENABLE currently requires ATTITUDE_ESTIMATOR_ENABLE=1)
endif

endif


ifeq ($(INNER_RATE_CONTROL_UART_DEBUG),1)

ifneq ($(INNER_RATE_CONTROL_ENABLE),1)
$(error INNER_RATE_CONTROL_UART_DEBUG requires INNER_RATE_CONTROL_ENABLE=1)
endif

endif


#
# Outer roll/pitch control consumes estimator Euler angles and produces
# rate setpoints for the existing inner rate controller.
#
ifeq ($(OUTER_ATTITUDE_CONTROL_ENABLE),1)

ifneq ($(ATTITUDE_ESTIMATOR_ENABLE),1)
$(error OUTER_ATTITUDE_CONTROL_ENABLE requires ATTITUDE_ESTIMATOR_ENABLE=1)
endif

ifneq ($(INNER_RATE_CONTROL_ENABLE),1)
$(error OUTER_ATTITUDE_CONTROL_ENABLE requires INNER_RATE_CONTROL_ENABLE=1)
endif

endif


ifeq ($(OUTER_ATTITUDE_CONTROL_UART_DEBUG),1)

ifneq ($(OUTER_ATTITUDE_CONTROL_ENABLE),1)
$(error OUTER_ATTITUDE_CONTROL_UART_DEBUG requires OUTER_ATTITUDE_CONTROL_ENABLE=1)
endif

endif


#
# The motor mixer consumes the abstract roll/pitch/yaw corrections
# produced by the inner angular-rate controller and combines them
# with collective.
#
ifeq ($(MOTOR_MIXER_ENABLE),1)

ifneq ($(INNER_RATE_CONTROL_ENABLE),1)
$(error MOTOR_MIXER_ENABLE requires INNER_RATE_CONTROL_ENABLE=1)
endif

endif


#
# Mixer UART diagnostics are an independent debug path.
#
# They require the mixer itself, but do not require any of the
# existing controller/pipeline UART debug flags.
#
ifeq ($(MOTOR_MIXER_UART_DEBUG),1)

ifneq ($(MOTOR_MIXER_ENABLE),1)
$(error MOTOR_MIXER_UART_DEBUG requires MOTOR_MIXER_ENABLE=1)
endif

endif


ifneq ($(filter 1,$(ACCEL_CAPTURE_TEST) $(GYRO_CAPTURE_TEST)),)

ifeq ($(GYRO_PIPELINE_ENABLE),1)
$(error Gyro pipeline cannot run while a capture test owns the main loop)
endif

endif


ifeq ($(MEASURE_PIPELINE_TIMES),1)

ifneq ($(ACCEL_PIPELINE_ENABLE),1)
$(error MEASURE_PIPELINE_TIMES requires ACCEL_PIPELINE_ENABLE=1)
endif

endif


ifeq ($(MEASURE_RAW_TO_ATTITUDE_TIMES),1)

ifneq ($(ATTITUDE_ESTIMATOR_ENABLE),1)
$(error MEASURE_RAW_TO_ATTITUDE_TIMES requires ATTITUDE_ESTIMATOR_ENABLE=1)
endif


ifneq ($(INNER_RATE_CONTROL_ENABLE),0)
$(error Raw-to-attitude timing must be measured with INNER_RATE_CONTROL_ENABLE=0)
endif


ifneq ($(MEASURE_PIPELINE_TIMES),0)
$(error Do not enable MEASURE_PIPELINE_TIMES and MEASURE_RAW_TO_ATTITUDE_TIMES together)
endif


ifneq ($(ACQ_UART_RAW_DEBUG)$(GYRO_PIPELINE_UART_DEBUG)$(ATTITUDE_ESTIMATOR_UART_DEBUG)$(INNER_RATE_CONTROL_UART_DEBUG)$(OUTER_ATTITUDE_CONTROL_UART_DEBUG)$(MOTOR_MIXER_UART_DEBUG),000000)
$(error Raw-to-attitude timing requires all continuous UART debug outputs to be disabled)
endif


ifneq ($(filter 1,$(ACCEL_PIPELINE_DEBUG_VALUES)),)
$(error Raw-to-attitude timing requires accelerometer UART debug outputs to be disabled)
endif

endif


ifeq ($(ACCEL_PIPELINE_ENABLE),1)

ifneq ($(IMU_MODEL),6500)
$(error Current accelerometer calibration constants belong to IMU_MODEL=6500)
endif

endif


ifeq ($(GYRO_PIPELINE_ENABLE),1)

ifneq ($(IMU_MODEL),6500)
$(error Current gyroscope bias constants belong to IMU_MODEL=6500)
endif

endif


ifeq ($(ACCEL_CAPTURE_TEST)$(ACCEL_PIPELINE_ENABLE),11)

$(warning ACCEL_CAPTURE_TEST owns the main loop; pipeline processing/debug will not run)

endif


ifeq ($(GYRO_CAPTURE_TEST)$(ACCEL_PIPELINE_ENABLE),11)

$(warning GYRO_CAPTURE_TEST owns the main loop; accelerometer pipeline processing/debug will not run)

endif


VALID_IMU_MODELS := 6500 9250


ifeq ($(filter $(IMU_MODEL),$(VALID_IMU_MODELS)),)

$(error Unsupported IMU_MODEL='$(IMU_MODEL)'. Use IMU_MODEL=6500 or IMU_MODEL=9250)

endif


CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
NM      := arm-none-eabi-nm

HOST_CC ?= cc


PLATFORM_DIR := ../../platform/stm32f103c8
MPU_DIR      := ../../drivers/mpu9250


CPU_FLAGS := -mcpu=cortex-m3 -mthumb


CFLAGS := $(CPU_FLAGS) \
	-std=c11 \
	-Wall \
	-Wextra \
	-Werror \
	-O0 \
	-g3 \
	-ffreestanding \
	-ffunction-sections \
	-fdata-sections \
	-fno-common \
	-fno-unwind-tables \
	-fno-asynchronous-unwind-tables \
	-DACQ_UART_RAW_DEBUG=$(ACQ_UART_RAW_DEBUG) \
	-DMPU_WHO_AM_I_TEST=$(MPU_WHO_AM_I_TEST) \
	-DIMU_INIT_DEBUG=$(IMU_INIT_DEBUG) \
	-DI2C_RECOVERY_TEST=$(I2C_RECOVERY_TEST) \
	-DI2C_BUS_HZ=$(I2C_BUS_HZ) \
	-DIMU_MODEL=$(IMU_MODEL) \
	-DACCEL_CAPTURE_TEST=$(ACCEL_CAPTURE_TEST) \
	-DGYRO_CAPTURE_TEST=$(GYRO_CAPTURE_TEST) \
	-DACCEL_PIPELINE_ENABLE=$(ACCEL_PIPELINE_ENABLE) \
	-DGYRO_PIPELINE_ENABLE=$(GYRO_PIPELINE_ENABLE) \
	-DGYRO_PIPELINE_UART_DEBUG=$(GYRO_PIPELINE_UART_DEBUG) \
	-DGYRO_INITIAL_BIAS_CAL_ENABLE=$(GYRO_INITIAL_BIAS_CAL_ENABLE) \
	-DGYRO_PIPELINE_UART_RATE_HZ=$(GYRO_PIPELINE_UART_RATE_HZ) \
	-DATTITUDE_ESTIMATOR_ENABLE=$(ATTITUDE_ESTIMATOR_ENABLE) \
	-DATTITUDE_ESTIMATOR_UART_DEBUG=$(ATTITUDE_ESTIMATOR_UART_DEBUG) \
	-DATTITUDE_ESTIMATOR_UART_RATE_HZ=$(ATTITUDE_ESTIMATOR_UART_RATE_HZ) \
	-DATTITUDE_EULER_RATE_HZ=$(ATTITUDE_EULER_RATE_HZ) \
	-DINNER_RATE_CONTROL_ENABLE=$(INNER_RATE_CONTROL_ENABLE) \
	-DINNER_RATE_CONTROL_UART_DEBUG=$(INNER_RATE_CONTROL_UART_DEBUG) \
	-DOUTER_ATTITUDE_CONTROL_ENABLE=$(OUTER_ATTITUDE_CONTROL_ENABLE) \
	-DOUTER_ATTITUDE_CONTROL_UART_DEBUG=$(OUTER_ATTITUDE_CONTROL_UART_DEBUG) \
	-DMOTOR_MIXER_ENABLE=$(MOTOR_MIXER_ENABLE) \
	-DMOTOR_MIXER_UART_DEBUG=$(MOTOR_MIXER_UART_DEBUG) \
	-DMEASURE_PIPELINE_TIMES=$(MEASURE_PIPELINE_TIMES) \
	-DMEASURE_RAW_TO_ATTITUDE_TIMES=$(MEASURE_RAW_TO_ATTITUDE_TIMES) \
	-DACCEL_PIPELINE_DEBUG_RAW=$(ACCEL_PIPELINE_DEBUG_RAW) \
	-DACCEL_PIPELINE_DEBUG_MEDIAN=$(ACCEL_PIPELINE_DEBUG_MEDIAN) \
	-DACCEL_PIPELINE_DEBUG_CALIBRATED_G=$(ACCEL_PIPELINE_DEBUG_CALIBRATED_G) \
	-DACCEL_PIPELINE_DEBUG_FILTERED_G=$(ACCEL_PIPELINE_DEBUG_FILTERED_G) \
	-DACCEL_PIPELINE_DEBUG_MS2=$(ACCEL_PIPELINE_DEBUG_MS2) \
	-DACCEL_PIPELINE_DEBUG_DECIMATION=$(ACCEL_PIPELINE_DEBUG_DECIMATION) \
	-Iinclude \
	-I$(PLATFORM_DIR)/include \
	-I$(MPU_DIR)/include \
	-MMD \
	-MP


LDFLAGS := $(CPU_FLAGS) \
	-nostartfiles \
	-nostdlib \
	-Tlinker/stm32f103c8.ld \
	-Wl,--gc-sections \
	-Wl,-Map=$(BUILD_DIR)/$(PROJECT).map \
	-Wl,--print-memory-usage


SOURCES := \
	src/main.c \
	src/imu_acquisition.c \
	src/flight_fault_handlers.c \
	src/freestanding_mem.c \
	startup/startup_stm32f103c8.c \
	$(MPU_DIR)/src/mpu9250.c \
	$(PLATFORM_DIR)/src/i2c1.c \
	$(PLATFORM_DIR)/src/micros.c \
	$(PLATFORM_DIR)/src/system_clock.c \
	$(PLATFORM_DIR)/src/system_time.c \
	$(PLATFORM_DIR)/src/uart_diag.c


ifeq ($(ACCEL_PIPELINE_ENABLE),1)

SOURCES += src/accel_pipeline.c

endif


ifeq ($(GYRO_PIPELINE_ENABLE),1)

SOURCES += src/gyro_pipeline.c

endif


ifeq ($(ATTITUDE_ESTIMATOR_ENABLE),1)

SOURCES += src/imu_body_frame.c
SOURCES += src/attitude_estimator.c

endif


ifeq ($(INNER_RATE_CONTROL_ENABLE),1)

SOURCES += src/pid.c
SOURCES += src/rate_controller.c

endif


ifeq ($(OUTER_ATTITUDE_CONTROL_ENABLE),1)

SOURCES += src/attitude_controller.c

endif


ifeq ($(MOTOR_MIXER_ENABLE),1)

SOURCES += src/motor_mixer.c

endif


OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.c=.o)))


DEPS := $(OBJECTS:.o.d)

CONFIG_STAMP := $(BUILD_DIR)/.build-config


ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin
HEX := $(BUILD_DIR)/$(PROJECT).hex


.PHONY: all clean size inspect \
	test-host \
	test-accel-host \
	test-gyro-host \
	test-attitude-host \
	test-pid-host \
	test-rate-host \
	test-attitude-controller-host \
	test-mixer-host \
	FORCE


all: $(ELF) $(BIN) $(HEX) size


$(BUILD_DIR):
	mkdir -p $@


FORCE:


$(CONFIG_STAMP): FORCE | $(BUILD_DIR)
	@printf '%s\n' \
		'IMU_MODEL=$(IMU_MODEL)' \
		'I2C_BUS_HZ=$(I2C_BUS_HZ)' \
		'ACQ_UART_RAW_DEBUG=$(ACQ_UART_RAW_DEBUG)' \
		'MPU_WHO_AM_I_TEST=$(MPU_WHO_AM_I_TEST)' \
		'IMU_INIT_DEBUG=$(IMU_INIT_DEBUG)' \
		'I2C_RECOVERY_TEST=$(I2C_RECOVERY_TEST)' \
		'ACCEL_CAPTURE_TEST=$(ACCEL_CAPTURE_TEST)' \
		'GYRO_CAPTURE_TEST=$(GYRO_CAPTURE_TEST)' \
		'ACCEL_PIPELINE_ENABLE=$(ACCEL_PIPELINE_ENABLE)' \
		'GYRO_PIPELINE_ENABLE=$(GYRO_PIPELINE_ENABLE)' \
		'GYRO_PIPELINE_UART_DEBUG=$(GYRO_PIPELINE_UART_DEBUG)' \
		'GYRO_INITIAL_BIAS_CAL_ENABLE=$(GYRO_INITIAL_BIAS_CAL_ENABLE)' \
		'GYRO_PIPELINE_UART_RATE_HZ=$(GYRO_PIPELINE_UART_RATE_HZ)' \
		'ATTITUDE_ESTIMATOR_ENABLE=$(ATTITUDE_ESTIMATOR_ENABLE)' \
		'ATTITUDE_ESTIMATOR_UART_DEBUG=$(ATTITUDE_ESTIMATOR_UART_DEBUG)' \
		'ATTITUDE_ESTIMATOR_UART_RATE_HZ=$(ATTITUDE_ESTIMATOR_UART_RATE_HZ)' \
		'ATTITUDE_EULER_RATE_HZ=$(ATTITUDE_EULER_RATE_HZ)' \
		'INNER_RATE_CONTROL_ENABLE=$(INNER_RATE_CONTROL_ENABLE)' \
		'INNER_RATE_CONTROL_UART_DEBUG=$(INNER_RATE_CONTROL_UART_DEBUG)' \
		'OUTER_ATTITUDE_CONTROL_ENABLE=$(OUTER_ATTITUDE_CONTROL_ENABLE)' \
		'OUTER_ATTITUDE_CONTROL_UART_DEBUG=$(OUTER_ATTITUDE_CONTROL_UART_DEBUG)' \
		'MOTOR_MIXER_ENABLE=$(MOTOR_MIXER_ENABLE)' \
		'MOTOR_MIXER_UART_DEBUG=$(MOTOR_MIXER_UART_DEBUG)' \
		'MEASURE_PIPELINE_TIMES=$(MEASURE_PIPELINE_TIMES)' \
		'MEASURE_RAW_TO_ATTITUDE_TIMES=$(MEASURE_RAW_TO_ATTITUDE_TIMES)' \
		'ACCEL_PIPELINE_DEBUG_RAW=$(ACCEL_PIPELINE_DEBUG_RAW)' \
		'ACCEL_PIPELINE_DEBUG_MEDIAN=$(ACCEL_PIPELINE_DEBUG_MEDIAN)' \
		'ACCEL_PIPELINE_DEBUG_CALIBRATED_G=$(ACCEL_PIPELINE_DEBUG_CALIBRATED_G)' \
		'ACCEL_PIPELINE_DEBUG_FILTERED_G=$(ACCEL_PIPELINE_DEBUG_FILTERED_G)' \
		'ACCEL_PIPELINE_DEBUG_MS2=$(ACCEL_PIPELINE_DEBUG_MS2)' \
		'ACCEL_PIPELINE_DEBUG_DECIMATION=$(ACCEL_PIPELINE_DEBUG_DECIMATION)' \
		> $@.tmp
	@if ! cmp -s $@.tmp $@; then mv $@.tmp $@; else rm -f $@.tmp; fi


$(OBJECTS): $(CONFIG_STAMP)


define COMPILE_template
$(BUILD_DIR)/$(notdir $(1:.c=.o)): $(1) | $(BUILD_DIR)
	$$(CC) $$(CFLAGS) -c $$< -o $$@
endef


$(foreach source,$(SOURCES),\
	$(eval $(call COMPILE_template,$(source))))


$(ELF): $(OBJECTS) linker/stm32f103c8.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -lgcc -o $@


$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@


$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@


size: $(ELF)
	$(SIZE) $<


inspect: all
	$(OBJDUMP) -h $(ELF)
	$(NM) -n $(ELF) | grep -E \
		'EXTI0_IRQHandler|TIM2_IRQHandler|imu_acquisition_process|attitude_controller_update|rate_controller_update|motor_mixer_update|g_latest_raw_imu_sample|g_latest_gyro_pipeline_output|g_latest_attitude_estimator_output|g_latest_attitude_controller_output|g_latest_rate_controller_output|g_latest_motor_mixer_output|g_raw_to_attitude_timing|g_imu_acquisition_stats|g_pipeline_time_|g_i2c1_diag|micros|i2c1_read_registers|i2c1_recover'


test-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-I$(PLATFORM_DIR)/include \
		-I$(MPU_DIR)/include \
		tests/mpu9250_decode_test.c \
		$(MPU_DIR)/src/mpu9250.c \
		-o build/host/mpu9250_decode_test
	./build/host/mpu9250_decode_test


test-accel-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-Iinclude \
		tests/accel_pipeline_test.c \
		src/accel_pipeline.c \
		-lm \
		-o build/host/accel_pipeline_test
	./build/host/accel_pipeline_test


test-gyro-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-Iinclude \
		tests/gyro_pipeline_test.c \
		src/gyro_pipeline.c \
		-lm \
		-o build/host/gyro_pipeline_test
	./build/host/gyro_pipeline_test


test-attitude-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-Iinclude \
		tests/attitude_estimator_test.c \
		src/attitude_estimator.c \
		src/imu_body_frame.c \
		-lm \
		-o build/host/attitude_estimator_test
	./build/host/attitude_estimator_test


test-pid-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-Iinclude \
		tests/pid_test.c \
		src/pid.c \
		-lm \
		-o build/host/pid_test
	./build/host/pid_test


test-rate-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-Iinclude \
		tests/rate_controller_test.c \
		src/rate_controller.c \
		src/pid.c \
		-lm \
		-o build/host/rate_controller_test
	./build/host/rate_controller_test


test-attitude-controller-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-Iinclude \
		tests/attitude_controller_test.c \
		src/attitude_controller.c \
		-lm \
		-o build/host/attitude_controller_test
	./build/host/attitude_controller_test


test-mixer-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-Iinclude \
		tests/motor_mixer_test.c \
		src/motor_mixer.c \
		-lm \
		-o build/host/motor_mixer_test
	./build/host/motor_mixer_test


clean:
	rm -rf $(BUILD_DIR)


-include $(DEPS)