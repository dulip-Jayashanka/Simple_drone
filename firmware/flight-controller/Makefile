PROJECT := flight-controller
BUILD_DIR := build
ACQ_UART_RAW_DEBUG ?= 0
MPU_WHO_AM_I_TEST ?= 0
IMU_INIT_DEBUG ?= 0
I2C_RECOVERY_TEST ?= 0
IMU_MODEL ?= 9250

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
	-DIMU_MODEL=$(IMU_MODEL) \
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

OBJECTS := \
	$(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.c=.o)))

DEPS := $(OBJECTS:.o=.d)

ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin
HEX := $(BUILD_DIR)/$(PROJECT).hex

.PHONY: all clean size inspect test-host

all: $(ELF) $(BIN) $(HEX) size

$(BUILD_DIR):
	mkdir -p $@

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
		'EXTI0_IRQHandler|imu_acquisition_process|g_latest_raw_imu_sample|g_imu_acquisition_stats|micros|i2c1_read_registers'

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

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)