# Motor-node Makefile
#
# Compatible with the attached repository layout:
#
#   firmware/motor-node/
#   ├── Makefile
#   ├── include/
#   ├── linker/stm32f103c8.ld
#   ├── src/
#   ├── startup/
#   └── tests/
#
# Run this Makefile from firmware/motor-node with:
#
#   make
#

PROJECT := motor-node
FAULT_TEST_MODE ?= 0

ifeq ($(FAULT_TEST_MODE),0)
BUILD_DIR := build
else
BUILD_DIR := build/fault-test-$(FAULT_TEST_MODE)
endif

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
NM      := arm-none-eabi-nm

# These paths are relative to firmware/motor-node.
SRC_DIR      := src
INC_DIR      := include
TEST_DIR     := tests
STARTUP_DIR  := startup
PLATFORM_DIR := ../../platform/stm32f103c8

# Keep the exact lowercase linker-script name used by the attached project.
LINKER_SCRIPT := linker/stm32f103c8.ld

OBJECTS := \
	$(BUILD_DIR)/main.o \
	$(BUILD_DIR)/motor_outputs.o \
	$(BUILD_DIR)/fault_test.o \
	$(BUILD_DIR)/startup_stm32f103c8.o \
	$(BUILD_DIR)/system_clock.o \
	$(BUILD_DIR)/fault_handlers.o \
	$(BUILD_DIR)/status_led.o \
	$(BUILD_DIR)/system_time.o \
	$(BUILD_DIR)/uart_diag.o

DEPS := $(OBJECTS:.o=.d)

CPU_FLAGS := -mcpu=cortex-m3 -mthumb

CFLAGS := \
	$(CPU_FLAGS) \
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
	-DFAULT_TEST_MODE=$(FAULT_TEST_MODE) \
	-I$(INC_DIR) \
	-I$(TEST_DIR) \
	-I$(PLATFORM_DIR)/include \
	-MMD \
	-MP

LDFLAGS := \
	$(CPU_FLAGS) \
	-nostartfiles \
	-nostdlib \
	-T$(LINKER_SCRIPT) \
	-Wl,--gc-sections \
	-Wl,-Map=$(BUILD_DIR)/$(PROJECT).map \
	-Wl,--print-memory-usage

ELF     := $(BUILD_DIR)/$(PROJECT).elf
BIN     := $(BUILD_DIR)/$(PROJECT).bin
HEX     := $(BUILD_DIR)/$(PROJECT).hex
LIST    := $(BUILD_DIR)/$(PROJECT).list
SYMBOLS := $(BUILD_DIR)/$(PROJECT).symbols

.PHONY: all clean size inspect \
	test-normal test-default test-nmi test-hardfault

all: $(ELF) $(BIN) $(HEX) $(LIST) $(SYMBOLS) size

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/main.o: $(SRC_DIR)/main.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/motor_outputs.o: $(SRC_DIR)/motor_outputs.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/status_led.o: \
	$(SRC_DIR)/status_led.c \
	$(INC_DIR)/status_led.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/fault_test.o: $(TEST_DIR)/fault_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup_stm32f103c8.o: \
	$(STARTUP_DIR)/startup_stm32f103c8.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/system_clock.o: \
	$(PLATFORM_DIR)/src/system_clock.c \
	$(PLATFORM_DIR)/include/system_clock.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/system_time.o: \
	$(PLATFORM_DIR)/src/system_time.c \
	$(PLATFORM_DIR)/include/system_time.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/uart_diag.o: \
	$(PLATFORM_DIR)/src/uart_diag.c \
	$(PLATFORM_DIR)/include/uart_diag.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/fault_handlers.o: \
	$(PLATFORM_DIR)/src/fault_handlers.c \
	$(PLATFORM_DIR)/include/fault_handlers.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJECTS) $(LINKER_SCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -lgcc -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(LIST): $(ELF)
	$(OBJDUMP) -h -S $< > $@

$(SYMBOLS): $(ELF)
	$(NM) -n $< > $@

size: $(ELF)
	$(SIZE) $<

inspect: all
	$(OBJDUMP) -h $(ELF)
	$(OBJDUMP) -s -j .isr_vector $(ELF)
		$(NM) -n $(ELF) | grep -E \
		'Reset_Handler|Default_Handler|NMI_Handler|HardFault_Handler|SysTick_Handler|g_fault_record|g_motor_safe_state_requested|g_motor_outputs_safe|motor_outputs_are_safe|g_system_time_status|g_milliseconds|g_uart_diag_status|g_uart_diag_output_ok|uart_diag_init|uart_diag_write_char|uart_diag_write_string|uart_diag_write_line|uart_diag_write_uint32|uart_diag_write_hex32'

test-normal:
	$(MAKE) FAULT_TEST_MODE=0 all

test-default:
	$(MAKE) FAULT_TEST_MODE=1 all

test-nmi:
	$(MAKE) FAULT_TEST_MODE=2 all

test-hardfault:
	$(MAKE) FAULT_TEST_MODE=3 all

clean:
	rm -rf build

-include $(DEPS)