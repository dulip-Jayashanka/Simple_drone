# Motor-node Makefile
#
# Compatible with the current repository layout plus the shared
# motor-link protocol added by FMCOM Phase 6.1:
#
#   firmware/
#   ├── motor-link/
#   │   ├── include/
#   │   ├── src/
#   │   └── tests/
#   └── motor-node/
#       ├── Makefile
#       ├── include/
#       ├── linker/stm32f103c8.ld
#       ├── src/
#       ├── startup/
#       └── tests/
#
# Run from firmware/motor-node with the communication link disabled:
#
#   make
#
# Enable the FMCOM Phase 6.1 USART2 receive/validation path with:
#
#   make MOTOR_NODE_LINK_ENABLE=1
#
# Enable the independent USART1 debug observer for validated received
# M1..M4 commands with:
#
#   make MOTOR_NODE_LINK_ENABLE=1 MOTOR_NODE_LINK_UART_DEBUG=1
#

PROJECT := motor-node

FAULT_TEST_MODE ?= 0

# FMCOM Phase 6.1 motor-command communication feature switch.
#
# 0:
#     Preserve the existing DISARMED/safe motor-node firmware without
#     compiling or running the USART2 motor-command receive path.
#
# 1:
#     Compile and run USART2 RX + command receiver + shared motor-link
#     protocol validation. Motor PWM remains outside this phase.
MOTOR_NODE_LINK_ENABLE ?= 0
MOTOR_NODE_LINK_BAUD ?= 230400

# Independent USART1 observer for validated motor commands received over
# the USART2 motor-node link.
#
# This debug path does not alter command_receiver, motor_node_state or the
# motor outputs. It is deliberately rate-limited because uart_diag uses
# polling writes.
MOTOR_NODE_LINK_UART_DEBUG ?= 0
MOTOR_NODE_LINK_UART_RATE_HZ ?= 10


ifneq ($(filter-out 0 1,$(MOTOR_NODE_LINK_ENABLE)),)
$(error MOTOR_NODE_LINK_ENABLE must be 0 or 1)
endif


ifneq ($(filter-out 0 1,$(MOTOR_NODE_LINK_UART_DEBUG)),)
$(error MOTOR_NODE_LINK_UART_DEBUG must be 0 or 1)
endif


ifeq ($(MOTOR_NODE_LINK_UART_DEBUG),1)

ifneq ($(MOTOR_NODE_LINK_ENABLE),1)
$(error MOTOR_NODE_LINK_UART_DEBUG requires MOTOR_NODE_LINK_ENABLE=1)
endif

endif


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

HOST_CC ?= cc


SRC_DIR        := src
INC_DIR        := include
TEST_DIR       := tests
STARTUP_DIR    := startup

PLATFORM_DIR   := ../../platform/stm32f103c8

MOTOR_LINK_DIR := ../motor-link


LINKER_SCRIPT := linker/stm32f103c8.ld


# Existing motor-node core is always compiled.
OBJECTS := \
	$(BUILD_DIR)/main.o \
	$(BUILD_DIR)/motor_outputs.o \
	$(BUILD_DIR)/motor_node_state.o \
	$(BUILD_DIR)/freestanding_mem.o \
	$(BUILD_DIR)/fault_test.o \
	$(BUILD_DIR)/startup_stm32f103c8.o \
	$(BUILD_DIR)/system_clock.o \
	$(BUILD_DIR)/fault_handlers.o \
	$(BUILD_DIR)/status_led.o \
	$(BUILD_DIR)/system_time.o \
	$(BUILD_DIR)/uart_diag.o


# FMCOM communication objects exist only in a link-enabled build.
ifeq ($(MOTOR_NODE_LINK_ENABLE),1)

OBJECTS += \
	$(BUILD_DIR)/command_receiver.o \
	$(BUILD_DIR)/motor_link_protocol.o \
	$(BUILD_DIR)/uart2_link.o

endif


DEPS := $(OBJECTS:.o=.d)

CONFIG_STAMP := $(BUILD_DIR)/.build-config


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
	-DMOTOR_NODE_LINK_ENABLE=$(MOTOR_NODE_LINK_ENABLE) \
	-DMOTOR_NODE_LINK_BAUD=$(MOTOR_NODE_LINK_BAUD) \
	-DMOTOR_NODE_LINK_UART_DEBUG=$(MOTOR_NODE_LINK_UART_DEBUG) \
	-DMOTOR_NODE_LINK_UART_RATE_HZ=$(MOTOR_NODE_LINK_UART_RATE_HZ) \
	-I$(INC_DIR) \
	-I$(TEST_DIR) \
	-I$(PLATFORM_DIR)/include \
	-I$(MOTOR_LINK_DIR)/include \
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


# main.c only depends on communication headers when that feature is enabled.
MAIN_DEPS := \
	$(SRC_DIR)/main.c

ifeq ($(MOTOR_NODE_LINK_ENABLE),1)

MAIN_DEPS += \
	$(INC_DIR)/command_receiver.h \
	$(PLATFORM_DIR)/include/uart2_link.h

endif


.PHONY: all clean size inspect \
	test-normal \
	test-default \
	test-nmi \
	test-hardfault \
	test-state-host \
	test-command-receiver-host \
	test-motor-link-protocol-host \
	FORCE


all: $(ELF) $(BIN) $(HEX) $(LIST) $(SYMBOLS) size


$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)


# Force configuration comparison on every make invocation.  The stamp is
# updated only when a relevant build setting actually changes, which prevents
# stale objects when switching MOTOR_NODE_LINK_ENABLE or baud rate without
# requiring a manual clean first.
FORCE:


$(CONFIG_STAMP): FORCE | $(BUILD_DIR)
	@printf '%s\n' \
		'FAULT_TEST_MODE=$(FAULT_TEST_MODE)' \
		'MOTOR_NODE_LINK_ENABLE=$(MOTOR_NODE_LINK_ENABLE)' \
		'MOTOR_NODE_LINK_BAUD=$(MOTOR_NODE_LINK_BAUD)' \
		'MOTOR_NODE_LINK_UART_DEBUG=$(MOTOR_NODE_LINK_UART_DEBUG)' \
		'MOTOR_NODE_LINK_UART_RATE_HZ=$(MOTOR_NODE_LINK_UART_RATE_HZ)' \
		> $@.tmp
	@if ! cmp -s $@.tmp $@; then mv $@.tmp $@; else rm -f $@.tmp; fi


$(OBJECTS): $(CONFIG_STAMP)


$(BUILD_DIR)/main.o: $(MAIN_DEPS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_outputs.o: \
	$(SRC_DIR)/motor_outputs.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_node_state.o: \
	$(SRC_DIR)/motor_node_state.c \
	$(INC_DIR)/motor_node_state.h \
	$(INC_DIR)/motor_outputs.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/command_receiver.o: \
	$(SRC_DIR)/command_receiver.c \
	$(INC_DIR)/command_receiver.h \
	$(MOTOR_LINK_DIR)/include/motor_link_protocol.h \
	$(PLATFORM_DIR)/include/uart2_link.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_link_protocol.o: \
	$(MOTOR_LINK_DIR)/src/motor_link_protocol.c \
	$(MOTOR_LINK_DIR)/include/motor_link_protocol.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/status_led.o: \
	$(SRC_DIR)/status_led.c \
	$(INC_DIR)/status_led.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/fault_test.o: \
	$(TEST_DIR)/fault_test.c | $(BUILD_DIR)
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


$(BUILD_DIR)/uart2_link.o: \
	$(PLATFORM_DIR)/src/uart2_link.c \
	$(PLATFORM_DIR)/include/uart2_link.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/fault_handlers.o: \
	$(PLATFORM_DIR)/src/fault_handlers.c \
	$(PLATFORM_DIR)/include/fault_handlers.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(ELF): $(OBJECTS) $(LINKER_SCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -lgcc -o $@

$(BUILD_DIR)/freestanding_mem.o: \
	$(SRC_DIR)/freestanding_mem.c | $(BUILD_DIR)

	$(CC) $(CFLAGS) -c $< -o $@

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
		'Reset_Handler|Default_Handler|NMI_Handler|HardFault_Handler|SysTick_Handler|USART2_IRQHandler|DMA1_Channel7_IRQHandler|g_fault_record|g_motor_safe_state_requested|g_motor_outputs_safe|motor_outputs_are_safe|g_system_time_status|g_milliseconds|g_uart_diag_status|g_uart_diag_output_ok|uart_diag_init|g_motor_command_uart_status|g_uart2_link_diag|g_latest_received_motor_command|g_command_receiver_stats|command_receiver_init|command_receiver_process|g_motor_node_state|g_motor_node_state_initialized|g_disarmed_enforcement_count|g_disarmed_safety_failure_count|g_unexpected_state_count|motor_node_state_init|motor_node_state_process|motor_node_is_disarmed'


test-normal:
	$(MAKE) FAULT_TEST_MODE=0 all


test-default:
	$(MAKE) FAULT_TEST_MODE=1 all


test-nmi:
	$(MAKE) FAULT_TEST_MODE=2 all


test-hardfault:
	$(MAKE) FAULT_TEST_MODE=3 all


test-state-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-I$(INC_DIR) \
		$(SRC_DIR)/motor_node_state.c \
		$(TEST_DIR)/motor_node_state_test.c \
		-o build/host/motor_node_state_test
	./build/host/motor_node_state_test


test-command-receiver-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-I$(INC_DIR) \
		-I$(MOTOR_LINK_DIR)/include \
		-I$(PLATFORM_DIR)/include \
		$(TEST_DIR)/command_receiver_test.c \
		$(SRC_DIR)/command_receiver.c \
		$(MOTOR_LINK_DIR)/src/motor_link_protocol.c \
		-o build/host/command_receiver_test
	./build/host/command_receiver_test


test-motor-link-protocol-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-I$(MOTOR_LINK_DIR)/include \
		$(MOTOR_LINK_DIR)/tests/motor_link_protocol_test.c \
		$(MOTOR_LINK_DIR)/src/motor_link_protocol.c \
		-o build/host/motor_link_protocol_test
	./build/host/motor_link_protocol_test


clean:
	rm -rf build


-include $(DEPS)
