# Motor-node Makefile
#
# Current motor-node architecture:
#
#   Phase 6.1
#       USART2 motor command receive
#       packet validation
#       optional USART1 received-value debug
#
#   Phase 6.2
#       valid-command watchdog
#       DISARMED / ARMED / FAILSAFE state model
#       latched command-timeout failsafe
#
#   Phase 6.2.5
#       requested DISARMED / ARMED state carried with M1..M4
#       local state-request safety gate
#       DISARM-before-ARM communication-session interlock
#
# PWM remains outside this phase.
#
#
# Normal safe motor-node:
#
#   make
#
#
# Phase 6.1 command receiver:
#
#   make MOTOR_NODE_LINK_ENABLE=1
#
#
# Phase 6.1 with received-value USART1 debug:
#
#   make \
#       MOTOR_NODE_LINK_ENABLE=1 \
#       MOTOR_NODE_LINK_UART_DEBUG=1
#
#
# Phase 6.2 / 6.2.5 command watchdog + requested-state gate:
#
#   make \
#       MOTOR_NODE_LINK_ENABLE=1 \
#       MOTOR_COMMAND_WATCHDOG_ENABLE=1
#
#
# Phase 6.2 / 6.2.5 + received-value USART1 debug:
#
#   make \
#       MOTOR_NODE_LINK_ENABLE=1 \
#       MOTOR_COMMAND_WATCHDOG_ENABLE=1 \
#       MOTOR_NODE_LINK_UART_DEBUG=1
#


PROJECT := motor-node


FAULT_TEST_MODE ?= 0


# ============================================================
# FMCOM PHASE 6.1 — MOTOR COMMAND LINK
# ============================================================

MOTOR_NODE_LINK_ENABLE ?= 0

MOTOR_NODE_LINK_BAUD ?= 230400


# Independent USART1 observer for validated commands received
# through USART2.
#
# uart_diag is polling, therefore this output remains deliberately
# rate-limited and diagnostic only.
MOTOR_NODE_LINK_UART_DEBUG ?= 0

MOTOR_NODE_LINK_UART_RATE_HZ ?= 10


# ============================================================
# FMCOM PHASE 6.2 — COMMAND WATCHDOG
# ============================================================
#
# 20 ms is the initial bench value.
#
# With a nominal 500 Hz command stream:
#
#     expected period ~= 2 ms
#
# 20 ms represents approximately ten expected command periods.
#
# Final flight timeout must later be chosen using measured
# command period and jitter.
MOTOR_COMMAND_WATCHDOG_ENABLE ?= 0

MOTOR_COMMAND_TIMEOUT_MS ?= 20


# ============================================================
# BUILD-FLAG VALIDATION
# ============================================================

ifneq ($(filter-out 0 1,$(MOTOR_NODE_LINK_ENABLE)),)
$(error MOTOR_NODE_LINK_ENABLE must be 0 or 1)
endif


ifneq ($(filter-out 0 1,$(MOTOR_NODE_LINK_UART_DEBUG)),)
$(error MOTOR_NODE_LINK_UART_DEBUG must be 0 or 1)
endif


ifneq ($(filter-out 0 1,$(MOTOR_COMMAND_WATCHDOG_ENABLE)),)
$(error MOTOR_COMMAND_WATCHDOG_ENABLE must be 0 or 1)
endif


ifeq ($(MOTOR_NODE_LINK_UART_DEBUG),1)

ifneq ($(MOTOR_NODE_LINK_ENABLE),1)
$(error MOTOR_NODE_LINK_UART_DEBUG requires MOTOR_NODE_LINK_ENABLE=1)
endif

endif


ifeq ($(MOTOR_COMMAND_WATCHDOG_ENABLE),1)

ifneq ($(MOTOR_NODE_LINK_ENABLE),1)
$(error MOTOR_COMMAND_WATCHDOG_ENABLE requires MOTOR_NODE_LINK_ENABLE=1)
endif

endif


# ============================================================
# BUILD DIRECTORY
# ============================================================

ifeq ($(FAULT_TEST_MODE),0)

BUILD_DIR := build

else

BUILD_DIR := build/fault-test-$(FAULT_TEST_MODE)

endif


# ============================================================
# TOOLS
# ============================================================

CC      := arm-none-eabi-gcc

OBJCOPY := arm-none-eabi-objcopy

OBJDUMP := arm-none-eabi-objdump

SIZE    := arm-none-eabi-size

NM      := arm-none-eabi-nm


HOST_CC ?= cc


# ============================================================
# DIRECTORIES
# ============================================================

SRC_DIR :=
SRC_DIR := src


INC_DIR :=
INC_DIR := include


TEST_DIR :=
TEST_DIR := tests


STARTUP_DIR :=
STARTUP_DIR := startup


PLATFORM_DIR :=
PLATFORM_DIR := ../../platform/stm32f103c8


MOTOR_LINK_DIR :=
MOTOR_LINK_DIR := ../motor-link


LINKER_SCRIPT :=
LINKER_SCRIPT := linker/stm32f103c8.ld


# ============================================================
# OBJECTS — ALWAYS PRESENT
# ============================================================

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


# ============================================================
# PHASE 6.1 COMMUNICATION OBJECTS
# ============================================================

ifeq ($(MOTOR_NODE_LINK_ENABLE),1)

OBJECTS += \
	$(BUILD_DIR)/command_receiver.o \
	$(BUILD_DIR)/motor_link_protocol.o \
	$(BUILD_DIR)/uart2_link.o

endif


# ============================================================
# PHASE 6.2 / 6.2.5 WATCHDOG + STATE-GATE OBJECTS
# ============================================================

ifeq ($(MOTOR_COMMAND_WATCHDOG_ENABLE),1)

OBJECTS += \
	$(BUILD_DIR)/command_watchdog.o \
	$(BUILD_DIR)/motor_command_gate.o

endif


DEPS :=
DEPS := $(OBJECTS:.o=.d)


CONFIG_STAMP :=
CONFIG_STAMP := $(BUILD_DIR)/.build-config


# ============================================================
# COMPILER FLAGS
# ============================================================

CPU_FLAGS :=
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
	-DMOTOR_COMMAND_WATCHDOG_ENABLE=$(MOTOR_COMMAND_WATCHDOG_ENABLE) \
	-DMOTOR_COMMAND_TIMEOUT_MS=$(MOTOR_COMMAND_TIMEOUT_MS) \
	-I$(INC_DIR) \
	-I$(TEST_DIR) \
	-I$(PLATFORM_DIR)/include \
	-I$(MOTOR_LINK_DIR)/include \
	-MMD \
	-MP


# ============================================================
# LINKER FLAGS
# ============================================================

LDFLAGS := \
	$(CPU_FLAGS) \
	-nostartfiles \
	-nostdlib \
	-T$(LINKER_SCRIPT) \
	-Wl,--gc-sections \
	-Wl,-Map=$(BUILD_DIR)/$(PROJECT).map \
	-Wl,--print-memory-usage


# ============================================================
# OUTPUT FILES
# ============================================================

ELF :=
ELF := $(BUILD_DIR)/$(PROJECT).elf


BIN :=
BIN := $(BUILD_DIR)/$(PROJECT).bin


HEX :=
HEX := $(BUILD_DIR)/$(PROJECT).hex


LIST :=
LIST := $(BUILD_DIR)/$(PROJECT).list


SYMBOLS :=
SYMBOLS := $(BUILD_DIR)/$(PROJECT).symbols


# ============================================================
# MAIN.C CONDITIONAL DEPENDENCIES
# ============================================================

MAIN_DEPS := \
	$(SRC_DIR)/main.c


ifeq ($(MOTOR_NODE_LINK_ENABLE),1)

MAIN_DEPS += \
	$(INC_DIR)/command_receiver.h \
	$(PLATFORM_DIR)/include/uart2_link.h

endif


ifeq ($(MOTOR_COMMAND_WATCHDOG_ENABLE),1)

MAIN_DEPS += \
	$(INC_DIR)/command_watchdog.h \
	$(INC_DIR)/motor_command_gate.h

endif


# ============================================================
# PHONY TARGETS
# ============================================================

.PHONY: all clean size inspect \
	test-normal \
	test-default \
	test-nmi \
	test-hardfault \
	test-state-host \
	test-command-receiver-host \
	test-command-watchdog-host \
	test-motor-command-gate-host \
	test-motor-link-protocol-host \
	FORCE


all: $(ELF) $(BIN) $(HEX) $(LIST) $(SYMBOLS) size


# ============================================================
# BUILD DIRECTORY
# ============================================================

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)


# ============================================================
# BUILD CONFIGURATION STAMP
# ============================================================
#
# Prevent stale objects when changing feature flags without
# manually cleaning first.

FORCE:


$(CONFIG_STAMP): FORCE | $(BUILD_DIR)
	@printf '%s\n' \
		'FAULT_TEST_MODE=$(FAULT_TEST_MODE)' \
		'MOTOR_NODE_LINK_ENABLE=$(MOTOR_NODE_LINK_ENABLE)' \
		'MOTOR_NODE_LINK_BAUD=$(MOTOR_NODE_LINK_BAUD)' \
		'MOTOR_NODE_LINK_UART_DEBUG=$(MOTOR_NODE_LINK_UART_DEBUG)' \
		'MOTOR_NODE_LINK_UART_RATE_HZ=$(MOTOR_NODE_LINK_UART_RATE_HZ)' \
		'MOTOR_COMMAND_WATCHDOG_ENABLE=$(MOTOR_COMMAND_WATCHDOG_ENABLE)' \
		'MOTOR_COMMAND_TIMEOUT_MS=$(MOTOR_COMMAND_TIMEOUT_MS)' \
		> $@.tmp
	@if ! cmp -s $@.tmp $@; then mv $@.tmp $@; else rm -f $@.tmp; fi


$(OBJECTS): $(CONFIG_STAMP)


# ============================================================
# APPLICATION OBJECTS
# ============================================================

$(BUILD_DIR)/main.o: $(MAIN_DEPS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_outputs.o: \
	$(SRC_DIR)/motor_outputs.c \
	$(INC_DIR)/motor_outputs.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_node_state.o: \
	$(SRC_DIR)/motor_node_state.c \
	$(INC_DIR)/motor_node_state.h \
	$(INC_DIR)/motor_outputs.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/command_receiver.o: \
	$(SRC_DIR)/command_receiver.c \
	$(INC_DIR)/command_receiver.h \
	$(INC_DIR)/motor_command_gate.h \
	$(MOTOR_LINK_DIR)/include/motor_link_protocol.h \
	$(PLATFORM_DIR)/include/uart2_link.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/command_watchdog.o: \
	$(SRC_DIR)/command_watchdog.c \
	$(INC_DIR)/command_watchdog.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_command_gate.o: \
	$(SRC_DIR)/motor_command_gate.c \
	$(INC_DIR)/motor_command_gate.h \
	$(INC_DIR)/command_receiver.h \
	$(INC_DIR)/motor_node_state.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_link_protocol.o: \
	$(MOTOR_LINK_DIR)/src/motor_link_protocol.c \
	$(MOTOR_LINK_DIR)/include/motor_link_protocol.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/freestanding_mem.o: \
	$(SRC_DIR)/freestanding_mem.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/status_led.o: \
	$(SRC_DIR)/status_led.c \
	$(INC_DIR)/status_led.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/fault_test.o: \
	$(TEST_DIR)/fault_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


# ============================================================
# STARTUP
# ============================================================

$(BUILD_DIR)/startup_stm32f103c8.o: \
	$(STARTUP_DIR)/startup_stm32f103c8.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


# ============================================================
# PLATFORM OBJECTS
# ============================================================

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


# ============================================================
# LINK
# ============================================================

$(ELF): $(OBJECTS) $(LINKER_SCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -lgcc -o $@


# ============================================================
# OUTPUT CONVERSION
# ============================================================

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


# ============================================================
# INSPECTION
# ============================================================

inspect: all
	$(OBJDUMP) -h $(ELF)
	$(OBJDUMP) -s -j .isr_vector $(ELF)
	$(NM) -n $(ELF) | grep -E \
		'Reset_Handler|Default_Handler|NMI_Handler|HardFault_Handler|SysTick_Handler|USART2_IRQHandler|DMA1_Channel7_IRQHandler|g_fault_record|g_motor_safe_state_requested|g_motor_outputs_safe|motor_outputs_are_safe|g_system_time_status|g_milliseconds|g_uart_diag_status|g_uart_diag_output_ok|uart_diag_init|g_motor_command_uart_status|g_uart2_link_diag|g_latest_received_motor_command|g_command_receiver_stats|command_receiver_init|command_receiver_process|command_receiver_reset_sequence_history|g_command_watchdog_status|g_command_watchdog_timeout_ms|g_command_watchdog_last_age_ms|g_command_watchdog_timeout_event_count|command_watchdog_init|command_watchdog_process|command_watchdog_is_healthy|g_motor_command_gate_status|g_motor_command_gate_diag|motor_command_gate_init|motor_command_gate_reset_session|motor_command_gate_apply|motor_command_gate_disarm_seen|g_motor_node_state|g_motor_node_failsafe_reason|g_motor_node_state_initialized|g_disarmed_enforcement_count|g_armed_safe_enforcement_count|g_failsafe_enforcement_count|g_unexpected_state_count|g_arm_request_count|g_arm_accept_count|g_arm_reject_count|g_disarm_request_count|g_failsafe_entry_count|motor_node_state_init|motor_node_state_process|motor_node_state_request_arm|motor_node_state_request_disarm|motor_node_state_enter_failsafe|motor_node_is_disarmed|motor_node_is_armed|motor_node_is_failsafe'


# ============================================================
# EMBEDDED FAULT-TEST BUILDS
# ============================================================

test-normal:
	$(MAKE) FAULT_TEST_MODE=0 all


test-default:
	$(MAKE) FAULT_TEST_MODE=1 all


test-nmi:
	$(MAKE) FAULT_TEST_MODE=2 all


test-hardfault:
	$(MAKE) FAULT_TEST_MODE=3 all


# ============================================================
# HOST TEST — MOTOR NODE STATE
# ============================================================

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


# ============================================================
# HOST TEST — COMMAND RECEIVER
# ============================================================

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


# ============================================================
# HOST TEST — COMMAND WATCHDOG
# ============================================================

test-command-watchdog-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 \
		-Wall \
		-Wextra \
		-Werror \
		-O0 \
		-g3 \
		-I$(INC_DIR) \
		$(SRC_DIR)/command_watchdog.c \
		$(TEST_DIR)/command_watchdog_test.c \
		-o build/host/command_watchdog_test
	./build/host/command_watchdog_test


# ============================================================
# HOST TEST — MOTOR COMMAND STATE GATE
# ============================================================

test-motor-command-gate-host:
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
		$(SRC_DIR)/motor_command_gate.c \
		$(TEST_DIR)/motor_command_gate_test.c \
		-o build/host/motor_command_gate_test
	./build/host/motor_command_gate_test


# ============================================================
# HOST TEST — SHARED MOTOR LINK PROTOCOL
# ============================================================

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


# ============================================================
# CLEAN
# ============================================================

clean:
	rm -rf build


-include $(DEPS)
