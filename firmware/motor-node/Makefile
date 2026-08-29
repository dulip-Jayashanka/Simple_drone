# Motor-node Makefile
#
# FMCOM development state:
#
#   Phase 6.1
#       USART2 motor-command receiver + protocol validation
#
#   Phase 6.2 / 6.2.5
#       valid-command watchdog
#       DISARMED / ARMED / FAILSAFE
#       requested-state safety gate
#       DISARM-before-ARM session interlock
#
#   Phase 6.3
#       TIM3 four-channel ESC PWM
#       final 0..1000 command -> electrical pulse mapping
#       separate hard-safe GPIO LOW and normal ESC-safe PWM policies
#       DISARMED / FAILSAFE -> safe pulse
#       ARMED + healthy validated command -> M1..M4 PWM
#       isolated raw-PWM and actuator-mapping bench modes
#
# -----------------------------------------------------------------
# Normal pre-PWM/default safe firmware
# -----------------------------------------------------------------
#
#   make
#
# -----------------------------------------------------------------
# Phase 6.3 raw PWM scope/logic-analyzer test
# -----------------------------------------------------------------
#
#   make \
#       MOTOR_PWM_ENABLE=1 \
#       MOTOR_PWM_TEST_MODE=1 \
#       MOTOR_PWM_TEST_M1_US=1000 \
#       MOTOR_PWM_TEST_M2_US=1000 \
#       MOTOR_PWM_TEST_M3_US=1000 \
#       MOTOR_PWM_TEST_M4_US=1000
#
# -----------------------------------------------------------------
# Phase 6.3 actuator conversion test
# -----------------------------------------------------------------
#
#   make \
#       MOTOR_PWM_ENABLE=1 \
#       MOTOR_ACTUATOR_TEST_MODE=1 \
#       MOTOR_ACTUATOR_TEST_M1=0 \
#       MOTOR_ACTUATOR_TEST_M2=250 \
#       MOTOR_ACTUATOR_TEST_M3=500 \
#       MOTOR_ACTUATOR_TEST_M4=750
#
# -----------------------------------------------------------------
# Phase 6.3 complete runtime path
# -----------------------------------------------------------------
#
#   make \
#       MOTOR_NODE_LINK_ENABLE=1 \
#       MOTOR_COMMAND_WATCHDOG_ENABLE=1 \
#       MOTOR_PWM_ENABLE=1
#
# Add MOTOR_NODE_LINK_UART_DEBUG=1 when the rate-limited USART1
# validated-command observer is useful.
#


PROJECT := motor-node

FAULT_TEST_MODE ?= 0


# ============================================================
# FMCOM MOTOR COMMAND LINK
# ============================================================

MOTOR_NODE_LINK_ENABLE ?= 0
MOTOR_NODE_LINK_BAUD ?= 230400

MOTOR_NODE_LINK_UART_DEBUG ?= 0
MOTOR_NODE_LINK_UART_RATE_HZ ?= 10


# ============================================================
# COMMAND WATCHDOG
# ============================================================

MOTOR_COMMAND_WATCHDOG_ENABLE ?= 0
MOTOR_COMMAND_TIMEOUT_MS ?= 20


# ============================================================
# PHASE 6.3 — TIM3 PWM / ESC ELECTRICAL CONFIGURATION
# ============================================================
#
# These are initial BENCH settings. The installed SimonK ESC range
# must be physically verified/calibrated before flight use.

MOTOR_PWM_ENABLE ?= 0

MOTOR_ESC_PWM_HZ ?= 400
MOTOR_ESC_SAFE_US ?= 1000
MOTOR_ESC_MIN_US ?= 1000
MOTOR_ESC_MAX_US ?= 2000


# Raw TIM3 output owner for Tests 1-4.
MOTOR_PWM_TEST_MODE ?= 0

MOTOR_PWM_TEST_M1_US ?= 1000
MOTOR_PWM_TEST_M2_US ?= 1000
MOTOR_PWM_TEST_M3_US ?= 1000
MOTOR_PWM_TEST_M4_US ?= 1000


# Command-domain -> PWM owner for Test 5.
MOTOR_ACTUATOR_TEST_MODE ?= 0

MOTOR_ACTUATOR_TEST_M1 ?= 0
MOTOR_ACTUATOR_TEST_M2 ?= 0
MOTOR_ACTUATOR_TEST_M3 ?= 0
MOTOR_ACTUATOR_TEST_M4 ?= 0


# ============================================================
# BUILD-FLAG VALIDATION / OUTPUT OWNERSHIP
# ============================================================

BOOLEAN_BUILD_VALUES := \
	$(MOTOR_NODE_LINK_ENABLE) \
	$(MOTOR_NODE_LINK_UART_DEBUG) \
	$(MOTOR_COMMAND_WATCHDOG_ENABLE) \
	$(MOTOR_PWM_ENABLE) \
	$(MOTOR_PWM_TEST_MODE) \
	$(MOTOR_ACTUATOR_TEST_MODE)


ifneq ($(filter-out 0 1,$(BOOLEAN_BUILD_VALUES)),)
$(error All motor-node enable/test flags must be 0 or 1)
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


ifeq ($(MOTOR_PWM_TEST_MODE),1)
ifneq ($(MOTOR_PWM_ENABLE),1)
$(error MOTOR_PWM_TEST_MODE requires MOTOR_PWM_ENABLE=1)
endif
endif


ifeq ($(MOTOR_ACTUATOR_TEST_MODE),1)
ifneq ($(MOTOR_PWM_ENABLE),1)
$(error MOTOR_ACTUATOR_TEST_MODE requires MOTOR_PWM_ENABLE=1)
endif
endif


ifeq ($(MOTOR_PWM_TEST_MODE)$(MOTOR_ACTUATOR_TEST_MODE),11)
$(error MOTOR_PWM_TEST_MODE and MOTOR_ACTUATOR_TEST_MODE are mutually exclusive)
endif


# Isolated tests are the sole owner of TIM3/CCRx. The command link and
# watchdog must not compete with them.
ifneq ($(filter 1,$(MOTOR_PWM_TEST_MODE) $(MOTOR_ACTUATOR_TEST_MODE)),)
ifneq ($(MOTOR_NODE_LINK_ENABLE),0)
$(error Phase 6.3 isolated output tests require MOTOR_NODE_LINK_ENABLE=0)
endif
ifneq ($(MOTOR_COMMAND_WATCHDOG_ENABLE),0)
$(error Phase 6.3 isolated output tests require MOTOR_COMMAND_WATCHDOG_ENABLE=0)
endif
endif


# Active normal PWM is allowed only through the complete local safety
# chain. There is deliberately no "PWM without watchdog" runtime mode.
ifeq ($(MOTOR_PWM_ENABLE),1)
ifeq ($(MOTOR_PWM_TEST_MODE)$(MOTOR_ACTUATOR_TEST_MODE),00)
ifneq ($(MOTOR_NODE_LINK_ENABLE),1)
$(error Normal MOTOR_PWM_ENABLE=1 requires MOTOR_NODE_LINK_ENABLE=1)
endif
ifneq ($(MOTOR_COMMAND_WATCHDOG_ENABLE),1)
$(error Normal MOTOR_PWM_ENABLE=1 requires MOTOR_COMMAND_WATCHDOG_ENABLE=1)
endif
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
# TOOLS / DIRECTORIES
# ============================================================

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
NM      := arm-none-eabi-nm

HOST_CC ?= cc

SRC_DIR := src
INC_DIR := include
TEST_DIR := tests
STARTUP_DIR := startup
PLATFORM_DIR := ../../platform/stm32f103c8
MOTOR_LINK_DIR := ../motor-link
LINKER_SCRIPT := linker/stm32f103c8.ld


# ============================================================
# OBJECTS
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


ifeq ($(MOTOR_NODE_LINK_ENABLE),1)
OBJECTS += \
	$(BUILD_DIR)/command_receiver.o \
	$(BUILD_DIR)/motor_link_protocol.o \
	$(BUILD_DIR)/uart2_link.o
endif


ifeq ($(MOTOR_COMMAND_WATCHDOG_ENABLE),1)
OBJECTS += \
	$(BUILD_DIR)/command_watchdog.o \
	$(BUILD_DIR)/motor_command_gate.o
endif


ifeq ($(MOTOR_PWM_ENABLE),1)
OBJECTS += \
	$(BUILD_DIR)/motor_pwm.o \
	$(BUILD_DIR)/motor_actuator.o
endif


DEPS := $(OBJECTS:.o=.d)
CONFIG_STAMP := $(BUILD_DIR)/.build-config


# ============================================================
# COMPILER / LINKER FLAGS
# ============================================================

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
	-DMOTOR_PWM_ENABLE=$(MOTOR_PWM_ENABLE) \
	-DMOTOR_ESC_PWM_HZ=$(MOTOR_ESC_PWM_HZ) \
	-DMOTOR_ESC_SAFE_US=$(MOTOR_ESC_SAFE_US) \
	-DMOTOR_ESC_MIN_US=$(MOTOR_ESC_MIN_US) \
	-DMOTOR_ESC_MAX_US=$(MOTOR_ESC_MAX_US) \
	-DMOTOR_PWM_TEST_MODE=$(MOTOR_PWM_TEST_MODE) \
	-DMOTOR_PWM_TEST_M1_US=$(MOTOR_PWM_TEST_M1_US) \
	-DMOTOR_PWM_TEST_M2_US=$(MOTOR_PWM_TEST_M2_US) \
	-DMOTOR_PWM_TEST_M3_US=$(MOTOR_PWM_TEST_M3_US) \
	-DMOTOR_PWM_TEST_M4_US=$(MOTOR_PWM_TEST_M4_US) \
	-DMOTOR_ACTUATOR_TEST_MODE=$(MOTOR_ACTUATOR_TEST_MODE) \
	-DMOTOR_ACTUATOR_TEST_M1=$(MOTOR_ACTUATOR_TEST_M1) \
	-DMOTOR_ACTUATOR_TEST_M2=$(MOTOR_ACTUATOR_TEST_M2) \
	-DMOTOR_ACTUATOR_TEST_M3=$(MOTOR_ACTUATOR_TEST_M3) \
	-DMOTOR_ACTUATOR_TEST_M4=$(MOTOR_ACTUATOR_TEST_M4) \
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


# ============================================================
# OUTPUT FILES
# ============================================================

ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin
HEX := $(BUILD_DIR)/$(PROJECT).hex
LIST := $(BUILD_DIR)/$(PROJECT).list
SYMBOLS := $(BUILD_DIR)/$(PROJECT).symbols


# ============================================================
# MAIN DEPENDENCIES
# ============================================================

MAIN_DEPS := $(SRC_DIR)/main.c

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

ifeq ($(MOTOR_PWM_ENABLE),1)
MAIN_DEPS += \
	$(INC_DIR)/motor_pwm.h \
	$(INC_DIR)/motor_actuator.h
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
	test-state-pwm-host \
	test-command-receiver-host \
	test-command-watchdog-host \
	test-motor-command-gate-host \
	test-motor-link-protocol-host \
	test-motor-pwm-host \
	test-motor-actuator-host \
	test-phase63-host \
	FORCE


all: $(ELF) $(BIN) $(HEX) $(LIST) $(SYMBOLS) size


$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)


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
		'MOTOR_PWM_ENABLE=$(MOTOR_PWM_ENABLE)' \
		'MOTOR_ESC_PWM_HZ=$(MOTOR_ESC_PWM_HZ)' \
		'MOTOR_ESC_SAFE_US=$(MOTOR_ESC_SAFE_US)' \
		'MOTOR_ESC_MIN_US=$(MOTOR_ESC_MIN_US)' \
		'MOTOR_ESC_MAX_US=$(MOTOR_ESC_MAX_US)' \
		'MOTOR_PWM_TEST_MODE=$(MOTOR_PWM_TEST_MODE)' \
		'MOTOR_PWM_TEST_M1_US=$(MOTOR_PWM_TEST_M1_US)' \
		'MOTOR_PWM_TEST_M2_US=$(MOTOR_PWM_TEST_M2_US)' \
		'MOTOR_PWM_TEST_M3_US=$(MOTOR_PWM_TEST_M3_US)' \
		'MOTOR_PWM_TEST_M4_US=$(MOTOR_PWM_TEST_M4_US)' \
		'MOTOR_ACTUATOR_TEST_MODE=$(MOTOR_ACTUATOR_TEST_MODE)' \
		'MOTOR_ACTUATOR_TEST_M1=$(MOTOR_ACTUATOR_TEST_M1)' \
		'MOTOR_ACTUATOR_TEST_M2=$(MOTOR_ACTUATOR_TEST_M2)' \
		'MOTOR_ACTUATOR_TEST_M3=$(MOTOR_ACTUATOR_TEST_M3)' \
		'MOTOR_ACTUATOR_TEST_M4=$(MOTOR_ACTUATOR_TEST_M4)' \
		> $@.tmp
	@if ! cmp -s $@.tmp $@; then mv $@.tmp $@; else rm -f $@.tmp; fi


$(OBJECTS): $(CONFIG_STAMP)


# ============================================================
# APPLICATION OBJECT RULES
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
	$(INC_DIR)/motor_outputs.h \
	$(INC_DIR)/motor_actuator.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/command_receiver.o: \
	$(SRC_DIR)/command_receiver.c \
	$(INC_DIR)/command_receiver.h \
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


$(BUILD_DIR)/motor_pwm.o: \
	$(SRC_DIR)/motor_pwm.c \
	$(INC_DIR)/motor_pwm.h \
	$(INC_DIR)/motor_outputs.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/motor_actuator.o: \
	$(SRC_DIR)/motor_actuator.c \
	$(INC_DIR)/motor_actuator.h \
	$(INC_DIR)/motor_pwm.h \
	$(MOTOR_LINK_DIR)/include/motor_link_protocol.h | $(BUILD_DIR)
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
# STARTUP / PLATFORM OBJECT RULES
# ============================================================

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


# ============================================================
# LINK / OUTPUT CONVERSION
# ============================================================

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


# ============================================================
# INSPECTION
# ============================================================

inspect: all
	$(OBJDUMP) -h $(ELF)
	$(OBJDUMP) -s -j .isr_vector $(ELF)
	$(NM) -n $(ELF) | grep -E \
		'Reset_Handler|HardFault_Handler|SysTick_Handler|USART2_IRQHandler|DMA1_Channel7_IRQHandler|g_fault_record|g_motor_outputs_safe|g_motor_command_uart_status|g_latest_received_motor_command|g_command_receiver_stats|g_command_watchdog_status|g_motor_command_gate_status|g_motor_command_gate_diag|g_motor_node_state|g_motor_node_failsafe_reason|g_motor_pwm_timer_clock_hz|g_motor_pwm_status|g_motor_pwm_diag|motor_pwm_init|motor_pwm_set_us|motor_pwm_set_safe|motor_pwm_hard_disable|g_motor_actuator_status|g_motor_actuator_diag|motor_actuator_apply|motor_actuator_set_safe|motor_node_state_request_arm|motor_node_state_request_disarm|motor_node_state_enter_failsafe'


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
# HOST TESTS
# ============================================================

test-state-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-I$(INC_DIR) \
		$(SRC_DIR)/motor_node_state.c \
		$(TEST_DIR)/motor_node_state_test.c \
		-o build/host/motor_node_state_test
	./build/host/motor_node_state_test


test-state-pwm-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-DMOTOR_PWM_ENABLE=1 \
		-I$(INC_DIR) \
		-I$(MOTOR_LINK_DIR)/include \
		$(SRC_DIR)/motor_node_state.c \
		$(TEST_DIR)/motor_node_state_pwm_test.c \
		-o build/host/motor_node_state_pwm_test
	./build/host/motor_node_state_pwm_test


test-command-receiver-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-I$(INC_DIR) \
		-I$(MOTOR_LINK_DIR)/include \
		-I$(PLATFORM_DIR)/include \
		$(TEST_DIR)/command_receiver_test.c \
		$(SRC_DIR)/command_receiver.c \
		$(MOTOR_LINK_DIR)/src/motor_link_protocol.c \
		-o build/host/command_receiver_test
	./build/host/command_receiver_test


test-command-watchdog-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-I$(INC_DIR) \
		$(SRC_DIR)/command_watchdog.c \
		$(TEST_DIR)/command_watchdog_test.c \
		-o build/host/command_watchdog_test
	./build/host/command_watchdog_test


test-motor-command-gate-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-I$(INC_DIR) \
		-I$(MOTOR_LINK_DIR)/include \
		$(SRC_DIR)/motor_command_gate.c \
		$(TEST_DIR)/motor_command_gate_test.c \
		-o build/host/motor_command_gate_test
	./build/host/motor_command_gate_test


test-motor-link-protocol-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-I$(MOTOR_LINK_DIR)/include \
		$(MOTOR_LINK_DIR)/tests/motor_link_protocol_test.c \
		$(MOTOR_LINK_DIR)/src/motor_link_protocol.c \
		-o build/host/motor_link_protocol_test
	./build/host/motor_link_protocol_test


test-motor-pwm-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-I$(INC_DIR) \
		$(SRC_DIR)/motor_pwm.c \
		$(TEST_DIR)/motor_pwm_test.c \
		-o build/host/motor_pwm_test
	./build/host/motor_pwm_test


test-motor-actuator-host:
	mkdir -p build/host
	$(HOST_CC) \
		-std=c11 -Wall -Wextra -Werror -O0 -g3 \
		-I$(INC_DIR) \
		-I$(MOTOR_LINK_DIR)/include \
		$(SRC_DIR)/motor_actuator.c \
		$(TEST_DIR)/motor_actuator_test.c \
		-o build/host/motor_actuator_test
	./build/host/motor_actuator_test


# Dependency-based Phase 6.3 host regression set.
test-phase63-host: \
	test-motor-pwm-host \
	test-motor-actuator-host \
	test-state-pwm-host \
	test-command-receiver-host \
	test-command-watchdog-host \
	test-motor-command-gate-host \
	test-motor-link-protocol-host


clean:
	rm -rf build


-include $(DEPS)
