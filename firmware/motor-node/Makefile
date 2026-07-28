PROJECT := motor-node

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

BUILD_DIR   := build
SRC_DIR     := src
INC_DIR     := include
PLATFORM_DIR := ../../platform/stm32f103c8

APP_SOURCES := $(SRC_DIR)/main.c
APP_OBJECTS := $(APP_SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

STARTUP_SOURCE := startup/startup_stm32f103c8.c
CLOCK_SOURCE   := $(PLATFORM_DIR)/src/system_clock.c
CLOCK_HEADER   := $(PLATFORM_DIR)/include/system_clock.h

OBJECTS := $(APP_OBJECTS) \
           $(BUILD_DIR)/startup_stm32f103c8.o \
           $(BUILD_DIR)/system_clock.o

CPU_FLAGS := -mcpu=cortex-m3 -mthumb
LINKER_SCRIPT := linker/stm32f103c8.ld
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
          -I$(INC_DIR) \
          -I$(PLATFORM_DIR)/include

LDFLAGS := $(CPU_FLAGS) \
           -nostartfiles \
           -T$(LINKER_SCRIPT) \
           -Wl,--gc-sections \
           -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map

ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin

.PHONY: all clean size

all: $(ELF) $(BIN) size

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup_stm32f103c8.o: $(STARTUP_SOURCE) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/system_clock.o: $(CLOCK_SOURCE) $(CLOCK_HEADER) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $(CLOCK_SOURCE) -o $@

$(ELF): $(OBJECTS) $(LINKER_SCRIPT)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

size: $(ELF)
	$(SIZE) $<

clean:
	rm -rf $(BUILD_DIR)
