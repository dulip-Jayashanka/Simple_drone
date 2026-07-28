PROJECT := motor-node

CC      := arm-none-eabi-gcc   ## Cross compiler for ARM Cortex-M microcontrollers
OBJCOPY := arm-none-eabi-objcopy ## Tool to convert ELF files to binary format
SIZE    := arm-none-eabi-size ##tool to display the size of the ELF file

BUILD_DIR := build ## Directory to store build artifacts
SRC_DIR   := src ## Directory containing source files
INC_DIR   := include ## Directory containing header files

SOURCES := $(SRC_DIR)/main.c
OBJECTS := $(SOURCES:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

CPU_FLAGS := -mcpu=cortex-m3 -mthumb

CFLAGS := $(CPU_FLAGS) \
          -std=c11 \ ## Use C11 standard
          -Wall \    ## Enable all warnings
          -Wextra \  ## Enable extra warnings
          -Werror \  ## Treat warnings as errors
          -O0 \      ### No optimization for easier debugging
          -g3 \      ### Generate debug informations in elf file 
          -ffreestanding \  ### Indicate that the program is freestanding (no standard library)
          -ffunction-sections \  ### Place each function in its own section exmple  .text.main .text.led_init   
          -fdata-sections \  ### Place each data item in its own section example .data.counter .data.led_state
          -I$(INC_DIR)   ## Include directory for header files

LDFLAGS := $(CPU_FLAGS) \
           -nostartfiles \ ### Do not use standard system startup files
           -Wl,--gc-sections \  ### Remove unused sections from the final binary
           -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map  ### Generate a map file for the final binary

ELF := $(BUILD_DIR)/$(PROJECT).elf
BIN := $(BUILD_DIR)/$(PROJECT).bin

.PHONY: all clean size

all: $(ELF) $(BIN) size

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

size: $(ELF)
	$(SIZE) $<

clean:
	rm -rf $(BUILD_DIR)