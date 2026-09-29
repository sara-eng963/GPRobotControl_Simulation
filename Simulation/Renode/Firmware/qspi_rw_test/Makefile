TOOLCHAIN ?= arm-none-eabi-

CC      := $(TOOLCHAIN)gcc
SIZE    := $(TOOLCHAIN)size
OBJCOPY := $(TOOLCHAIN)objcopy

BUILD_DIR := build
TARGET    := $(BUILD_DIR)/qspi_rw_test

SOURCES := startup.c qspi_rw_test.c
OBJECTS := $(addprefix $(BUILD_DIR)/,$(SOURCES:.c=.o))

CFLAGS := \
    -mcpu=cortex-m7 \
    -mthumb \
    -ffreestanding \
    -fno-builtin \
    -ffunction-sections \
    -fdata-sections \
    -Wall \
    -Wextra \
    -Werror \
    -Os

LDFLAGS := \
    -mcpu=cortex-m7 \
    -mthumb \
    -nostdlib \
    -nostartfiles \
    -T stm32h7_renode.ld \
    -Wl,--gc-sections \
    -Wl,-Map=$(TARGET).map

.PHONY: all clean

all: $(TARGET).elf $(TARGET).bin
	$(SIZE) $(TARGET).elf

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS) stm32h7_renode.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)
