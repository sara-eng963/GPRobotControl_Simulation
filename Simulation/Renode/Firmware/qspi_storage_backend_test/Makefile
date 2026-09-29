TOOLCHAIN ?= arm-none-eabi-

CC      := $(TOOLCHAIN)gcc
SIZE    := $(TOOLCHAIN)size
OBJCOPY := $(TOOLCHAIN)objcopy

BUILD_DIR := build
TARGET    := $(BUILD_DIR)/qspi_storage_backend_test

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

OBJECTS := \
    $(BUILD_DIR)/startup.o \
    $(BUILD_DIR)/qspi_storage_backend_test.o \
    $(BUILD_DIR)/qspi_nor_validated_storage.o

.PHONY: all clean

all: $(TARGET).elf $(TARGET).bin
	$(SIZE) $(TARGET).elf

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/startup.o: startup.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/qspi_storage_backend_test.o: qspi_storage_backend_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/qspi_nor_validated_storage.o: ../../Storage/qspi_nor_validated_storage.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS) stm32h7_renode.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)
