TOOLCHAIN ?= arm-none-eabi-

CC      := $(TOOLCHAIN)gcc
SIZE    := $(TOOLCHAIN)size
OBJCOPY := $(TOOLCHAIN)objcopy

BUILD_DIR := build
TARGET    := $(BUILD_DIR)/qspi_streaming_test

CFLAGS := -DW25Q_STM32H7_BACKEND -DW25Q_RENODE_VIRTUAL_TIME \
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
    $(BUILD_DIR)/qspi_streaming_test.o \
    $(BUILD_DIR)/qspi_nor_validated_storage.o \
    $(BUILD_DIR)/w25q512jv_flash.o \
    $(BUILD_DIR)/stm32h7_w25q_bus.o \
    $(BUILD_DIR)/validated_stream_buffer.o \
    $(BUILD_DIR)/freestanding_mem.o

.PHONY: all clean

all: $(TARGET).elf $(TARGET).bin
	$(SIZE) $(TARGET).elf

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/startup.o: startup.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/qspi_streaming_test.o: qspi_streaming_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


$(BUILD_DIR)/w25q512jv_flash.o: ../../Storage/w25q512jv_flash.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/stm32h7_w25q_bus.o: ../../Storage/stm32h7_w25q_bus.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/qspi_nor_validated_storage.o: ../../Storage/qspi_nor_validated_storage.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/validated_stream_buffer.o: ../../../Storage/validated_stream_buffer.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/freestanding_mem.o: freestanding_mem.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS) stm32h7_renode.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)
