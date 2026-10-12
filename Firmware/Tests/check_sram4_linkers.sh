#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
NM="${ARM_NONE_EABI_NM:-arm-none-eabi-nm}"
RE="${ARM_NONE_EABI_READELF:-arm-none-eabi-readelf}"
for cmd in "$CC" "$NM" "$RE"; do command -v "$cmd" >/dev/null || exit 1; done
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
for core in CM7 CM4; do
    if [[ "$core" == CM7 ]]; then cpu=cortex-m7; fpu=fpv5-d16
    else cpu=cortex-m4; fpu=fpv4-sp-d16; fi
    "$CC" -std=c11 -O0 -Wall -Wextra -Werror \
        -ffunction-sections -fdata-sections \
        -mcpu="$cpu" -mthumb -mfpu="$fpu" -mfloat-abi=hard \
        -DSTM32H745xx -DUSE_HAL_DRIVER -D"CORE_$core=1" \
        -D"GP_IPC_CORE_M${core#CM}=1" -DGP_IPC_FREERTOS_BACKEND=1 \
        -I"$FW/$core/Core/Inc" -I"$FW/Common/Inc" \
        -I"$ROOT/Intercore" \
        -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include" \
        -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" \
        -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include" \
        -I"$FW/Drivers/CMSIS/Include" \
        -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc" \
        -c "$ROOT/Firmware/Tests/shared_linker_probe.c" -o "$OUT/$core.o"
    for mode in FLASH RAM; do
        echo "[Stage 2C] $core $mode linker..."
        elf="$OUT/$core-$mode.elf"
        "$CC" -mcpu="$cpu" -mthumb -mfpu="$fpu" -mfloat-abi=hard \
            -nostdlib -Wl,--gc-sections \
            "-Wl,-T,$FW/$core/STM32H745IITX_$mode.ld" \
            "-Wl,-Map,$OUT/$core-$mode.map" "$OUT/$core.o" -o "$elf"
        for pair in "gp_stage2_linker_ipc 38000000 .gp_shared_ipc 000008a0" \
                    "gp_stage2_linker_trajectory 38001000 .gp_shared_trajectory 00003040"; do
            read -r symbol addr section size <<< "$pair"
            actual="$("$NM" --defined-only "$elf" | awk -v n="$symbol" '$3==n {print $1}')"
            [[ "$actual" == "$addr" ]] || { echo "FAIL: $symbol at $actual, expected $addr"; exit 1; }
            "$RE" -W -S "$elf" | grep -Eq "\] $section[[:space:]]+NOBITS[[:space:]]+$addr" ||
                { echo "FAIL: $section not NOBITS at $addr"; exit 1; }
            actual_size="$("$NM" -S --defined-only "$elf" | awk -v n="$symbol" '$4==n {print $2}')"
            [[ "$actual_size" == "$size" ]] || { echo "FAIL: $symbol size $actual_size != $size"; exit 1; }
            echo "PASS: $symbol at 0x$addr, $size bytes, NOLOAD"
        done
    done
done
echo "PASS: Stage 2C synthetic linker placement on both cores and both link modes"
echo "NOTE: full CubeIDE firmware builds and hardware MPU remain unverified."
