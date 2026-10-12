#!/usr/bin/env bash
# Stage 2E-1: compile the *actual* M7-owned SRAM4 object definitions.
# This is an ARM ELF link test, not a running two-core firmware or hardware test.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
NM="${ARM_NONE_EABI_NM:-arm-none-eabi-nm}"
RE="${ARM_NONE_EABI_READELF:-arm-none-eabi-readelf}"
for c in "$CC" "$NM" "$RE"; do
    command -v "$c" >/dev/null 2>&1 || { echo "FAIL: missing $c" >&2; exit 1; }
done
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
GCC_FLAGS=(-std=c11 -O0 -Wall -Wextra -Werror -ffunction-sections
    -fdata-sections -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
    -DSTM32H745xx -DUSE_HAL_DRIVER -DCORE_CM7=1
    -I"$FW/CM7/Core/Inc" -I"$ROOT/Intercore"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    -I"$FW/Drivers/CMSIS/Include"
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc")

"$CC" "${GCC_FLAGS[@]}" -c \
    "$FW/CM7/Core/Src/intercore_shared_memory.c" -o "$OUT/objects.o"
printf 'void Reset_Handler(void) { for (;;) { __asm volatile ("nop"); } }\n' > "$OUT/reset.c"
"$CC" "${GCC_FLAGS[@]}" -c "$OUT/reset.c" -o "$OUT/reset.o"

for mode in FLASH RAM; do
    echo "[Stage 2E-1] Production M7 objects with $mode linker..."
    elf="$OUT/objects-$mode.elf"
    "$CC" -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
        -nostdlib -Wl,--gc-sections -Wl,-T,"$FW/CM7/STM32H745IITX_$mode.ld" \
        "$OUT/objects.o" "$OUT/reset.o" -o "$elf"
    for item in "intercore_ipc_shared 38000000 000008a0 .gp_shared_ipc" \
                "intercore_trajectory_shared 38001000 00003040 .gp_shared_trajectory"; do
        read -r symbol address size section <<< "$item"
        line="$("$NM" -S --defined-only "$elf" | awk -v n="$symbol" '$4==n{print $1, $2}')"
        [[ "$line" == "$address $size" ]] || {
            echo "FAIL: $symbol expected $address $size, got [$line]" >&2; exit 1;
        }
        "$RE" -W -S "$elf" | grep -Eq "\] $section[[:space:]]+NOBITS[[:space:]]+$address" || {
            echo "FAIL: $symbol section $section not NOBITS at $address" >&2; exit 1;
        }
        echo "PASS: $symbol at 0x$address ($size bytes, NOLOAD)"
    done
done
echo "PASS: production shared allocations at correct M7 SRAM4 addresses"
echo "NOTE: full firmware builds and two-core runtime behavior still pending."
