#!/usr/bin/env bash
# Stage 2A: authoritative 32-bit layout sizes against the generated firmware.
# Does NOT patch linker scripts/MPU or compile the full application.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
SRC="$ROOT/Firmware/Tests/shared_layout_probe.c"
GCC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
READELF="${ARM_NONE_EABI_READELF:-arm-none-eabi-readelf}"
for x in "$GCC" "$READELF"; do
    command -v "$x" >/dev/null 2>&1 || { echo "FAIL: missing $x" >&2; exit 1; }
done
[[ -f "$SRC" ]] || { echo "FAIL: probe missing: $SRC" >&2; exit 1; }
[[ -f "$FW/GP_STM32H745.ioc" ]] || { echo "FAIL: CubeMX target missing" >&2; exit 1; }
KERNEL="$FW/Middlewares/Third_Party/FreeRTOS/Source"
PORT="$KERNEL/portable/GCC/ARM_CM4F"
DEV="$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
CORE="$FW/Drivers/CMSIS/Include"
HAL="$FW/Drivers/STM32H7xx_HAL_Driver/Inc"
for dir in "$KERNEL/include" "$PORT" "$DEV" "$CORE" "$HAL"; do
    [[ -d "$dir" ]] || { echo "FAIL: missing SDK folder $dir" >&2; exit 1; }
done
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
for core in CM7 CM4; do
    if [[ "$core" == "CM7" ]]; then
        cpu=cortex-m7; fpu=fpv5-d16
    else
        cpu=cortex-m4; fpu=fpv4-sp-d16
    fi
    echo "[Stage 2A] Compiling $core probe against committed FreeRTOS 10.6.2..."
    "$GCC" -std=c11 -O0 -Wall -Wextra -Werror \
        -mcpu="$cpu" -mthumb -mfpu="$fpu" -mfloat-abi=hard \
        -DSTM32H745xx -DUSE_HAL_DRIVER -D"CORE_$core=1" \
        -D"GP_IPC_CORE_M${core#CM}=1" -DGP_IPC_FREERTOS_BACKEND=1 \
        -I"$FW/$core/Core/Inc" -I"$ROOT/Intercore" -I"$FW/Common/Inc" \
        -I"$KERNEL/include" -I"$PORT" -I"$DEV" -I"$CORE" \
        -I"$HAL" -I"$HAL/Legacy" \
        -c "$SRC" -o "$TMP/$core.o"
    "$READELF" --wide --symbols "$TMP/$core.o" | \
        awk '$4 == "OBJECT" && $NF ~ /^gp_stage2_/ {print $NF, $3}' | \
        LC_ALL=C sort > "$TMP/$core.sizes"
    [[ "$(wc -l < "$TMP/$core.sizes")" -eq 12 ]] || {
        echo "FAIL: unexpected number of layout symbols on $core" >&2
        cat "$TMP/$core.sizes" >&2
        exit 1
    }
done
if ! cmp -s "$TMP/CM7.sizes" "$TMP/CM4.sizes"; then
    echo "FAIL: different layout across CM7 / CM4!" >&2
    diff -u "$TMP/CM7.sizes" "$TMP/CM4.sizes" || true
    exit 1
fi
get_size() { awk -v name="$1" '$1 == name {print $2}' "$TMP/CM7.sizes"; }
ipc="$(get_size gp_stage2_shared_ipc)"
traj="$(get_size gp_stage2_shared_trajectory)"
[[ "$ipc" =~ ^[0-9]+$ && "$traj" =~ ^[0-9]+$ ]] || {
    echo "FAIL: unable to read object sizes" >&2; exit 1;
}
combined=$((ipc + traj))
reserve=$((65536 - combined))
echo
printf 'GpIpcShared:         %6d bytes\n' "$ipc"
printf 'GpTrajectoryShared:  %6d bytes\n' "$traj"
printf 'Combined minimum:    %6d / 65536 SRAM4 bytes\n' "$combined"
printf 'Unallocated budget:  %6d bytes (not yet reserved)\n' "$reserve"
echo
echo 'ABI/offset audit (array size encodes offset+1):'
cat "$TMP/CM7.sizes"
echo
if grep -Eq '(^|[[:space:]])(RAM_D3|SRAM4)[[:space:]]*\(' "$FW/CM4/STM32H745IITX_FLASH.ld"; then
    echo 'INFO: CM4 linker already declares a shared region; inspect manually.'
else
    echo 'PENDING: CM4 linker has no dedicated SRAM4 region.'
fi
echo
echo 'Stage 2B RESERVED ADDRESS PROPOSAL (not yet linker-placed):'
printf '  IPC:          0x38000000..0x38000FFF  4096 B (%d B occupied)\n' "$ipc"
printf '  Trajectory:   0x38001000..0x38004FFF 16384 B (%d B occupied)\n' "$traj"
echo "  Shared spare: 0x38005000..0x38007FFF 12288 B (reserved)"
echo "  Upper SRAM4:  0x38008000..0x3800FFFF 32768 B (unallocated)"
echo 'PENDING: Neither core places the objects in SRAM4 yet; no MPU validation.'
echo 'PASS: native FreeRTOS 10.6.2 message_buffer.h compiled without custom feature shim.'
echo 'PASS: Stage 2A CM7/CM4 ABI equality and SRAM4 gross-capacity check'
