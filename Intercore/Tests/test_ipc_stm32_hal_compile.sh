#!/usr/bin/env bash
# Cross-compile the real STM32H745 HSEM HAL adapter against STM32CubeH7.
# No mock HAL headers, target link, hardware execution or GitHub downloads.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CUBE="${STM32CUBEH7_ROOT:-$ROOT/../STM32CubeH7}"
KERNEL="${FREERTOS_KERNEL_ROOT:-$ROOT/../FreeRTOS-Kernel}"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"

if ! command -v "$CC" >/dev/null 2>&1; then
    echo "FAIL: ARM compiler not found: $CC" >&2
    exit 1
fi
if [[ ! -f "$KERNEL/include/FreeRTOS.h" ]]; then
    echo "FAIL: FreeRTOS-Kernel missing at $KERNEL" >&2
    exit 1
fi

CORE_HEADERS="$CUBE/Drivers/CMSIS/Core/Include"
DEVICE_HEADERS="$CUBE/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
HAL_HEADERS="$CUBE/Drivers/STM32H7xx_HAL_Driver/Inc"
HAL_HSEM="$CUBE/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c"
EXAMPLE="$CUBE/Projects/STM32H745I-DISCO/Examples/HSEM/HSEM_CoreNotification"

for file in \
    "$CORE_HEADERS/core_cm7.h" \
    "$DEVICE_HEADERS/stm32h745xx.h" \
    "$HAL_HEADERS/stm32h7xx_hal.h" \
    "$HAL_HEADERS/stm32h7xx_hal_hsem.h" \
    "$HAL_HSEM" \
    "$EXAMPLE/CM7/Inc/stm32h7xx_hal_conf.h" \
    "$EXAMPLE/CM4/Inc/stm32h7xx_hal_conf.h"; do
    if [[ ! -f "$file" ]]; then
        echo "FAIL: missing ST source: $file" >&2
        echo "Install the official STM32CubeH7 package with its submodules," >&2
        echo "or set STM32CUBEH7_ROOT to the package root." >&2
        exit 1
    fi
done

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# Reuse host config only as a compile-check starting point. The actual
# CM7/CM4 firmware MUST have independent, validated FreeRTOS configurations.
sed -E \
    's/^#define[[:space:]]+configSUPPORT_STATIC_ALLOCATION[[:space:]]+0[[:space:]]*$/#define configSUPPORT_STATIC_ALLOCATION 1/' \
    "$ROOT/FreeRTOSConfig.h" > "$TMP/FreeRTOSConfig.h"
cat >> "$TMP/FreeRTOSConfig.h" <<'CONFIG'
/* ARM port settings for compilation only, not a deployment configuration. */
#define configPRIO_BITS 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY (15 << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (5 << (8 - configPRIO_BITS))
#define configUSE_STREAM_BUFFERS 1
CONFIG

for spec in "M7 cortex-m7 ARM_CM7 fpv5-d16 CM7" \
            "M4 cortex-m4 ARM_CM4F fpv4-sp-d16 CM4"; do
    read -r core cpu port_dir fpu st_core <<< "$spec"
    port="$(find "$KERNEL/portable/GCC/$port_dir" -name portmacro.h -print -quit 2>/dev/null || true)"
    if [[ -z "$port" ]]; then
        echo "FAIL: FreeRTOS $port_dir port not found" >&2
        exit 1
    fi

    echo "[IPC HAL] Compiling $core with official STM32CubeH7 headers..."
    common=(
        -std=c11 -mcpu="$cpu" -mthumb
        -mfpu="$fpu" -mfloat-abi=hard
        -DSTM32H745xx -DUSE_HAL_DRIVER
        "-DCORE_$st_core=1"
        "-DGP_IPC_CORE_$core=1"
        -DGP_IPC_FREERTOS_BACKEND=1
        -I"$TMP"
        -I"$KERNEL/include" -I"$(dirname "$port")"
        -I"$ROOT/Intercore"
        -I"$EXAMPLE/$st_core/Inc"
        -I"$HAL_HEADERS" -I"$HAL_HEADERS/Legacy"
        -I"$DEVICE_HEADERS" -I"$CORE_HEADERS"
    )

    "$CC" "${common[@]}" -Wall -Wextra -Werror \
        -c "$ROOT/Intercore/ipc_stm32h745_hsem.c" \
        -o "$TMP/gp_hsem_$core.o"

    # Compile ST's real HSEM implementation in its own translation unit,
    # rather than merely accepting mocked HAL symbols.
    "$CC" "${common[@]}" -Wall -Wextra \
        -c "$HAL_HSEM" -o "$TMP/st_hsem_$core.o"

    echo "PASS: $core adapter + official HAL HSEM driver compile"
done

echo "PASS: STM32H745 dual-core real-HAL compilation gate"
