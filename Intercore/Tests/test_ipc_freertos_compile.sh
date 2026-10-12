#!/usr/bin/env bash
# Cross-compile the STM32-only IPC backend against the installed FreeRTOS
# kernel headers. This test intentionally does NOT run or link target firmware.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
KERNEL="${FREERTOS_KERNEL_ROOT:-$ROOT/../FreeRTOS-Kernel}"
COMPILER="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"

if ! command -v "$COMPILER" >/dev/null 2>&1; then
    echo "FAIL: arm-none-eabi-gcc is required for the IPC target compile test" >&2
    exit 1
fi
if [[ ! -f "$KERNEL/include/FreeRTOS.h" ]]; then
    echo "FAIL: FreeRTOS-Kernel not found at $KERNEL" >&2
    exit 1
fi

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

# The committed root FreeRTOSConfig.h is intended for the POSIX simulator.
# Adapt a throwaway copy for ARM port header compilation, without modifying
# the host's FreeRTOSConfig.h or pretending to provide a real board config.
sed -E \
    's/^#define[[:space:]]+configSUPPORT_STATIC_ALLOCATION[[:space:]]+0[[:space:]]*$/#define configSUPPORT_STATIC_ALLOCATION 1/' \
    "$ROOT/FreeRTOSConfig.h" > "$TMP_DIR/FreeRTOSConfig.h"

cat >> "$TMP_DIR/FreeRTOSConfig.h" <<'CONFIG'
/* Test-only STM32H745 NVIC settings; production requires board verification. */
#ifndef configPRIO_BITS
#define configPRIO_BITS 4
#endif
#ifndef configLIBRARY_LOWEST_INTERRUPT_PRIORITY
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#endif
#ifndef configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#endif
#ifndef configKERNEL_INTERRUPT_PRIORITY
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#endif
#ifndef configMAX_SYSCALL_INTERRUPT_PRIORITY
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#endif
#ifndef configUSE_STREAM_BUFFERS
#define configUSE_STREAM_BUFFERS 1
#endif
CONFIG

if ! grep -Eq '^#define[[:space:]]+configSUPPORT_STATIC_ALLOCATION[[:space:]]+1' "$TMP_DIR/FreeRTOSConfig.h"; then
    echo "FAIL: temporary ARM FreeRTOSConfig.h did not enable static allocation" >&2
    exit 1
fi

compile_core() {
    local cpu="$1"
    local port_subdir="$2"
    local fpu="$3"
    local port
    port="$(find "$KERNEL/portable/GCC/$port_subdir" -name portmacro.h -print -quit 2>/dev/null || true)"
    if [[ -z "$port" ]]; then
        echo "FAIL: FreeRTOS portmacro.h missing for $cpu ($port_subdir)" >&2
        exit 1
    fi

    echo "[IPC] Compiling $cpu against actual FreeRTOS headers..."
    "$COMPILER" \
        -std=c11 -O2 -Wall -Wextra -Werror \
        -mcpu="$cpu" -mthumb -mfpu="$fpu" -mfloat-abi=hard \
        -DGP_IPC_FREERTOS_BACKEND=1 \
        -I"$TMP_DIR" -I"$KERNEL/include" -I"$(dirname "$port")" \
        -I"$ROOT/Intercore" \
        -c "$ROOT/Intercore/ipc_mailbox_freertos.c" \
        -o "$TMP_DIR/ipc_${cpu}.o"
    echo "PASS: $cpu IPC target cross-compilation"
}

compile_core cortex-m7 ARM_CM7 fpv5-d16
compile_core cortex-m4 ARM_CM4F fpv4-sp-d16
echo "PASS: Both STM32H745 cores compile the target IPC source"
