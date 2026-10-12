#!/usr/bin/env bash
# Build and run the real FreeRTOS message-buffer backend on FreeRTOS POSIX.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
KERNEL="${FREERTOS_KERNEL_ROOT:-$ROOT/../FreeRTOS-Kernel}"
BUILD="${GP_IPC_RUNTIME_BUILD_DIR:-$ROOT/build-intercore-ipc-posix}"
if [[ ! -f "$KERNEL/include/FreeRTOS.h" ]]; then
    echo "FAIL: FreeRTOS kernel not found at $KERNEL" >&2
    exit 1
fi
cmake -S "$ROOT/Intercore/Tests/ipc_freertos_runtime" \
    -B "$BUILD" -DFREERTOS_KERNEL_ROOT="$KERNEL" \
    -DCMAKE_BUILD_TYPE=Debug
cmake --build "$BUILD" --target ipc_freertos_runtime_test -j 2

# POSIX scheduler deadlocks should never hang CI or the user terminal.
timeout 35s "$BUILD/ipc_freertos_runtime_test"
