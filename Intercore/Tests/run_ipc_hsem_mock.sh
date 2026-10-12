#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
KERNEL="${FREERTOS_KERNEL_ROOT:-$ROOT/../FreeRTOS-Kernel}"
if [[ ! -f "$KERNEL/include/task.h" ]]; then
    echo "FAIL: real FreeRTOS kernel headers not found at $KERNEL" >&2
    exit 1
fi
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
for core in M7 M4; do
    echo "[IPC HSEM] Testing $core with real FreeRTOS headers and mock HAL..."
    cc -std=c11 -Wall -Wextra -Werror -DGP_IPC_FREERTOS_BACKEND=1 \
       -DGP_IPC_CORE_$core=1 \
       -I"$ROOT/Intercore/Tests/ipc_freertos_runtime/config" \\
       -I"$KERNEL/include" -I"$KERNEL/portable/ThirdParty/GCC/Posix" \\
       -I"$ROOT/Intercore/Tests/hsem_mocks" -I"$ROOT/Intercore" \
       "$ROOT/Intercore/ipc_stm32h745_hsem.c" \
       "$ROOT/Intercore/Tests/test_ipc_hsem_mock.c" \
       -o "$OUT/test_$core"
    "$OUT/test_$core"
done
