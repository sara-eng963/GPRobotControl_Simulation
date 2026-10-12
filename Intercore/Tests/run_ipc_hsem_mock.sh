#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
for core in M7 M4; do
    echo "[IPC HSEM] Testing $core against HAL/FreeRTOS test doubles..."
    cc -std=c11 -Wall -Wextra -Werror -DGP_IPC_FREERTOS_BACKEND=1 \
       -DGP_IPC_CORE_$core=1 \
       -I"$ROOT/Intercore/Tests/hsem_mocks" -I"$ROOT/Intercore" \
       "$ROOT/Intercore/ipc_stm32h745_hsem.c" \
       "$ROOT/Intercore/Tests/test_ipc_hsem_mock.c" \
       -o "$OUT/test_$core"
    "$OUT/test_$core"
done
