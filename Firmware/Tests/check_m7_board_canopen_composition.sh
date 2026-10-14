#!/usr/bin/env bash
# Stage 3 Batch A: ARM compile and source-level fail-closed invariants.
# This is NOT proof of real electrical safety, motors or 2-ms deadlines.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
NM="${ARM_NONE_EABI_NM:-arm-none-eabi-nm}"
command -v "$CC" >/dev/null
command -v "$NM" >/dev/null
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
flags=(
    -std=gnu11 -O0 -Wall -Wextra -Werror
    -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
    -DUSE_HAL_DRIVER -DSTM32H745xx -DCORE_CM7=1
    -DGP_IPC_FREERTOS_BACKEND=1
    -I"$ROOT/StateMachine" -I"$ROOT/Intercore" -I"$ROOT/CANComm"
    -I"$FW/CM7/Core/Inc" -I"$FW/CM7/Board/Inc"
    -I"$FW/Common/Inc"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    -I"$FW/Drivers/CMSIS/Include" -I"$FW/Drivers/CMSIS/RTOS2/Include"
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc"
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
)
"$CC" "${flags[@]}" -c "$FW/CM7/Core/Src/application_supervisor.c" -o "$OUT/app.o"
"$CC" "${flags[@]}" -c "$FW/CM7/Board/Src/fdcan_classic_backend.c" -o "$OUT/fdcan.o"

# The CANopen adapter is genuinely referenced in the production composition,
# not forcibly retained via a linker KEEP directive.
"$NM" -u "$OUT/app.o" | grep -Eq '[[:space:]]U gp_fdcan_classic_backend_open$'
"$NM" --defined-only "$OUT/fdcan.o" | grep -Eq '[[:space:]]T gp_fdcan_classic_backend_open$'

# Inspect the compiled object for real HAL_FDCAN_Init dependencies.
# References in source comments are harmless.
if "$NM" -u "$OUT/app.o" | grep -Eq '[[:space:]]U HAL_FDCAN_Init$'; then
    echo "FAIL: CubeMX (not application glue) must own HAL_FDCAN_Init" >&2
    exit 1
fi

# No board supplier is defined by the firmware. A user must provide a real
# hardware board implementation. A weak unresolved reference is intentional.
"$NM" "$OUT/app.o" | grep -Eq '[[:space:]]w gp_board_supply_commissioned_config$'
if "$NM" --defined-only "$OUT/app.o" | grep -Eq '[[:space:]]T gp_board_supply_commissioned_config$'; then
    echo "FAIL: default production board supplier unexpectedly provided" >&2
    exit 1
fi

python3 - "$FW" <<'PY'
from pathlib import Path
import sys
fw=Path(sys.argv[1])
a=(fw/'CM7/Core/Src/application_supervisor.c').read_text()
b=(fw/'CM7/Board/Inc/board_commissioning.h').read_text()
assert 'gp_board_supply_commissioned_config == NULL' in a
assert 'hardware_interlocks_verified' in a and 'hardware_interlocks_verified' in b
assert 'io->read_inputs' in a and 'SUP_IO_ESTOP' in a and 'SUP_IO_PROTECTIVE_STOP' in a
assert 'gp_fdcan_classic_backend_open' in a and 'canopen_master_init' in a
assert 'gp_ipc_valid(shared)' in a and 'supervisor_task_start(4U)' in a
assert 'revoke_unstarted_board(board)' in a and 'canopen_master_close(&g_canopen_master)' in a
assert 'CANOPEN_NODE_IDS' not in a
assert not (fw/'CM7/Board/Src/board_config_real.c').exists()
print('PASS: board-supplied CANopen composition, fail-closed startup invariants')
PY

echo "PASS: HAL-active M7 CANopen composition ARM-compiled; adapter linked by reference"
echo "NOTE: Physical board provider absent, Supervisor refuses startup by design."
