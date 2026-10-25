#!/usr/bin/env bash
# Stage 3B-2 ARM compile only; full firmware linking remains a CubeIDE gate.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
command -v "$CC" >/dev/null
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
flags=(
  -std=gnu11 -O0 -Wall -Wextra -Werror
  -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  -DSTM32H745xx -DUSE_HAL_DRIVER -DCORE_CM7=1
  -DIPC_FREERTOS_BACKEND=1
  -I"$FW/CM7/Application/StateMachine" -I"$FW/Common/Inc" -I"$FW/CM7/Application/Intercore" -I"$FW/CM7/Application/CANComm"
  -I"$FW/CM7/Core/Inc" -I"$FW/CM7/Board/Inc" -I"$FW/Common/Inc"
  -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include"
  -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
  -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
  -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
  -I"$FW/Drivers/CMSIS/Include" -I"$FW/Drivers/CMSIS/RTOS2/Include"
  -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc"
  -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
)
"$CC" "${flags[@]}" -c "$FW/CM7/Core/Src/application_supervisor.c" -o "$OUT/application.o"
"$CC" "${flags[@]}" -c "$FW/CM7/Core/Src/main.c" -o "$OUT/main.o"
python3 - "$FW" <<'PY'
from pathlib import Path
import sys
fw=Path(sys.argv[1])
p=(fw/'CM7/.cproject').read_text()
s=(fw/'CM7/Core/Src/main.c').read_text()
a=(fw/'CM7/Core/Src/application_supervisor.c').read_text()
assert '#define IPC_FREERTOS_BACKEND 1' in a
assert p.count('name="Application"')==2
assert p.count('value="../Application/Intercore"')==2
assert 'application_supervisor_board_configure(&board_config)' in s
assert 'application_supervisor_start(&board_config)' in s
assert '__attribute__((weak))' in a
assert 'config.ipc = shared' in a
print('PASS: M7 board gating, IPC ownership, and CubeIDE settings')
PY
echo "PASS: CM7 application ARM compile-only check (NOT target link)"
