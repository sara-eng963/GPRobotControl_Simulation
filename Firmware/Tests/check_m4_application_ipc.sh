#!/usr/bin/env bash
# Stage 3B-1: ARM compile and CubeMX USER CODE binding checks for CM4 service.
# Does not prove link, scheduler behavior, physical IPC or safety response.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
NM="${ARM_NONE_EABI_NM:-arm-none-eabi-nm}"
command -v "$CC" >/dev/null || { echo "FAIL: missing $CC" >&2; exit 1; }
command -v "$NM" >/dev/null || { echo "FAIL: missing $NM" >&2; exit 1; }
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
flags=(
  -std=gnu11 -O0 -Wall -Wextra -Werror
  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  -DSTM32H745xx -DUSE_HAL_DRIVER -DCORE_CM4=1
  -I"$FW/CM4/Core/Inc" -I"$FW/Common/Inc"
  -I"$ROOT/Intercore" -I"$ROOT/StateMachine"
  -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include"
  -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
  -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
  -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
  -I"$FW/Drivers/CMSIS/Include"
  -I"$FW/Drivers/CMSIS/RTOS2/Include"
  -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc"
  -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
)
"$CC" "${flags[@]}" -c "$FW/CM4/Core/Src/application_ipc_service.c" -o "$OUT/ipc_service.o"
"$CC" "${flags[@]}" -c "$FW/CM4/Core/Src/main.c" -o "$OUT/main.o"
for symbol in application_ipc_service_initialize application_ipc_service_run application_ipc_service_submit_hmi application_ipc_service_read_status; do
  "$NM" --defined-only "$OUT/ipc_service.o" | grep -Eq " T $symbol$" || {
    echo "FAIL: missing $symbol" >&2; exit 1;
  }
done
python3 - "$FW/CM4/Core/Src/main.c" "$ROOT/Intercore/ipc_mailbox_freertos.c" <<'PY'
from pathlib import Path
import sys
main = Path(sys.argv[1]).read_text()
ipc = Path(sys.argv[2]).read_text()
start = main.index('/* USER CODE BEGIN 5 */')
end = main.index('/* USER CODE END 5 */', start)
block = main[start:end]
assert 'application_ipc_service_initialize()' in block
assert 'application_ipc_service_run()' in block
assert block.index('application_ipc_service_initialize()') < block.index('application_ipc_service_run()')
assert 'intercore_startup_validate()' in main and 'intercore_notifications_initialize()' in main
assert 'xMessageBufferSend(handle, msg, sizeof(*msg), 0)' in ipc
assert 'xMessageBufferReceive(handle, msg, sizeof(*msg), 0)' in ipc
print('PASS: CM4 task wiring and nonblocking shared transport contract')
PY
echo "PASS: CM4 application IPC ARM compile-only gate (not a firmware link)"
