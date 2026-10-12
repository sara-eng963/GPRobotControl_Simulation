#!/usr/bin/env bash
# Stage 2E-2: verify the actual CM7 initialization implementation compiles
# against the checked-in STM32CubeH7 FreeRTOS 10.6.2 / CMSIS headers.
# This is a SOURCE-LEVEL test; it neither boots both cores nor proves IPC.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
NM="${ARM_NONE_EABI_NM:-arm-none-eabi-nm}"
for command_name in "$CC" "$NM" python3; do
    command -v "$command_name" >/dev/null || {
        echo "FAIL: missing $command_name" >&2; exit 1;
    }
done
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
INCLUDE_FLAGS=(
    -I"$FW/CM7/Core/Inc" -I"$ROOT/Intercore"
    -I"$FW/Common/Inc"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    -I"$FW/Drivers/CMSIS/Include"
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc"
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
)
GCC_FLAGS=(-std=gnu11 -O0 -Wall -Wextra -Werror
    -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
    -DSTM32H745xx -DUSE_HAL_DRIVER -DCORE_CM7=1)
echo "[Stage 2E-2] Compiling production inter-core initialization sources..."
for f in \
    "$FW/CM7/Core/Src/intercore_shared_memory.c" \
    "$ROOT/Intercore/ipc_mailbox_freertos.c" \
    "$ROOT/Intercore/trajectory_stream.c"; do
    "$CC" "${GCC_FLAGS[@]}" "${INCLUDE_FLAGS[@]}" -c "$f" \
        -o "$OUT/$(basename "$f").o"
    echo "PASS: $(basename "$f") compiles for CM7"
done
"$NM" "$OUT/intercore_shared_memory.c.o" | grep -Eq ' T intercore_shared_memory_initialize$' || {
    echo "FAIL: M7 initializer symbol not exported" >&2; exit 1;
}
"$NM" "$OUT/ipc_mailbox_freertos.c.o" | grep -Eq ' T gp_ipc_initialize$' || {
    echo "FAIL: native FreeRTOS IPC initializer not linked in source" >&2; exit 1;
}
"$NM" "$OUT/trajectory_stream.c.o" | grep -Eq ' T gp_traj_shared_initialize$' || {
    echo "FAIL: existing trajectory initializer not built" >&2; exit 1;
}
python3 - "$FW/CM7/Core/Src/main.c" <<'PY'
from pathlib import Path
import sys
s = Path(sys.argv[1]).read_text()
assert '#include "intercore_shared_memory.h"' in s, "CM7 bootstrap header missing"
assert s.count('intercore_shared_memory_initialize()') == 1, "CM7 bootstrap call missing or duplicated"
before, _, region = s.partition("/* USER CODE BEGIN Boot_Mode_Sequence_2 */")
assert region, "CubeMX boot user-code section missing"
boot, _, rest = region.partition("/* USER CODE END Boot_Mode_Sequence_2 */")
assert rest, "Unterminated CubeMX boot user-code section"
assert "SystemClock_Config();" in before, "CM7 clock init must precede bootstrap"
initialize = boot.index("intercore_shared_memory_initialize()")
hsem_take = boot.index("HAL_HSEM_FastTake(HSEM_ID_0)")
hsem_release = boot.index("HAL_HSEM_Release(HSEM_ID_0,0)")
assert initialize < hsem_take < hsem_release, "CM7 IPC must initialize before HSEM-0 release"
assert "Error_Handler();" in boot[initialize:hsem_release], "Failed initialization must prevent CM4 release"
print("PASS: CubeMX user-code region initializes IPC before HSEM-0 M4 wakeup")
PY
echo "PASS: Stage 2E-2 CM7 source integration checks"
echo "NOTE: full CubeIDE linking and physical dual-core runtime remain pending."
