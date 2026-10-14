#!/usr/bin/env bash
# Stage 3C-2a: compile the CM7 physical transport in its *current* inactive
# CubeMX state. This is not a test of the ST FDCAN HAL-enabled branch.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
command -v "$CC" >/dev/null
OBJ="$(mktemp --suffix=.o)"
trap 'rm -f "$OBJ"' EXIT

# The current vendor snapshot lacks HAL FDCAN entirely. Do not fake it.
if grep -Eq '^[[:space:]]*#define[[:space:]]+HAL_FDCAN_MODULE_ENABLED' \
    "$FW/CM7/Core/Inc/stm32h7xx_hal_conf.h"; then
    echo "FAIL: this inactive-mode gate is no longer applicable; run real HAL FDCAN integration checks" >&2
    exit 1
fi
"$CC" -std=gnu11 -Wall -Wextra -Werror -O0 \
    -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
    -DUSE_HAL_DRIVER -DSTM32H745xx -DCORE_CM7=1 \
    -I"$FW/CM7/Core/Inc" -I"$FW/Common/Inc" \
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc" \
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" \
    -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include" \
    -I"$FW/Drivers/CMSIS/Include" \
    -c "$FW/CM7/Board/Src/fdcan_classic_backend.c" -o "$OBJ"

python3 - "$FW" <<'PY'
import sys
from pathlib import Path
fw=Path(sys.argv[1])
p=(fw/'CM7/.cproject').read_text()
b=(fw/'CM7/Board/Src/fdcan_classic_backend.c').read_text()
config=(fw/'CM7/Core/Inc/stm32h7xx_hal_conf.h').read_text()
assert p.count('name="Board"')>=2
assert p.count('value="../Board/Inc"')==4
assert p.count('value="../Application/CANComm"')==4
assert '#if defined(HAL_FDCAN_MODULE_ENABLED)' in b
assert 'HAL_FDCAN_ConfigGlobalFilter' in b
assert 'HAL_FDCAN_GetProtocolStatus' in b
assert 'FDCAN_CLASSIC_CAN' in b
import re
assert not re.search(r'^\\s*#\\s*define\\s+HAL_FDCAN_MODULE_ENABLED\\b', config, re.M)
print("PASS: FDCAN Board source registration and CubeMX-disabled guard")
PY
echo "PASS: conditional CM7 board source compiles without unconfigured FDCAN"
echo "NOTE: HAL-enabled FDCAN branch, bus timing and physical transceiver are NOT verified."
