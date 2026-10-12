#!/usr/bin/env bash
# Stage 2E-4: compile native ST HAL EXTI notification adapter on BOTH cores.
# Does not prove SW interrupt delivery or scheduler wakeup on hardware.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"
command -v "$CC" >/dev/null || { echo "FAIL: $CC missing"; exit 1; }
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
for core in CM7 CM4; do
    if [[ "$core" == CM7 ]]; then
        cpu=cortex-m7
        fpu=fpv5-d16
    else
        cpu=cortex-m4
        fpu=fpv4-sp-d16
    fi
    echo "[Stage 2E-4] Compiling $core ST-HAL EXTI notification adapter..."
    flags=(-std=gnu11 -O0 -Wall -Wextra -Werror
        -mcpu="$cpu" -mthumb -mfpu="$fpu" -mfloat-abi=hard
        -DSTM32H745xx -DUSE_HAL_DRIVER -D"CORE_$core=1"
        -I"$FW/$core/Core/Inc" -I"$FW/Common/Inc"
        -I"$ROOT/Intercore"
        -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include"
        -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
        -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
        -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
        -I"$FW/Drivers/CMSIS/Include"
        -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc"
        -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy")
    "$CC" "${flags[@]}" -c "$FW/Common/Src/intercore_notifications.c" -o "$OUT/$core-notifications.o"
    echo "PASS: $core intercore_notifications.c compiled against real HAL/FreeRTOS headers"
    "$CC" "${flags[@]}" -c "$FW/$core/Core/Src/stm32h7xx_it.c" -o "$OUT/$core-irqs.o"
    echo "PASS: $core generated IRQ file compiles with notification handler"
done
python3 - "$FW" <<'PY'
from pathlib import Path
import sys
fw = Path(sys.argv[1])
notify = (fw/"Common/Src/intercore_notifications.c").read_text()
assert 'EXTI_MODE_CORE2_INTERRUPT' in notify and 'EXTI_MODE_CORE1_INTERRUPT' in notify
assert 'HAL_EXTI_GenerateSWI(&exti_tx)' in notify
assert 'xMessageBufferSendCompletedFromISR' in notify
assert 'HAL_EXTI_IRQHandler(&exti_rx)' in notify
assert 'gp_ipc_set_notifier' in notify
for core, irq, other in [('CM7', 'EXTI1_IRQHandler', 'EXTI0_IRQHandler'),
                         ('CM4', 'EXTI0_IRQHandler', 'EXTI1_IRQHandler')]:
    main = (fw/core/"Core/Src/main.c").read_text()
    it = (fw/core/"Core/Src/stm32h7xx_it.c").read_text()
    assert 'intercore_notifications_initialize()' in main
    assert irq+'(void)' in it and other+'(void)' not in it
    assert 'intercore_notifications_handle_irq()' in it
    if core == 'CM7':
        assert main.index('intercore_shared_memory_initialize()') < main.index('intercore_notifications_initialize()') < main.index('HAL_HSEM_Release(HSEM_ID_0,0)')
    else:
        assert main.index('intercore_startup_validate()') < main.index('intercore_notifications_initialize()') < main.index('osKernelInitialize()')
print("PASS: CM7/CM4 EXTI line ownership, IRQ binding and startup ordering")
PY
echo "PASS: Stage 2E-4 ARM compile + source-order integration checks"
echo "NOTE: STM32CubeIDE full builds and physical EXTI delivery still pending."
