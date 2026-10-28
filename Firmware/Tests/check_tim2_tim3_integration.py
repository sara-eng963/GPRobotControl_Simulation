#!/usr/bin/env python3
"""Read-only source/contract gate for STM32H745 TIM2/TIM3 integration.

This verifies CubeMX and application wiring, NOT target compilation or runtime
determinism. Pass/fail messages intentionally distinguish this static gate from
real HAL timer/FreeRTOS/hardware tests.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "Firmware" / "STM32H745"
read = lambda name: (ROOT / name).read_text(encoding="utf-8")
ioc = read("Firmware/STM32H745/STM32H745ZIT6_BoardConfig.ioc")
main = read("Firmware/STM32H745/CM7/Core/Src/main.c")
isr = read("Firmware/STM32H745/CM7/Core/Src/stm32h7xx_it.c")
msp = read("Firmware/STM32H745/CM7/Core/Src/stm32h7xx_hal_msp.c")
app = read("Firmware/STM32H745/CM7/Core/Src/application_supervisor.c")
sup = read("StateMachine/supervisor_task.c")
hdr = read("StateMachine/supervisor_task.h")
a_hdr = read("Firmware/STM32H745/CM7/Core/Inc/application_supervisor.h")
checks = {
    "TIM3 persisted 2 ms period": "TIM3.Period=1999" in ioc,
    "TIM3 persisted divider": "TIM3.Prescaler=199" in ioc,
    "TIM2 persisted divider": "TIM2.Prescaler=199" in ioc,
    "TIM2 1 MHz counter": "htim2.Init.Period = 4294967295;" in main and "htim2.Init.Prescaler = 199;" in main,
    "TIM3 500 Hz counter": "htim3.Init.Period = 1999;" in main and "htim3.Init.Prescaler = 199;" in main,
    "Both timer interrupt handlers from CubeMX": all(x in isr for x in ("HAL_TIM_IRQHandler(&htim2);", "HAL_TIM_IRQHandler(&htim3);")),
    "TIM2 NVIC priority 5": "HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);" in msp,
    "TIM3 NVIC priority 5": "HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);" in msp,
    "CubeMX USER CODE HAL callback integration": "application_supervisor_timer_period_elapsed_isr(htim);" in main,
    "Existing ST HAL tick preserved": "htim->Instance == TIM6" in main and "HAL_IncTick();" in main,
    "Start both HAL timers with interrupts": all(x in app for x in ("HAL_TIM_Base_Start_IT(&htim2)", "HAL_TIM_Base_Start_IT(&htim3)")),
    "Clear stale update flags": all(x in app for x in ("__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE)", "__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE)")),
    "TIM2 pending overflow accounted": all(x in app for x in ("g_tim2_wraps", "TIM_FLAG_UPDATE", "<< 32U")),
    "Counted FreeRTOS ISR notify": "vTaskNotifyGiveFromISR" in app and "portYIELD_FROM_ISR" in app,
    "Counted FreeRTOS task wait": "ulTaskNotifyTake(pdTRUE, timeout_ticks)" in app,
    "Supervisor optional clock hooks": "SupervisorTimingPort timing;" in hdr and "g_timing.monotonic_us" in sup,
    "PC RTOS tick fallback remains": "xTaskGetTickCount()" in sup and "vTaskDelayUntil" in sup,
    "Supervisor scheduling failures classified": all(x in sup for x in ("0xE002U", "0xE003U", "0xE004U")),
    "Supervisor runtime diagnostics exposed": all(x in hdr for x in ("timer_missed_periods", "timer_timeouts", "timer_overruns", "timer_max_cycle_us")),
    "Missing hardware board still fail closed": "board_supply_commissioned_config == NULL" in app,
    "Public HAL timer callback declaration": "application_supervisor_timer_period_elapsed_isr" in a_hdr,
}
errors = 0
for label, yes in checks.items():
    print(f'{"PASS" if yes else "FAIL"}: {label}')
    errors += not yes
if errors:
    print(f"FAIL: {errors} STM32 timer source wiring checks")
    sys.exit(1)

# Mathematical reference cases for review of the intended 1 MHz timer
# and counted wakeups. This is NOT a test of the embedded implementation.
assert 200000000 // (199 + 1) == 1000000
assert 1000000 // (1999 + 1) == 500
for high, low, pending, result in [
    (0, 0, False, 0),
    (0, 0xFFFFFFFF, False, 0xFFFFFFFF),
    (0, 0, True, 0x100000000),
    (1, 17, False, 0x100000011),
    (1, 23, True, 0x200000017),
]:
    assert ((high + int(pending)) << 32) | low == result
for delivered, missed in [(0, None), (1, 0), (2, 1), (5, 4)]:
    if delivered:
        assert delivered - 1 == missed
print("PASS: TIM2/TIM3 source wiring and arithmetic reference checks (static only)")
