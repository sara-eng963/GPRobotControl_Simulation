#!/usr/bin/env python3
"""Static, read-only firmware C2A integration check (not a build/test of ISR)."""
from pathlib import Path
import sys
root = Path(__file__).resolve().parents[2]
fw = root / 'Firmware/STM32H745'
get = lambda name: (root / name).read_text()
cm7 = get('Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.c')
cm7_h = get('Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.h')
cm7_main = get('Firmware/STM32H745/CM7/Core/Src/main.c')
cm4 = get('Firmware/STM32H745/CM4/Core/Src/application_ipc_service.c')
cm4_h = get('Firmware/STM32H745/CM4/Core/Inc/application_ipc_service.h')
policy = get('Firmware/STM32H745/Common/Inc/supervision_progress.h')
checks = {
    'CM7 publishes only after timing completion':
        cm7.index('supervision_record_completion(&context->progress') >
        cm7.index('supervisor_timing_assess('),
    'CM7 publishes from the Supervisor task body':
        cm7.index('supervision_record_completion(&context->progress') <
        cm7.index('bool supervisor_task_init('),
    'CM7 snapshots under same-core critical section':
        'bool supervisor_task_get_progress(SupervisionProgressSample *out)' in cm7_h and
        'taskENTER_CRITICAL();\n    *out = g_supervisor.progress;\n    taskEXIT_CRITICAL();' in cm7,
    'CM7 background task observes but cannot feed':
        'supervision_progress_eligible(&progress_policy' in cm7_main and
        'HAL_IWDG_Refresh(' not in cm7_main and
        'HAL_IWDG_Init(' not in cm7_main,
    'CM4 publishes after IPC work':
        cm4.index('supervision_record_completion(&ipc_progress') >
        cm4.index('const bool state_changed = receive_controller_status();') and
        'bool application_ipc_service_get_progress(SupervisionProgressSample *out)' in cm4_h,
    'No IPC message structure or CANopen interface changes':
        'supervision_progress_eligible' in policy and
        not (fw / 'Common/Inc/supervision_progress.c').exists(),
    'Stage B deadline logic preserved':
        all(x in cm7 for x in ('0xE003U', '0xE004U', 'supervisor_timing_assess(',
                               'a.service_gap_severe', 'a.wake_gap_severe')),
}
for name, value in checks.items():
    print(('PASS' if value else 'FAIL')+': '+name)
sys.exit(0 if all(checks.values()) else 1)
