#!/usr/bin/env python3
"""Supplementary source-graph integration assertions; not executable safety proof.

The live portable policy and mutations run separately in
run_drive_restart_gate_tests.py; this rejects accidental deletion of the
production call sites that those portable tests cannot directly observe.
"""
from pathlib import Path
root = Path(__file__).resolve().parents[2]
cm7 = root / 'Firmware/STM32H745/CM7'
boot = (cm7 / 'Application/StateMachine/States/state_boot.c').read_text()
bh = (cm7 / 'Application/StateMachine/States/state_boot.h').read_text()
master = (cm7 / 'Application/CANComm/CANopen/canopen_master.c').read_text()
sup = (cm7 / 'Application/StateMachine/supervisor_task.c').read_text()
main = (cm7 / 'Core/Src/main.c').read_text()
app = (cm7 / 'Core/Src/application_supervisor.c').read_text()
assert 'BOOT_PHASE_WAIT_AUTHORIZATION' in bh
assert 'case BOOT_PHASE_PRE_ENABLE_STATUS:' in boot
assert 'case BOOT_PHASE_WAIT_AUTHORIZATION:' in boot
assert 'if (!boot->drive_enable_authorized)' in boot
assert boot.index('case BOOT_PHASE_WAIT_AUTHORIZATION:') < boot.index('case BOOT_PHASE_ENABLE_DRIVES:')
assert master.count('!master->drive_command_permission') >= 3
assert 'controlword != 0x0000U && controlword != 0x0002U' in master
assert 'drive_restart_gate_tick(' in sup
assert 'canopen_master_set_drive_permission(' in sup
assert 'drive_restart_gate_inhibit(&context->restart_gate);' in sup
assert 'context->restart_gate.authorized &&' in sup
assert main.index('g_cm7_reset_evidence = reset_diagnostics_from_raw(raw_reset)') < main.index('__HAL_RCC_CLEAR_RESET_FLAGS()')
assert 'config->reset_diagnostics = *application_supervisor_reset_info();' in app
print('PASS: C2B production wiring anchors (supplementary, non-executable)')
