#!/usr/bin/env python3
"""Execute exact production CM7 CANopen coordinator against a fake CAN backend.

No repository writes; compiler artifacts and mutants live in temporary directories.
Requires the already-C2B-patched local checkout, NOT the C2A remote baseline.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'Firmware/STM32H745/CM7/Application'
MASTER = FW / 'CANComm/CANopen/canopen_master.c'
TEST = ROOT / 'Firmware/Tests/test_c2b_canopen_master.c'
DEPS = [
    FW / 'CANComm/CANopen/canopen_heartbeat.c',
    FW / 'CANComm/CANopen/canopen_nmt.c',
    FW / 'CANComm/CANopen/canopen_sdo.c',
    ROOT / 'ServoDrive/AvatarM/avatar_m_drive.c',
    ROOT / 'ServoDrive/AvatarM/avatar_m_pdo.c',
    ROOT / 'ServoDrive/CiA402/cia402.c',
]

# Each mutation removes one distinct safety decision; each MUST compile and
# then fail by violating an executed fake-CAN behavioral assertion.
MUTANTS = {
    'controlword veto': (
        '(!master->drive_command_permission &&\n         controlword != 0x0000U && controlword != 0x0002U)',
        'false'),
    'SYNC veto': (
        '!master->drive_command_permission ||\n        !master->initialized\n    )\n    {\n        return false;\n    }\n\n    CanFrame frame;',
        '!master->initialized\n    )\n    {\n        return false;\n    }\n\n    CanFrame frame;'),
    'RPDO target veto': (
        'master == NULL ||\n        !master->drive_command_permission ||\n        !master->initialized ||\n        target_positions == NULL',
        'master == NULL ||\n        !master->initialized ||\n        target_positions == NULL'),
    'owner-bound permission setter': (
        'master == NULL || master->owner_check == NULL ||\n        !owner_access_allowed(master)',
        'master == NULL'),
    'pending SDO cancellation': (
        'if (!permitted && master->sdo.state == CANOPEN_MASTER_SDO_PENDING &&',
        'if (false && !permitted && master->sdo.state == CANOPEN_MASTER_SDO_PENDING &&'),
    'NMT reset revocation': (
        'if (master != NULL &&\n        (command == CANOPEN_NMT_RESET_COMMUNICATION ||\n         command == CANOPEN_NMT_RESET_APPLICATION))\n        canopen_master_set_drive_permission(master, false);',
        'if (false) canopen_master_set_drive_permission(master, false);'),
    'foreign-owner policy': (
        'return master == NULL || master->owner_check == NULL ||\n           master->owner_check(master->owner_check_context);',
        'return master == NULL || master->initialized;'),
}

def compile_run(tmp: Path, source: Path, label: str):
    binary = tmp / 'bin'
    cmd = [os.environ.get('CC', 'cc'), '-std=c11', '-O1', '-g',
           '-Wall', '-Wextra', '-Werror', '-pedantic',
           '-I', str(FW / 'CANComm/CANopen'),
           str(TEST), str(source), *map(str, DEPS), '-lm', '-o', str(binary)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f'{label} COMPILE FAILED:\n{result.stdout}\n{result.stderr}')
    return subprocess.run([str(binary)], capture_output=True, text=True)

def main():
    for p in [MASTER, TEST, *DEPS]:
        if not p.is_file():
            raise RuntimeError(f'Production source or test missing: {p}')
    # Supplemental source wiring check. Behavior is exercised in the fake CAN
    # tests; this also ensures the live Supervisor cannot restore permission
    # automatically after an NMT reset. Does not replace an integration test.
    supervisor = (FW / 'StateMachine/supervisor_task.c').read_text()
    guard = ('if (was_authorized &&\n'
             '            !context->config.canopen_master->drive_command_permission)')
    if guard not in supervisor:
        raise RuntimeError('Supervisor missing NMT/session-loss restart inhibition')
    original = MASTER.read_text()
    for label, (needle, _) in MUTANTS.items():
        if original.count(needle) != 1:
            raise RuntimeError(f'{label}: expected one precise source anchor, found {original.count(needle)}; inspect real checkout before changing tests')
    with tempfile.TemporaryDirectory(prefix='c2b_canopen_', dir=os.environ.get('TMPDIR')) as path:
        tmp = Path(path)
        result = compile_run(tmp, MASTER, 'baseline')
        if result.returncode:
            raise RuntimeError(f'Production baseline FAIL:\n{result.stdout}\n{result.stderr}')
        print(result.stdout.strip())
        for label, (needle, replacement) in MUTANTS.items():
            mutant = tmp / 'canopen_master.c'
            mutant.write_text(original.replace(needle, replacement, 1))
            result = compile_run(tmp, mutant, label)
            if result.returncode == 0:
                raise RuntimeError(f'MUTATION SURVIVED: {label}')
            if 'FAIL ' not in result.stderr:
                raise RuntimeError(f'MUTATION ABNORMAL TERMINATION: {label}: {result.stderr}')
            print(f'PASS: mutation rejected at runtime: {label}')
    print('PASS: 7/7 executable production-master mutations rejected')

if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError) as error:
        sys.exit(f'FAIL: {error}')
