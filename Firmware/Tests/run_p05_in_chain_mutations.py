#!/usr/bin/env python3
"""Recompile production Supervisor mutants into the live P05 CANopen process.

Reuse an already built CMake target's exact flags and link graph. A compile
failure never counts as a kill; the executable must fail its P05 assertions.
All mutant objects, binaries and traces remain outside the checkout.
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SM = ROOT / 'Firmware/STM32H745/CM7/Application/StateMachine'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build-host', type=Path, required=True)
args = parser.parse_args()
work = args.build_host.resolve() / 'HandGuiding'
target = work / 'CMakeFiles/p05_virtual_system.dir'
flags = (target / 'flags.make').read_text()
options = []
for name in ('C_DEFINES', 'C_INCLUDES', 'C_FLAGS'):
    options += shlex.split(re.search(r'^'+name+r' = (.*)$', flags, re.M)[1])
link = shlex.split((target / 'link.txt').read_text())
adapter_name = 'CMakeFiles/p05_virtual_system.dir/__/Simulation/P05/p05_production_supervisor.c.o'
assert link.count(adapter_name) == 1
source = (SM / 'supervisor_task.c').read_text()
adapter = (ROOT / 'Simulation/P05/p05_production_supervisor.c').read_text()
mutations = {
    'dispatch_adjacent_timing_veto': (
        'if (hardware_timed &&\n            !supervisor_timing_pre_dispatch_check(context, supervisor_now_us()))',
        'if (false && hardware_timed &&\n            !supervisor_timing_pre_dispatch_check(context, supervisor_now_us()))'),
    'cm4_expiry_veto': (
        'else if (!runtime_peer_fresh(&context->peer_evidence, now_ms))',
        'else if (false && !runtime_peer_fresh(&context->peer_evidence, now_ms))'),
}

with tempfile.TemporaryDirectory(prefix='p05-in-chain-mutations-') as td:
    out = Path(td)
    for label, replacements in [('nominal', None), *mutations.items()]:
        mutant = out / 'supervisor_task.c'
        code = source
        if replacements:
            old, new = replacements
            assert code.count(old) == 1, label
            code = code.replace(old, new, 1)
        # Preserve production header ownership when compiling a relocated TU.
        mutant.write_text(code.replace('#include "supervisor_task.h"',
            '#include "' + str(SM / 'supervisor_task.h') + '"', 1))
        unit = out / 'adapter.c'
        unit.write_text(adapter.replace(
            '"../../Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.c"',
            '"' + str(mutant) + '"').replace(
            '"../../Firmware/Tests/test_production_supervisor.c"',
            '"' + str(ROOT / 'Firmware/Tests/test_production_supervisor.c') + '"').replace(
            '"p05_production_supervisor.h"',
            '"' + str(ROOT / 'Simulation/P05/p05_production_supervisor.h') + '"'))
        obj, binary = out/'adapter.o', out/'live-chain'
        subprocess.run([link[0], '-I'+str(SM), *options,
            '-Wno-error=pedantic', '-c', str(unit), '-o', str(obj)],
            cwd=work, check=True, capture_output=True, text=True)
        command = [str(obj) if x == adapter_name else x for x in link]
        command[command.index('-o') + 1] = str(binary)
        subprocess.run(command, cwd=work, check=True, capture_output=True, text=True)
        evidence = out / label
        evidence.mkdir()
        result = subprocess.run([str(binary), str(evidence)],
                                capture_output=True, text=True, timeout=90)
        if replacements is None:
            assert result.returncode == 0, result.stdout + result.stderr
            trace = json.loads((evidence/'production_supervisor_trace.json').read_text())
            assert trace['same_live_canopen_process'] and trace['task_entry_executed']
            print('PASS: live production Supervisor nominal baseline', flush=True)
        else:
            assert result.returncode != 0 and 'P05_ASSERT_FAIL' in result.stderr, (
                label, result.stdout, result.stderr)
            print('REJECTED (compiled, live-chain behavioral assertion): '+label, flush=True)
print('PASS: 2/2 production Supervisor in-chain mutations rejected')
