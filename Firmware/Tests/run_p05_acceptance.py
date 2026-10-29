#!/usr/bin/env python3
"""P05 laptop-only acceptance runner. Evidence is written outside the repo.

No commits, branch edits, resets, or simulation claims about hardware.
Outputs are WSL wall-clock runtimes, not STM32 WCET measurements.
"""
import argparse
import json
import subprocess
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
BASE = 'ba5ab122868bd50a4626c568574be106dfce98fe'
BRANCH = 'sara/can-replacement'


def command(args, label, out):
    started = time.perf_counter()
    try:
        result = subprocess.run(args, cwd=ROOT, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=1800)
        status, log = ('PASS' if result.returncode == 0 else 'FAIL'), result.stdout
    except (OSError, subprocess.TimeoutExpired) as error:
        status, log = 'FAIL', str(error)
    wall = round(time.perf_counter() - started, 3)
    (out / (label + '.log')).write_text(log, encoding='utf-8')
    print(f'{label}: {status} (WSL wall {wall}s)')
    return {'status': status, 'wall_seconds': wall,
            'log': label + '.log', 'argv': [str(v) for v in args]}


def main():
    arg = argparse.ArgumentParser(description=__doc__)
    arg.add_argument('--output', type=Path, help='new, nonexistent evidence directory')
    ns = arg.parse_args()
    branch = subprocess.check_output(['git', 'branch', '--show-current'],
                                     cwd=ROOT, text=True).strip()
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'],
                                   cwd=ROOT, text=True).strip()
    # Verified P05 commits retain the original baseline in their ancestry.
    ancestry = subprocess.run(['git','merge-base','--is-ancestor',BASE,head],
                              cwd=ROOT, check=False).returncode == 0
    if branch != BRANCH or not ancestry:
        raise SystemExit(f'NO TEST RUN: expected {BRANCH} descending from {BASE}; got {branch}@{head}')
    if ns.output is None:
        output = Path(tempfile.mkdtemp(prefix='p05_acceptance_'))
    else:
        output = ns.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
    print('Evidence directory:', output)
    build = output / 'host'
    data = {'baseline': head, 'branch': branch,
            'note': 'Host or WSL execution, no real physical hardware evidence',
            'results': {}}
    def record(label, args):
        data['results'][label] = command(args, label, output)
        (output / 'results.json').write_text(json.dumps(data, indent=2) + '\n')
        return data['results'][label]['status'] == 'PASS'

    configured = record('configure_host', ['cmake','-S',str(ROOT),'-B',str(build),
                                           '-DCMAKE_BUILD_TYPE=Debug'])
    compiled = configured and record('build_host', ['cmake','--build',str(build),
                                                     '--parallel','4'])
    if compiled:
        record('p05_connected_host_runner', ['bash', str(ROOT / 'scripts/run_p05_virtual_end_to_end.sh'), str(output / 'p05_connected')])
        record('p05_and_full_ctest', ['ctest','--test-dir',str(build),
                                        '--output-on-failure','--no-tests=error',
                                        '--parallel','2'])
        record('compiled_p05_mutations', [sys.executable,
                str(ROOT / 'Firmware/Tests/run_p05_mutations.py')])
        record('compiled_p04_mutations', [sys.executable,
                str(ROOT / 'Firmware/Tests/run_p04_mutations.py')])
        record('p05_in_chain_mutations', [sys.executable,
                str(ROOT / 'Firmware/Tests/run_p05_in_chain_mutations.py'),
                '--build-host',str(build)])
        record('p05_production_supervisor_source', [sys.executable,
                str(ROOT / 'Firmware/Tests/run_p05_supervisor_source.py')])
        record('p05_acceptance_negative', [sys.executable,
                str(ROOT / 'Firmware/Tests/test_p05_acceptance_gate.py')])
        record('p05_complete_evidence_gate', [sys.executable,
                str(ROOT / 'Firmware/Tests/check_p05_integrated_evidence.py'),
                str(output / 'p05_connected')])
    else:
        for label in ('p05_connected_host_runner','p05_and_full_ctest','compiled_p05_mutations',
                      'compiled_p04_mutations', 'p05_in_chain_mutations', 'p05_production_supervisor_source',
                      'p05_acceptance_negative','p05_complete_evidence_gate'):
            data['results'][label] = {'status':'NOT RUN',
                                     'reason':'Host configure/build failed'}

    for config in ('Debug','Release'):
        arm_root = output / 'stm32' / config
        for core in ('CM7','CM4'):
            record(f'arm_{config}_{core}', [sys.executable,
                str(ROOT / 'Firmware/Tests/build_stm32.py'),
                '--core',core,'--config',config,
                '--output',str(arm_root / core)])
        record(f'p05_uart_arm_link_{config}', [sys.executable,
            str(ROOT / 'Firmware/Tests/check_p05_uart_arm.py'),
            '--build-root',str(arm_root)])
        record(f'firmware_integration_{config}', [sys.executable,
            str(ROOT / 'Firmware/Tests/check_firmware_integration.py'),
            '--build-root',str(arm_root)])

    record('git_diff_check', ['git','diff','--check'])
    (output/'results.json').write_text(json.dumps(data, indent=2)+'\n')
    failed = [key for key,val in data['results'].items() if val['status']!='PASS']
    print('Final result:', 'PASS' if not failed else 'INCOMPLETE',
          ', '.join(failed))
    return 1 if failed else 0

if __name__ == '__main__':
    sys.exit(main())
