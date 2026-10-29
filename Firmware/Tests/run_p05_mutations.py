#!/usr/bin/env python3
"""P05 compiled behavioral mutations for actual hand_guiding serial source.

A mutation is counted as rejected only if its compilation succeeds AND the
real host test returns failure. Never count a build error as a test kill.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'HandGuiding/ft_serial_transport.c'
TEST = ROOT / 'HandGuiding/Tests/test_ft_serial_transport.c'
INCLUDE = ROOT / 'HandGuiding'

MUTANTS = {
    'P05-MUT-01-bypass-sequence-replay': (
        'if (delta == 0U || delta >= UINT32_C(0x80000000)) {',
        '(void)delta; if (false) {'),
    'P05-MUT-02-admit-old-dma-packet': (
        'if (elapsed_exceeds(now_us, observed_us, p->max_sample_age_us)) {',
        '(void)now_us; if (false) {'),
    'P05-MUT-03-bypass-overflow-containment': (
        'if (consume_errors(p)) return 0U;',
        'if (false) return 0U;'),
}

def compile_and_run(cc, source, exe):
    cmd = [cc, '-std=c11', '-Wall', '-Wextra', '-Wpedantic',
           '-Werror', '-UNDEBUG', '-I', str(INCLUDE),
           str(source), str(TEST), '-o', str(exe)]
    build = subprocess.run(cmd, text=True, capture_output=True)
    if build.returncode:
        return 'COMPILE_FAIL', build.stderr
    run = subprocess.run([str(exe)], text=True, capture_output=True)
    return ('PASS' if run.returncode == 0 else 'BEHAVIOR_FAIL',
            run.stdout + run.stderr)

def main():
    if not SOURCE.is_file() or not TEST.is_file():
        raise SystemExit('P05 sources absent')
    original = SOURCE.read_text(encoding='utf-8')
    cc = os.environ.get('CC', 'cc')
    with tempfile.TemporaryDirectory(prefix='p05_mut_') as tmp:
        out = Path(tmp)
        nominal, log = compile_and_run(cc, SOURCE, out / 'nominal')
        if nominal != 'PASS':
            print(log)
            raise SystemExit(f'Nominal production-source test not PASS: {nominal}')
        print('P05-MUT-00 nominal compiled and passed')
        for i, (name, (old, new)) in enumerate(MUTANTS.items()):
            if original.count(old) != 1:
                raise SystemExit(f'{name}: mutation anchor count !=1')
            candidate = out / f'mutant_{i}.c'
            candidate.write_text(original.replace(old, new, 1), encoding='utf-8')
            status, evidence = compile_and_run(cc, candidate, out / f'mutant_{i}')
            if status != 'BEHAVIOR_FAIL':
                print(evidence)
                raise SystemExit(f'{name}: not behaviorally rejected ({status})')
            print(f'{name}: compiled, behaviorally rejected')
    print('PASS: 3/3 compiled source mutations behaviorally rejected')

if __name__ == '__main__':
    main()
