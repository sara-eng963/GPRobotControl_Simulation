#!/usr/bin/env python3
"""C2A executable policy tests + mutations; no hardware watchdog required.

Runs the actual portable production header with host C11. No claim that the
full CM7/CM4 firmware, cross-core transport or MCU reset was exercised.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

TESTS = Path(__file__).resolve().parent
HEADER = TESTS.parent / 'STM32H745/Common/Inc/supervision_progress.h'
SOURCE = TESTS / 'test_supervision_policy.c'
MUTATIONS = {
    'sequence check': ('!supervision_sequence_forward(sample->sequence,',
                       'false && !supervision_sequence_forward(sample->sequence,'),
    'freshness': ('age > max_age_ms', 'false'),
    'session': ('sample->session != policy->expected_session', 'false'),
    'completed cycle': ('!sample->valid || !sample->completed_ok',
                        '!sample->valid || false'),
    'fault response': ('sample->mode == SUP_PROGRESS_FAULT && !sample->response_ok',
                       'sample->mode == SUP_PROGRESS_FAULT && false'),
    'manual reauthorization': ('!input->operator_reauthorized', 'false'),
}

def check_run(header_contents, name, expect_success):
    with tempfile.TemporaryDirectory(prefix='c2a_mutation_') as tmp:
        t = Path(tmp)
        (t / 'supervision_progress.h').write_text(header_contents)
        exe = t / 'run'
        subprocess.run(['cc', '-std=c11', '-O0', '-Wall', '-Wextra', '-Werror',
                        '-pedantic', '-I', str(t), str(SOURCE), '-o', str(exe)],
                       check=True, capture_output=True, text=True)
        # Avoid creating large core dumps when deliberately asserting failure.
        def no_core():
            import resource
            resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        result = subprocess.run([str(exe)], capture_output=True, text=True,
                                preexec_fn=no_core)
        assert (result.returncode == 0) == expect_success, (
            name, result.returncode, result.stdout, result.stderr)
        print(('PASS' if expect_success else 'PASS: mutation rejected') + ': ' + name)

def main():
    source = HEADER.read_text()
    check_run(source, 'actual production policy', True)
    for name, (before, after) in MUTATIONS.items():
        if source.count(before) != 1:
            raise AssertionError(f'{name}: expected unique mutation marker')
        check_run(source.replace(before, after, 1), name, False)
    print(f'PASS: {len(MUTATIONS)} deliberate defective variants rejected')

if __name__ == '__main__':
    try:
        main()
    except (AssertionError, OSError, subprocess.CalledProcessError) as exc:
        sys.exit('FAIL: supervision policy tests: ' + str(exc))
