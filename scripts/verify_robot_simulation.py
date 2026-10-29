#!/usr/bin/env python3
"""Execute software integration/regression gates; never a zero-force smoke test.

No native Windows MATLAB graphics are claimed. MATLAB process supervision is
contract-tested here; regular interactive startup performs the native handshake.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_TESTS = {'robot_production_silkit_sequence', 'robot_production_virtual_ft',
                  'robot_launcher_contracts', 'robot_windows_matlab_contracts',
                  'robot_verification_contracts', 'robot_installer_contracts', 'renode_qspi_jedec_test',
                  'renode_qspi_rw_test', 'renode_qspi_pv_sample_test',
                  'renode_qspi_storage_backend_test', 'renode_qspi_streaming_test'}


def record(results, output, label, argv, timeout=3600):
    started = time.monotonic()
    try:
        with (output / (label + '.log')).open('w') as log:
            completed = subprocess.run(argv, cwd=ROOT, stdout=log,
                                       stderr=subprocess.STDOUT, timeout=timeout)
        code = completed.returncode
    except (OSError, subprocess.TimeoutExpired) as error:
        with (output / (label + '.log')).open('a') as log:
            log.write('\n' + str(error) + '\n')
        code = -1
    results[label] = {'status': 'PASS' if code == 0 else 'FAIL', 'exit_code': code,
                      'argv': list(map(str, argv)), 'wall_seconds': round(time.monotonic()-started, 3)}
    (output / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(f'{label}: {results[label]["status"]}; log: {output / (label + ".log")}', flush=True)
    return code == 0


def registered_tests(build):
    result = subprocess.run(['ctest', '--test-dir', str(build), '--show-only=json-v1'],
                            check=True, text=True, capture_output=True)
    return {test['name'] for test in json.loads(result.stdout)['tests']}


def execute(output, build, sdk):
    results = {}
    built = record(results, output, 'configure', ['cmake', '-S', str(ROOT), '-B', str(build),
        '-DCMAKE_BUILD_TYPE=Debug', '-DENABLE_SILKIT=ON', '-DENABLE_RENODE_TESTS=ON',
        '-DSILKIT_ROOT=' + str(sdk)])
    if built:
        built = record(results, output, 'build_all', ['cmake', '--build', str(build),
                         '--parallel', os.environ.get('BUILD_JOBS', '4')])
    if built:
        try:
            names = registered_tests(build)
            missing = REQUIRED_TESTS - names
            if missing:
                raise RuntimeError('Required tests not registered: ' + ', '.join(sorted(missing)))
            (output / 'registered-tests.json').write_text(json.dumps(sorted(names), indent=2)+'\n')
            results['required_test_registration'] = {'status': 'PASS', 'count': len(names)}
        except (RuntimeError, subprocess.SubprocessError, ValueError) as error:
            built = False
            results['required_test_registration'] = {'status': 'FAIL', 'reason': str(error)}
    if built:
        # All registered regressions, including legacy failures, affect exit status.
        record(results, output, 'all_ctest', ['ctest', '--test-dir', str(build),
            '--output-on-failure', '--no-tests=error', '--parallel', '2'])
    else:
        results['all_ctest'] = {'status': 'NOT RUN', 'reason': 'Build or required registration failed'}
    # Retain independent mutations, ARM builds and P05 evidence, even if CTest failed.
    record(results, output, 'p05_acceptance', [sys.executable,
        str(ROOT/'Firmware/Tests/run_p05_acceptance.py'), '--output', str(output/'p05')])
    record(results, output, 'git_diff_check', ['git', 'diff', '--check'])
    failed = [label for label, result in results.items() if result['status'] != 'PASS']
    summary = {'scope': 'software integration and all registered regressions',
               'zero_force_smoke_only': False, 'physical_hardware_tested': False,
               'native_windows_matlab_tested': False,
               'status': 'FAIL' if failed else 'PASS', 'failed_or_not_run': failed,
               'results': results}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print('VERIFICATION ' + summary['status'] + (': ' + ', '.join(failed) if failed else ''), flush=True)
    return 1 if failed else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify', action='store_true', required=True)
    parser.add_argument('--output', type=Path, help='new external evidence directory')
    ns = parser.parse_args()
    sdk = Path(os.environ.get('SILKIT_ROOT', str(Path.home()/'silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit'))).resolve()
    required = ('cmake', 'ctest', 'cc', 'c++', 'arm-none-eabi-gcc', 'renode')
    missing = [name for name in required if not shutil.which(name)]
    if missing or not (sdk/'bin/sil-kit-registry').is_file():
        parser.error('Verification dependencies missing: ' + ', '.join(missing) +
                     '; SILKIT_ROOT must contain bin/sil-kit-registry. See docs/UNIFIED_ROBOT_SIMULATION.md')
    if ns.output:
        output = ns.output.resolve()
        if output == ROOT or ROOT in output.parents:
            parser.error('--output must be outside the source checkout')
        output.mkdir(parents=True, exist_ok=False)
    else:
        output = Path(tempfile.mkdtemp(prefix='robot-verification-'))
    build = Path(os.environ.get('BUILD_DIR', str(ROOT/'build/robot-simulation'))).resolve()
    print('SOFTWARE VERIFICATION; evidence:', output, flush=True)
    print('Includes nonzero-force teaching, QSPI replay, faults, MATLAB UDP, all CTests and P05 gates.', flush=True)
    return execute(output, build, sdk)


if __name__ == '__main__':
    sys.exit(main())
