#!/usr/bin/env python3
"""Production-source P04 scenarios via repo-owned Supervisor alignment builder.
Deterministic fake-clock scenarios; JSON is NOT a measured STM32 timing trace.
"""
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
RUNNER=ROOT/'Firmware/Tests/run_state_machine_alignment_tests.py'
EXPECTED=[f'P04-{i:02d}' for i in range(1,11)]+['P04-BOUNDARY']


def main():
    env=os.environ.copy()
    env['P04_FAST_ONLY']='1'
    result=subprocess.run([sys.executable,str(RUNNER)],
                          env=env,capture_output=True,text=True,timeout=180)
    log=result.stdout+'\n'+result.stderr
    found=re.findall(r'^P04_SCENARIO (P04-[\w-]+) PASS injected-time$',
                     log,re.M)
    success=(result.returncode==0 and len(found)==len(EXPECTED)
             and set(found)==set(EXPECTED) and len(set(found))==len(found))
    payload={
        'baseline':'916e7a85fd2d20f445fe513901f8d149260c09cb',
        'producer':'source-compiled production Supervisor fixture',
        'timing_origin':'deterministic injected CM7 clock values, not measured',
        'scheduling_origin':'host fake FreeRTOS tasks/queue, not real dual core',
        'status':'PASS' if success else 'FAIL',
        'scenarios':{name:('PASS' if name in found else 'FAIL')
                     for name in EXPECTED},
        'duration_inputs_us':{
            'wake_boundaries':[1999,2000,2001],
            'period':2000,
            'injected_service_gap':4000,
            'measurement':'test stimulus only; not firmware WCET'
        },
        'process_exit_code':result.returncode,
    }
    out=os.environ.get('P04_TRACE_FILE')
    if out:
        dest=Path(out);dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_text(json.dumps(payload,indent=2)+'\n')
    print(log, end='' if log.endswith('\n') else '\n')
    print(json.dumps(payload,indent=2))
    if not success:
        raise SystemExit('FAIL: scenario output, compilation or assertion mismatch')
    print('PASS: P04 deterministic integrated fixture (NOT hardware timing)')

if __name__=='__main__': main()
