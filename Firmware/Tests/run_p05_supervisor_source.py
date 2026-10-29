#!/usr/bin/env python3
"""Compile and run P05 assertions against real STM32 CM7 Supervisor source.

This uses the P04 alignment compiler, production translation units and
injected timing/devices. It is not the physical firmware scheduler.
"""
import os
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,default=None)
args=parser.parse_args()
required = ('feedback_loss','cm4_lost_and_recovered',
            'deadline_and_no_reauth','real_task_entry')
env=os.environ.copy()
env.pop('P04_FAST_ONLY',None)
env['P05_SUPERVISOR_ONLY']='1'
p=subprocess.run([sys.executable,str(ROOT/'Firmware/Tests/run_state_machine_alignment_tests.py')],
                 cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                 text=True,timeout=300)
print(p.stdout,flush=True)
passed=set(re.findall(r'^P05_SUPERVISOR ([a-z0-9_]+) PASS$',p.stdout,re.M))
if p.returncode or passed!=set(required):
    raise SystemExit('P05 production Supervisor FAIL: '+repr(sorted(set(required)-passed)))
if args.output is not None:
    if args.output.exists(): raise SystemExit('refusing to overwrite production evidence')
    args.output.write_text(json.dumps({
        'production_supervisor_executed':True,
        'fixture':'actual STM32 CM7 supervisor_task.c with P04 production harness',
        'time_domain':'deterministic injected host time, not physical STM32',
        'scenarios':{scenario:'PASS' for scenario in required},
        'same_live_canopen_process':False,
    },indent=2)+'\n')
print('PASS: actual production Supervisor compiled P05 safety contracts')
