#!/usr/bin/env python3
"""Negative executable test: missing QSPI or Supervisor cannot pass acceptance."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
helper=Path(__file__).with_name('check_p05_integrated_evidence.py')
with tempfile.TemporaryDirectory(prefix='p05-acceptance-gate-') as td:
    d=Path(td)
    for mode in ('nominal_host_only','supervisor_only','qspi_only','fake_flags'):
        summary={'runner_status':'PASS','physical_hardware_tested':False,
                 'full_p05_acceptance': mode=='fake_flags',
                 'qspi_end_to_end':'PASS' if mode in ('qspi_only','fake_flags') else 'NOT RUN',
                 'p04_production_supervisor_in_chain': 'PASS' if mode in ('supervisor_only','fake_flags') else 'NOT RUN',
                 'integration':{'result':'PASS','qspi_proven': mode in ('qspi_only','fake_flags'),
                                'supervisor_production_invoked':mode in ('supervisor_only','fake_flags')}}
        (d/'summary.json').write_text(json.dumps(summary))
        proc=subprocess.run([sys.executable,str(helper),str(d)],capture_output=True,text=True)
        if proc.returncode==0 or 'P05_FINAL_FAIL' not in (proc.stderr+proc.stdout):
            raise SystemExit('Acceptance false positive: '+mode)
print('PASS: incomplete host/production/QSPI permutations all rejected')
