#!/usr/bin/env python3
"""Fail closed: a nominal host simulation is not final P05 software acceptance."""
import json
import sys
from pathlib import Path

if len(sys.argv)!=2: raise SystemExit('usage: check_p05_integrated_evidence.py EVIDENCE_DIR')
root=Path(sys.argv[1])
summary=root/'summary.json'
if not summary.is_file(): raise SystemExit('P05_FINAL_FAIL: missing summary.json')
data=json.loads(summary.read_text())
required={
 'runner_status':'PASS',
 'full_p05_acceptance':True,
 'qspi_end_to_end':'PASS',
 'p04_production_supervisor_in_chain':'PASS',
 'physical_hardware_tested':False,
}
missing=[k for k,v in required.items() if data.get(k)!=v]
if missing:
    raise SystemExit('P05_FINAL_FAIL: unresolved '+','.join(missing))
proof=data.get('integration')
if not isinstance(proof,dict):raise SystemExit('P05_FINAL_FAIL: no integration proof')
for k in ('qspi_proven','supervisor_production_invoked'):
    if proof.get(k) is not True:raise SystemExit('P05_FINAL_FAIL: '+k+' != true')
if proof.get('result')!='PASS':raise SystemExit('P05_FINAL_FAIL: integrated run not PASS')
for name in ('joint_trace.csv','cartesian_trace.csv','trajectory_compare.csv',
             'faults.csv','qspi_readback.json','production_supervisor_trace.json'):
    if not (root/name).is_file():
        raise SystemExit('P05_FINAL_FAIL: missing evidence '+name)
q=json.loads((root/'qspi_readback.json').read_text())
s=json.loads((root/'production_supervisor_trace.json').read_text())
for k in ('same_artifact','same_samples','crc_verified','read_from_simulated_flash'):
    if q.get(k) is not True:raise SystemExit('P05_FINAL_FAIL: QSPI '+k)
if s.get('production_supervisor_executed') is not True:
    raise SystemExit('P05_FINAL_FAIL: production Supervisor missing')
# A separately passing production fixture cannot stand in for live dispatch.
for key in ('same_live_canopen_process','task_entry_executed'):
    if s.get(key) is not True:
        raise SystemExit('P05_FINAL_FAIL: production Supervisor '+key)
scenarios=('revoked_motor_authorization','missing_drive_feedback',
           'stale_drive_feedback','cm4_liveness_failure',
           'missed_control_deadline','latched_fault','can_tx_failure')
for name in scenarios:
    case=s.get('scenarios',{}).get(name,{})
    expected={'status':'PASS','motion_denied':True,'restart_denied':True,
              'permission':False,'inhibited':True,
              'denied_rpdo_frames':0,'denied_syncs':0}
    if any(case.get(k)!=v for k,v in expected.items()) or not case.get('fault_code'):
        raise SystemExit('P05_FINAL_FAIL: in-chain scenario '+name)
if not (q.get('sample_count',0)>0 and
        q['sample_count']==proof.get('validated_samples')==
        proof.get('stream_replayed_samples')):
    raise SystemExit('P05_FINAL_FAIL: playback sample count mismatch')
# Cross-check executable trace rows against committed readback, not only flags.
import csv
with (root/'trajectory_compare.csv').open() as trace:
    rows=list(csv.DictReader(trace))
if (len(rows)!=q['sample_count'] or
    any(int(row['cycle'])!=index or row['all_six_equal']!='1' or
        row['axis1_committed']!=row['axis1_replayed']
        for index,row in enumerate(rows))):
    raise SystemExit('P05_FINAL_FAIL: durable playback comparison mismatch')
print('PASS: complete P05 software-only acceptance; physical commissioning not claimed')
