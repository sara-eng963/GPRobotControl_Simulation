#!/usr/bin/env python3
"""Source mutations: compile MUST pass and P04 assertions MUST reject.
Creates a symlink mirror outside repo; never writes/commits/reset existing Git files.
"""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[2]
TARGET=Path('Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.c')
ALIGN=Path('Firmware/Tests/run_state_machine_alignment_tests.py')
TIMER=Path('Firmware/Tests/run_m7_timer_fault_injection.py')
BASE='916e7a85fd2d20f445fe513901f8d149260c09cb'

SUPERVISOR={
 'task_entry_early_callsite_bypass':(
     'if (hardware_timed &&\n            !supervisor_timing_pre_dispatch_check(context, cycle_start_us))',
     'if (false && hardware_timed &&\n            !supervisor_timing_pre_dispatch_check(context, cycle_start_us))'),
 'task_entry_predispatch_recheck_bypass':(
     'if (hardware_timed &&\n            !supervisor_timing_pre_dispatch_check(context, supervisor_now_us()))',
     'if (false && hardware_timed &&\n            !supervisor_timing_pre_dispatch_check(context, supervisor_now_us()))'),
 'late_dispatch_veto':('else if (a.late_completion)\n        detail = 0xE004U;',
                   'else if (false && a.late_completion)\n        detail = 0xE004U;'),
 'invalid_timing_veto':('if (!a.valid)\n        detail = 0xE019U;',
                        'if (false && !a.valid)\n        detail = 0xE019U;'),
 'severe_service_gap_veto':('else if (a.service_gap_severe || a.wake_gap_severe)',
                            'else if (false && (a.service_gap_severe || a.wake_gap_severe))'),
 'timing_fault_containment':('emit_step_failure(context, detail);\n    return false;\n}\n\nstatic void supervisor_task_entry',
                            '(void)detail;\n    return false;\n}\n\nstatic void supervisor_task_entry'),
 'peer_expiry_veto':('else if (!runtime_peer_fresh(&context->peer_evidence, now_ms))',
                     'else if (false && !runtime_peer_fresh(&context->peer_evidence, now_ms))'),
 'safety_expiry_veto':('if (!runtime_safety_fresh(&context->safety_evidence,',
                       'if (false && !runtime_safety_fresh(&context->safety_evidence,'),
 'queue_overload_veto':('if (result == SUPERVISOR_DRAIN_OVERLOAD &&',
                        'if (false && result == SUPERVISOR_DRAIN_OVERLOAD &&'),
 'can_feedback_veto':('!feedback.feedback_valid || !feedback.operation_enabled ||',
                      'false || !feedback.operation_enabled ||'),
}
TIMING={
 'missed_release_veto':('if (periods != 1U) {','if (false && periods != 1U) {'),
 'execution_deadline_veto':('if (elapsed >= UINT64_C(2000)) {',
                             'if (false && elapsed >= UINT64_C(2000)) {'),
}

def mirror(output,rel,mutable):
    src=ROOT/rel;dst=output/rel
    if rel==TARGET:
        dst.parent.mkdir(parents=True,exist_ok=True)
        dst.write_text(mutable);return
    if rel in (ALIGN,TIMER):
        dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dst);return
    if src.is_dir() and any(t.is_relative_to(rel) for t in (TARGET,ALIGN,TIMER)):
        dst.mkdir(parents=True,exist_ok=True)
        for child in src.iterdir(): mirror(output,rel/child.name,mutable)
    else:
        dst.parent.mkdir(parents=True,exist_ok=True)
        dst.symlink_to(src,target_is_directory=src.is_dir())

def execute(source,timer):
    with tempfile.TemporaryDirectory(prefix='p04-mutant-') as tmp:
        dst=Path(tmp)
        for child in ROOT.iterdir():
            if child.name=='.git':continue
            mirror(dst,Path(child.name),source)
        path=dst/(TIMER if timer else ALIGN)
        env=os.environ.copy()
        if not timer:env['P04_FAST_ONLY']='1'
        return subprocess.run([sys.executable,str(path)],env=env,
                              capture_output=True,text=True,timeout=240)

def ensure_compiled_assertion_failure(name, result, timer):
    out=result.stdout+'\n'+result.stderr
    required='Assertion' if timer else 'P04_ASSERT_FAIL'
    if result.returncode==0 or required not in out:
        raise RuntimeError(name+': mutant survived or did not reach behavioral assertion\n'+out[-3500:])
    if ('error:' in out or 'COMPILE FAIL' in out or
        'No such file' in out or 'undefined reference' in out or
        'Traceback' in out and required not in out):
        raise RuntimeError(name+': compile/infrastructure failure is NOT mutation rejection\n'+out[-3500:])
    print('REJECTED (compiled, behavioral assertion): '+name,flush=True)

def main():
    original=(ROOT/TARGET).read_text()
    for label,timer in [('integrated',False),('timer_extraction',True)]:
        baseline=execute(original,timer)
        if baseline.returncode:
            raise RuntimeError('BASELINE '+label+' FAIL\n'+baseline.stdout+baseline.stderr)
        if not timer and 'P04_SCENARIO P04-10 PASS' not in baseline.stdout:
            raise RuntimeError('P04 fixture missing from baseline test')
        print('PASS: P04 '+label+' baseline',flush=True)
    for timer,mutations in [(False,SUPERVISOR),(True,TIMING)]:
        for name,(good,bad) in mutations.items():
            if original.count(good)!=1:
                raise RuntimeError(name+': unique exact production anchor missing')
            result=execute(original.replace(good,bad),timer)
            ensure_compiled_assertion_failure(name,result,timer)
    print('PASS: '+str(len(SUPERVISOR)+len(TIMING))+
          ' compiled negative P04 source mutations rejected')

if __name__=='__main__':main()
