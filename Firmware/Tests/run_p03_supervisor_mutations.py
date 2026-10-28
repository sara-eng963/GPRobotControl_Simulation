#!/usr/bin/env python3
"""Run negative mutations on ACTUAL production CM7 Supervisor TU without
modifying repository. Mirrors repo with symlinks except mutated one file and
copied alignment runner. WSL/Linux only. Compile failures are NOT accepted as
negative-test detection. No Git changes/commits/build cleanup.
"""
from pathlib import Path
import shutil, os, subprocess, tempfile
ROOT=Path(__file__).resolve().parents[2]
TARGET=Path('Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.c')
RUNNER=Path('Firmware/Tests/run_state_machine_alignment_tests.py')
original=(ROOT/TARGET).read_text()
mutations={
 'omit_safety_expiry':(
    'if (!runtime_safety_fresh(&context->safety_evidence,',
    'if (false && !runtime_safety_fresh(&context->safety_evidence,'),
 'omit_peer_expiry':(
    'else if (!runtime_peer_fresh(&context->peer_evidence, now_ms))',
    'else if (false && !runtime_peer_fresh(&context->peer_evidence, now_ms))'),
 'permit_old_HMI_session':(
    'msg.data[1] != context->peer_evidence.expected_session ||',
    'false ||'),
 'permit_bad_challenge_publication':(
    'msg.data[9] = challenge;', 'msg.data[9] = 0U;'),
}
# The mirror is a fresh tempfile. Only TEST and production source are copied;
# everything else is symlinked. No branch/checkout mutation happens.
def make_mirror(dst, relative, mutable):
    src=ROOT/relative
    output=dst/relative
    if relative==TARGET:
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(mutable)
        return
    if relative==RUNNER:
        output.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,output)
        return
    if src.is_dir() and (TARGET.is_relative_to(relative) or
                         RUNNER.is_relative_to(relative)):
        output.mkdir(parents=True,exist_ok=True)
        for child in src.iterdir():
            make_mirror(dst,relative/child.name,mutable)
    else:
        output.parent.mkdir(parents=True,exist_ok=True)
        output.symlink_to(src,target_is_directory=src.is_dir())

def execute(source):
    with tempfile.TemporaryDirectory(prefix='p03-supervisor-mutant-') as name:
        dst=Path(name)
        for root_entry in ROOT.iterdir():
            if root_entry.name=='.git': continue
            make_mirror(dst,Path(root_entry.name),source)
        cmd=[os.environ.get('PYTHON','python3'),str(dst/RUNNER)]
        return subprocess.run(cmd,capture_output=True,text=True,timeout=900)

if __name__=='__main__':
    baseline=execute(original)
    if baseline.returncode:
        raise RuntimeError('BASELINE PRODUCTION TEST FAILED:\n'+baseline.stdout+baseline.stderr)
    print('PASS: unmutated production Supervisor fixture',flush=True)
    for name,(correct,bad) in mutations.items():
        if original.count(correct)!=1: raise RuntimeError('Mutation anchor invalid: '+name)
        result=execute(original.replace(correct,bad))
        if result.returncode==0 or 'FAIL' not in result.stdout+result.stderr:
            raise RuntimeError('MUTATION SURVIVED / NON-ASSERTION FAILURE '+name+'\n'+result.stderr)
        # A compile error is not evidence of a behavioral negative test.
        if 'COMPILE FAIL' in result.stderr or 'error:' in result.stderr:
            raise RuntimeError('Mutation compile failure NOT accepted '+name+'\n'+result.stderr)
        print('PASS: rejected production Supervisor mutation '+name,flush=True)
