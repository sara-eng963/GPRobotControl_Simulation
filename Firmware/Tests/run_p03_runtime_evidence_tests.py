#!/usr/bin/env python3
"""Execute production P03 header with actual repo types and reject negative mutants.
Each mutant must COMPILE and FAIL an executable assertion to count rejected.
"""
from pathlib import Path
import os, subprocess, tempfile
ROOT=Path(__file__).resolve().parents[2]
HEADER=ROOT/'Firmware/STM32H745/Common/Inc/runtime_evidence.h'
TEST=ROOT/'Firmware/Tests/test_p03_runtime_evidence.c'
INC=ROOT/'Firmware/STM32H745/Common/Inc'
SM=ROOT/'Firmware/STM32H745/CM7/Application/StateMachine'
MUTATIONS={
 'safety_duplicate': ('delta != 0U && delta < UINT32_C(0x80000000)',
                      'delta < UINT32_C(0x80000000)'),
 'safety_future': ('age < UINT32_C(0x80000000) && age <= limit',
                   '(age <= limit || age >= UINT32_C(0x80000000))'),
 'expiry_boundary': ('age <= limit;', 'age <= limit + 1U;'),
 'safety_session': ('session != state->producer_session',
                    'session == state->producer_session'),
 'safety_timestamp_order': ('!runtime_forward(sample->timestampMs, state->last_source_ms)',
                            'runtime_forward(sample->timestampMs, state->last_source_ms)'),
 'safety_contradiction': ('sample->motionPermitted && (sample->estopActive ||',
                           'false && (sample->estopActive ||'),
 'hello_required': ('!state->hello_seen ||\n        state->compromised) return false;', 'false ||\n        state->compromised) return false;'),
 'hello_reset_latch': ('if (state->hello_seen || state->compromised)',
                       'if (!state->hello_seen && state->compromised)'),
 'challenge_roundtrip': ('!runtime_age_valid(now_ms,state->challenge_issued_ms,\n                           RUNTIME_CHALLENGE_MAX_RTT_MS) ||',
                         'false ||'),
 'challenge_match': ('msg->data[6]!=state->challenge_pending',
                     'msg->data[6]==state->challenge_pending'),
 'peer_sequence': ('if (state->have_sequence &&\n        !runtime_forward(msg->sequence,state->last_sequence)) return false;',
                   'if (false) return false;'),
 'peer_session': ('msg->type!=IPC_CM4_PROGRESS ||\n        msg->data[0]!=state->expected_session',
                  'msg->type!=IPC_CM4_PROGRESS ||\n        msg->data[0]==state->expected_session'),
 'peer_completion': ('msg->data[3]!=1U', 'msg->data[3]==1U'),
 'peer_age': ('runtime_age_valid(now_ms,state->last_response_ms,\n                          RUNTIME_PEER_MAX_AGE_MS)',
              'runtime_age_valid(now_ms,state->last_response_ms,\n                          RUNTIME_PEER_MAX_AGE_MS+100U)'),
}
def run(source,tmp,mutant=False):
    output=tmp/'test'
    cc=os.environ.get('CC','cc')
    compile=subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror',
        '-I'+str(tmp),'-I'+str(INC),'-I'+str(SM),str(source),'-o',str(output)],
        capture_output=True,text=True,timeout=30)
    if compile.returncode:
        raise RuntimeError('COMPILE FAIL (not a test rejection):\n'+compile.stderr)
    executed=subprocess.run([str(output)],capture_output=True,text=True,timeout=10)
    if mutant:
        if executed.returncode == 0 or 'Assertion' not in executed.stderr:
            raise RuntimeError('MUTATION NOT REJECTED BY ASSERTION: '+executed.stdout+executed.stderr)
    elif executed.returncode:
        raise RuntimeError('TEST FAIL:\n'+executed.stdout+executed.stderr)
with tempfile.TemporaryDirectory(prefix='p03-runtime-evidence-') as d:
    tmp=Path(d)
    text=HEADER.read_text()
    (tmp/'test.c').write_text(TEST.read_text())
    (tmp/'runtime_evidence.h').write_text(text)
    run(tmp/'test.c',tmp)
    print('PASS: production P03 policy tests',flush=True)
    for name,(original,defective) in MUTATIONS.items():
        if text.count(original)!=1:
            raise RuntimeError(f'MUTATION ANCHOR invalid {name}, count={text.count(original)}')
        (tmp/'runtime_evidence.h').write_text(text.replace(original,defective))
        run(tmp/'test.c',tmp,mutant=True)
        print('PASS: rejected mutation '+name,flush=True)
        (tmp/'runtime_evidence.h').write_text(text)
