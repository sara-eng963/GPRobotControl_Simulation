#!/usr/bin/env bash
# Stage 3 B5: one consolidated SOFTWARE-only B1–B5 acceptance gate.
# Source ABI, host tests, ARM compilation, independently built target ELFs.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || {
  echo "Usage: $0 <independent-CubeIDE-verifier>" >&2; exit 2;
}
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
    echo "FAIL: expected independent dual-core CubeIDE verifier" >&2; exit 1;
}
cd "$ROOT"
for name in \
  stage_m7_application_sources.sh stage_stage3_batch_b1.sh \
  stage_stage3_batch_b2.sh stage_stage3_batch_b3.sh \
  stage_stage3_batch_b4.sh stage_stage3_batch_b5.sh \
  check_stage3_batch_b1_acceptance.sh \
  check_stage3_batch_b2_acceptance.sh \
  check_stage3_batch_b3_acceptance.sh \
  check_stage3_batch_b4_acceptance.sh \
  check_stage3_batch_b5_acceptance.sh; do
    bash -n "Firmware/Tests/$name"
done
python3 - "$ROOT" "$DEST" <<'PY'
from pathlib import Path
from xml.etree import ElementTree as ET
import sys
repo, dest = map(Path, sys.argv[1:])
pairs = {
    'CM4/Core/Src/application_ipc_service.c':
      'Firmware/STM32H745/CM4/Core/Src/application_ipc_service.c',
    'CM4/Core/Inc/application_ipc_service.h':
      'Firmware/STM32H745/CM4/Core/Inc/application_ipc_service.h',
    'CM4/Application/Intercore/hmi_lifecycle.c':
      'Intercore/hmi_lifecycle.c',
    'CM4/Application/Intercore/hmi_lifecycle.h':
      'Intercore/hmi_lifecycle.h',
    'CM7/Application/StateMachine/supervisor_task.c':
      'StateMachine/supervisor_task.c',
}
for dst, src in pairs.items():
    a,b=repo/src,dest/dst
    if not a.is_file() or not b.is_file() or a.read_bytes()!=b.read_bytes():
        raise SystemExit(f'FAIL: B5 staged source stale/missing: {dst}')
m4=ET.parse(dest/'CM4/.cproject').getroot()
values=[e.get('value') for e in m4.iter('listOptionValue')]
assert values.count('../Application/Intercore')==4, (
    'FAIL: M4 Debug/Release C/ASM include roots do not cover HMI lifecycle'
)
print('PASS: B5 M7/M4 staged sources and CubeIDE HMI include paths')
PY

TEMP="$(mktemp -d)"
trap 'rm -rf "$TEMP"' EXIT
gcc -std=gnu11 -O0 -Wall -Wextra -Werror \
  Intercore/hmi_lifecycle.c Intercore/Tests/test_hmi_lifecycle.c \
  -o "$TEMP/hmi_lifecycle_test"
"$TEMP/hmi_lifecycle_test"
echo "PASS: B5 HMI command/state/freshness transport-neutral host regression"

# Full regression across all previously accepted batch contracts. Each
# existing gate includes its own source parity, host/ARM and ELF checks.
bash Firmware/Tests/check_stage3_batch_b1_acceptance.sh "$DEST"
bash Firmware/Tests/check_stage3_batch_b2_acceptance.sh "$DEST"
bash Firmware/Tests/check_stage3_batch_b3_acceptance.sh "$DEST"
bash Firmware/Tests/check_stage3_batch_b4_acceptance.sh "$DEST"

M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
[[ -s "$M7" && -s "$M4" ]] || {
    echo "FAIL: independent M7/M4 Debug ELF files missing" >&2; exit 1;
}
for rel in CM4/Core/Src/application_ipc_service.c \
           CM4/Application/Intercore/hmi_lifecycle.c; do
    [[ "$M4" -nt "$DEST/$rel" ]] || {
        echo "FAIL: M4 ELF predates B5 source $rel — clean rebuild needed" >&2; exit 1;
    }
done
[[ "$M7" -nt "$DEST/CM7/Application/StateMachine/supervisor_task.c" ]] || {
    echo "FAIL: M7 ELF predates new Supervisor HMI sequencing" >&2; exit 1;
}
arm-none-eabi-nm --defined-only "$M7" > "$TEMP/m7.nm"
arm-none-eabi-nm --defined-only "$M4" > "$TEMP/m4.nm"
strong() {
    local symbols="$1" name="$2"
    awk -v n="$name" '$2=="T" && $3==n {found=1} END {exit !found}' "$symbols" || {
        echo "FAIL: required strong firmware symbol missing: $name" >&2; exit 1;
    }
    echo "PASS: linked $name"
}
# Unused external API entry points can be removed by --gc-sections when
# no physical HMI provider is linked. Require only live, reachable
# application/runtime symbols in ELFs; the ARM object compilation above
# verifies public API definitions such as supervisor_task_post_hmi.
for symbol in gp_ipc_m7_receive gp_ipc_m7_send \
              application_supervisor_start; do
    strong "$TEMP/m7.nm" "$symbol"
done
for symbol in gp_ipc_m4_receive gp_ipc_m4_send \
              application_ipc_service_initialize \
              application_ipc_service_read_status \
              application_ipc_service_run \
              gp_hmi_status_decode gp_hmi_status_is_fresh \
              gp_hmi_command_allowed gp_hmi_should_drop_pending; do
    strong "$TEMP/m4.nm" "$symbol"
done
echo "PASS: Stage 3 Batch B5 consolidated B1-B5 software and dual-core ELF gates"
echo "DEFERRED: powered STM32, physical HMI transport, NOR persistence, motors, real 500 Hz jitter and safety certification"
