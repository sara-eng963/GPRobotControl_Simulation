#!/usr/bin/env bash
# B4 source staging: reuse B3's managed CubeIDE project wiring.
# Do not change CubeMX .ioc, HAL pins, clocks or physical flash configuration.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent-CubeIDE-verifier>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* ]] || {
    echo "FAIL: verifier cannot be the canonical Git checkout" >&2; exit 1;
}
for path in \
  Firmware/Tests/stage_m7_application_sources.sh \
  Firmware/Tests/stage_stage3_batch_b3.sh \
  Firmware/Tests/stage_stage3_batch_b4.sh; do
    bash -n "$ROOT/$path"
done
bash "$ROOT/Firmware/Tests/stage_stage3_batch_b3.sh" "$DEST"

# B3 staging now stages both canonical B4 service files into both cores.
# Check all required newly compiled translation units before handing to IDE.
for relative in \
  CM7/Application/Intercore/trajectory_storage_service.c \
  CM7/Application/Intercore/trajectory_storage_service.h \
  CM7/Application/Intercore/trajectory_stream_path_port.c \
  CM7/Application/StateMachine/States/state_path_execution.c \
  CM4/Application/Intercore/trajectory_storage_service.c \
  CM4/Application/Intercore/trajectory_storage_service.h \
  CM4/Core/Src/application_validation_service.c; do
    [[ -s "$DEST/$relative" ]] || {
        echo "FAIL: missing B4 staged source $relative" >&2; exit 1;
    }
done
echo "PASS: Stage B4 canonical M7 request/M4 producer staged; ready to CLEAN-BUILD both cores."
