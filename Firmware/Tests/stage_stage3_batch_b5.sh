#!/usr/bin/env bash
# Stage 3 B5: mirror canonical M4 HMI lifecycle and M7 Supervisor files.
# Keep CubeMX-generated pins/clock/HAL and physical HMI transport untouched.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent-CubeIDE-verifier>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" &&
   -f "$DEST/CM7/.project" && -f "$DEST/CM4/.project" ]] || {
    echo "FAIL: expected independent dual-core CubeIDE verifier" >&2; exit 1;
}
for script in stage_m7_application_sources.sh stage_stage3_batch_b3.sh \
              stage_stage3_batch_b4.sh stage_stage3_batch_b5.sh; do
    bash -n "$ROOT/Firmware/Tests/$script"
done
# B4 staging already mirrors canonical M7 Supervisor and M4 Intercore sources.
bash "$ROOT/Firmware/Tests/stage_stage3_batch_b4.sh" "$DEST"
for name in application_ipc_service.c; do
    cp "$ROOT/Firmware/STM32H745/CM4/Core/Src/$name" \
       "$DEST/CM4/Core/Src/$name"
done
for name in application_ipc_service.h; do
    cp "$ROOT/Firmware/STM32H745/CM4/Core/Inc/$name" \
       "$DEST/CM4/Core/Inc/$name"
done
for rel in \
    CM4/Core/Src/application_ipc_service.c \
    CM4/Core/Inc/application_ipc_service.h \
    CM4/Application/Intercore/hmi_lifecycle.c \
    CM4/Application/Intercore/hmi_lifecycle.h \
    CM7/Application/StateMachine/supervisor_task.c; do
    [[ -s "$DEST/$rel" ]] || {
        echo "FAIL: B5 missing staged file: $rel" >&2; exit 1;
    }
    echo "PASS: B5 staged $rel"
done
echo "PASS: Stage 3 B5 HMI lifecycle and Supervisor sources staged."
echo "NEXT: Refresh, clean-build CM7 and CM4 independently in CubeIDE."
