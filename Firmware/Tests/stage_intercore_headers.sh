#!/usr/bin/env bash
# Stage canonical IPC/trajectory headers into a separate STM32CubeIDE
# verification project. Do not hand-edit these generated copies.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
if [[ $# -ne 1 ]]; then
    echo "Usage: bash Firmware/Tests/stage_intercore_headers.sh <CubeIDE-project-root>" >&2
    exit 2
fi
DEST="$1"
[[ -f "$DEST/GP_STM32H745.ioc" ]] || {
    echo "FAIL: not a CubeIDE STM32H745 project directory: $DEST" >&2; exit 1;
}
# Only files needed by the first production M7 allocation source. The
# canonical source remains under Intercore/ in the Git repository.
for core in CM7 CM4; do
    dir="$DEST/$core/Core/Inc"
    [[ -d "$dir" ]] || { echo "FAIL: missing $dir" >&2; exit 1; }
    for name in ipc_mailbox.h trajectory_stream.h; do
        cp "$ROOT/Intercore/$name" "$dir/$name"
        cmp -s "$ROOT/Intercore/$name" "$dir/$name" || {
            echo "FAIL: staged copy diverged: $dir/$name" >&2; exit 1;
        }
    done
    if [[ "$core" == CM4 ]]; then
        cp "$ROOT/StateMachine/supervisor_hmi.h" "$dir/supervisor_hmi.h"
        cmp -s "$ROOT/StateMachine/supervisor_hmi.h" "$dir/supervisor_hmi.h" || {
            echo "FAIL: staged M4 HMI command ABI diverged" >&2; exit 1;
        }
    fi
    echo "PASS: staged canonical Intercore headers for $core"
done
echo "NOTE: source-of-truth headers are Intercore/*.h; regenerate these copies."
