#!/usr/bin/env bash
# Mirror canonical FreeRTOS IPC/trajectory source files into the separate
# STM32CubeIDE verification workspace. The source of truth stays in Intercore/.
# Regenerate staged files when the canonical sources change.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
if [[ $# -ne 1 ]]; then
    echo "Usage: bash Firmware/Tests/stage_intercore_sources.sh <CubeIDE-project-root>" >&2
    exit 2
fi
DEST="$1"
[[ -f "$DEST/GP_STM32H745.ioc" ]] || {
    echo "FAIL: not an STM32H745 CubeIDE verification project: $DEST" >&2
    exit 1
}
CM7="$DEST/CM7/Core/Src"
[[ -f "$CM7/intercore_shared_memory.c" ]] || {
    echo "FAIL: sync Firmware/STM32H745 to the verifier before staging sources" >&2
    exit 1
}
for name in ipc_mailbox_freertos.c trajectory_stream.c; do
    cp "$ROOT/Intercore/$name" "$CM7/$name"
    cmp -s "$ROOT/Intercore/$name" "$CM7/$name" || {
        echo "FAIL: staged copy diverged: $name" >&2
        exit 1
    }
    echo "PASS: staged canonical CM7 source $name"
done
echo "NOTE: do not compile Intercore/ipc_mailbox.c in the CM7 target."
echo "NOTE: these are build inputs for the separate verifier, not forked sources."
