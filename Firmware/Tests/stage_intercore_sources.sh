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
for core in CM7 CM4; do
    target="$DEST/$core/Core/Src"
    if [[ "$core" == CM7 ]]; then
        owner=intercore_shared_memory.c
    else
        owner=intercore_startup_validation.c
    fi
    [[ -f "$target/$owner" ]] || {
        echo "FAIL: sync Firmware/STM32H745 $core into verifier first" >&2
        exit 1
    }
    for name in ipc_mailbox_freertos.c trajectory_stream.c; do
        cp "$ROOT/Intercore/$name" "$target/$name"
        cmp -s "$ROOT/Intercore/$name" "$target/$name" || {
            echo "FAIL: staged copy diverged: $core/$name" >&2
            exit 1
        }
        echo "PASS: staged canonical $core source $name"
    done
done
echo "NOTE: never compile host-only Intercore/ipc_mailbox.c in either firmware target."
echo "NOTE: these are build inputs for the separate verifier, not forked sources."
