#!/usr/bin/env bash
# Synchronize only the canonical STM32H745 CM7 timing integration and
# Supervisor application sources into the existing, separate CubeIDE verifier.
# No backup/CubeMX generation changes; does not build or write to GitHub.
set -euo pipefail
[[ $# -eq 1 ]] || {
    echo "Usage: $0 <CubeIDE-verifier-project-root>" >&2
    exit 2
}
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
DEST="$1"
[[ -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" ]] || {
    echo "FAIL: not a valid dual-core CubeIDE verifier" >&2
    exit 1
}
python3 "$REPO/Firmware/Tests/check_tim2_tim3_integration.py"
for path in \
    CM7/Core/Src/main.c \
    CM7/Core/Src/application_supervisor.c \
    CM7/Core/Inc/application_supervisor.h; do
    canonical="$REPO/Firmware/STM32H745/$path"
    staged="$DEST/$path"
    [[ -f "$staged" ]] || {
        echo "FAIL: required existing verifier file missing: $staged" >&2
        exit 1
    }
    cp "$canonical" "$staged"
    cmp -s "$canonical" "$staged" || {
        echo "FAIL: target staging mismatch: $path" >&2
        exit 1
    }
    echo "PASS: staged $path"
done
bash "$REPO/Firmware/Tests/stage_m7_application_sources.sh" "$DEST"
echo "PASS: CM7 HAL timer callbacks and canonical Supervisor sources staged."
echo "NOTE: refresh and CLEAN-BUILD BOTH cores; report compiler/linker diagnostics."
