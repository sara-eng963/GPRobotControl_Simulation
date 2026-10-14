#!/usr/bin/env bash
# C3 release build checkpoint. Never rebuild/modify generated MCU code here.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent-CubeIDE-verifier>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
    echo "FAIL: independent two-core CubeIDE verifier required" >&2; exit 1;
}
cd "$ROOT"
[[ "$(git branch --show-current)" == "sara/can-replacement" ]] || {
  echo "FAIL: switch to sara/can-replacement before C3 qualification" >&2; exit 1;
}
git diff --quiet HEAD -- && git diff --cached --quiet -- || {
  echo "FAIL: tracked source changes present; commit/review before release" >&2; exit 1;
}
# Source ancestry establishes accepted historical baselines without
# confusing their old SHA with the new candidate SHA.
for accepted in \
  26217a6ce44bdd1b941edefcdb1bee98ae54e003 \
  ee6783f57fd7c9fa18c32dcba664fa4297d1bd1e \
  9c0dd0d3f02240c663f4fa415121be7c75c12edd; do
  git merge-base --is-ancestor "$accepted" HEAD || {
    echo "FAIL: source has diverged from accepted software revision $accepted" >&2
    exit 1
  }
done
# Prevent an old, pre-C2 M7 image from becoming a C3 candidate.
for p in CANComm/CANopen/canopen_master.c CANComm/CANopen/canopen_master.h; do
  cmp -s "$ROOT/$p" "$DEST/CM7/Application/$p" || {
    echo "FAIL: staging mismatch in $p; run C2 staging BEFORE preparing C3" >&2
    exit 1
  }
done
OUT_PARENT="$ROOT/.simulation-logs/stage-c3"
mkdir -p "$OUT_PARENT"
PREP="$(mktemp -d "$OUT_PARENT/prep-XXXXXXXX")"
{
  git rev-parse HEAD
  printf '%s\n' "$DEST"
} > "$PREP/context.txt"
date -u +%Y-%m-%dT%H:%M:%SZ > "$PREP/start_utc.txt"
# This mtime MUST predate actual, new linked M7/M4 binaries and their maps.
touch "$PREP/rebuild_after_this.marker"
echo "PASS: C3 frozen source revision $(git rev-parse --short HEAD)"
echo "PREP DIRECTORY: $PREP"
echo "NEXT: STM32CubeIDE Refresh, Project > Clean both CM7/CM4, then Build both Debug targets."
echo "Run check_stage3_c3_release.sh with THIS PREP DIRECTORY after both builds."
