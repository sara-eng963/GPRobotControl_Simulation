#!/usr/bin/env bash
# Stage C2: only the two changed M7 CANopen sources. No generated ST edits.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent CubeIDE project>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: expected independent CubeIDE repro" >&2; exit 1;
}
for rel in CANComm/CANopen/canopen_master.c CANComm/CANopen/canopen_master.h; do
  src="$ROOT/$rel"
  target="$DEST/CM7/Application/$rel"
  [[ -f "$src" && -f "$target" ]] || {
    echo "FAIL: missing source or staged file: $rel" >&2; exit 1;
  }
done
for rel in CANComm/CANopen/canopen_master.c CANComm/CANopen/canopen_master.h; do
  cp "$ROOT/$rel" "$DEST/CM7/Application/$rel"
  cmp -s "$ROOT/$rel" "$DEST/CM7/Application/$rel"
  echo "STAGED: CM7/Application/$rel"
done
echo "PASS: C2 bounded CANopen source staged; generated MCU and M4 unchanged"
echo "NEXT: Refresh and CLEAN-BUILD CM7 Debug in STM32CubeIDE."
