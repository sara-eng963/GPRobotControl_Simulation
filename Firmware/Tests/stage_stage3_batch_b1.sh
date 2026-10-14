#!/usr/bin/env bash
# Stage only the M7 B1 integration files into an independent CubeIDE verifier.
# Generated CubeMX sources/pins/clock configurations remain untouched.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent-CubeIDE-verifier>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* ]] || {
  echo "FAIL: destination cannot be the canonical Git checkout" >&2; exit 1;
}
[[ -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: dual-core CubeIDE verifier not found" >&2; exit 1;
}
source_files=(
  "Firmware/STM32H745/CM7/Core/Src/application_supervisor.c"
  "Intercore/trajectory_stream_path_port.c"
  "Intercore/trajectory_stream_path_port.h"
)
target_files=(
  "CM7/Core/Src/application_supervisor.c"
  "CM7/Application/Intercore/trajectory_stream_path_port.c"
  "CM7/Application/Intercore/trajectory_stream_path_port.h"
)
# Preflight all inputs before changing the verifier.
for i in "${!source_files[@]}"; do
  [[ -f "$ROOT/${source_files[$i]}" && -f "$DEST/${target_files[$i]}" ]] || {
    echo "FAIL: missing source or staged destination: ${target_files[$i]}" >&2
    exit 1
  }
done
for i in "${!source_files[@]}"; do
  src="$ROOT/${source_files[$i]}"
  dst="$DEST/${target_files[$i]}"
  if cmp -s "$src" "$dst"; then
    echo "UNCHANGED: ${target_files[$i]}"
  else
    cp "$src" "$dst"
    cmp -s "$src" "$dst"
    echo "STAGED: ${target_files[$i]}"
  fi
done
echo "PASS: Batch B1 M7 application and stream port staged."
echo "Clean-build CM7 and CM4 in CubeIDE; this script does not build the ELFs."
