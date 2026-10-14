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
  "StateMachine/States/state_path_execution.c"
)
target_files=(
  "CM7/Core/Src/application_supervisor.c"
  "CM7/Application/Intercore/trajectory_stream_path_port.c"
  "CM7/Application/Intercore/trajectory_stream_path_port.h"
  "CM7/Application/StateMachine/States/state_path_execution.c"
)
# B1 requires the project-managed include path for the canonical Intercore
# header: application_supervisor.c lives in CM7/Core/Src, whereas the header
# lives in CM7/Application/Intercore. Preserve unrelated CubeIDE settings.
# The tracked .cproject already carries this setting for Debug/Release C+ASM.
[[ -f "$DEST/CM7/.cproject" ]] || { echo "FAIL: missing M7 .cproject"; exit 1; }
python3 - "$DEST/CM7/.cproject" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
file = Path(sys.argv[1])
text = file.read_text()
anchor = '<listOptionValue builtIn="false" value="../Application/StateMachine"/>'
target = '<listOptionValue builtIn="false" value="../Application/Intercore"/>'
# Do not partially rewrite a changed/user-maintained project configuration.
old = text.count(anchor)
existing = text.count(target)
if old != 4 or existing not in (0, 4):
    raise SystemExit(f"FAIL: unexpected CM7 include-path shape: StateMachine={old} Intercore={existing}")
if existing == 0:
    lines = text.splitlines(keepends=True)
    output = []
    for line in lines:
        output.append(line)
        if anchor in line:
            indentation = line[:len(line)-len(line.lstrip())]
            eol = '\r\n' if line.endswith('\r\n') else '\n'
            output.append(indentation + target + eol)
    proposed = ''.join(output)
    ET.fromstring(proposed)
    file.write_bytes(proposed.encode())
    print("STAGED: CM7 .cproject Intercore include path (Debug/Release C and ASM)")
else:
    ET.fromstring(text)
    print("UNCHANGED: CM7 .cproject Intercore include path already configured")
PY
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
echo "PASS: Batch B1 M7 application, stream port and runtime fault path staged."
echo "Clean-build CM7 and CM4 in CubeIDE; this script does not build the ELFs."
