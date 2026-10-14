#!/usr/bin/env bash
# B3 verifier staging. Canonical source stays in Git; never edit CubeMX .ioc.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent-CubeIDE-project>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" &&
   -f "$DEST/CM7/.project" && -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: expected independent CubeIDE dual-core project" >&2; exit 1;
}

# Existing canonical Stage B1 M7 app staging (includes M7 validation bridge).
bash -n "$ROOT/Firmware/Tests/stage_m7_application_sources.sh"
bash -n "$ROOT/Firmware/Tests/stage_stage3_batch_b3.sh"
bash "$ROOT/Firmware/Tests/stage_m7_application_sources.sh" "$DEST"

# Apply ONLY the new M4 include/source-root entries, preserving other
# project settings and refusing unrecognized project configuration shapes.
python3 - "$DEST/CM4/.cproject" <<'PY'
from pathlib import Path
from xml.etree import ElementTree as ET
import sys
p = Path(sys.argv[1])
s = p.read_text()
include = '<listOptionValue builtIn="false" value="../Core/Inc"/>'
extra = '<listOptionValue builtIn="false" value="../Application/Intercore"/>'
source = '<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="Common"/>'
app = '<entry flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="Application"/>'
if s.count(include) != 4 or s.count(source) != 2 or s.count(extra) not in (0,4) or s.count(app) not in (0,2):
    raise SystemExit("FAIL: unfamiliar M4 .cproject shape; no edits made")
lines = s.splitlines(keepends=True)
out = []
for line in lines:
    if app not in s and source in line:
        indent = line[:len(line)-len(line.lstrip())]
        out.append(indent + app + ('\r\n' if line.endswith('\r\n') else '\n'))
    out.append(line)
    if extra not in s and include in line:
        indent = line[:len(line)-len(line.lstrip())]
        out.append(indent + extra + ('\r\n' if line.endswith('\r\n') else '\n'))
new = ''.join(out)
ET.fromstring(new)
if new != s:
    p.write_bytes(new.encode())
    print("STAGED: M4 project Intercore includes / Application sources")
else:
    print("UNCHANGED: M4 project settings already present")
PY

# Copy canonical firmware files only after checking their intended homes.
paths=(
  "CM7/Core/Src/intercore_shared_memory.c"
  "CM7/Core/Src/application_supervisor.c"
  "CM4/Core/Src/intercore_startup_validation.c"
  "CM4/Core/Src/application_validation_service.c"
  "CM4/Core/Inc/application_validation_service.h"
  "CM4/Core/Src/main.c"
)
for path in "${paths[@]}"; do
  src="$ROOT/Firmware/STM32H745/$path"
  target="$DEST/$path"
  [[ -f "$src" ]] || { echo "FAIL: canonical firmware source missing $path" >&2; exit 1; }
  [[ -f "$target" || "$path" == "CM4/Core/Src/application_validation_service.c" ||
     "$path" == "CM4/Core/Inc/application_validation_service.h" ]] || {
    echo "FAIL: verifier source missing $path" >&2; exit 1
  }
done
for path in "${paths[@]}"; do
  cp "$ROOT/Firmware/STM32H745/$path" "$DEST/$path"
  cmp -s "$ROOT/Firmware/STM32H745/$path" "$DEST/$path"
  echo "STAGED: $path"
done
for core in CM7 CM4; do
  cp "$ROOT/Intercore/validation_storage_wire.h" "$DEST/$core/Core/Inc/validation_storage_wire.h"
done

# The M4 builds only the existing canonical bridge C file. All other
# dependency folders are header-only mirrors, not duplicate runtime modules.
APP="$DEST/CM4/Application"
for dir in ControlCore/Analysis ControlCore/Config ControlCore/Execution \
  ControlCore/Kinematics ControlCore/Math ControlCore/Pipeline \
  ControlCore/Trajectory StateMachine StateMachine/States \
  ServoDrive/AvatarM ServoDrive/CiA402 ServoDrive/JointDrive; do
  mkdir -p "$APP/$dir"
  find "$APP/$dir" -maxdepth 1 -type f -name '*.h' -delete
  while IFS= read -r -d '' file; do
    cp "$file" "$APP/$dir/$(basename "$file")"
  done < <(find "$ROOT/$dir" -maxdepth 1 -type f -name '*.h' -print0)
done
mkdir -p "$APP/Intercore"
for name in validation_storage_bridge.c validation_storage_bridge.h validation_storage_wire.h; do
  cp "$ROOT/Intercore/$name" "$APP/Intercore/$name"
done
echo "PASS: B3 M7/M4 typed validation ABI staged; physical flash provider absent by default"
echo "Clean-build both CubeIDE targets before final acceptance."
