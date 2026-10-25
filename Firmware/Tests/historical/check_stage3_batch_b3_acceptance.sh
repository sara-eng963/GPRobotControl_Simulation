#!/usr/bin/env bash
# Batch B3: typed validation SRAM4, source/ARM builds, host protocol, ELF symbols.
# Does NOT claim physical NOR or dual-core runtime verification.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent-CubeIDE-verifier>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: expected independent dual-core CubeIDE verifier" >&2; exit 1
}
python3 - "$ROOT" "$DEST" <<'PY'
from pathlib import Path
from xml.etree import ElementTree as ET
import sys
root, dest = map(Path, sys.argv[1:])
files = {
    'CM7/Core/Src/intercore_shared_memory.c':
      root/'Firmware/STM32H745/CM7/Core/Src/intercore_shared_memory.c',
    'CM7/Core/Src/application_supervisor.c':
      root/'Firmware/STM32H745/CM7/Core/Src/application_supervisor.c',
    'CM4/Core/Src/intercore_startup_validation.c':
      root/'Firmware/STM32H745/CM4/Core/Src/intercore_startup_validation.c',
    'CM4/Core/Src/application_validation_service.c':
      root/'Firmware/STM32H745/CM4/Core/Src/application_validation_service.c',
    'CM4/Core/Inc/application_validation_service.h':
      root/'Firmware/STM32H745/CM4/Core/Inc/application_validation_service.h',
    'CM4/Core/Src/main.c':
      root/'Firmware/STM32H745/CM4/Core/Src/main.c',
    'CM7/Application/Intercore/validation_storage_bridge.c':
      root/'Intercore/validation_storage_bridge.c',
    'CM4/Application/Intercore/validation_storage_bridge.c':
      root/'Intercore/validation_storage_bridge.c',
    'CM7/Application/Intercore/validation_storage_bridge.h':
      root/'Intercore/validation_storage_bridge.h',
    'CM4/Application/Intercore/validation_storage_bridge.h':
      root/'Intercore/validation_storage_bridge.h',
    'CM4/Core/Inc/validation_storage_wire.h':
      root/'Intercore/validation_storage_wire.h',
}
for rel, src in files.items():
    dst = dest/rel
    assert src.is_file() and dst.is_file() and src.read_bytes() == dst.read_bytes(), (
        f"FAIL: B3 verifier input stale/missing {rel}"
    )
for core in ('CM7', 'CM4'):
    r = ET.parse(dest/core/'.cproject').getroot()
    include = [e.get('value') for e in r.iter('listOptionValue')]
    assert include.count('../Application/Intercore') == 4, (
        f"FAIL: {core} Debug/Release C/ASM Intercore include paths"
    )
r = ET.parse(dest/'CM4/.cproject').getroot()
entries = [e.get('name') for e in r.iter('entry')]
assert entries.count('Application') == 2, "FAIL: M4 Application sources not selected"
print("PASS: canonical B3 M7/M4 sources and CubeIDE include/source settings")
PY

cd "$ROOT"
# Guard against broken continuation or malformed Bash staging logic before
# interpreting any ELF/runtime output as an integration acceptance.
bash -n Firmware/Tests/stage_m7_application_sources.sh
bash -n Firmware/Tests/stage_stage3_batch_b3.sh
bash -n Firmware/Tests/check_stage3_batch_b3_acceptance.sh
echo "PASS: B3 staging/acceptance Bash syntax"
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_production_shared_objects.sh
bash Firmware/Tests/check_application_arm_compilation.sh

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
gcc -std=c11 -O0 -Wall -Wextra -Werror -pthread \
  "$ROOT/Intercore/validation_storage_bridge.c" \
  "$ROOT/Intercore/Tests/test_validation_storage_bridge.c" \
  -o "$tmp/validation_bridge_test"
"$tmp/validation_bridge_test"
echo "PASS: host async M7/M4 queue, backpressure, flash failure, cancellation, backend offline"

M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
[[ -s "$M7" && -s "$M4" ]] || { echo "FAIL: target ELFs missing"; exit 1; }
for rel in CM7/Core/Src/intercore_shared_memory.c \
  CM7/Core/Src/application_supervisor.c \
  CM7/Application/Intercore/validation_storage_bridge.c; do
  [[ "$M7" -nt "$DEST/$rel" ]] || {
    echo "FAIL: M7 ELF predates staged $rel; rebuild required" >&2; exit 1;
  }
done
for rel in CM4/Core/Src/main.c CM4/Core/Src/intercore_startup_validation.c \
  CM4/Core/Src/application_validation_service.c \
  CM4/Application/Intercore/validation_storage_bridge.c; do
  [[ "$M4" -nt "$DEST/$rel" ]] || {
    echo "FAIL: M4 ELF predates staged $rel; rebuild required" >&2; exit 1;
  }
done
arm-none-eabi-nm --defined-only "$M7" > "$tmp/m7.nm"
arm-none-eabi-nm --defined-only "$M4" > "$tmp/m4.nm"
verify() {
  local file="$1" symbol="$2"
  awk -v n="$symbol" '$2=="T" && $3==n {found=1} END {exit !found}' "$file" || {
    echo "FAIL: required linked symbol missing: $symbol" >&2; exit 1;
  }
  echo "PASS: linked $symbol"
}
for sym in gp_validation_shared_initialize gp_validation_m7_init \
  gp_validation_m7_storage gp_validation_m4_idle \
  application_supervisor_start; do verify "$tmp/m7.nm" "$sym"; done
for sym in intercore_startup_validate application_validation_service_start \
  gp_validation_m4_init gp_validation_m4_step gp_validation_m4_shutdown; do
  verify "$tmp/m4.nm" "$sym"
done
# Current firmware must have no default fabricated physical flash provider.
if arm-none-eabi-nm --defined-only "$M4" | grep -Eq ' [TW] gp_board_m4_supply_validation_backend$'; then
  echo "FAIL: a physical M4 board storage provider unexpectedly exists" >&2
  exit 1
fi
arm-none-eabi-nm -S --defined-only "$M7" | awk \
  '$4=="intercore_validation_storage" &&
   $1=="38005000" && $2=="00003000" {ok=1} END {exit !ok}' || {
  echo "FAIL: typed validation allocation changed B2 12 KiB ABI" >&2; exit 1;
}
arm-none-eabi-readelf -W -S "$M7" | grep -E \
  '\] \.gp_shared_validation[[:space:]]+NOBITS[[:space:]]+38005000' >/dev/null || {
  echo "FAIL: validation NOLOAD SRAM4 section missing" >&2; exit 1;
}
echo "PASS: Stage 3 Batch B3 source, host protocol, ARM and both target ELF gates"
echo "DEFERRED: actual NOR backend, M4 worker runtime, independent reset and hardware tests"
