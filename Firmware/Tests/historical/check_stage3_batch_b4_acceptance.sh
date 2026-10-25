#!/usr/bin/env bash
# B4 integrated software acceptance. No NOR / hardware / 500 Hz runtime claim.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent-CubeIDE-verifier>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
    echo "FAIL: expected an independent STM32CubeIDE dual-core verifier" >&2; exit 1;
}
cd "$ROOT"
for script in \
  Firmware/Tests/stage_m7_application_sources.sh \
  Firmware/Tests/stage_stage3_batch_b3.sh \
  Firmware/Tests/stage_stage3_batch_b4.sh \
  Firmware/Tests/check_stage3_batch_b4_acceptance.sh; do
    bash -n "$script"
done
python3 - "$ROOT" "$DEST" <<'PY'
from pathlib import Path
import sys
repo, dest = map(Path, sys.argv[1:])
canonical = {
    'CM7/Core/Src/application_supervisor.c':
        'Firmware/STM32H745/CM7/Core/Src/application_supervisor.c',
    'CM7/Application/StateMachine/States/state_path_execution.c':
        'StateMachine/States/state_path_execution.c',
    'CM7/Application/StateMachine/States/state_path_execution.h':
        'StateMachine/States/state_path_execution.h',
    'CM7/Application/Intercore/trajectory_stream_path_port.c':
        'Intercore/trajectory_stream_path_port.c',
    'CM7/Application/Intercore/trajectory_stream_path_port.h':
        'Intercore/trajectory_stream_path_port.h',
    'CM4/Core/Src/application_validation_service.c':
        'Firmware/STM32H745/CM4/Core/Src/application_validation_service.c',
    'CM4/Core/Src/intercore_startup_validation.c':
        'Firmware/STM32H745/CM4/Core/Src/intercore_startup_validation.c',
    'CM4/Core/Src/main.c':
        'Firmware/STM32H745/CM4/Core/Src/main.c',
}
for core in ("CM7", "CM4"):
    for name in ("trajectory_storage_service.c", "trajectory_storage_service.h",
                 "validation_storage_bridge.c", "validation_storage_bridge.h",
                 "validation_storage_wire.h"):
        canonical[f"{core}/Application/Intercore/{name}"] = f"Intercore/{name}"
for rel, src in canonical.items():
    x, y = repo/src, dest/rel
    if not x.is_file() or not y.is_file() or x.read_bytes() != y.read_bytes():
        raise SystemExit(f"FAIL: stale or missing B4 verifier input: {rel}")
print(f"PASS: B4 canonical staging parity for {len(canonical)} firmware sources")
PY
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_production_shared_objects.sh
bash Firmware/Tests/check_application_arm_compilation.sh

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
common=(
  "$ROOT/Intercore/trajectory_stream.c"
  "$ROOT/Intercore/validation_storage_bridge.c"
  "$ROOT/Intercore/trajectory_storage_service.c"
)
gcc -std=gnu11 -O0 -Wall -Wextra -Werror -pthread \
    "${common[@]}" \
    "$ROOT/Intercore/Tests/test_trajectory_storage_service.c" \
    -o "$TMP/playback"
"$TMP/playback"
gcc -std=gnu11 -O0 -Wall -Wextra -Werror \
    "${common[@]}" \
    "$ROOT/Intercore/trajectory_stream_path_port.c" \
    "$ROOT/Intercore/Tests/test_trajectory_stream_path_port.c" \
    -o "$TMP/path_adapter"
"$TMP/path_adapter"
gcc -std=gnu11 -O0 -Wall -Wextra -Werror \
    "$ROOT/StateMachine/States/state_path_execution.c" \
    "$ROOT/Simulation/Storage/trajectory_prefetch.c" \
    "$ROOT/StateMachine/Tests/test_path_execution_prefetch.c" \
    -o "$TMP/path_faults"
"$TMP/path_faults"
echo "PASS: B4 host CRC/readback, replay, cancellation, prefill and failure tests"

M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
[[ -s "$M7" && -s "$M4" ]] || {
    echo "FAIL: both CubeIDE target ELFs required" >&2; exit 1;
}
for source in \
  CM7/Core/Src/application_supervisor.c \
  CM7/Application/Intercore/trajectory_storage_service.c \
  CM7/Application/Intercore/trajectory_stream_path_port.c \
  CM7/Application/StateMachine/States/state_path_execution.c; do
    [[ "$M7" -nt "$DEST/$source" ]] || {
        echo "FAIL: M7 ELF stale vs $source; clean-build CM7" >&2; exit 1;
    }
done
for source in \
  CM4/Core/Src/application_validation_service.c \
  CM4/Application/Intercore/trajectory_storage_service.c \
  CM4/Core/Src/main.c; do
    [[ "$M4" -nt "$DEST/$source" ]] || {
        echo "FAIL: M4 ELF stale vs $source; clean-build CM4" >&2; exit 1;
    }
done
arm-none-eabi-nm --defined-only "$M7" > "$TMP/m7"
arm-none-eabi-nm --defined-only "$M4" > "$TMP/m4"
require_symbol() {
    local file="$1" name="$2"
    awk -v n="$name" '$2=="T" && $3==n {found=1} END {exit !found}' "$file" || {
        echo "FAIL: target image missing strong symbol $name" >&2; exit 1;
    }
    echo "PASS: linked $name"
}
for name in gp_playback_m7_request gp_playback_m7_cancel \
  gp_traj_execution_attach_playback application_supervisor_start; do
    require_symbol "$TMP/m7" "$name"
done
for name in gp_playback_m4_init gp_playback_m4_step \
  gp_traj_m4_prepare gp_traj_m4_step application_validation_service_start; do
    require_symbol "$TMP/m4" "$name"
done
for name in gp_board_m4_supply_validation_backend \
            gp_board_m4_supply_trajectory_reader; do
    if grep -Eq "[[:space:]][TW][[:space:]]+${name}$" "$TMP/m4"; then
        echo "FAIL: unreviewed physical storage provider is defined: $name" >&2
        exit 1
    fi
done
arm-none-eabi-nm -S --defined-only "$M7" | awk \
  '$4=="intercore_validation_storage" &&
   $1=="38005000" && $2=="00003000" {pass=1} END {exit !pass}' || {
    echo "FAIL: changed Stage B2 validation SRAM4 footprint" >&2; exit 1;
}
echo "PASS: Stage 3 Batch B4 M4 durable-playback software and dual-core ELF gates"
echo "DEFERRED: official CubeMX NOR pinout, physical read/write/persistence, restart behavior and 500 Hz motion"
