#!/usr/bin/env bash
# Stage 3 B1: source, ARM object, host runtime, and previously CubeIDE-built ELF.
# Hardware motion, timing jitter and CAN bus behavior are NOT verified here.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || {
  echo "Usage: $0 <independent-CubeIDE-verifier>" >&2
  exit 2
}
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: expected independent dual-core CubeIDE verifier" >&2; exit 1
}
pairs=(
  "Firmware/STM32H745/CM7/Core/Src/application_supervisor.c|CM7/Core/Src/application_supervisor.c"
  "Intercore/trajectory_stream_path_port.c|CM7/Application/Intercore/trajectory_stream_path_port.c"
  "Intercore/trajectory_stream_path_port.h|CM7/Application/Intercore/trajectory_stream_path_port.h"
  "StateMachine/States/state_path_execution.c|CM7/Application/StateMachine/States/state_path_execution.c"
)
for pair in "${pairs[@]}"; do
  IFS='|' read -r left right <<< "$pair"
  if [[ ! -f "$DEST/$right" ]] || ! cmp -s "$ROOT/$left" "$DEST/$right"; then
    echo "FAIL: stale verifier source $right" >&2
    echo "Run: bash Firmware/Tests/stage_stage3_batch_b1.sh \"$DEST\"" >&2
    exit 1
  fi
done
echo "PASS: B1 source parity across independent CubeIDE verifier"

cd "$ROOT"
bash Firmware/Tests/check_m7_application_integration.sh
bash Firmware/Tests/check_m7_board_canopen_composition.sh
bash Firmware/Tests/check_application_arm_compilation.sh

# Test the real shared trajectory and composed callback implementation on host.
# Temporary out-of-tree host build; no SIL Kit, simulated drive or hardware bus.
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
cmake -S "$ROOT" -B "$tmp/host" -DENABLE_SILKIT=OFF
cmake --build "$tmp/host" --target \
  intercore_trajectory_stream_test \
  intercore_trajectory_path_port_test \
  avatar_m_canopen_test canopen_master_test
ctest --test-dir "$tmp/host" --output-on-failure \
  -R '^(intercore_trajectory_stream_test|intercore_trajectory_path_port_test|avatar_m_canopen_test|canopen_master_test)$'
# Run the actual PathExecution state machine with a host six-axis fake;
# this tests software hold ordering and fault latching, NOT a physical stop.
gcc -std=c11 -O0 -Wall -Wextra -Werror \
  "$ROOT/StateMachine/States/state_path_execution.c" \
  "$ROOT/Simulation/Storage/trajectory_prefetch.c" \
  "$ROOT/StateMachine/Tests/test_path_execution_prefetch.c" \
  -o "$tmp/path_execution_prefetch_test"
"$tmp/path_execution_prefetch_test"
echo "PASS: B1 host stream/consumer, execution faults and CANopen tests"

M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
[[ -s "$M7" && -s "$M4" ]] || {
  echo "FAIL: independently linked M7/M4 target images required" >&2
  exit 1
}
for pair in "${pairs[@]}"; do
  IFS='|' read -r left right <<< "$pair"
  [[ "$M7" -nt "$DEST/$right" ]] || {
    echo "FAIL: CM7 ELF predates staged B1 input $right — CLEAN BUILD required" >&2
    exit 1
  }
done
arm-none-eabi-nm --defined-only "$M7" > "$tmp/m7.nm"
for symbol in \
  gp_traj_execution_bind gp_traj_m7_begin gp_traj_m7_take \
  gp_traj_m7_finish gp_traj_m7_discard \
  state_path_execution_step canopen_master_init \
  application_supervisor_start; do
  if ! awk -v n="$symbol" '$2=="T" && $3==n {found=1} END {exit !found}' "$tmp/m7.nm"; then
    echo "FAIL: strong M7 symbol missing: $symbol" >&2; exit 1
  fi
  echo "PASS: strong M7 $symbol"
done
bash Firmware/Tests/check_stage3_batch_a_target_elfs.sh "$DEST"
echo "PASS: Stage 3 Batch B1 composed M7 motion software gates and ELF inventory"
echo "DEFERRED: physical drive PDO, flash-backed M4 production, measured 2 ms latency and safety."
