#!/usr/bin/env bash
# C3: source-frozen, dual-ELF, software-only release qualification.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -ge 1 && $# -le 2 ]] || {
  echo "Usage: $0 <CubeIDE-root> [C3-PREP-directory]" >&2; exit 2;
}
DEST="$(realpath -m "$1")"
if [[ $# -eq 2 ]]; then
  PREP="$(realpath -m "$2")"
else
  LATEST="$ROOT/.simulation-logs/stage-c3/latest_prep.txt"
  [[ -s "$LATEST" ]] || {
    echo "FAIL: no C3 preparation recorded; run prepare_stage3_c3_release.sh" >&2
    exit 1
  }
  PREP="$(realpath -m "$(cat "$LATEST")")"
fi
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: independent CubeIDE project required" >&2; exit 1;
}
[[ "$PREP" == "$ROOT/.simulation-logs/stage-c3/prep-"* &&
   -f "$PREP/context.txt" && -f "$PREP/config_sha256.txt" &&
   -f "$PREP/rebuild_after_this.marker" ]] || {
  echo "FAIL: invalid C3 checkpoint; run prepare_stage3_c3_release.sh" >&2; exit 1;
}
cd "$ROOT"
bash -n Firmware/Tests/prepare_stage3_c3_release.sh
bash -n Firmware/Tests/check_stage3_c3_release.sh
python3 -m py_compile Firmware/Tests/stage_c3_release_manifest.py
[[ "$(git branch --show-current)" == "sara/can-replacement" ]] || {
  echo "FAIL: wrong Git branch" >&2; exit 1;
}
git diff --quiet HEAD -- && git diff --cached --quiet -- || {
  echo "FAIL: tracked source changed since C3 snapshot" >&2; exit 1;
}
[[ "$(wc -l < "$PREP/context.txt")" -eq 2 &&
   "$(sed -n '1p' "$PREP/context.txt")" == "$(git rev-parse HEAD)" &&
   "$(sed -n '2p' "$PREP/context.txt")" == "$DEST" ]] || {
  echo "FAIL: C3 source SHA or project changed since checkpoint" >&2; exit 1;
}
sha256sum --check --status "$PREP/config_sha256.txt" || {
  echo "FAIL: .ioc/.cproject/linker changed during C3 rebuild" >&2; exit 1;
}
M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
for elf in "$M7" "$M4"; do
  [[ -s "$elf" && "$elf" -nt "$PREP/rebuild_after_this.marker" ]] || {
    echo "FAIL: missing/stale ELF $(basename "$elf") after C3 checkpoint" >&2
    exit 1;
  }
done
echo "PASS: both Cortex M7 and M4 ELFs freshly linked after frozen checkpoint"
for accepted in \
  26217a6ce44bdd1b941edefcdb1bee98ae54e003 \
  ee6783f57fd7c9fa18c32dcba664fa4297d1bd1e \
  9c0dd0d3f02240c663f4fa415121be7c75c12edd; do
  git merge-base --is-ancestor "$accepted" HEAD || {
    echo "FAIL: missing accepted source ancestor $accepted" >&2; exit 1;
  }
done
echo "PASS: candidate descends from B5, C1 and C2 accepted source revisions"

OUT_PARENT="$ROOT/.simulation-logs/stage-c3"
mkdir -p "$OUT_PARENT"
OUT="$(mktemp -d "$OUT_PARENT/run-XXXXXXXX")"
echo "C3 evidence directory: $OUT"

run_gate() {
  local name="$1"; shift
  echo "===== C3 $name ====="
  if "$@" > "$OUT/$name.log" 2>&1; then
    echo "PASS: $name"
    grep -E '^(PASS:|100% tests passed)' "$OUT/$name.log" | tail -n 7 || true
  else
    local result=$?
    echo "FAIL: $name (exit $result), see $OUT/$name.log" >&2
    tail -n 85 "$OUT/$name.log" >&2
    exit "$result"
  fi
}
# This C2 run recursively qualifies C1 + all B1–B5 source/ARM/ELF gates.
run_gate "c2_regression" bash Firmware/Tests/check_stage3_c2_acceptance.sh "$DEST"

# Curated headless portable robot-state/CANopen/IPC regression suites.
# Excludes interactive HMI and external hardware/network simulation.
set -- \
  ipc_mailbox_test \
  intercore_trajectory_stream_test \
  intercore_trajectory_path_port_test \
  intercore_hmi_lifecycle_test \
  intercore_trajectory_storage_test \
  intercore_validation_storage_test \
  host_timing_trace_test \
  path_validation_test \
  runtime_states_test \
  state_machine_test \
  avatar_m_canopen_test \
  canopen_master_test \
  canopen_bounded_rx_test \
  can_backend_test \
  joint_drive_idle_test \
  joint_drive_homing_test \
  joint_drive_approach_test \
  joint_drive_teaching_test \
  joint_drive_boot_test
TEST_COUNT="$#"
REGEX="^($(printf '%s|' "$@" | sed 's/|$//'))$"
run_gate "host_configure" cmake -S "$ROOT" -B "$OUT/host" \
  -DENABLE_SILKIT=OFF -DCMAKE_BUILD_TYPE=Debug
run_gate "host_build" cmake --build "$OUT/host" --target "$@" -j 2
ctest --test-dir "$OUT/host" -N -R "$REGEX" > "$OUT/ctest_inventory.log"
grep -F "Total Tests: $TEST_COUNT" "$OUT/ctest_inventory.log" >/dev/null || {
  echo "FAIL: curated CTest inventory incomplete" >&2
  cat "$OUT/ctest_inventory.log" >&2; exit 1;
}
run_gate "host_ctest" ctest --test-dir "$OUT/host" \
  --output-on-failure --parallel 2 -R "$REGEX"
echo "PASS: all $TEST_COUNT curated portable control and IPC host test suites"

# No unexpected physical motion/storage supplier is allowed in this
# intentionally uncommissioned firmware baseline.
arm-none-eabi-nm --defined-only "$M7" > "$OUT/m7_defined_symbols.txt"
arm-none-eabi-nm --defined-only "$M4" > "$OUT/m4_defined_symbols.txt"
if awk '$3=="gp_board_supply_commissioned_config" &&
        $2 ~ /^[TW]$/ {found=1} END {exit !found}' "$OUT/m7_defined_symbols.txt"; then
  echo "FAIL: a live CM7 physical board supplier was linked" >&2; exit 1;
fi
for name in gp_board_m4_supply_validation_backend \
            gp_board_m4_supply_trajectory_reader; do
  if awk -v n="$name" '$3==n && $2 ~ /^[TW]$/ {f=1} END {exit !f}' \
     "$OUT/m4_defined_symbols.txt"; then
    echo "FAIL: physical NOR provider unexpectedly linked: $name" >&2; exit 1;
  fi
done
echo "PASS: no commissioned motion or NOR board provider linked"

C2_DIR="$(awk '/^REPORT DIRECTORY:/ {p=$3} END {print p}' "$OUT/c2_regression.log")"
[[ "$C2_DIR" == "$ROOT/.simulation-logs/stage-c2/run-"* &&
   -d "$C2_DIR" ]] || {
  echo "FAIL: nested C2 evidence folder missing" >&2; exit 1;
}
printf '%s\n' "$C2_DIR" > "$OUT/c2_evidence_path.txt"
mkdir -p "$OUT/reports"
for file in can_wire_budget.json can_wire_budget.md \
            rtos_static_audit.json rtos_static_audit.md \
            evidence_manifest.txt qualification_status.md; do
  [[ -s "$C2_DIR/$file" ]] || {
    echo "FAIL: C2 evidence missing $file" >&2; exit 1;
  }
  cp "$C2_DIR/$file" "$OUT/reports/c2_$file"
done
MEMORY="$(find "$C2_DIR/c1" -name memory_regions.json -type f -print -quit)"
[[ -n "$MEMORY" && -s "$MEMORY" ]] || {
  echo "FAIL: nested C1 memory report missing" >&2; exit 1;
}
cp "$MEMORY" "$OUT/reports/c1_memory_regions.json"

{
  echo "SOURCE_SHA=$(git rev-parse HEAD)"
  echo "BRANCH=$(git branch --show-current)"
  echo "UTC_COMPLETED=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "C3_PREP=$PREP"
  echo "C2_EVIDENCE=$C2_DIR"
  echo "ARM_GCC=$(arm-none-eabi-gcc -dumpversion)"
  echo "GCC=$(gcc -dumpversion)"
  sha256sum "$M7" "$M4"
} > "$OUT/qualification_manifest.txt"
cat > "$OUT/qualification_status.md" <<'MD'
# C3 scope: software-only release candidate
PASS: B1–B5/C1/C2 integrated gates, frozen-source dual-core freshly
linked target images, 19 portable controls/IPC CTests, board-provider
fail-closed symbols, linked memory reports, and release SHA256 manifest.

NOT VERIFIED: clean-click provenance from IDE (save IDE console separately);
powered STM32 HSEM/cache/MPU/restart, 500Hz WCET and real six-drive CAN,
NOR physical persistence, stack/heap runtime watermarks, hand-guiding
handle force sensor, physical HMI, safety-rated STO/E-stop, any robot motion.
MD
run_gate "candidate_manifest" python3 \
  Firmware/Tests/stage_c3_release_manifest.py \
  --project "$DEST" --prep "$PREP" --out "$OUT"

echo "PASS: Stage C3 final pre-hardware software qualification and dual-core candidate"
echo "REPORT DIRECTORY: $OUT"
echo "DEFERRED: H0-H10 real hardware/safety commissioning"
