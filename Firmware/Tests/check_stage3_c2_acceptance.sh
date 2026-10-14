#!/usr/bin/env bash
# Stage C2: consolidated software-only integration + resource evidence.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <independent CubeIDE verifier>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: expected independent CubeIDE verifier" >&2; exit 1;
}
cd "$ROOT"
bash -n Firmware/Tests/check_stage3_c2_acceptance.sh
bash -n Firmware/Tests/stage_stage3_c2_canopen.sh
for p in CANComm/CANopen/canopen_master.c CANComm/CANopen/canopen_master.h; do
  cmp -s "$ROOT/$p" "$DEST/CM7/Application/$p" || {
    echo "FAIL: mismatched M7 CANopen source: $p" >&2; exit 1;
  }
done
M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
[[ -s "$M7" && -s "$M4" &&
   "$M7" -nt "$DEST/CM7/Application/CANComm/CANopen/canopen_master.c" &&
   "$M7" -nt "$DEST/CM7/Application/CANComm/CANopen/canopen_master.h" ]] || {
  echo "FAIL: M7 ELF stale vs C2 CANopen sources; clean-build M7" >&2; exit 1;
}
python3 Firmware/Tests/stage_c2_can_wire_model.py --self-test
python3 Firmware/Tests/stage_c2_rtos_audit.py --self-test
OUT_PARENT="$ROOT/.simulation-logs/stage-c2"
mkdir -p "$OUT_PARENT"
OUT="$(mktemp -d "$OUT_PARENT/run-XXXXXXXX")"
echo "Stage C2 report directory: $OUT"

run_gate() {
  local name="$1"; shift
  echo "===== C2 $name ====="
  if "$@" > "$OUT/$name.log" 2>&1; then
    echo "PASS: $name"
    grep -E '^(PASS:|100% tests passed)' "$OUT/$name.log" | tail -n 6 || true
  else
    local result=$?
    echo "FAIL: $name (exit $result): $OUT/$name.log" >&2
    tail -n 75 "$OUT/$name.log" >&2
    exit "$result"
  fi
}
mkdir -p "$OUT/c1"
run_gate "c1_regression" env GP_STAGE3_C1_REPORT_ROOT="$OUT/c1" \
  bash Firmware/Tests/check_stage3_c1_release.sh "$DEST"

run_gate "host_configure" cmake -S "$ROOT" -B "$OUT/host" -DCMAKE_BUILD_TYPE=Debug
run_gate "host_build" cmake --build "$OUT/host" \
  --target canopen_bounded_rx_test canopen_master_test -j 2
run_gate "bounded_and_existing_canopen_tests" ctest \
  --test-dir "$OUT/host" --output-on-failure \
  -R "^(canopen_bounded_rx_test|canopen_master_test)$"
run_gate "m7_timer_host_faults" python3 Firmware/Tests/run_m7_timer_fault_injection.py

KERNEL="$(printenv FREERTOS_KERNEL_ROOT || true)"
if [[ -z "$KERNEL" ]]; then
  KERNEL="$ROOT/../FreeRTOS-Kernel"
fi
if [[ -f "$KERNEL/include/FreeRTOS.h" ]]; then
  run_gate "ipc_hsem_hal_mock" env FREERTOS_KERNEL_ROOT="$KERNEL" \
    bash Intercore/Tests/run_ipc_hsem_mock.sh
else
  echo "DEFERRED: optional HSEM mock; external FreeRTOS-Kernel tree absent" \
    | tee "$OUT/optional_ipc_hsem.txt"
fi

python3 Firmware/Tests/stage_c2_can_wire_model.py --out "$OUT"
python3 Firmware/Tests/stage_c2_rtos_audit.py --out "$OUT"
{
  echo "SOURCE_COMMIT=$(git rev-parse HEAD)"
  echo "C2_SOURCE_BRANCH=$(git rev-parse --abbrev-ref HEAD)"
  echo "UTC_COMPLETED=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "M7 M4 ELF SHA256"
  sha256sum "$M7" "$M4"
  echo "CANOPEN SOURCE SHA256"
  sha256sum CANComm/CANopen/canopen_master.c CANComm/CANopen/canopen_master.h
} > "$OUT/evidence_manifest.txt"
cat > "$OUT/qualification_status.md" <<'MD'
# C2 verification boundary
- PASS after full script completes: C1 regression, bounded real CANopen C
  owner RX test, original CANopen master tests, real-source timer faults,
  static RTOS/IRQ audit and CAN bus framing feasibility model.
- Hardware DEFERRED: true simultaneous M7/M4 reset coherency, NOR power
  fail, physical CAN motor timing / bus off, 500Hz deadlines, worst-case
  task latency and stack/heap watermarks, E-stop/STO and safety qualification.
- Optional external FreeRTOS POSIX HSEM mock is separately marked DEFERRED
  if its source dependency is missing; do not count it as PASS.
This is NOT physical or production release acceptance.
MD
echo "PASS: Stage C2 bounded CANopen RX, timer faults, CAN budget, RTOS contracts and C1 regression"
echo "REPORT DIRECTORY: $OUT"
echo "DEFERRED: C3 release handoff, physical real-time/safety"
