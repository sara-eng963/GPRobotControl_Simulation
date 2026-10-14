#!/usr/bin/env bash
# Stage C1: release baseline; existing B5 target firmware remains unchanged.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || {
  echo "Usage: $0 <independent-GP_Stage3_Repro-project-root>" >&2; exit 2;
}
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* &&
   -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: independent STM32CubeIDE dual-core verifier required" >&2; exit 1;
}
for tool in arm-none-eabi-readelf arm-none-eabi-nm gcc; do
  command -v "$tool" >/dev/null || {
    echo "FAIL: missing $tool" >&2; exit 1;
  }
done
M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
[[ -s "$M7" && -s "$M4" ]] || {
  echo "FAIL: target Debug ELFs missing; build both cores in CubeIDE" >&2; exit 1;
}
cd "$ROOT"
python3 Firmware/Tests/stage_c1_memory_audit.py --self-test
bash -n Firmware/Tests/check_stage3_c1_release.sh

OUT_PARENT="$(printenv GP_STAGE3_C1_REPORT_ROOT || true)"
if [[ -z "$OUT_PARENT" ]]; then
  OUT_PARENT="$ROOT/.simulation-logs/stage-c1"
fi
mkdir -p "$OUT_PARENT"
OUT="$(mktemp -d "$OUT_PARENT/run-XXXXXXXX")"
echo "Stage C1 report directory: $OUT"

run_gate() {
  local name="$1"; shift
  echo "===== C1 $name ====="
  if "$@" > "$OUT/$name.log" 2>&1; then
    echo "PASS: $name"
    grep -E '^(PASS: Stage 3 Batch B5|PASS: Stage 2E-4|PASS: Stage 3|PASS: .+fault|PASS:.*CM4)' "$OUT/$name.log" | tail -n 8 || true
  else
    local result=$?
    echo "FAIL: $name (exit $result); diagnostic: $OUT/$name.log" >&2
    tail -n 65 "$OUT/$name.log" >&2
    exit "$result"
  fi
}

run_gate "batch_b_consolidated" bash Firmware/Tests/check_stage3_batch_b5_acceptance.sh "$DEST"
run_gate "hal_exti_contract" bash Firmware/Tests/check_intercore_notifications.sh
run_gate "fdcan_hal1_fault_mocks" bash Firmware/Tests/check_fdcan_hal_adapter_mock.sh
run_gate "m4_ipc_api" bash Firmware/Tests/check_m4_application_ipc.sh

python3 Firmware/Tests/stage_c1_memory_audit.py \
  --cm7 "$M7" --cm4 "$M4" \
  --cm7-linker "$DEST/CM7/STM32H745IITX_FLASH.ld" \
  --cm4-linker "$DEST/CM4/STM32H745IITX_FLASH.ld" \
  --out "$OUT"

{
  echo "STAGE_C1_SOURCE_SHA=$(git rev-parse HEAD)"
  echo "STAGE_C1_SOURCE_BRANCH=$(git rev-parse --abbrev-ref HEAD)"
  echo "UTC_COMPLETED=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  arm-none-eabi-gcc --version | head -n 1
  gcc --version | head -n 1
  echo "ELF_CHECKSUMS"
  sha256sum "$M7" "$M4"
  echo "LINKER_SCRIPT_CHECKSUMS"
  sha256sum "$DEST/CM7/STM32H745IITX_FLASH.ld" \
            "$DEST/CM4/STM32H745IITX_FLASH.ld"
  for core in CM7 CM4; do
    map="$DEST/$core/Debug/GP_STM32H745_$core.map"
    if [[ -s "$map" ]]; then
      echo "MAP_SHA256 $core $(sha256sum "$map")"
    else
      echo "MAP_NOT_CAPTURED $core — ELF sections audited directly"
    fi
  done
} > "$OUT/evidence_manifest.txt"

cat > "$OUT/qualification_status.md" <<'MD'
# Stage C1 release qualification evidence
- **PASS (after this script exits zero):** consolidated B1–B5 host/ARM/ELF gates.
- **PASS:** official ST HAL EXTI notification compile and startup-order checks.
- **PASS:** host FDCAN HAL1 fault mocks and M4 IPC application API checks.
- **PASS:** real linked M7/M4 ELF region allocations and flash PT_LOAD audit.
- **NOT TESTED:** simultaneous two-core boot; live HSEM/EXTI, MPU and caches.
- **NOT TESTED:** independent M4/M7 reset during storage or IPC traffic.
- **NOT TESTED:** physical six-drive CAN bus, 500 Hz jitter, PDO latency or WCET.
- **NOT TESTED:** actual NOR persistence, HMI transport and physical safety circuits.
- **NOT YET MEASURED:** FreeRTOS task stack watermarks and peak heap at runtime.
This is Stage C1 static/host software evidence, not a production robot release.
MD
echo "PASS: Stage C1 software release baseline, HAL fault mocks and actual ELF memory audit"
echo "REPORT DIRECTORY: $OUT"
echo "DEFERRED: C2 integrated restart/fault injection and scheduling, C3 signoff and physical commissioning"
