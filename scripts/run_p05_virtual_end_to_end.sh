#!/usr/bin/env bash
# Connected software demonstration. All builds/evidence stay outside checkout.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -gt 1 ]]; then echo 'usage: run_p05_virtual_end_to_end.sh [NEW_OUTPUT_DIRECTORY]' >&2; exit 2; fi
if [[ $# == 1 ]]; then
    OUT="$(realpath -m -- "$1")"
    case "$OUT/" in "$ROOT/"*) echo 'Refusing evidence inside source checkout' >&2; exit 2;; esac
    mkdir -m700 -- "$OUT" || exit 2
else
    OUT="$(mktemp -d "${TMPDIR:-/tmp}/p05_virtual_XXXXXXXX")"
fi
printf 'P05 evidence: %s\n' "$OUT"
status=FAIL
write_summary() {
    P05_STATUS="$status" python3 - "$OUT" <<'PY'
import json,os,sys
from pathlib import Path
out=Path(sys.argv[1])
def read(name):
    p=out/name
    return json.loads(p.read_text()) if p.is_file() else {}
i=read('integration.json')
s=read('production_supervisor_trace.json')
q=read('qspi_readback.json')
qspi=(i.get('qspi_proven') is True and all(q.get(k) is True for k in
    ('same_artifact','same_samples','crc_verified','read_from_simulated_flash')))
supervisor=(i.get('supervisor_production_invoked') is True and
    all(s.get(k) is True for k in ('production_supervisor_executed',
        'same_live_canopen_process','task_entry_executed')))
passed=os.environ['P05_STATUS']=='PASS'
data={'runner_status':os.environ['P05_STATUS'],
      'full_p05_acceptance':passed and qspi and supervisor,
      'physical_hardware_tested':False,
      'qspi_end_to_end':'PASS' if qspi else 'NOT RUN',
      'p04_production_supervisor_in_chain':'PASS' if supervisor else 'NOT RUN',
      'integration':i}
(out/'summary.json').write_text(json.dumps(data,indent=2)+'\n')
PY
}
trap write_summary EXIT
cmake -S "$ROOT" -B "$OUT/build" -DCMAKE_BUILD_TYPE=Debug >"$OUT/configure.log" 2>&1 || {
    cat "$OUT/configure.log" >&2; exit 1;
}
cmake --build "$OUT/build" --target p05_virtual_system --parallel 4 >"$OUT/build.log" 2>&1 || {
    tail -n 100 "$OUT/build.log" >&2; exit 1;
}
"$OUT/build/HandGuiding/p05_virtual_system" "$OUT" >"$OUT/virtual.log" 2>&1 || {
    tail -n 100 "$OUT/virtual.log" >&2; exit 1;
}
# Retain the independent P04-source contracts in addition to the live test.
python3 "$ROOT/Firmware/Tests/run_p05_supervisor_source.py" \
    --output "$OUT/production_supervisor_source_trace.json" >"$OUT/supervisor_source.log" 2>&1 || {
    tail -n 100 "$OUT/supervisor_source.log" >&2; exit 1;
}
status=PASS
write_summary
if ! python3 "$ROOT/Firmware/Tests/check_p05_integrated_evidence.py" "$OUT"; then
    status=FAIL
    exit 1
fi
cat "$OUT/virtual.log"
echo "P05 SOFTWARE SIMULATION ACCEPTED; no physical commissioning evidence."
