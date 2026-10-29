#!/usr/bin/env bash
# One-command connected host demonstration. Builds and writes OUTSIDE repo.
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
cleanup() {
    P05_STATUS="$status" python3 - "$OUT" <<'PY'
import json,os,sys
from pathlib import Path
out=Path(sys.argv[1]);integration=out/'integration.json'
data={'runner_status':os.environ['P05_STATUS'],
      'full_p05_acceptance':False,'physical_hardware_tested':False,
      'qspi_end_to_end':'NOT RUN','p04_production_supervisor_in_chain':'NOT RUN',
      'integration':json.loads(integration.read_text()) if integration.exists() else None}
(out/'summary.json').write_text(json.dumps(data,indent=2)+'\n')
PY
}
trap cleanup EXIT
cmake -S "$ROOT" -B "$OUT/build" -DCMAKE_BUILD_TYPE=Debug >"$OUT/configure.log" 2>&1 || {
    cat "$OUT/configure.log" >&2; exit 1;
}
cmake --build "$OUT/build" --target p05_virtual_system --parallel 4 >"$OUT/build.log" 2>&1 || {
    tail -100 "$OUT/build.log" >&2; exit 1;
}
"$OUT/build/HandGuiding/p05_virtual_system" "$OUT" >"$OUT/virtual.log" 2>&1 || {
    tail -100 "$OUT/virtual.log" >&2; exit 1;
}
status=PASS
cat "$OUT/virtual.log"
echo "HOST SIMULATION COMPLETE (QSPI physical/production Supervisor not accepted)."
