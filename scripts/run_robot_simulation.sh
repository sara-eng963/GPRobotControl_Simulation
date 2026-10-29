#!/usr/bin/env bash
# Unified host-only production runtime. Never selects a physical CAN backend.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROBOT_UNIFIED_MODE=1
export BUILD_DIR="${BUILD_DIR:-$ROOT/build/robot-simulation}"
VERIFY=0
HELP=0
for argument in "$@"; do
    [[ "$argument" != --verify ]] || VERIFY=1
    [[ "$argument" != --help && "$argument" != -h ]] || HELP=1
    if [[ "$argument" == --matlab ]]; then
        echo '[FAIL] Automatic MATLAB startup is disabled in this launcher.' >&2
        echo 'Run without --matlab and open your Windows MATLAB visualizer manually.' >&2
        exit 2
    fi
done
if ((VERIFY)); then
    # Python validates the complete argument list; incompatible smoke/GUI flags fail.
    exec python3 "$ROOT/scripts/verify_robot_simulation.py" "$@"
fi
if ((HELP)); then
    cat <<'EOF'
Unified robot simulation — manual Windows MATLAB mode
Usage: ./scripts/run_robot_simulation.sh [options]
  (no options)                 Full interactive simulation + UDP telemetry to Windows:5005
  --no-matlab                  Explicit manual MATLAB mode (default)
  --verify [--output DIR]      Run integrated software/regression verification
  --headless --duration N      Zero-force startup smoke test (no GUI)
  --no-build                   Reuse current compiled simulation
  --help                       Show this help
Environment: MATLAB_IP overrides the detected Windows destination IP.
MATLAB is never launched or closed by this command. Open can_robot_visualizer.m manually.
EOF
    exit 0
fi
# Force independent MATLAB and UDP telemetry regardless of external defaults.
exec bash "$ROOT/scripts/run_full_can_silkit_simulation.sh" --virtual-ft "$@" --no-matlab
