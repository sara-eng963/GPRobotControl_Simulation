#!/usr/bin/env bash
set -Eeuo pipefail

# Full PC graduation-project pipeline:
#   SIL Kit registry
#   six AVATAR M virtual CANopen nodes
#   FreeRTOS Supervisor / StateMachine
#   operator HMI
#   teaching simulator
#
# MATLAB is a Windows-side process in the current setup. Keep
# MATLAB/can_robot_visualizer.m running; this script auto-detects the Windows
# host IP used by the Supervisor for UDP telemetry.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/build-can-silkit}"
SILKIT_ROOT="${SILKIT_ROOT:-$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit}"
REGISTRY_URI="${SILKIT_REGISTRY_URI:-silkit://localhost:8500}"
MATLAB_IP="${MATLAB_IP:-}"

DO_BUILD=1
START_GUI=1

usage() {
    cat <<'EOF'
Usage:
  ./scripts/run_full_can_silkit_simulation.sh [options]

Options:
  --no-build   Skip CMake configure/build.
  --no-gui     Do not start supervisor_panel or teaching_sim.
  -h, --help   Show this help.

Environment overrides:
  SILKIT_ROOT          SIL Kit installation root.
  BUILD_DIR            CMake build directory.
  SILKIT_REGISTRY_URI  Registry URI (default silkit://localhost:8500).
  MATLAB_IP            Windows host IP for UDP telemetry.
EOF
}

while (($# > 0)); do
    case "$1" in
        --no-build)
            DO_BUILD=0
            ;;
        --no-gui)
            START_GUI=0
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if [[ -z "$MATLAB_IP" ]]; then
    MATLAB_IP="$(
        ip route 2>/dev/null |
        awk '/default/ {print $3; exit}'
    )"
fi

if [[ -z "$MATLAB_IP" ]]; then
    MATLAB_IP="127.0.0.1"
    echo "[WARN] Could not detect Windows/host IP; MATLAB_IP=$MATLAB_IP"
fi

REGISTRY_BIN="$SILKIT_ROOT/bin/sil-kit-registry"
MOTOR_BIN="$BUILD_DIR/avatar_m_silkit_motor_bank"
SUPERVISOR_BIN="$BUILD_DIR/supervisor_silkit_sim"
PANEL_BIN="$BUILD_DIR/supervisor_panel"
TEACH_BIN="$BUILD_DIR/teaching_sim"

LOG_DIR="$REPO_ROOT/.simulation-logs"
mkdir -p "$LOG_DIR"

REGISTRY_LOG="$LOG_DIR/silkit-registry.log"
MOTOR_LOG="$LOG_DIR/avatar-m-motors.log"
SUPERVISOR_LOG="$LOG_DIR/supervisor.log"
PANEL_LOG="$LOG_DIR/supervisor-panel.log"
TEACH_LOG="$LOG_DIR/teaching-sim.log"

PIDS=()

cleanup() {
    local status=$?
    trap - EXIT INT TERM

    if (("${#PIDS[@]}" > 0)); then
        echo
        echo "Stopping simulation..."
        for pid in "${PIDS[@]}"; do
            kill "$pid" 2>/dev/null || true
        done

        sleep 0.3

        for pid in "${PIDS[@]}"; do
            kill -9 "$pid" 2>/dev/null || true
        done
    fi

    exit "$status"
}

trap cleanup EXIT INT TERM

wait_for_log() {
    local file="$1"
    local pattern="$2"
    local timeout_s="$3"
    local pid="$4"
    local label="$5"

    local deadline=$((SECONDS + timeout_s))

    while ((SECONDS < deadline)); do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "[FAIL] $label exited early." >&2
            tail -n 80 "$file" 2>/dev/null || true
            return 1
        fi

        if grep -Fq "$pattern" "$file" 2>/dev/null; then
            return 0
        fi

        sleep 0.1
    done

    echo "[FAIL] Timed out waiting for $label." >&2
    tail -n 80 "$file" 2>/dev/null || true
    return 1
}

cd "$REPO_ROOT"

echo "============================================================"
echo " GP Robot — Full SIL Kit CANopen Simulation"
echo "============================================================"
echo "Repository : $REPO_ROOT"
echo "Build      : $BUILD_DIR"
echo "SIL Kit    : $SILKIT_ROOT"
echo "Registry   : $REGISTRY_URI"
echo "MATLAB IP  : $MATLAB_IP"
echo "CAN        : CAN1 @ 1 Mbit/s"
echo "Motors     : AVATAR nodes 1..6"
echo "============================================================"
echo

if [[ ! -x "$REGISTRY_BIN" ]]; then
    echo "[FAIL] SIL Kit registry not found:" >&2
    echo "       $REGISTRY_BIN" >&2
    exit 1
fi

if ((DO_BUILD)); then
    echo "[1/5] Configuring SIL Kit build..."
    cmake -S "$REPO_ROOT" -B "$BUILD_DIR"         -DENABLE_SILKIT=ON         -DSILKIT_ROOT="$SILKIT_ROOT"

    echo
    echo "[2/5] Building controller, motors and GUIs..."
    cmake --build "$BUILD_DIR"         --target             supervisor_silkit_sim             avatar_m_silkit_motor_bank             supervisor_panel             teaching_sim         -j "$(nproc)"
else
    echo "[1/5] Configure/build skipped."
    echo "[2/5] Build skipped."
fi

for required in "$MOTOR_BIN" "$SUPERVISOR_BIN"; do
    if [[ ! -x "$required" ]]; then
        echo "[FAIL] Missing executable: $required" >&2
        exit 1
    fi
done

: > "$REGISTRY_LOG"
: > "$MOTOR_LOG"
: > "$SUPERVISOR_LOG"
: > "$PANEL_LOG"
: > "$TEACH_LOG"

echo
echo "[3/5] Starting SIL Kit CAN network..."

"$REGISTRY_BIN"     --listen-uri "$REGISTRY_URI"     >"$REGISTRY_LOG" 2>&1 &
REGISTRY_PID=$!
PIDS+=("$REGISTRY_PID")

wait_for_log     "$REGISTRY_LOG"     "SIL Kit Registry listening"     8     "$REGISTRY_PID"     "SIL Kit registry"

SILKIT_REGISTRY_URI="$REGISTRY_URI"     "$MOTOR_BIN"     >"$MOTOR_LOG" 2>&1 &
MOTOR_PID=$!
PIDS+=("$MOTOR_PID")

wait_for_log     "$MOTOR_LOG"     "AVATAR SIL Kit motor bank ready."     10     "$MOTOR_PID"     "AVATAR motor bank"

SILKIT_REGISTRY_URI="$REGISTRY_URI" MATLAB_IP="$MATLAB_IP"     "$SUPERVISOR_BIN"     >"$SUPERVISOR_LOG" 2>&1 &
SUPERVISOR_PID=$!
PIDS+=("$SUPERVISOR_PID")

wait_for_log     "$SUPERVISOR_LOG"     "[SILKIT] Controller connected:"     10     "$SUPERVISOR_PID"     "Supervisor"

echo
echo "[4/5] SIL Kit participants are running."
echo "      Registry + RobotControllerSupervisor + AVATAR nodes 1..6"

if ((START_GUI)); then
    if [[ ! -x "$PANEL_BIN" || ! -x "$TEACH_BIN" ]]; then
        echo "[FAIL] HMI executables are missing." >&2
        exit 1
    fi

    echo
    echo "[5/5] Starting HMI + teaching simulator..."

    "$PANEL_BIN"         >"$PANEL_LOG" 2>&1 &
    PANEL_PID=$!
    PIDS+=("$PANEL_PID")

    "$TEACH_BIN"         >"$TEACH_LOG" 2>&1 &
    TEACH_PID=$!
    PIDS+=("$TEACH_PID")
else
    echo
    echo "[5/5] GUI startup skipped."
fi

echo
echo "============================================================"
echo " FULL PIPELINE RUNNING"
echo "============================================================"
echo " HMI"
echo "   -> FreeRTOS Supervisor / StateMachine"
echo "   -> CanopenMaster"
echo "   -> SIL Kit CAN1 @ 1 Mbit/s"
echo "   -> six AVATAR virtual motors"
echo "   -> TPDO4 feedback"
echo "   -> HMI + MATLAB UDP :5005"
echo
echo " MATLAB:"
echo "   Keep MATLAB/can_robot_visualizer.m running on Windows."
echo "   Telemetry destination: $MATLAB_IP:5005"
echo
echo " Logs:"
echo "   $LOG_DIR"
echo
echo " Press Ctrl-C here to stop the complete simulation."
echo "============================================================"

# Keep the launcher alive and fail loudly if a core process exits.
while true; do
    for entry in         "$REGISTRY_PID:registry:$REGISTRY_LOG"         "$MOTOR_PID:motor-bank:$MOTOR_LOG"         "$SUPERVISOR_PID:supervisor:$SUPERVISOR_LOG"
    do
        IFS=: read -r pid name log <<< "$entry"

        if ! kill -0 "$pid" 2>/dev/null; then
            echo
            echo "[FAIL] Core process '$name' stopped unexpectedly." >&2
            tail -n 100 "$log" 2>/dev/null || true
            exit 1
        fi
    done

    sleep 1
done
