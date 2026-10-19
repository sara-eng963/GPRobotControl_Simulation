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
VIRTUAL_FT=0

usage() {
    cat <<'EOF'
Usage:
  ./scripts/run_full_can_silkit_simulation.sh [options]

Options:
  --no-build   Skip CMake configure/build.
  --no-gui     Do not start graphical applications (normal mode only).
  --virtual-ft Run interactive F/T hand-guiding instead of teaching_sim.
               Starts virtual_ft_gui + supervisor_panel + CAN monitor.
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
        --virtual-ft)
            VIRTUAL_FT=1
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

# The interactive F/T receiver needs one persistent zero-wrench source.
# Running an F/T Supervisor without its GUI would trigger a watchdog fault
# as soon as TEACHING begins; force explicit intent instead.
if ((VIRTUAL_FT && !START_GUI)); then
    echo "[FAIL] --virtual-ft cannot be combined with --no-gui." >&2
    exit 2
fi

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
VFT_GUI_BIN="$BUILD_DIR/virtual_ft_gui"
CAN_MONITOR_BIN="$BUILD_DIR/can_live_monitor"

if ((VIRTUAL_FT)); then
    # Do not overwrite normal full-simulation operator/timing logs.
    LOG_DIR="$REPO_ROOT/.simulation-logs/virtual-ft"
else
    LOG_DIR="$REPO_ROOT/.simulation-logs"
fi
mkdir -p "$LOG_DIR"

REGISTRY_LOG="$LOG_DIR/silkit-registry.log"
MOTOR_LOG="$LOG_DIR/avatar-m-motors.log"
SUPERVISOR_LOG="$LOG_DIR/supervisor.log"
EVENT_LOG="$LOG_DIR/operator-events.txt"
TRACE_LOG="$LOG_DIR/simulation-trace.csv"
PANEL_LOG="$LOG_DIR/supervisor-panel.log"
TEACH_LOG="$LOG_DIR/teaching-sim.log"
VFT_GUI_LOG="$LOG_DIR/virtual-ft-gui.log"
GUIDANCE_TRACE_LOG="$LOG_DIR/guidance.csv"
CAN_MONITOR_LOG="$LOG_DIR/can-monitor.log"
CAN_FRAME_LOG="$LOG_DIR/can-frames.csv"
CAN_METRICS_LOG="$LOG_DIR/can-metrics.csv"
TIMING_CSV="$LOG_DIR/timing-metrics.csv"

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
if ((VIRTUAL_FT)); then
    echo "Mode       : Interactive virtual F/T (2 ms guidance pipeline)"
else
    echo "Mode       : Normal full simulation"
fi
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
    echo "[2/5] Building controller, motors, CAN dashboard and GUIs..."
    local_gui_target="teaching_sim"
    if ((VIRTUAL_FT)); then
        local_gui_target="virtual_ft_gui"
    fi
    cmake --build "$BUILD_DIR" \
        --target \
            supervisor_silkit_sim \
            avatar_m_silkit_motor_bank \
            can_live_monitor \
            supervisor_panel \
            "$local_gui_target" \
        -j "$(nproc)"
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
: > "$CAN_MONITOR_LOG"
: > "$CAN_FRAME_LOG"
: > "$CAN_METRICS_LOG"
if ((VIRTUAL_FT)); then
    : > "$VFT_GUI_LOG"
    : > "$GUIDANCE_TRACE_LOG"
fi

echo
echo "[3/5] Starting SIL Kit CAN network..."

"$REGISTRY_BIN"     --listen-uri "$REGISTRY_URI"     >"$REGISTRY_LOG" 2>&1 &
REGISTRY_PID=$!
PIDS+=("$REGISTRY_PID")

wait_for_log     "$REGISTRY_LOG"     "SIL Kit Registry listening"     8     "$REGISTRY_PID"     "SIL Kit registry"

if ((START_GUI)); then
    if [[ ! -x "$CAN_MONITOR_BIN" ]]; then
        echo "[FAIL] CAN live monitor executable is missing." >&2
        exit 1
    fi

    SILKIT_REGISTRY_URI="$REGISTRY_URI" \
    CAN_FRAME_LOG="$CAN_FRAME_LOG" \
    CAN_METRICS_LOG="$CAN_METRICS_LOG" \
        "$CAN_MONITOR_BIN" \
        >"$CAN_MONITOR_LOG" 2>&1 &
    CAN_MONITOR_PID=$!
    PIDS+=("$CAN_MONITOR_PID")

    wait_for_log \
        "$CAN_MONITOR_LOG" \
        "[CAN MONITOR] ready:" \
        10 \
        "$CAN_MONITOR_PID" \
        "CAN live monitor"
fi

SILKIT_REGISTRY_URI="$REGISTRY_URI"     "$MOTOR_BIN"     >"$MOTOR_LOG" 2>&1 &
MOTOR_PID=$!
PIDS+=("$MOTOR_PID")

wait_for_log     "$MOTOR_LOG"     "AVATAR SIL Kit motor bank ready."     10     "$MOTOR_PID"     "AVATAR motor bank"

# The opt-in F/T mode preserves the real C controller and command gates.
# It cannot silently inherit stale guidance env vars in normal mode.
SUPERVISOR_ENV=(
    "SILKIT_REGISTRY_URI=$REGISTRY_URI"
    "MATLAB_IP=$MATLAB_IP"
    "SIM_EVENT_LOG=$EVENT_LOG"
    "SIM_TRACE_FILE=$TRACE_LOG"
    "GP_TIMING_CSV=$TIMING_CSV"
)
if ((VIRTUAL_FT)); then
    SUPERVISOR_ENV+=(
        "GP_GUIDANCE_DRY_RUN=1"
        "GP_GUIDANCE_SIM_MOTION=1"
        "GP_GUIDANCE_VIRTUAL_FT=1"
        "GP_GUIDANCE_TRACE_FILE=$GUIDANCE_TRACE_LOG"
    )
else
    SUPERVISOR_ENV+=(
        "GP_GUIDANCE_DRY_RUN=0"
        "GP_GUIDANCE_SIM_MOTION=0"
        "GP_GUIDANCE_VIRTUAL_FT=0"
    )
fi
env "${SUPERVISOR_ENV[@]}" "$SUPERVISOR_BIN" \
    >"$SUPERVISOR_LOG" 2>&1 &
SUPERVISOR_PID=$!
PIDS+=("$SUPERVISOR_PID")

wait_for_log     "$SUPERVISOR_LOG"     "[SILKIT] Controller connected:"     10     "$SUPERVISOR_PID"     "Supervisor"

echo
echo "[4/5] SIL Kit participants are running."
echo "      Registry + RobotControllerSupervisor + AVATAR nodes 1..6"
if ((START_GUI)); then
    echo "      + CanLiveMonitor passive CAN1 observer"
fi

if ((START_GUI)); then
    if [[ ! -x "$PANEL_BIN" ]]; then
        echo "[FAIL] Missing supervisor panel: $PANEL_BIN" >&2
        exit 1
    fi

    echo
    if ((VIRTUAL_FT)); then
        if [[ ! -x "$VFT_GUI_BIN" ]]; then
            echo "[FAIL] Missing F/T GUI: $VFT_GUI_BIN" >&2
            exit 1
        fi
        echo "[5/5] Starting the F/T sensor first, then the operator panel..."
        # Start the zero-wrench stream before TEACHING can be selected.
        "$VFT_GUI_BIN" >"$VFT_GUI_LOG" 2>&1 &
        VFT_GUI_PID=$!
        PIDS+=("$VFT_GUI_PID")
        sleep 0.3
        if ! kill -0 "$VFT_GUI_PID" 2>/dev/null; then
            echo "[FAIL] Virtual F/T GUI failed to start." >&2
            tail -n 80 "$VFT_GUI_LOG" >&2 || true
            exit 1
        fi
    else
        if [[ ! -x "$TEACH_BIN" ]]; then
            echo "[FAIL] Missing teaching simulator: $TEACH_BIN" >&2
            exit 1
        fi
        echo "[5/5] Starting HMI + teaching simulator..."
        "$TEACH_BIN" >"$TEACH_LOG" 2>&1 &
        TEACH_PID=$!
        PIDS+=("$TEACH_PID")
    fi

    "$PANEL_BIN" >"$PANEL_LOG" 2>&1 &
    PANEL_PID=$!
    PIDS+=("$PANEL_PID")
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
if ((VIRTUAL_FT)); then
    echo "   Virtual F/T GUI -> UDP :5013 -> existing C admittance controller"
    echo "   GUI status feedback <- UDP :5014"
    echo "   Use supervisor_panel: wait for IDLE, then select LINE/TEACHING."
    echo "   Hold Left/Right arrow in virtual_ft_gui to apply -X/+X force."
    echo "   Do not launch the Python virtual-F/T sender simultaneously."
fi
echo
echo " MATLAB:"
echo "   Keep MATLAB/can_robot_visualizer.m running on Windows."
echo "   Telemetry destination: $MATLAB_IP:5005"
echo
echo " Operator event log:"
echo "   $EVENT_LOG"
echo
echo " Detailed state trace:"
echo "   $TRACE_LOG"
echo
echo " CAN frame log:"
echo "   $CAN_FRAME_LOG"
if ((VIRTUAL_FT)); then
    echo " Guidance trace (wrench, twist, qref, actual joint feedback):"
    echo "   $GUIDANCE_TRACE_LOG"
    echo " Virtual F/T GUI log:"
    echo "   $VFT_GUI_LOG"
    echo
fi
echo " Host timing CSV:"
echo "   $TIMING_CSV"
echo " Analyze: python3 scripts/analyze_timing_csv.py $TIMING_CSV"
echo
echo " CAN metrics log:"
echo "   $CAN_METRICS_LOG"
echo " CAN monitor console log:"
echo "   $CAN_MONITOR_LOG"
echo
echo " Full runtime logs:"
echo "   $LOG_DIR"
echo
echo " Press Ctrl-C here to stop the complete simulation."
echo "============================================================"

# In interactive F/T mode mirror only meaningful Supervisor events.
# The full 10 Hz TRACE remains in supervisor.log; never flood the terminal.
# Byte-offset incremental reads run in this launcher once per second, not in
# FreeRTOS or the 2 ms control callback.
DIAG_BYTE_OFFSET=0
print_ft_diagnostics() {
    ((VIRTUAL_FT)) || return 0
    local size
    size=$(wc -c < "$SUPERVISOR_LOG")
    if ((size < DIAG_BYTE_OFFSET)); then
        DIAG_BYTE_OFFSET=0
    fi
    if ((size > DIAG_BYTE_OFFSET)); then
        tail -c +"$((DIAG_BYTE_OFFSET + 1))" "$SUPERVISOR_LOG" |
            grep -E '^\\[(FAULT|VFT|SIM|HMI RX|GUIDANCE)\\]' || true
        DIAG_BYTE_OFFSET=$size
    fi
}

echo
if ((VIRTUAL_FT)); then
    echo "[DIAGNOSTICS] Live state transitions, GUI input faults and first-fault cause:"
    print_ft_diagnostics
fi

# Keep the launcher alive and fail loudly if a core process exits.
while true; do
    print_ft_diagnostics
    core_entries=(
        "$REGISTRY_PID:registry:$REGISTRY_LOG"
        "$MOTOR_PID:motor-bank:$MOTOR_LOG"
        "$SUPERVISOR_PID:supervisor:$SUPERVISOR_LOG"
    )
    # Closing the F/T sender is an intentional end to the interactive
    # session. Stop the entire simulation instead of leaving a live
    # Supervisor that may fault after its source disappears.
    if ((VIRTUAL_FT)); then
        core_entries+=("$VFT_GUI_PID:virtual-ft-gui:$VFT_GUI_LOG")
    fi
    for entry in "${core_entries[@]}"
    do
        IFS=: read -r pid name log <<< "$entry"

        if ! kill -0 "$pid" 2>/dev/null; then
            echo
            if ((VIRTUAL_FT)) && [[ "$name" == "virtual-ft-gui" ]]; then
                echo "Virtual F/T GUI closed; stopping the simulation."
                exit 0
            fi
            echo "[FAIL] Core process '$name' stopped unexpectedly." >&2
            tail -n 100 "$log" 2>/dev/null || true
            exit 1
        fi
    done

    sleep 1
done
