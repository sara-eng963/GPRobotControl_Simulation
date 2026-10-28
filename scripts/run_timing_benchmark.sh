#!/usr/bin/env bash
# Issue 5: repeatable host-only SIL Kit timing experiment.
# Each scenario writes its own raw CSV and analyzed summary/plots.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
BUILD_DIR="${BUILD_DIR:-$ROOT/build-can-silkit}"
LOG_DIR="${TIMING_RESULTS_DIR:-$BUILD_DIR/timing-benchmark}"
mkdir -p "$LOG_DIR"

# Resume a completed baseline without repeating the 66-second scenario.
FLASH_ONLY=0
PATH_ONLY=0
INCLUDE_FAULTS=0
NO_PLOTS=0
PATH_RUNS=1
for arg in "$@"; do
    case "$arg" in
        --flash-only) FLASH_ONLY=1 ;;
        --path-only) PATH_ONLY=1 ;;
        --include-faults) INCLUDE_FAULTS=1 ;;
        --no-plots) NO_PLOTS=1 ;;
        --path-runs=*) PATH_RUNS="${arg#--path-runs=}" ;;
        *)
            echo "Usage: bash scripts/run_timing_benchmark.sh [--flash-only|--path-only] [--path-runs=N] [--include-faults] [--no-plots]" >&2
            exit 2 ;;
    esac
done
if ! [[ "$PATH_RUNS" =~ ^[1-9][0-9]*$ ]] || ((PATH_RUNS > 20)); then
    echo "[FAIL] --path-runs must be 1..20" >&2
    exit 2
fi
PLOT_ARGS=()
if ((NO_PLOTS)); then PLOT_ARGS+=(--no-plots); fi

echo "[TIMING] CMake configure"
cmake -S . -B "$BUILD_DIR"

echo "[TIMING] Building required binaries"
cmake --build "$BUILD_DIR" --target \
    host_timing_trace_test \
    supervisor_silkit_sim \
    avatar_m_silkit_motor_bank \
    -j "$(nproc)"

echo "[TIMING] Checking recorder and analyzer"
ctest --test-dir "$BUILD_DIR" \
    -R '^(host_timing_trace_test|host_timing_analyzer_test)$' \
    --output-on-failure

run_scenario() {
    local name="$1"
    local test="$2"
    local csv="$LOG_DIR/${name}.csv"
    local analysis_dir="$LOG_DIR/${name}-analysis"
    rm -f "$csv"
    echo "[TIMING] Scenario: $name ($test)"
    HOST_TIMING_CSV="$csv" \
        ctest --test-dir "$BUILD_DIR" \
        -R "^${test}$" \
        --output-on-failure
    if [[ ! -s "$csv" ]]; then
        echo "[FAIL] Scenario produced no timing CSV: $csv" >&2
        exit 1
    fi
    python3 scripts/analyze_timing_csv.py "$csv" \
        --output-dir "$analysis_dir" "${PLOT_ARGS[@]}"
    echo "[TIMING] Raw: $csv"
    echo "[TIMING] Summary: $analysis_dir/summary_metrics.csv"
    echo "[TIMING] Report: $analysis_dir/timing_report.md"
}

if ((PATH_ONLY)); then
    # Preserve legacy single-run filenames. Multi-run results have unique
    # names; baseline and modified branches must use separate LOG_DIRs.
    for ((run = 1; run <= PATH_RUNS; ++run)); do
        if ((PATH_RUNS == 1)); then
            scenario=path_diagnostics
        else
            scenario="path_diagnostics_run${run}"
        fi
        run_scenario "$scenario" supervisor_silkit_flash_sequence
    done
    echo "[TIMING] $PATH_RUNS complete path run(s). Baseline/flash unchanged."
    exit 0
fi

if ((!FLASH_ONLY)); then
    run_scenario baseline supervisor_silkit_sequence
elif [[ -s "$LOG_DIR/baseline.csv" ]]; then
    echo "[TIMING] Reusing previous baseline.csv (not rerunning)."
else
    echo "[WARN] --flash-only requested but baseline.csv does not exist." >&2
fi
run_scenario flash supervisor_silkit_flash_sequence

if ((INCLUDE_FAULTS)); then
    run_scenario flash_cancel supervisor_silkit_flash_cancel
    run_scenario flash_program_fail supervisor_silkit_flash_program_fail
fi

echo "[TIMING] Complete. Compare baseline-analysis and flash-analysis."
echo "[TIMING] Results directory: $LOG_DIR"
