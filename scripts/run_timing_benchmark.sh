#!/usr/bin/env bash
# Issue 5: repeatable host-only SIL Kit timing experiment.
# Each scenario writes its own raw CSV and analyzed summary/plots.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "\${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
BUILD_DIR="\${BUILD_DIR:-$ROOT/build-can-silkit}"
LOG_DIR="\${TIMING_RESULTS_DIR:-$ROOT/.simulation-logs/timing-benchmark}"
mkdir -p "$LOG_DIR"

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
    local csv="$LOG_DIR/\${name}.csv"
    local analysis_dir="$LOG_DIR/\${name}-analysis"
    rm -f "$csv"
    echo "[TIMING] Scenario: $name ($test)"
    GP_TIMING_CSV="$csv" \
        ctest --test-dir "$BUILD_DIR" \
        -R "^\${test}$" \
        --output-on-failure
    if [[ ! -s "$csv" ]]; then
        echo "[FAIL] Scenario produced no timing CSV: $csv" >&2
        exit 1
    fi
    python3 scripts/analyze_timing_csv.py "$csv" \
        --output-dir "$analysis_dir"
    echo "[TIMING] Raw: $csv"
    echo "[TIMING] Summary: $analysis_dir/summary_metrics.csv"
    echo "[TIMING] Report: $analysis_dir/timing_report.md"
}

run_scenario baseline supervisor_silkit_sequence
run_scenario flash supervisor_silkit_flash_sequence

if [[ "\${1:-}" == "--include-faults" ]]; then
    run_scenario flash_cancel supervisor_silkit_flash_cancel
    run_scenario flash_program_fail supervisor_silkit_flash_program_fail
elif [[ $# -gt 0 ]]; then
    echo "Usage: bash scripts/run_timing_benchmark.sh [--include-faults]" >&2
    exit 2
fi

echo "[TIMING] Complete. Compare baseline-analysis and flash-analysis."
echo "[TIMING] Results directory: $LOG_DIR"
