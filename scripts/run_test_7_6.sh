#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"

cd "${ROOT_DIR}"

echo "============================================================"
echo "TEST 7.6 - OFFLINE TRAJECTORY STORAGE CAPACITY"
echo "============================================================"
echo "No HMI, KickCAT, or full robot simulator is required."
echo "The test builds deterministic taught programs, runs them through"
echo "ControlCore -> Path Validation, and measures the validated CSP"
echo "sample storage required in external flash."
echo

cmake -S . -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target path_validation_test -j

echo
"${BUILD_DIR}/path_validation_test"
