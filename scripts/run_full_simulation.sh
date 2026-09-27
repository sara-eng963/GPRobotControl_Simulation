#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

printf '\n============================================================\n'
printf ' FULL ROBOT PC SIMULATION\n'
printf '============================================================\n'
printf 'Building and launching:\n'
printf '  - FreeRTOS robot_simulator\n'
printf '  - KickCAT 6-axis A6-EC simulation\n'
printf '  - Real-style HMI window\n'
printf '  - Simulated hand-guiding window\n'
printf '  - MATLAB telemetry on UDP port 5005\n'
printf '============================================================\n\n'

printf '[1/3] Configuring CMake...\n'
cmake -S "${REPO_ROOT}" -B "${REPO_ROOT}/build"

printf '\n[2/3] Building simulator and HMI targets...\n'
cmake --build "${REPO_ROOT}/build" \
    --target robot_simulator robot_hmi teaching_sim \
    -j "$(nproc)"

printf '\n[3/3] Starting full simulation...\n\n'
exec bash "${SCRIPT_DIR}/run_hmi_state_simulation.sh"
