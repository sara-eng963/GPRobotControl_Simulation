#!/usr/bin/env bash

set -Eeuo pipefail

# The old pre-global-FSM simulation launcher used freertos_pc_test + HMI-Mock.
# The canonical simulator is now the FreeRTOS/global-FSM workflow.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_hmi_state_simulation.sh" "$@"
