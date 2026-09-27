#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
KICKCAT_DIR="${KICKCAT_DIR:-${HOME}/KickCAT}"

KICKCAT_BIN="${KICKCAT_DIR}/build/simulation/network_simulator"
VETH_SCRIPT="${KICKCAT_DIR}/simulation/create_virtual_ethernet.sh"
A6_CONFIG="${KICKCAT_DIR}/simulation/slave_configs/A6-EC.json"

CONTROLLER_BIN="${REPO_ROOT}/build/robot_simulator"
PANEL_BIN="${REPO_ROOT}/build/robot_hmi"
TEACH_BIN="${REPO_ROOT}/build/teaching_sim"

LOG_DIR="${TMPDIR:-/tmp}/gprobot_hmi_state_${USER}"
mkdir -p "${LOG_DIR}"

KICKCAT_PID=""
CONTROLLER_PID=""
PANEL_PID=""
TEACH_PID=""

cleanup()
{
    set +e

    [[ -n "${PANEL_PID}" ]] && kill "${PANEL_PID}" 2>/dev/null || true
    [[ -n "${TEACH_PID}" ]] && kill "${TEACH_PID}" 2>/dev/null || true

    if [[ -n "${CONTROLLER_PID}" ]]
    then
        sudo kill -INT "${CONTROLLER_PID}" 2>/dev/null || true
    fi

    if [[ -n "${KICKCAT_PID}" ]]
    then
        sudo kill -TERM "${KICKCAT_PID}" 2>/dev/null || true
    fi

    sleep 0.4

    if ip link show ecatA >/dev/null 2>&1
    then
        sudo ip link del ecatA 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

for required in "${KICKCAT_BIN}" "${VETH_SCRIPT}" "${A6_CONFIG}"                 "${CONTROLLER_BIN}" "${PANEL_BIN}" "${TEACH_BIN}"
do
    if [[ ! -e "${required}" ]]
    then
        printf 'Missing required file: %s\n' "${required}" >&2
        exit 1
    fi
done

printf '\n============================================================\n'
printf ' Real HMI + actual state-machine PC simulation\n'
printf '============================================================\n'

sudo -v

# Recreate a clean virtual EtherCAT link.
if ip link show ecatA >/dev/null 2>&1
then
    sudo ip link del ecatA 2>/dev/null || true
fi

if ip link show ecatB >/dev/null 2>&1
then
    sudo ip link del ecatB 2>/dev/null || true
fi

sudo "${VETH_SCRIPT}" create ecat

printf '[1/4] Starting KickCAT (6 A6-EC virtual drives)...\n'

sudo "${KICKCAT_BIN}" ecatB     "${A6_CONFIG}" "${A6_CONFIG}" "${A6_CONFIG}"     "${A6_CONFIG}" "${A6_CONFIG}" "${A6_CONFIG}"     >"${LOG_DIR}/kickcat.log" 2>&1 &

KICKCAT_PID=$!

sleep 1

printf '[2/4] Starting FreeRTOS global-state simulator...\n'

sudo "${CONTROLLER_BIN}"     >"${LOG_DIR}/controller.log" 2>&1 &

CONTROLLER_PID=$!

sleep 2

if ! sudo kill -0 "${CONTROLLER_PID}" 2>/dev/null
then
    printf 'Controller failed to start.\n' >&2
    tail -n 80 "${LOG_DIR}/controller.log" >&2 || true
    exit 1
fi

printf '[3/4] Opening real HMI panel window...\n'

"${PANEL_BIN}"     >"${LOG_DIR}/panel.log" 2>&1 &

PANEL_PID=$!

printf '[4/4] Opening simulated hand-guiding window...\n'

"${TEACH_BIN}"     >"${LOG_DIR}/teaching.log" 2>&1 &

TEACH_PID=$!

printf '\nTwo HMI windows are running.\n'
printf 'Controller log: %s\n' "${LOG_DIR}/controller.log"
printf 'MATLAB telemetry remains on UDP port 5005.\n'
printf 'Close the main HMI window or press Ctrl+C here to stop.\n\n'

while kill -0 "${PANEL_PID}" 2>/dev/null
do
    if ! sudo kill -0 "${CONTROLLER_PID}" 2>/dev/null
    then
        printf 'Controller stopped unexpectedly.\n' >&2
        tail -n 80 "${LOG_DIR}/controller.log" >&2 || true
        exit 1
    fi

    sleep 1
done
