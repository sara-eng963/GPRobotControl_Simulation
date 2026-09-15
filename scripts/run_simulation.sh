#!/usr/bin/env bash

set -Eeuo pipefail

# ============================================================================
# Full PC EtherCAT simulation launcher
# ============================================================================
#
# Starts the complete simulated control stack with the CPU isolation that keeps
# the 1 ms SOEM/KickCAT exchange stable while the Raylib HMI is open:
#
#   CPU 1 -> KickCAT network_simulator
#   CPU 2 -> FreeRTOS + SOEM controller
#   CPU 3 -> Raylib HMI (low priority)
#
# It also recreates the virtual EtherCAT link every run:
#
#   SOEM master -> ecatA <---- veth pair ----> ecatB -> KickCAT slaves
#
# Close the HMI or press Ctrl+C in this terminal to stop the whole simulation.
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

KICKCAT_DIR="${KICKCAT_DIR:-${HOME}/KickCAT}"

KICKCAT_CPU="${KICKCAT_CPU:-1}"
CONTROLLER_CPU="${CONTROLLER_CPU:-2}"
HMI_CPU="${HMI_CPU:-3}"

KICKCAT_BIN="${KICKCAT_DIR}/build/simulation/network_simulator"
VETH_SCRIPT="${KICKCAT_DIR}/simulation/create_virtual_ethernet.sh"
A6_CONFIG="${KICKCAT_DIR}/simulation/slave_configs/A6-EC.json"

CONTROLLER_BIN="${REPO_ROOT}/build/freertos_pc_test"
HMI_BIN="${REPO_ROOT}/build/mock_hmi"

LOG_DIR="${TMPDIR:-/tmp}/gprobot_simulation_${USER}"
mkdir -p "${LOG_DIR}"

KICKCAT_LOG="${LOG_DIR}/kickcat.log"
CONTROLLER_LOG="${LOG_DIR}/controller.log"
HMI_LOG="${LOG_DIR}/hmi.log"

KICKCAT_PID=""
CONTROLLER_PID=""
HMI_PID=""
SUDO_KEEPALIVE_PID=""
CLEANUP_DONE=0

print_header()
{
    printf '\n============================================================\n'
    printf ' GPRobotControl full EtherCAT simulation\n'
    printf '============================================================\n'
}

require_file()
{
    local path="$1"
    local description="$2"

    if [[ ! -f "${path}" ]]
    then
        printf 'ERROR: %s not found:\n  %s\n' "${description}" "${path}" >&2
        exit 1
    fi
}

require_executable()
{
    local path="$1"
    local description="$2"

    if [[ ! -x "${path}" ]]
    then
        printf 'ERROR: %s not found or not executable:\n  %s\n' "${description}" "${path}" >&2
        exit 1
    fi
}

process_alive()
{
    local pid="$1"
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

stop_old_processes()
{
    printf '[1/6] Stopping stale simulation processes...\n'

    pkill -TERM -f "${HMI_BIN}" 2>/dev/null || true
    sudo pkill -TERM -f "${CONTROLLER_BIN}" 2>/dev/null || true
    sudo pkill -TERM -f "${KICKCAT_BIN}" 2>/dev/null || true

    sleep 0.5

    sudo pkill -KILL -f "${CONTROLLER_BIN}" 2>/dev/null || true
    sudo pkill -KILL -f "${KICKCAT_BIN}" 2>/dev/null || true
}

remove_virtual_ethernet()
{
    # Removing either end of a veth pair normally removes both. Check both so
    # the script also recovers cleanly from a partially stale setup.
    if ip link show ecatA >/dev/null 2>&1
    then
        sudo ip link del ecatA 2>/dev/null || true
    fi

    if ip link show ecatB >/dev/null 2>&1
    then
        sudo ip link del ecatB 2>/dev/null || true
    fi
}

cleanup()
{
    if [[ "${CLEANUP_DONE}" -eq 1 ]]
    then
        return
    fi

    CLEANUP_DONE=1
    trap - EXIT INT TERM

    printf '\nStopping simulation...\n'

    if [[ -n "${HMI_PID}" ]]
    then
        kill "${HMI_PID}" 2>/dev/null || true
    fi

    # Give the controller SIGINT first so main.c can perform its clean servo /
    # EtherCAT shutdown path before anything is force-killed.
    sudo pkill -INT -f "${CONTROLLER_BIN}" 2>/dev/null || true
    sudo pkill -INT -f "${KICKCAT_BIN}" 2>/dev/null || true

    sleep 1

    sudo pkill -TERM -f "${CONTROLLER_BIN}" 2>/dev/null || true
    sudo pkill -TERM -f "${KICKCAT_BIN}" 2>/dev/null || true
    pkill -TERM -f "${HMI_BIN}" 2>/dev/null || true

    remove_virtual_ethernet

    if [[ -n "${SUDO_KEEPALIVE_PID}" ]]
    then
        kill "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
    fi

    printf 'Stopped. ecatA/ecatB removed.\n'
    printf 'Logs kept in: %s\n' "${LOG_DIR}"
}

trap cleanup EXIT INT TERM

print_header

require_executable "${VETH_SCRIPT}" "KickCAT virtual-Ethernet helper"
require_executable "${KICKCAT_BIN}" "KickCAT network_simulator"
require_file "${A6_CONFIG}" "A6-EC KickCAT slave configuration"
require_executable "${CONTROLLER_BIN}" "FreeRTOS/SOEM controller"
require_executable "${HMI_BIN}" "Raylib HMI"

CPU_COUNT="$(nproc)"

for cpu in "${KICKCAT_CPU}" "${CONTROLLER_CPU}" "${HMI_CPU}"
do
    if ! [[ "${cpu}" =~ ^[0-9]+$ ]] || (( cpu >= CPU_COUNT ))
    then
        printf 'ERROR: requested CPU %s is unavailable; WSL reports %s logical CPUs.\n' \
            "${cpu}" "${CPU_COUNT}" >&2
        exit 1
    fi
done

if [[ "${KICKCAT_CPU}" == "${CONTROLLER_CPU}" || \
      "${KICKCAT_CPU}" == "${HMI_CPU}" || \
      "${CONTROLLER_CPU}" == "${HMI_CPU}" ]]
then
    printf 'ERROR: KickCAT, controller, and HMI must use different CPUs.\n' >&2
    exit 1
fi

printf 'CPU isolation: KickCAT=%s  Controller=%s  HMI=%s\n' \
    "${KICKCAT_CPU}" "${CONTROLLER_CPU}" "${HMI_CPU}"
printf 'KickCAT: %s\n' "${KICKCAT_DIR}"
printf 'Project: %s\n' "${REPO_ROOT}"
printf 'Logs:    %s\n\n' "${LOG_DIR}"

# Authenticate once before any process is sent to the background. This avoids a
# background sudo process waiting invisibly for a password.
sudo -v

# Keep the sudo timestamp alive for clean shutdown even during long sessions.
(
    while sleep 60
    do
        sudo -n true 2>/dev/null || exit 0
    done
) &
SUDO_KEEPALIVE_PID=$!

stop_old_processes

printf '[2/6] Recreating EtherCAT virtual Ethernet pair...\n'
remove_virtual_ethernet

(
    cd "${KICKCAT_DIR}"
    sudo ./simulation/create_virtual_ethernet.sh create ecat
)

sudo ip link set ecatA up
sudo ip link set ecatB up

if ! ip link show ecatA >/dev/null 2>&1 || ! ip link show ecatB >/dev/null 2>&1
then
    printf 'ERROR: ecatA/ecatB were not created successfully.\n' >&2
    exit 1
fi

printf '      ecatA UP -> SOEM master\n'
printf '      ecatB UP -> KickCAT simulator\n'

: > "${KICKCAT_LOG}"
: > "${CONTROLLER_LOG}"
: > "${HMI_LOG}"

printf '[3/6] Starting six A6-EC KickCAT slaves on CPU %s...\n' "${KICKCAT_CPU}"

sudo taskset -c "${KICKCAT_CPU}" \
    "${KICKCAT_BIN}" \
    -i ecatB \
    -s \
    "${A6_CONFIG}" \
    "${A6_CONFIG}" \
    "${A6_CONFIG}" \
    "${A6_CONFIG}" \
    "${A6_CONFIG}" \
    "${A6_CONFIG}" \
    >"${KICKCAT_LOG}" 2>&1 &
KICKCAT_PID=$!

sleep 1

if ! process_alive "${KICKCAT_PID}"
then
    printf 'ERROR: KickCAT exited during startup. Last log lines:\n' >&2
    tail -n 40 "${KICKCAT_LOG}" >&2 || true
    exit 1
fi

printf '[4/6] Starting FreeRTOS + SOEM controller on CPU %s...\n' "${CONTROLLER_CPU}"

sudo taskset -c "${CONTROLLER_CPU}" \
    nice -n -10 \
    "${CONTROLLER_BIN}" \
    >"${CONTROLLER_LOG}" 2>&1 &
CONTROLLER_PID=$!

sleep 2

if ! process_alive "${CONTROLLER_PID}"
then
    printf 'ERROR: controller exited during startup. Last log lines:\n' >&2
    tail -n 60 "${CONTROLLER_LOG}" >&2 || true
    exit 1
fi

printf '[5/6] Starting HMI on CPU %s at low priority...\n' "${HMI_CPU}"

taskset -c "${HMI_CPU}" \
    nice -n 15 \
    "${HMI_BIN}" \
    >"${HMI_LOG}" 2>&1 &
HMI_PID=$!

sleep 1

if ! process_alive "${HMI_PID}"
then
    printf 'ERROR: HMI exited during startup. Last log lines:\n' >&2
    tail -n 60 "${HMI_LOG}" >&2 || true
    exit 1
fi

printf '[6/6] Full simulation is running.\n\n'
printf 'Architecture:\n'
printf '  FreeRTOS/SOEM -- ecatA <== veth ==> ecatB -- KickCAT x6\n'
printf '        CPU %-2s                         CPU %s\n' "${CONTROLLER_CPU}" "${KICKCAT_CPU}"
printf '  HMI -> CPU %s (nice +15)\n\n' "${HMI_CPU}"
printf 'Logs:\n'
printf '  Controller: %s\n' "${CONTROLLER_LOG}"
printf '  KickCAT:    %s\n' "${KICKCAT_LOG}"
printf '  HMI:        %s\n\n' "${HMI_LOG}"
printf 'To watch WKC from another terminal:\n'
printf '  tail -f %q\n\n' "${CONTROLLER_LOG}"
printf 'Close the HMI window or press Ctrl+C here to stop everything.\n'

while true
do
    if ! process_alive "${HMI_PID}"
    then
        printf '\nHMI closed; ending simulation session.\n'
        break
    fi

    if ! process_alive "${CONTROLLER_PID}"
    then
        printf '\nERROR: controller stopped unexpectedly. Last log lines:\n' >&2
        tail -n 60 "${CONTROLLER_LOG}" >&2 || true
        exit 1
    fi

    if ! process_alive "${KICKCAT_PID}"
    then
        printf '\nERROR: KickCAT stopped unexpectedly. Last log lines:\n' >&2
        tail -n 60 "${KICKCAT_LOG}" >&2 || true
        exit 1
    fi

    sleep 1
done
