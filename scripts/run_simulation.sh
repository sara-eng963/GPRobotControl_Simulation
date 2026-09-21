#!/usr/bin/env bash

set -Eeuo pipefail

# ============================================================================
# Full PC EtherCAT simulation launcher
# ============================================================================
#
# Starts the complete simulated control stack with CPU isolation:
#
#   CPU 1 -> KickCAT network_simulator
#   CPU 2 -> FreeRTOS + SOEM controller
#   CPU 3 -> Raylib HMI (low priority)
#
# It recreates the virtual EtherCAT link every run:
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

KICKCAT_PIDFILE="${LOG_DIR}/kickcat.pid"
CONTROLLER_PIDFILE="${LOG_DIR}/controller.pid"

KICKCAT_PID=""
CONTROLLER_PID=""
HMI_PID=""
KICKCAT_LAUNCHER_PID=""
CONTROLLER_LAUNCHER_PID=""
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

root_process_alive()
{
    local pid="$1"
    [[ -n "${pid}" ]] && sudo -n kill -0 "${pid}" 2>/dev/null
}

wait_for_pidfile()
{
    local pidfile="$1"
    local launcher_pid="$2"
    local description="$3"

    for _ in $(seq 1 50)
    do
        if [[ -s "${pidfile}" ]]
        then
            cat "${pidfile}"
            return 0
        fi

        if ! process_alive "${launcher_pid}"
        then
            printf 'ERROR: %s launcher exited before writing its PID.\n' \
                "${description}" >&2
            return 1
        fi

        sleep 0.1
    done

    printf 'ERROR: timed out waiting for %s PID.\n' "${description}" >&2
    return 1
}

stop_old_processes()
{
    printf '[1/6] Stopping stale simulation processes...\n'

    # The bracketed first character prevents pkill from matching its own
    # command line. The previous launcher used raw -f patterns, which could
    # kill the sudo/pkill helper itself and made shutdown unreliable.
    pkill -TERM -f '[m]ock_hmi' 2>/dev/null || true
    sudo pkill -TERM -f '[f]reertos_pc_test' 2>/dev/null || true
    sudo pkill -TERM -f '[n]etwork_simulator.*ecatB' 2>/dev/null || true

    sleep 0.5

    pkill -KILL -f '[m]ock_hmi' 2>/dev/null || true
    sudo pkill -KILL -f '[f]reertos_pc_test' 2>/dev/null || true
    sudo pkill -KILL -f '[n]etwork_simulator.*ecatB' 2>/dev/null || true
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
    set +e

    printf '\nStopping simulation...\n'

    # HMI is an ordinary user process.
    if process_alive "${HMI_PID}"
    then
        kill -TERM "${HMI_PID}" 2>/dev/null || true
    fi

    # The privileged programs are launched through sudo, but their real PIDs
    # are written from inside the sudo child before exec(). Kill those exact
    # PIDs instead of using pkill -f.
    if root_process_alive "${CONTROLLER_PID}"
    then
        # SIGINT lets main.c execute its clean EtherCAT/servo shutdown path.
        sudo -n kill -INT "${CONTROLLER_PID}" 2>/dev/null || true
    fi

    if root_process_alive "${KICKCAT_PID}"
    then
        sudo -n kill -TERM "${KICKCAT_PID}" 2>/dev/null || true
    fi

    sleep 1

    if root_process_alive "${CONTROLLER_PID}"
    then
        sudo -n kill -TERM "${CONTROLLER_PID}" 2>/dev/null || true
    fi

    if root_process_alive "${KICKCAT_PID}"
    then
        sudo -n kill -TERM "${KICKCAT_PID}" 2>/dev/null || true
    fi

    sleep 0.5

    if root_process_alive "${CONTROLLER_PID}"
    then
        sudo -n kill -KILL "${CONTROLLER_PID}" 2>/dev/null || true
    fi

    if root_process_alive "${KICKCAT_PID}"
    then
        sudo -n kill -KILL "${KICKCAT_PID}" 2>/dev/null || true
    fi

    # Reap/stop the sudo launcher wrappers if they are still around.
    if process_alive "${CONTROLLER_LAUNCHER_PID}"
    then
        kill -TERM "${CONTROLLER_LAUNCHER_PID}" 2>/dev/null || true
    fi

    if process_alive "${KICKCAT_LAUNCHER_PID}"
    then
        kill -TERM "${KICKCAT_LAUNCHER_PID}" 2>/dev/null || true
    fi

    remove_virtual_ethernet

    if [[ -n "${SUDO_KEEPALIVE_PID}" ]]
    then
        kill "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
    fi

    rm -f "${KICKCAT_PIDFILE}" "${CONTROLLER_PIDFILE}"

    printf 'Stopped. ecatA/ecatB removed.\n'
    printf 'Logs kept in: %s\n' "${LOG_DIR}"
}

# Ctrl+C/TERM should leave the main loop immediately. EXIT performs the actual
# cleanup exactly once.
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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
rm -f "${KICKCAT_PIDFILE}" "${CONTROLLER_PIDFILE}"

printf '[3/6] Starting six A6-EC KickCAT slaves on CPU %s...\n' "${KICKCAT_CPU}"

sudo sh -c '
    pidfile="$1"
    shift
    printf "%s\n" "$$" > "$pidfile"
    exec "$@"
' sh \
    "${KICKCAT_PIDFILE}" \
    taskset -c "${KICKCAT_CPU}" \
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
KICKCAT_LAUNCHER_PID=$!

KICKCAT_PID="$(wait_for_pidfile "${KICKCAT_PIDFILE}" "${KICKCAT_LAUNCHER_PID}" "KickCAT")"

sleep 1

if ! root_process_alive "${KICKCAT_PID}"
then
    printf 'ERROR: KickCAT exited during startup. Last log lines:\n' >&2
    tail -n 40 "${KICKCAT_LOG}" >&2 || true
    exit 1
fi

printf '[4/6] Starting FreeRTOS + SOEM controller on CPU %s...\n' "${CONTROLLER_CPU}"

sudo sh -c '
    pidfile="$1"
    shift
    printf "%s\n" "$$" > "$pidfile"
    exec "$@"
' sh \
    "${CONTROLLER_PIDFILE}" \
    taskset -c "${CONTROLLER_CPU}" \
    nice -n -10 \
    "${CONTROLLER_BIN}" \
    >"${CONTROLLER_LOG}" 2>&1 &
CONTROLLER_LAUNCHER_PID=$!

CONTROLLER_PID="$(wait_for_pidfile "${CONTROLLER_PIDFILE}" "${CONTROLLER_LAUNCHER_PID}" "controller")"

sleep 2

if ! root_process_alive "${CONTROLLER_PID}"
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

    if ! root_process_alive "${CONTROLLER_PID}"
    then
        printf '\nERROR: controller stopped unexpectedly. Last log lines:\n' >&2
        tail -n 60 "${CONTROLLER_LOG}" >&2 || true
        exit 1
    fi

    if ! root_process_alive "${KICKCAT_PID}"
    then
        printf '\nERROR: KickCAT stopped unexpectedly. Last log lines:\n' >&2
        tail -n 60 "${KICKCAT_LOG}" >&2 || true
        exit 1
    fi

    sleep 1
done
