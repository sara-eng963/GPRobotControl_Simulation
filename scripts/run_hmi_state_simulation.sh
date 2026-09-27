#!/usr/bin/env bash

set -Eeuo pipefail

# ============================================================================
# Real-style HMI + FreeRTOS PC simulator launcher
# ============================================================================
#
# Timing matters for the 1 ms SOEM/KickCAT loop. Keep the EtherCAT simulator,
# controller and graphical HMIs on separate CPU resources so Raylib/WSLg does
# not intermittently starve Homing or cyclic PDO exchange.
#
#   CPU 1 -> KickCAT network_simulator
#   CPU 2 -> FreeRTOS + SOEM robot_simulator
#   CPU 3 -> robot_hmi + teaching_sim (nice +15)
#
#   robot_simulator/SOEM -> ecatA <== veth ==> ecatB -> KickCAT x6
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

CONTROLLER_BIN="${REPO_ROOT}/build/robot_simulator"
PANEL_BIN="${REPO_ROOT}/build/robot_hmi"
TEACH_BIN="${REPO_ROOT}/build/teaching_sim"

LOG_DIR="${TMPDIR:-/tmp}/gprobot_hmi_state_${USER}"
mkdir -p "${LOG_DIR}"

KICKCAT_LOG="${LOG_DIR}/kickcat.log"
CONTROLLER_LOG="${LOG_DIR}/controller.log"
PANEL_LOG="${LOG_DIR}/panel.log"
TEACH_LOG="${LOG_DIR}/teaching.log"

KICKCAT_PIDFILE="${LOG_DIR}/kickcat.pid"
CONTROLLER_PIDFILE="${LOG_DIR}/controller.pid"

KICKCAT_PID=""
CONTROLLER_PID=""
PANEL_PID=""
TEACH_PID=""
KICKCAT_LAUNCHER_PID=""
CONTROLLER_LAUNCHER_PID=""
SUDO_KEEPALIVE_PID=""
CLEANUP_DONE=0

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
            printf 'ERROR: %s launcher exited before writing its PID.\n'                 "${description}" >&2
            return 1
        fi

        sleep 0.1
    done

    printf 'ERROR: timed out waiting for %s PID.\n' "${description}" >&2
    return 1
}

remove_virtual_ethernet()
{
    if ip link show ecatA >/dev/null 2>&1
    then
        sudo ip link del ecatA 2>/dev/null || true
    fi

    if ip link show ecatB >/dev/null 2>&1
    then
        sudo ip link del ecatB 2>/dev/null || true
    fi
}

stop_old_processes()
{
    printf '[1/6] Stopping stale simulator processes...\n'

    pkill -TERM -f '[r]obot_hmi' 2>/dev/null || true
    pkill -TERM -f '[t]eaching_sim' 2>/dev/null || true
    sudo pkill -TERM -f '[r]obot_simulator' 2>/dev/null || true
    sudo pkill -TERM -f '[n]etwork_simulator.*ecatB' 2>/dev/null || true

    sleep 0.5

    pkill -KILL -f '[r]obot_hmi' 2>/dev/null || true
    pkill -KILL -f '[t]eaching_sim' 2>/dev/null || true
    sudo pkill -KILL -f '[r]obot_simulator' 2>/dev/null || true
    sudo pkill -KILL -f '[n]etwork_simulator.*ecatB' 2>/dev/null || true
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

    process_alive "${PANEL_PID}" && kill -TERM "${PANEL_PID}" 2>/dev/null || true
    process_alive "${TEACH_PID}" && kill -TERM "${TEACH_PID}" 2>/dev/null || true

    if root_process_alive "${CONTROLLER_PID}"
    then
        sudo -n kill -INT "${CONTROLLER_PID}" 2>/dev/null || true
    fi

    if root_process_alive "${KICKCAT_PID}"
    then
        sudo -n kill -TERM "${KICKCAT_PID}" 2>/dev/null || true
    fi

    sleep 1

    root_process_alive "${CONTROLLER_PID}" &&         sudo -n kill -TERM "${CONTROLLER_PID}" 2>/dev/null || true

    root_process_alive "${KICKCAT_PID}" &&         sudo -n kill -TERM "${KICKCAT_PID}" 2>/dev/null || true

    sleep 0.5

    root_process_alive "${CONTROLLER_PID}" &&         sudo -n kill -KILL "${CONTROLLER_PID}" 2>/dev/null || true

    root_process_alive "${KICKCAT_PID}" &&         sudo -n kill -KILL "${KICKCAT_PID}" 2>/dev/null || true

    process_alive "${CONTROLLER_LAUNCHER_PID}" &&         kill -TERM "${CONTROLLER_LAUNCHER_PID}" 2>/dev/null || true

    process_alive "${KICKCAT_LAUNCHER_PID}" &&         kill -TERM "${KICKCAT_LAUNCHER_PID}" 2>/dev/null || true

    remove_virtual_ethernet

    if [[ -n "${SUDO_KEEPALIVE_PID}" ]]
    then
        kill "${SUDO_KEEPALIVE_PID}" 2>/dev/null || true
    fi

    rm -f "${KICKCAT_PIDFILE}" "${CONTROLLER_PIDFILE}"

    printf 'Stopped. ecatA/ecatB removed.\n'
    printf 'Logs kept in: %s\n' "${LOG_DIR}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for required in     "${KICKCAT_BIN}"     "${VETH_SCRIPT}"     "${A6_CONFIG}"     "${CONTROLLER_BIN}"     "${PANEL_BIN}"     "${TEACH_BIN}"
do
    if [[ ! -e "${required}" ]]
    then
        printf 'Missing required file: %s\n' "${required}" >&2
        exit 1
    fi
done

CPU_COUNT="$(nproc)"

for cpu in "${KICKCAT_CPU}" "${CONTROLLER_CPU}" "${HMI_CPU}"
do
    if ! [[ "${cpu}" =~ ^[0-9]+$ ]] || (( cpu >= CPU_COUNT ))
    then
        printf 'ERROR: requested CPU %s is unavailable; WSL reports %s logical CPUs.\n'             "${cpu}" "${CPU_COUNT}" >&2
        exit 1
    fi
done

if [[ "${KICKCAT_CPU}" == "${CONTROLLER_CPU}" ||       "${KICKCAT_CPU}" == "${HMI_CPU}" ||       "${CONTROLLER_CPU}" == "${HMI_CPU}" ]]
then
    printf 'ERROR: KickCAT, controller and HMI CPU assignments must differ.\n' >&2
    exit 1
fi

printf '\n============================================================\n'
printf ' Real HMI + FreeRTOS state-machine PC simulation\n'
printf '============================================================\n'
printf 'CPU isolation: KickCAT=%s  Controller=%s  HMI=%s\n'     "${KICKCAT_CPU}" "${CONTROLLER_CPU}" "${HMI_CPU}"
printf 'Logs: %s\n' "${LOG_DIR}"
printf '============================================================\n\n'

sudo -v

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
: > "${PANEL_LOG}"
: > "${TEACH_LOG}"
rm -f "${KICKCAT_PIDFILE}" "${CONTROLLER_PIDFILE}"

printf '[3/6] Starting six A6-EC KickCAT slaves on CPU %s...\n' "${KICKCAT_CPU}"

sudo sh -c '
    pidfile="$1"
    shift
    printf "%s\n" "$$" > "$pidfile"
    exec "$@"
' sh     "${KICKCAT_PIDFILE}"     taskset -c "${KICKCAT_CPU}"     "${KICKCAT_BIN}"     -i ecatB     -s     "${A6_CONFIG}" "${A6_CONFIG}" "${A6_CONFIG}"     "${A6_CONFIG}" "${A6_CONFIG}" "${A6_CONFIG}"     >"${KICKCAT_LOG}" 2>&1 &
KICKCAT_LAUNCHER_PID=$!

KICKCAT_PID="$(wait_for_pidfile "${KICKCAT_PIDFILE}" "${KICKCAT_LAUNCHER_PID}" "KickCAT")"

sleep 1

if ! root_process_alive "${KICKCAT_PID}"
then
    printf 'ERROR: KickCAT exited during startup. Last log lines:\n' >&2
    tail -n 60 "${KICKCAT_LOG}" >&2 || true
    exit 1
fi

printf '[4/6] Starting FreeRTOS + SOEM simulator on CPU %s...\n' "${CONTROLLER_CPU}"

sudo sh -c '
    pidfile="$1"
    shift
    printf "%s\n" "$$" > "$pidfile"
    exec "$@"
' sh     "${CONTROLLER_PIDFILE}"     taskset -c "${CONTROLLER_CPU}"     nice -n -10     "${CONTROLLER_BIN}"     >"${CONTROLLER_LOG}" 2>&1 &
CONTROLLER_LAUNCHER_PID=$!

CONTROLLER_PID="$(wait_for_pidfile "${CONTROLLER_PIDFILE}" "${CONTROLLER_LAUNCHER_PID}" "robot simulator")"

sleep 2

if ! root_process_alive "${CONTROLLER_PID}"
then
    printf 'ERROR: robot_simulator exited during startup. Last log lines:\n' >&2
    tail -n 80 "${CONTROLLER_LOG}" >&2 || true
    exit 1
fi

printf '[5/6] Opening both HMI windows on CPU %s at low priority...\n' "${HMI_CPU}"

taskset -c "${HMI_CPU}"     nice -n 15     "${PANEL_BIN}"     >"${PANEL_LOG}" 2>&1 &
PANEL_PID=$!

taskset -c "${HMI_CPU}"     nice -n 15     "${TEACH_BIN}"     >"${TEACH_LOG}" 2>&1 &
TEACH_PID=$!

sleep 1

if ! process_alive "${PANEL_PID}"
then
    printf 'ERROR: robot_hmi exited during startup.\n' >&2
    tail -n 60 "${PANEL_LOG}" >&2 || true
    exit 1
fi

if ! process_alive "${TEACH_PID}"
then
    printf 'ERROR: teaching_sim exited during startup.\n' >&2
    tail -n 60 "${TEACH_LOG}" >&2 || true
    exit 1
fi

printf '[6/6] Full simulation is running.\n\n'
printf 'Architecture:\n'
printf '  robot_simulator/SOEM -- ecatA <== veth ==> ecatB -- KickCAT x6\n'
printf '        CPU %-2s                                  CPU %s\n'     "${CONTROLLER_CPU}" "${KICKCAT_CPU}"
printf '  robot_hmi + teaching_sim -> CPU %s, nice +15\n\n' "${HMI_CPU}"
printf 'MATLAB telemetry: UDP port 5005\n'
printf 'Controller log:  %s\n' "${CONTROLLER_LOG}"
printf 'KickCAT log:     %s\n' "${KICKCAT_LOG}"
printf 'Panel log:       %s\n' "${PANEL_LOG}"
printf 'Teaching log:    %s\n\n' "${TEACH_LOG}"
printf 'Close the main HMI or press Ctrl+C here to stop everything.\n'

while true
do
    if ! process_alive "${PANEL_PID}"
    then
        printf '\nMain HMI closed; ending simulation session.\n'
        break
    fi

    if ! root_process_alive "${CONTROLLER_PID}"
    then
        printf '\nERROR: robot_simulator stopped unexpectedly.\n' >&2
        tail -n 80 "${CONTROLLER_LOG}" >&2 || true
        exit 1
    fi

    if ! root_process_alive "${KICKCAT_PID}"
    then
        printf '\nERROR: KickCAT stopped unexpectedly.\n' >&2
        tail -n 80 "${KICKCAT_LOG}" >&2 || true
        exit 1
    fi

    sleep 1
done
