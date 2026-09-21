#!/usr/bin/env bash

set -Eeuo pipefail

# ============================================================================
# GPRobotControl teammate environment bootstrap
# ============================================================================
#
# Run this from a cloned GPRobotControl_Simulation repository inside WSL/Linux.
# It installs the build prerequisites, downloads the external dependencies,
# builds KickCAT, initializes the Raylib submodule, and builds this project.
#
# Optional first argument:
#   path to A6-EC.xml
#
# Example:
#   bash scripts/setup_environment.sh ~/A6-EC.xml
#
# If no argument is provided, the script also looks for:
#   <project-root>/A6-EC.xml
#
# The A6-EC XML is not stored in this repository. It must be shared separately.
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_PARENT="$(dirname "${REPO_ROOT}")"

FREERTOS_DIR="${FREERTOS_DIR:-${PROJECT_PARENT}/FreeRTOS-Kernel}"
SOEM_DIR="${SOEM_DIR:-${PROJECT_PARENT}/SOEM}"
KICKCAT_DIR="${KICKCAT_DIR:-${HOME}/KickCAT}"

TOOLS_VENV="${TOOLS_VENV:-${HOME}/.local/share/gprobot-tools/venv}"

FREERTOS_REPO="https://github.com/FreeRTOS/FreeRTOS-Kernel.git"
SOEM_REPO="https://github.com/OpenEtherCATsociety/SOEM.git"
KICKCAT_REPO="https://github.com/leducp/KickCAT.git"

# Pinned public revisions so every teammate gets the same dependency versions.
# These can be overridden without editing the script, for example:
#   KICKCAT_REF=<sha> bash scripts/setup_environment.sh
FREERTOS_REF="${FREERTOS_REF:-8be86d4a24fd4091f8f4192018423ab590f408db}"
SOEM_REF="${SOEM_REF:-88e8ed46efba7dfa7b94d08a512db25a33e3f8d5}"
KICKCAT_REF="${KICKCAT_REF:-43ad3e9ce390f0f6d9548118098bcaafc0d4f1f1}"

A6_CONFIG_DIR="${KICKCAT_DIR}/simulation/slave_configs"
A6_JSON="${A6_CONFIG_DIR}/A6-EC.json"
A6_XML="${A6_CONFIG_DIR}/A6-EC.xml"
A6_XML_SOURCE="${1:-${A6_XML_SOURCE:-}}"

if [[ -z "${A6_XML_SOURCE}" && -f "${REPO_ROOT}/A6-EC.xml" ]]
then
    A6_XML_SOURCE="${REPO_ROOT}/A6-EC.xml"
fi

print_header()
{
    printf '\n============================================================\n'
    printf ' GPRobotControl teammate setup\n'
    printf '============================================================\n'
}

step()
{
    printf '\n[%s] %s\n' "$1" "$2"
}

fail()
{
    printf '\nERROR: %s\n' "$*" >&2
    exit 1
}

require_command()
{
    command -v "$1" >/dev/null 2>&1 || fail "Required command not found: $1"
}

repo_is_clean()
{
    local dir="$1"
    [[ -z "$(git -C "${dir}" status --porcelain 2>/dev/null)" ]]
}

clone_or_pin_repo()
{
    local name="$1"
    local url="$2"
    local dir="$3"
    local ref="$4"

    if [[ ! -e "${dir}" ]]
    then
        printf 'Cloning %s -> %s\n' "${name}" "${dir}"
        git clone --recursive "${url}" "${dir}"
    elif [[ ! -d "${dir}/.git" ]]
    then
        fail "${dir} already exists but is not a Git repository. Move/remove it, then rerun."
    else
        printf '%s already exists -> reusing %s\n' "${name}" "${dir}"
    fi

    if ! repo_is_clean "${dir}"
    then
        fail "${name} has local changes in ${dir}. Commit/stash them before running setup."
    fi

    git -C "${dir}" fetch --tags origin
    git -C "${dir}" checkout --detach "${ref}"
    git -C "${dir}" submodule update --init --recursive

    printf 'Pinned %-18s %s\n' "${name}:" "$(git -C "${dir}" rev-parse --short HEAD)"
}

install_system_packages()
{
    sudo apt-get update

    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        git \
        pkg-config \
        python3 \
        python3-venv \
        python3-pip \
        libcap2-bin \
        iproute2 \
        util-linux \
        libasound2-dev \
        libx11-dev \
        libxrandr-dev \
        libxi-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libxcursor-dev \
        libxinerama-dev \
        libudev-dev
}

install_build_tools()
{
    mkdir -p "$(dirname "${TOOLS_VENV}")"

    if [[ ! -x "${TOOLS_VENV}/bin/python" ]]
    then
        python3 -m venv "${TOOLS_VENV}"
    fi

    "${TOOLS_VENV}/bin/python" -m pip install --upgrade pip
    "${TOOLS_VENV}/bin/python" -m pip install \
        'cmake>=3.28,<4' \
        'conan>=2.10,<3'

    export PATH="${TOOLS_VENV}/bin:${PATH}"

    require_command cmake
    require_command conan

    printf 'CMake: %s\n' "$(cmake --version | head -n1)"
    printf 'Conan: %s\n' "$(conan --version)"
    printf 'GCC:   %s\n' "$(gcc --version | head -n1)"
}

install_a6_config()
{
    mkdir -p "${A6_CONFIG_DIR}"

    cat > "${A6_JSON}" <<'JSON'
{
  "esi": "A6-EC.xml",
  "ds402_motor": true
}
JSON

    printf 'Installed simulator config: %s\n' "${A6_JSON}"

    if [[ -n "${A6_XML_SOURCE}" ]]
    then
        if [[ ! -f "${A6_XML_SOURCE}" ]]
        then
            fail "A6-EC XML source does not exist: ${A6_XML_SOURCE}"
        fi

        if [[ "$(readlink -f "${A6_XML_SOURCE}")" != "$(readlink -m "${A6_XML}")" ]]
        then
            cp "${A6_XML_SOURCE}" "${A6_XML}"
        fi

        printf 'Installed A6-EC ESI XML: %s\n' "${A6_XML}"
    elif [[ -f "${A6_XML}" ]]
    then
        printf 'A6-EC.xml already present: %s\n' "${A6_XML}"
    else
        printf '\nNOTE: A6-EC.xml has NOT been installed yet.\n'
        printf '      Copy the XML your teammate shared to:\n'
        printf '      %s\n' "${A6_XML}"
    fi
}

build_kickcat()
{
    (
        cd "${KICKCAT_DIR}"

        # Keep the KickCAT build minimal: only the ESI parser and simulator are
        # needed for the six software A6-EC slaves used by this project.
        ./scripts/configure.sh build \
            --without=all \
            --with=esi_parser \
            --with=simulation \
            --non-interactive

        ./scripts/setup_build.sh build --build-type Release
        cmake --build build -j "$(nproc)"
    )

    [[ -x "${KICKCAT_DIR}/build/simulation/network_simulator" ]] || \
        fail "KickCAT network_simulator was not built successfully."

    [[ -x "${KICKCAT_DIR}/simulation/create_virtual_ethernet.sh" ]] || \
        fail "KickCAT virtual-Ethernet helper is missing."
}

build_project()
{
    git -C "${REPO_ROOT}" submodule update --init --recursive

    cmake -S "${REPO_ROOT}" -B "${REPO_ROOT}/build"
    cmake --build "${REPO_ROOT}/build" -j "$(nproc)"

    [[ -x "${REPO_ROOT}/build/freertos_pc_test" ]] || \
        fail "freertos_pc_test was not built."

    [[ -x "${REPO_ROOT}/build/mock_hmi" ]] || \
        fail "mock_hmi was not built."
}

print_summary()
{
    local cpu_count
    cpu_count="$(nproc)"

    printf '\n============================================================\n'
    printf ' SETUP COMPLETE\n'
    printf '============================================================\n'
    printf 'FreeRTOS-Kernel: %s\n' "${FREERTOS_DIR}"
    printf 'SOEM:            %s\n' "${SOEM_DIR}"
    printf 'KickCAT:         %s\n' "${KICKCAT_DIR}"
    printf 'Project:         %s\n' "${REPO_ROOT}"
    printf 'Logical CPUs:    %s\n' "${cpu_count}"

    if (( cpu_count < 4 ))
    then
        printf '\nWARNING: run_simulation.sh expects CPUs 1, 2 and 3 by default.\n'
        printf '         This machine exposes fewer than 4 logical CPUs.\n'
    fi

    if [[ ! -f "${A6_XML}" ]]
    then
        printf '\nONE MANUAL FILE IS STILL REQUIRED:\n'
        printf '  A6-EC.xml -> %s\n' "${A6_XML}"
        printf '\nAfter copying it there, run:\n'
        printf '  cd %q\n' "${REPO_ROOT}"
        printf '  bash scripts/run_simulation.sh\n'
        return
    fi

    printf '\nEverything required by the simulator is present.\n'
    printf '\nRun the complete system with:\n'
    printf '  cd %q\n' "${REPO_ROOT}"
    printf '  bash scripts/run_simulation.sh\n'
}

print_header

if [[ "$(uname -s)" != "Linux" ]]
then
    fail "This setup script must be run inside Linux/WSL, not Windows PowerShell."
fi

if grep -qi microsoft /proc/version 2>/dev/null
then
    printf 'Environment: WSL detected\n'
else
    printf 'Environment: native Linux detected\n'
fi

printf 'Project: %s\n' "${REPO_ROOT}"
printf 'Dependencies will be installed at:\n'
printf '  FreeRTOS-Kernel -> %s\n' "${FREERTOS_DIR}"
printf '  SOEM            -> %s\n' "${SOEM_DIR}"
printf '  KickCAT         -> %s\n' "${KICKCAT_DIR}"

step '1/7' 'Installing Ubuntu/Linux build dependencies'
sudo -v
install_system_packages

step '2/7' 'Installing reproducible CMake + Conan tooling'
install_build_tools

step '3/7' 'Downloading and pinning FreeRTOS-Kernel'
clone_or_pin_repo 'FreeRTOS-Kernel' "${FREERTOS_REPO}" "${FREERTOS_DIR}" "${FREERTOS_REF}"

step '4/7' 'Downloading and pinning SOEM'
clone_or_pin_repo 'SOEM' "${SOEM_REPO}" "${SOEM_DIR}" "${SOEM_REF}"

step '5/7' 'Downloading, configuring and building KickCAT'
clone_or_pin_repo 'KickCAT' "${KICKCAT_REPO}" "${KICKCAT_DIR}" "${KICKCAT_REF}"
install_a6_config
build_kickcat

step '6/7' 'Initializing Raylib and building GPRobotControl'
build_project

step '7/7' 'Verifying installation'
require_command taskset
require_command ip
require_command nice

print_summary
