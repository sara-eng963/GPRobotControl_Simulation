#!/usr/bin/env bash
# Stage 3A: compile selected existing application sources against the vendored
# STM32H745 dual-core FreeRTOS 10.6.2 and CMSIS/HAL headers.
# Compile-only reconnaissance. Not a CubeIDE build or runtime validation.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
CC="${ARM_NONE_EABI_GCC:-arm-none-eabi-gcc}"

command -v "$CC" >/dev/null 2>&1 || { echo "FAIL: $CC is missing" >&2; exit 1; }
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

include_common=(
    # Mirror the existing Stage 3 M7 application compilation include contract.
    # ROOT alone does not make a bare "supervisor_task.h" resolvable.
    -I"$ROOT"
    -I"$ROOT/StateMachine"
    -I"$ROOT/CANComm"
    -I"$ROOT/Intercore"
    -I"$FW/Common/Inc"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/include"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
    -I"$FW/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    -I"$FW/Drivers/CMSIS/Device/ST/STM32H7xx/Include"
    -I"$FW/Drivers/CMSIS/Include"
    -I"$FW/Drivers/CMSIS/RTOS2/Include"
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc"
    -I"$FW/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
)
common_flags=(
    -std=gnu11 -O0 -Wall -Wextra -Werror
    -DSTM32H745xx -DUSE_HAL_DRIVER -DGP_IPC_FREERTOS_BACKEND=1
)
m7_sources=(
    StateMachine/state_machine.c
    StateMachine/supervisor_io.c
    StateMachine/supervisor_task.c
    StateMachine/States/state_path_execution.c
    CANComm/CANopen/canopen_master.c
    CANComm/CANopen/canopen_nmt.c
    CANComm/CANopen/canopen_sdo.c
    CANComm/CANopen/canopen_heartbeat.c
    ServoDrive/AvatarM/avatar_m_drive.c
    ServoDrive/AvatarM/avatar_m_pdo.c
    ServoDrive/AvatarM/avatar_m_position.c
    ServoDrive/CiA402/cia402.c
    ServoDrive/JointDrive/canopen_joint_drive_port.c
    ServoDrive/JointDrive/canopen_drive_commissioning_port.c
    Intercore/trajectory_stream.c
    Intercore/trajectory_stream_path_port.c
    Intercore/validation_storage_bridge.c
    Intercore/trajectory_storage_service.c
    Firmware/STM32H745/CM7/Core/Src/intercore_shared_memory.c
    Firmware/STM32H745/CM7/Core/Src/application_supervisor.c
)
m4_sources=(
    Intercore/trajectory_stream.c
    Intercore/validation_storage_bridge.c
    Intercore/trajectory_storage_service.c
    Firmware/STM32H745/CM4/Core/Src/intercore_startup_validation.c
    Firmware/STM32H745/CM4/Core/Src/application_validation_service.c
)

compile_core() {
    local core="$1" cpu="$2" fpu="$3"
    shift 3
    local -a files=("$@")
    local n=0 file
    for file in "${files[@]}"; do
        [[ -f "$ROOT/$file" ]] || { echo "FAIL: missing source $file" >&2; exit 1; }
        "$CC" "${common_flags[@]}" \
            -mcpu="$cpu" -mthumb -mfpu="$fpu" -mfloat-abi=hard \
            -D"CORE_$core=1" \
            -I"$FW/$core/Core/Inc" -I"$FW/$core/Board/Inc" \
            -I"$FW/CM7/Board/Inc" "${include_common[@]}" \
            -c "$ROOT/$file" -o "$OUT/$core-$n.o"
        echo "PASS: $core $file"
        n=$((n+1))
    done
    echo "PASS: $core compiled $n application translation units"
}

compile_core CM7 cortex-m7 fpv5-d16 "${m7_sources[@]}"
compile_core CM4 cortex-m4 fpv4-sp-d16 "${m4_sources[@]}"
echo "PASS: compile-only reconnaissance; target linking and hardware behavior NOT tested"
