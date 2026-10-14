#!/usr/bin/env bash
# Mirror source, NEVER make another canonical robot application copy.
# This writes solely inside the specified CubeIDE verifier's CM7/Application.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: $0 <CubeIDE-project-root>" >&2; exit 2; }
DEST="$1"
[[ -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" ]] || {
  echo "FAIL: invalid CubeIDE verifier" >&2; exit 1;
}
APP="$DEST/CM7/Application"
for dir in \
  ControlCore/Analysis ControlCore/Config ControlCore/Execution \
  ControlCore/Kinematics ControlCore/Math ControlCore/Pipeline \
  ControlCore/Trajectory StateMachine StateMachine/States \
  CANComm CANComm/CANopen ServoDrive/AvatarM ServoDrive/CiA402 \
  ServoDrive/JointDrive Intercore; do
  mkdir -p "$APP/$dir"
  find "$APP/$dir" -maxdepth 1 -type f \
    \( -name '*.c' -o -name '*.h' \) -delete
  while IFS= read -r -d '' file; do
    name="$(basename "$file")"
    # Match the DIRECTORY exactly: in a Bash case pattern, '*' also matches
    # '/', so 'CANComm/*.c' would incorrectly exclude CANComm/CANopen/*.c.
    case "$dir" in
      CANComm)
        [[ "$name" == *.h ]] || continue ;;
      Intercore)
        [[ "$name" == *.h ||
           "$name" == trajectory_stream_path_port.c ||
           "$name" == validation_storage_bridge.c ]] || continue ;;
    esac
    cp "$file" "$APP/$dir/$name"
  done < <(find "$ROOT/$dir" -maxdepth 1 -type f \
    \( -name '*.c' -o -name '*.h' \) -print0)
done
for required in \
  StateMachine/supervisor_task.c \
  ControlCore/Kinematics/adls_ik.c \
  CANComm/CANopen/canopen_master.c \
  CANComm/CANopen/canopen_nmt.c \
  CANComm/CANopen/canopen_sdo.c \
  CANComm/CANopen/canopen_heartbeat.c \
  Intercore/trajectory_stream_path_port.c
  Intercore/validation_storage_bridge.c; do
  [[ -f "$APP/$required" ]] || {
    echo "FAIL: staged source missing: $required" >&2; exit 1;
  }
done
for forbidden in \
  Intercore/ipc_mailbox.c \
  Intercore/ipc_mailbox_freertos.c \
  Intercore/trajectory_stream.c; do
  [[ ! -f "$APP/$forbidden" ]] || {
    echo "FAIL: duplicate firmware source staged: $forbidden" >&2; exit 1;
  }
done
echo "PASS: staged canonical CM7 application C and headers"
