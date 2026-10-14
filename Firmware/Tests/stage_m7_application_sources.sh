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
    case "$dir/$name" in
      CANComm/*.c) continue ;;
      Intercore/*.c)
        [[ "$name" == trajectory_stream_path_port.c ]] || continue ;;
    esac
    cp "$file" "$APP/$dir/$name"
  done < <(find "$ROOT/$dir" -maxdepth 1 -type f \
    \( -name '*.c' -o -name '*.h' \) -print0)
done
[[ -f "$APP/StateMachine/supervisor_task.c" &&
   -f "$APP/ControlCore/Kinematics/adls_ik.c" &&
   -f "$APP/CANComm/CANopen/canopen_master.c" &&
   ! -f "$APP/Intercore/ipc_mailbox.c" &&
   ! -f "$APP/Intercore/ipc_mailbox_freertos.c" &&
   ! -f "$APP/Intercore/trajectory_stream.c" ]] || {
  echo "FAIL: source staging manifest incomplete/duplicated" >&2; exit 1;
}
echo "PASS: staged canonical CM7 application C and headers"
