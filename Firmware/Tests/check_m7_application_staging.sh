#!/usr/bin/env bash
# Stage 3B-2 regression: exercise the exact M7 source mirroring manifest.
# Uses an isolated temporary verifier; never modifies a CubeIDE project.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TEST_DEST="$(mktemp -d)"
trap 'rm -rf "$TEST_DEST"' EXIT
mkdir -p "$TEST_DEST/CM7"
: > "$TEST_DEST/GP_STM32H745.ioc"
: > "$TEST_DEST/CM7/.project"

bash "$ROOT/Firmware/Tests/stage_m7_application_sources.sh" "$TEST_DEST"
APP="$TEST_DEST/CM7/Application"

for required in \
  CANComm/can_backend.h \
  CANComm/CANopen/canopen_master.c \
  CANComm/CANopen/canopen_nmt.c \
  CANComm/CANopen/canopen_sdo.c \
  CANComm/CANopen/canopen_heartbeat.c \
  ControlCore/Kinematics/adls_ik.c \
  StateMachine/supervisor_task.c \
  Intercore/trajectory_stream_path_port.c; do
  test -f "$APP/$required" || {
    echo "FAIL: missing $required" >&2; exit 1;
  }
  cmp "$ROOT/$required" "$APP/$required" || {
    echo "FAIL: not an exact copy: $required" >&2; exit 1;
  }
done
for forbidden in \
  Intercore/ipc_mailbox.c \
  Intercore/ipc_mailbox_freertos.c \
  Intercore/trajectory_stream.c \
  Intercore/validation_storage_bridge.c; do
  test ! -f "$APP/$forbidden" || {
    echo "FAIL: incorrectly mirrored $forbidden" >&2; exit 1;
  }
done

# It must be safe to rerun when restoring the verifier on another day.
bash "$ROOT/Firmware/Tests/stage_m7_application_sources.sh" "$TEST_DEST"
cmp "$ROOT/CANComm/CANopen/canopen_master.c" \
    "$APP/CANComm/CANopen/canopen_master.c"
echo "PASS: CM7 application staging manifest and repeatability"
