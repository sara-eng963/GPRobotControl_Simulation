#!/usr/bin/env bash
# Executes the target service policy on Linux; this does not emulate FreeRTOS.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
FW="$ROOT/Firmware/STM32H745"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
for mode in static dynamic missing period; do
    flags=()
    case "$mode" in
      dynamic) flags+=(-DconfigSUPPORT_STATIC_ALLOCATION=0);;
      missing) flags+=(-DTEST_NO_PROVIDERS);;
      period) flags+=(-DTEST_TICK_HZ=100);;
    esac
    cc -std=gnu11 -O0 -Wall -Wextra -Werror "${flags[@]}" \
      -I"$ROOT/Firmware/Tests/cm4_validation" \
      -I"$FW/CM4/Core/Inc" -I"$FW/Common/Inc" \
      -I"$FW/CM7/Application/Intercore" \
      "$FW/CM4/Core/Src/application_validation_service.c" \
      "$FW/CM7/Application/Intercore/validation_storage_bridge.c" \
      "$FW/CM7/Application/Intercore/trajectory_storage_service.c" \
      "$FW/Common/Src/trajectory_stream.c" \
      "$ROOT/Firmware/Tests/cm4_validation/test_service.c" -o "$OUT/$mode"
    "$OUT/$mode"
done
