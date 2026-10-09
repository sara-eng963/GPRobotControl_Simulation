#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
OUT_DIR="$(mktemp -d /tmp/gp-batch5-flash-prefetch.XXXXXX)"
EXE="$OUT_DIR/flash_prefetch_500hz"
printf '[INFO] Build/log directory: %s\n' "$OUT_DIR"
cc -std=c11 -O2 -Wall -Wextra -Werror -pthread \
  Simulation/Tests/test_flash_prefetch_500hz.c \
  Simulation/Storage/trajectory_prefetch.c \
  Simulation/Renode/Storage/qspi_nor_validated_storage.c \
  Simulation/Renode/Storage/w25q512jv_flash.c \
  Simulation/Tests/w25q_nor_model.c -o "$EXE"
printf '[RUN] Actual flash-storage + TrajectoryPrefetch, host-paced 2ms\n'
set +e
"$EXE" "${1:-all}" 2>&1 | tee "$OUT_DIR/results.log"
status=${PIPESTATUS[0]}
set -e
printf '[LOG] %s\n' "$OUT_DIR/results.log"
exit "$status"
