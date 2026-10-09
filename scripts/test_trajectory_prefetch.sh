#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build-trajectory-prefetch
cc -std=c11 -O2 -g -Wall -Wextra -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -DTRAJECTORY_PREFETCH_HOST_TEST \
  Simulation/Storage/trajectory_prefetch.c \
  Simulation/Tests/test_trajectory_prefetch.c \
  -o build-trajectory-prefetch/test_trajectory_prefetch
ASAN_OPTIONS=detect_leaks=1 ./build-trajectory-prefetch/test_trajectory_prefetch
