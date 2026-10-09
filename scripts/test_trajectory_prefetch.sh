#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_build_dir="${PREFETCH_TEST_BUILD_DIR:-$(mktemp -d /tmp/trajectory-prefetch.XXXXXX)}"
mkdir -p "$test_build_dir"
for test_name in test_trajectory_prefetch test_trajectory_prefetch_concurrent; do
  cc -std=c11 -O2 -g -Wall -Wextra -Werror -pthread \
    -fsanitize="${PREFETCH_SANITIZERS:-address,undefined}" -fno-omit-frame-pointer \
    -DTRAJECTORY_PREFETCH_HOST_TEST \
    Simulation/Storage/trajectory_prefetch.c \
    "Simulation/Tests/$test_name.c" \
    -o "$test_build_dir/$test_name"
  ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=1}" "$test_build_dir/$test_name"
done
