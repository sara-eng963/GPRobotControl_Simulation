#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_build_dir="${W25Q_FIFO_TEST_BUILD_DIR:-$(mktemp -d /tmp/w25q-fifo.XXXXXX)}"
mkdir -p "$test_build_dir"
cc -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DW25Q_TEST_MMIO -I Simulation/Renode/Storage \
    Simulation/Renode/Storage/stm32h7_w25q_bus.c \
    Simulation/Tests/test_stm32h7_w25q_fifo.c \
    -o "$test_build_dir/w25q_stm32_fifo_mock_test"
"$test_build_dir/w25q_stm32_fifo_mock_test"
