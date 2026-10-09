#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
cc -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DW25Q_TEST_MMIO -I Simulation/Renode/Storage \
    Simulation/Renode/Storage/stm32h7_w25q_bus.c \
    Simulation/Tests/test_stm32h7_w25q_fifo.c \
    -o /tmp/w25q_stm32_fifo_mock_test
/tmp/w25q_stm32_fifo_mock_test
