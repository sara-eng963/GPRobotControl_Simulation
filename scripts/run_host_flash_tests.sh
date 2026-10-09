#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p build-w25q-host
gcc -std=c11 -Wall -Wextra -Werror -O2 \
    -I Simulation/Renode/Storage \
    Simulation/Tests/test_w25q512jv_verified.c \
    Simulation/Renode/Storage/w25q512jv_flash.c \
    Simulation/Renode/Storage/qspi_nor_validated_storage.c \
    -o build-w25q-host/w25q512jv_verified
./build-w25q-host/w25q512jv_verified
