#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
test_build_dir="${W25Q_TEST_BUILD_DIR:-$(mktemp -d /tmp/w25q-host.XXXXXX)}"
mkdir -p "$test_build_dir"
for test_name in test_w25q512jv_verified test_qspi_storage_power_loss test_w25q512jv_protocol; do
gcc -std=c11 -Wall -Wextra -Werror -O2 \
    -I Simulation/Renode/Storage \
    "Simulation/Tests/$test_name.c" \
    Simulation/Tests/w25q_nor_model.c \
    Simulation/Renode/Storage/w25q512jv_flash.c \
    Simulation/Renode/Storage/qspi_nor_validated_storage.c \
    -o "$test_build_dir/$test_name"
(cd "$test_build_dir" && "./$test_name")
done
