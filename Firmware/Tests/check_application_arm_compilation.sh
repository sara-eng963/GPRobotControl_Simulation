#!/usr/bin/env bash
# Compile every source selected by each current CubeIDE project, then link.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
python3 "$ROOT/Firmware/Tests/build_stm32.py" --core CM7
python3 "$ROOT/Firmware/Tests/build_stm32.py" --core CM4
