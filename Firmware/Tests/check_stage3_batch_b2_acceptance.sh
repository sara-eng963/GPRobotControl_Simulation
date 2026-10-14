#!/usr/bin/env bash
# Batch B2 source + linker acceptance. Does not build a CubeIDE target or test hardware.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_production_shared_objects.sh

if [[ $# -ne 1 ]]; then
  echo "SOURCE TESTS ONLY: provide independently rebuilt CubeIDE project directory to qualify ELF."
  echo "Usage: bash Firmware/Tests/check_stage3_batch_b2_acceptance.sh <CubeIDE-verifier>"
  exit 2
fi
ELF="$1/CM7/Debug/GP_STM32H745_CM7.elf"
MAP="$1/CM7/Debug/GP_STM32H745_CM7.map"
[[ -s "$ELF" && -s "$MAP" ]] || { echo "FAIL: fresh M7 ELF/map required"; exit 1; }
command -v arm-none-eabi-nm >/dev/null
command -v arm-none-eabi-readelf >/dev/null
temp="$(mktemp -d)"
trap 'rm -rf "$temp"' EXIT
arm-none-eabi-nm -S --defined-only "$ELF" > "$temp/nm"
arm-none-eabi-readelf -W -S "$ELF" > "$temp/sections"
awk '$4=="intercore_validation_storage" && $1=="38005000" && $2=="00003000" { found=1 } END {exit !found}' "$temp/nm" ||
  { echo "FAIL: invalid M7 SRAM4 validation symbol/size"; exit 1; }
grep -E '\] \.gp_shared_validation[[:space:]]+NOBITS[[:space:]]+38005000' "$temp/sections" >/dev/null ||
  { echo "FAIL: validation section not NOLOAD/NOBITS at reserved base"; exit 1; }
grep -F '__gp_shared_validation_start__' "$MAP" >/dev/null ||
  { echo "FAIL: linker map lacks validation region symbol"; exit 1; }
echo "PASS: Batch B2 source checks and independently built M7 SRAM4 validation allocation"
echo "NOT VERIFIED: application validation protocol, M4 NOR, physical SRAM coherence."
