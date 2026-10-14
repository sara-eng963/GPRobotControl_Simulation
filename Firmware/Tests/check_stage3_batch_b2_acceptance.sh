#!/usr/bin/env bash
# Batch B2 acceptance: independent CubeIDE ELF, source parity, and ARM layout.
# No build is performed by this script. No hardware is operated.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || {
  echo "Usage: bash Firmware/Tests/check_stage3_batch_b2_acceptance.sh <CubeIDE-project-root>" >&2
  exit 2
}
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* ]] || {
  echo "FAIL: independent CubeIDE verifier required, not a repository directory" >&2
  exit 1
}
[[ -f "$DEST/GP_STM32H745.ioc" && -f "$DEST/CM7/.project" &&
   -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: not a dual-core CubeIDE verifier: $DEST" >&2
  exit 1
}
SOURCE_FILES=(
  "Common/Inc/sram4_memory_map.h"
  "CM7/Core/Src/intercore_shared_memory.c"
  "CM7/STM32H745IITX_FLASH.ld"
  "CM7/STM32H745IITX_RAM.ld"
  "CM4/STM32H745IITX_FLASH.ld"
  "CM4/STM32H745IITX_RAM.ld"
)
for path in "${SOURCE_FILES[@]}"; do
  a="$ROOT/Firmware/STM32H745/$path"
  b="$DEST/$path"
  if [[ ! -f "$b" ]] || ! cmp -s "$a" "$b"; then
    echo "FAIL: stale verifier input: $path" >&2
    echo "Run: bash Firmware/Tests/stage_stage3_batch_b2.sh \"$DEST\"" >&2
    exit 1
  fi
done
echo "PASS: six Batch B2 verifier source/linker inputs match canonical Git files"

cd "$ROOT"
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_production_shared_objects.sh

M7="$DEST/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$DEST/CM4/Debug/GP_STM32H745_CM4.elf"
MAP="$DEST/CM7/Debug/GP_STM32H745_CM7.map"
[[ -s "$M7" && -s "$M4" && -s "$MAP" ]] || {
  echo "FAIL: independently built M7/M4 ELFs and M7 map required" >&2
  exit 1
}
for path in "${SOURCE_FILES[@]}"; do
  case "$path" in
    CM7/*) [[ "$M7" -nt "$DEST/$path" ]] || {
      echo "FAIL: M7 ELF not newer than staged $path. Rebuild CM7." >&2; exit 1; } ;;
    CM4/*) [[ "$M4" -nt "$DEST/$path" ]] || {
      echo "FAIL: M4 ELF not newer than staged $path. Rebuild CM4." >&2; exit 1; } ;;
    Common/*)
      for core in CM7 CM4; do
        elf="$M7"; [[ "$core" == CM4 ]] && elf="$M4"
        [[ "$elf" -nt "$DEST/$path" ]] || {
          echo "FAIL: $core ELF not newer than staged $path. Rebuild $core." >&2; exit 1; }
      done ;;
  esac
done
command -v arm-none-eabi-nm >/dev/null
command -v arm-none-eabi-readelf >/dev/null
temp="$(mktemp -d)"
trap 'rm -rf "$temp"' EXIT
arm-none-eabi-nm -S --defined-only "$M7" > "$temp/nm"
arm-none-eabi-readelf -W -S "$M7" > "$temp/sections"
awk '$4=="intercore_validation_storage" && $1=="38005000" && $2=="00003000" {found=1} END {exit !found}' "$temp/nm" || {
  echo "FAIL: M7 validation allocation absent/wrong; check linker, rebuild CM7" >&2
  exit 1
}
grep -E '\] \.gp_shared_validation[[:space:]]+NOBITS[[:space:]]+38005000' "$temp/sections" >/dev/null || {
  echo "FAIL: validation section must be NOLOAD/NOBITS at 0x38005000" >&2; exit 1
}
grep -F '__gp_shared_validation_start__' "$MAP" >/dev/null || {
  echo "FAIL: linker map lacks validation region symbol" >&2; exit 1
}
echo "PASS: Stage 3 Batch B2 source/ABI, four linker probes and independent target ELF/map"
echo "DEFERRED: typed validation protocol, M4 flash worker, storage device, hardware runtime."
