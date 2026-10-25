#!/usr/bin/env bash
# Copy ONLY the six Stage 3 Batch B2 canonical inputs into a separate verifier.
# Does not edit .ioc, regenerate HAL, build firmware, or touch other files.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
[[ $# -eq 1 ]] || { echo "Usage: bash Firmware/Tests/stage_stage3_batch_b2.sh <CubeIDE-project-root>" >&2; exit 2; }
DEST="$(realpath -m "$1")"
[[ "$DEST" != "$ROOT" && "$DEST" != "$ROOT/"* ]] || {
  echo "FAIL: refusing to stage into Git checkout" >&2; exit 1;
}
[[ -f "$DEST/GP_STM32H745.ioc" &&
   -f "$DEST/CM7/.project" && -f "$DEST/CM4/.project" ]] || {
  echo "FAIL: expected independent dual-core CubeIDE project" >&2; exit 1
}
FILES=(
  "Common/Inc/sram4_memory_map.h"
  "CM7/Core/Src/intercore_shared_memory.c"
  "CM7/STM32H745IITX_FLASH.ld"
  "CM7/STM32H745IITX_RAM.ld"
  "CM4/STM32H745IITX_FLASH.ld"
  "CM4/STM32H745IITX_RAM.ld"
)
# Preflight all targets BEFORE the first copy; do not create a partial verifier.
for path in "${FILES[@]}"; do
  [[ -f "$ROOT/Firmware/STM32H745/$path" && -f "$DEST/$path" ]] || {
    echo "FAIL: source/destination file absent: $path" >&2; exit 1
  }
done
for path in "${FILES[@]}"; do
  src="$ROOT/Firmware/STM32H745/$path"
  dst="$DEST/$path"
  if cmp -s "$src" "$dst"; then
    echo "UNCHANGED: $path"
  else
    cp "$src" "$dst"
    cmp -s "$src" "$dst" || { echo "FAIL: parity after copy: $path" >&2; exit 1; }
    echo "STAGED: $path"
  fi
done
echo "PASS: six Batch B2 inputs matched. Clean-build BOTH cores in CubeIDE before acceptance."
