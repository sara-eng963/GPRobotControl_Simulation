#!/usr/bin/env bash
# Stage 3 Batch A: validate both already-built target ELF images.
# Capturing nm once into files avoids 'nm | grep -q' + pipefail SIGPIPE.
# No target execution, bus timing or safety checks are implied.
set -euo pipefail
[[ $# -eq 1 ]] || { echo "Usage: $0 <STM32CubeIDE-dual-core-project-root>" >&2; exit 2; }
ROOT="$1"
M7="$ROOT/CM7/Debug/GP_STM32H745_CM7.elf"
M4="$ROOT/CM4/Debug/GP_STM32H745_CM4.elf"
NM="${ARM_NONE_EABI_NM:-arm-none-eabi-nm}"
SIZE="${ARM_NONE_EABI_SIZE:-arm-none-eabi-size}"
command -v "$NM" >/dev/null
command -v "$SIZE" >/dev/null
for elf in "$M7" "$M4"; do
    [[ -s "$elf" ]] || { echo "FAIL: missing/nonempty ELF $elf" >&2; exit 1; }
done
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
"$NM" "$M7" > "$tmp/m7.nm"
"$NM" "$M4" > "$tmp/m4.nm"

check_symbol() {
    local file="$1" symbol="$2" kind="$3"
    if ! grep -Eq "^[[:xdigit:]]+[[:space:]]+$kind[[:space:]]+$symbol\$" "$file"; then
        echo "FAIL: expected $kind $symbol in $file" >&2
        return 1
    fi
    echo "PASS: $kind $symbol"
}
echo "=== M7 HAL + CANopen + fail-closed composition ==="
check_symbol "$tmp/m7.nm" HAL_FDCAN_Init T
check_symbol "$tmp/m7.nm" HAL_FDCAN_MspInit T
check_symbol "$tmp/m7.nm" gp_fdcan_classic_backend_open T
check_symbol "$tmp/m7.nm" canopen_master_init T
check_symbol "$tmp/m7.nm" application_supervisor_start T
check_symbol "$tmp/m7.nm" application_supervisor_board_configure W

# Unresolved weak symbol is expected when real board supplier is absent.
if grep -Eq '^[[:xdigit:]]+[[:space:]]+[A-Z][[:space:]]+gp_board_supply_commissioned_config$' "$tmp/m7.nm"; then
    echo "FAIL: unexpected strong/defined board supplier in M7 ELF" >&2
    exit 1
fi
if ! grep -Eq '^[[:space:]]+w[[:space:]]+gp_board_supply_commissioned_config$' "$tmp/m7.nm"; then
    echo "NOTE: no unresolved weak board supplier symbol displayed (check linker/map manually)"
else
    echo "PASS: board supplier remains unresolved weak"
fi

echo "=== M4 IPC ==="
check_symbol "$tmp/m4.nm" application_ipc_service_run T
check_symbol "$tmp/m4.nm" gp_ipc_m4_receive T

echo "=== Firmware image sizes ==="
"$SIZE" "$M7" "$M4"
echo "PASS: Stage 3 Batch A target ELF symbol gate"
