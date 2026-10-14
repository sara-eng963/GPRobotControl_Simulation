#!/usr/bin/env bash
# Import only reviewed STM32CubeMX TIM2/TIM3 generation outputs from the
# independent Windows CubeIDE verifier. Never copy Debug, Backup, project
# metadata caches or custom robot application directories.
#
# Usage: bash Firmware/Tests/stage_tim2_tim3_generated.sh \
#          "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
# Does not git add, commit or push. Review the diff before committing.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <generated-CubeIDE-project-root>" >&2
    exit 2
fi
REPO="$(git rev-parse --show-toplevel)"
cd "$REPO"
[[ "$(git branch --show-current)" == "sara/can-replacement" ]] || {
    echo "FAIL: must be on sara/can-replacement, not a detached/other branch" >&2
    exit 1
}
SOURCE="$1"
ROOT="$REPO/Firmware/STM32H745"
[[ -f "$SOURCE/GP_STM32H745.ioc" && -f "$SOURCE/CM7/.project" ]] || {
    echo "FAIL: not a generated STM32H745 CubeIDE project: $SOURCE" >&2
    exit 1
}

# CubeMX writes the .ioc with CRLF on Windows. Match exact key/value pairs
# after removing only the optional line-final carriage return; accept LF too.
# Keep failures diagnostic rather than letting 'set -e' exit silently.
require_ioc_entry() {
    local expected="$1"
    if ! awk -v expected="$expected" '{
        sub(/\r$/, "", $0)
        if ($0 == expected) found = 1
    } END { exit found ? 0 : 1 }' "$SOURCE/GP_STM32H745.ioc"; then
        echo "FAIL: missing saved CubeMX setting: $expected" >&2
        exit 1
    fi
    echo "PASS: saved CubeMX setting: $expected"
}
require_ioc_entry 'TIM3.Period=1999'
require_ioc_entry 'TIM3.Prescaler=63'
require_ioc_entry 'TIM2.Prescaler=63'
for token in \
    'htim2.Init.Prescaler = 63;' \
    'htim2.Init.Period = 4294967295;' \
    'htim3.Init.Prescaler = 63;' \
    'htim3.Init.Period = 1999;' \
    'MX_FDCAN1_Init();' \
    'MX_TIM2_Init();' \
    'MX_TIM3_Init();' \
    'intercore_shared_memory_initialize()' \
    'intercore_notifications_initialize()' \
    'application_supervisor_start(&config)'
do
    grep -Fq "$token" "$SOURCE/CM7/Core/Src/main.c" || {
        echo "FAIL: missing generated source/startup token: $token" >&2
        exit 1
    }
done
for token in 'TIM2_IRQHandler(void)' 'TIM3_IRQHandler(void)'; do
    grep -Fq "$token" "$SOURCE/CM7/Core/Src/stm32h7xx_it.c" || {
        echo "FAIL: missing generated interrupt handler: $token" >&2
        exit 1
    }
done

# Preserve custom application files and untouched Cortex-M4.
for path in \
    CM7/Core/Src/application_supervisor.c \
    CM7/Board/Src/fdcan_classic_backend.c \
    CM7/Core/Inc/FreeRTOSConfig.h \
    CM4/Core/Src/main.c \
    CM7/.project \
    CM4/.project
do
    cmp -s "$ROOT/$path" "$SOURCE/$path" || {
        echo "FAIL: custom/core file diverged: $path" >&2
        exit 1
    }
done

FILES=(
    GP_STM32H745.ioc
    .mxproject
    CM7/Core/Src/main.c
    CM7/Core/Src/stm32h7xx_hal_msp.c
    CM7/Core/Src/stm32h7xx_it.c
    CM7/Core/Inc/stm32h7xx_it.h
    Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_tim.h
)

# Never clobber unrelated staged/unstaged edits or pre-existing untracked
# files. If a previous run copied the identical generated file, accept it
# idempotently. Validate every path before copying any of them.
for path in "${FILES[@]}"; do
    [[ -f "$SOURCE/$path" ]] || {
        echo "FAIL: generated file missing: $SOURCE/$path" >&2
        exit 1
    }
    relative="Firmware/STM32H745/$path"
    if [[ -f "$ROOT/$path" ]] &&
       ! cmp -s "$SOURCE/$path" "$ROOT/$path"; then
        if ! git ls-files --error-unmatch -- "$relative" >/dev/null 2>&1; then
            echo "FAIL: refusing to overwrite differing untracked file: $relative" >&2
            exit 1
        fi
        if ! git diff --quiet HEAD -- "$relative"; then
            echo "FAIL: refusing to overwrite local edits: $relative" >&2
            exit 1
        fi
    fi
done

for path in "${FILES[@]}"; do
    if [[ -f "$ROOT/$path" ]] &&
       cmp -s "$SOURCE/$path" "$ROOT/$path"; then
        echo "ALREADY SYNCED: $path"
        continue
    fi
    mkdir -p "$(dirname "$ROOT/$path")"
    cp "$SOURCE/$path" "$ROOT/$path"
    cmp -s "$SOURCE/$path" "$ROOT/$path" || {
        echo "FAIL: post-copy verification failed: $path" >&2
        exit 1
    }
    echo "SYNCED: $path"
done

# CubeMX emits Windows CRLF in generated source and .ioc/.mxproject metadata.
# Git's whitespace checker treats the CR as trailing whitespace unless
# cr-at-eol is enabled. This is an invocation-scoped setting: do not rewrite
# official generated files, change git config, or disable other checks.
git -c core.whitespace=cr-at-eol diff --check -- Firmware/STM32H745
echo
echo "=== Changed generated paths (not yet staged/committed) ==="
git status --short -- Firmware/STM32H745
echo
echo "=== Tracked-file diff summary ==="
git diff --stat -- Firmware/STM32H745
echo
echo "PASS: reviewed CubeMX-generated timing inputs copied into Git working tree."
echo "NOTE: no git add, commit or push performed; review diff before publication."
