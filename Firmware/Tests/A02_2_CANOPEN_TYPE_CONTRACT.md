# Stage A02.2 — Canonical CANopen type contract

Verified GitHub branch HEAD at package construction: `5e74a6933e4ff457a120f6ebf24afd6352332d71`.
This package is an **uncommitted patch**; no GitHub files or branches were changed.

## Architectural decision

`CANComm/CANopen/canopen_master_types.h` owns all CANopen master enum,
configuration, transaction, feedback, callback and master instance types.
The root and firmware public master headers include that **same exact file**.
The firmware header alone declares the production-only setter
`canopen_master_set_drive_permission()`; root desktop code does not implement or
call this API. The canonical struct contains the permission member in BOTH
builds, but it does not itself enforce permission behavior in the desktop
implementation. The production `.c` retains the C2B guards unchanged.

## Local application (from repo root, branch HEAD must still be 5e74a693)

```
git status --short
git rev-parse HEAD
git apply --check /path/to/A02_2.patch
git apply /path/to/A02_2.patch
python3 Firmware/Tests/run_a02_1_canopen_adapter_abi.py
python3 Firmware/Tests/run_c2b_canopen_master_tests.py
python3 Firmware/Tests/run_drive_restart_gate_tests.py
python3 Firmware/Tests/check_c2b_wiring.py
python3 Firmware/Tests/build_stm32.py --core CM7 --config Debug
python3 Firmware/Tests/build_stm32.py --core CM7 --config Release
python3 Firmware/Tests/build_stm32.py --core CM4 --config Debug
python3 Firmware/Tests/build_stm32.py --core CM4 --config Release
python3 Firmware/Tests/check_firmware_integration.py
python3 Firmware/Tests/check_firmware_integration.py --build-root build/stm32/Release
```

## Remaining firmware-side software verification

```
python3 Firmware/Tests/check_portable_protocol.py
python3 Firmware/Tests/check_stage_a_configuration.py --self-test
python3 Firmware/Tests/run_supervision_policy_tests.py
python3 Firmware/Tests/check_c2a_wiring.py
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_intercore_startup_compile.sh
bash Firmware/Tests/check_intercore_notifications.sh
bash Firmware/Tests/check_cm4_validation_service.sh
python3 Firmware/Tests/check_tim2_tim3_integration.py
python3 Firmware/Tests/run_m7_timer_fault_injection.py
```

## Desktop / CMake / SIL Kit when dependencies exist

```
# The repository requires the sibling ../FreeRTOS-Kernel checkout.
cmake -S . -B build/a02_2_desktop -DENABLE_SILKIT=OFF
cmake --build build/a02_2_desktop -j 4
ctest --test-dir build/a02_2_desktop --output-on-failure

# Requires installed SIL Kit SDK; optionally run the real full simulation:
SILKIT_ROOT=/path/to/SilKit bash scripts/run_full_can_silkit_simulation.sh --no-gui
```

Use a configured, pinned SIL Kit SDK and review the simulation script's
external tool and process expectations before running its interactive flow.


Consult `build_stm32.py --help` if your checked-out script uses different
--config flag spelling. All tests are read-only or emit temporary/build
artifacts; do not commit before independent acceptance.

## CI

The existing STM32 CI now triggers on `CANComm/**` changes and requires the
strict A02.1 real adapter composition. The historical `--expect-baseline-failure` mode belongs to the original
A02.1 revision (`5e74a693`). The current runner requires all ABI and
behavioral assertions to pass. CI executes strict mode without extra flags.

## Limitations

Local WSL verification passed the production adapter ABI regression, C2B/C2A
authorization and supervision checks, CM7/CM4 Debug and Release builds,
firmware integration, 54 desktop CTest cases, and timing/IPC regressions.
Physical timing, hardware behavior, and motor safety remain unverified.
