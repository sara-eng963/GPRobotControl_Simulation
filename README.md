# Robot control firmware and simulation

STM32H745ZIT6 dual-core firmware, FreeRTOS POSIX simulation, six virtual
CANopen motor nodes, SIL Kit 5.0.7, trajectory/storage tests, and hand-guiding
tools live in this repository.

The authoritative target project is [Firmware/STM32H745](Firmware/STM32H745).
See [firmware build instructions and ownership](Firmware/README.md).
The firmware is software-build-ready; physical commissioning and robot motion
authorization remain incomplete.

## Host build and regressions

Install CMake 3.28+, C/C++ compilers and the raylib Linux development dependencies.
The POSIX FreeRTOS kernel must be available at ../FreeRTOS-Kernel.
For SIL Kit, set SILKIT_ROOT to the installed 5.0.7 SilKit directory.

~~~bash
cmake -S . -B build/host -DENABLE_SILKIT=ON -DSILKIT_ROOT=/path/to/SilKit
cmake --build build/host -j8
ctest --test-dir build/host --output-on-failure -j4
~~~

Set ENABLE_SILKIT=OFF for portable tests without SIL Kit. Renode QSPI tests
register automatically when Renode and the ARM toolchain are installed.
Simulation tests do not establish physical safety or hardware timing.
Host APIs retain their existing names; the target APIs are isolated inside
Firmware/STM32H745. Host code does not depend on STM32 HAL headers.

## Tools and records

- [Simulation](Simulation): virtual drives, F/T interfaces, storage and Renode.
- [HandGuiding](HandGuiding): sensor processing, admittance and command gating.
- [scripts](scripts): host simulation, analysis and regression runners.
- [Firmware/Tests](Firmware/Tests): current target builds and acceptance checks.
- [Historical firmware tools](Firmware/Tests/historical/README.md): superseded
  II-package staging workflow; do not run against the ZIT6 project.
- [Previous local migration README](Firmware/History/README_migration_v4.md)
  and [W25Q FIFO notes](Firmware/History/README_w25q_fifo_fix.md):
  preserved reference material; current build instructions are above.

Historical archives are kept outside the repository in `~/GP_Workspace_Archive/`.
Current builds use the checked-in source tree without patch installers.

## Runtime configuration

Hand-guiding settings use `HAND_GUIDING_*`; simulation storage/fault selection
uses `SIM_STORAGE_BACKEND` and `SIM_FLASH_TEST_MODE`. Host timing recording uses
`HOST_TIMING_CSV`. Launchers, tests and module guides use these names throughout.
External launch scripts must adopt the current names; no legacy aliases are
provided. Binary IPC, trajectory, storage and F/T protocol formats are unchanged.

## Generated outputs

Use an out-of-tree CMake build, or an ignored `build/` or root `build-*`
directory. CTest writes storage images and CSVs in the build tree. The full
simulation launcher writes logs to `$BUILD_DIR/logs` (override with
`SIMULATION_LOG_DIR`); timing benchmarks use `$BUILD_DIR/timing-benchmark`
(override with `TIMING_RESULTS_DIR`). Run standalone test executables from a
build or temporary directory because some use relative output filenames.

See [prefetch ownership and tests](Simulation/Storage/PREFETCH_README.md) and
[W25Q512JV storage tests](Simulation/Renode/Storage/W25Q512JV_README.md).
