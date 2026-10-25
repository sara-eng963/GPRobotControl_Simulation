# STM32H745ZIT6 integration result - 2026-10-25

Integrated directly on sara/can-replacement, starting at
0f87f940e5496f6a1f7dd152b6c4009be68645dc. No commit, push, reset, flashing or
hardware motion was performed. Unrelated untracked files were preserved.

## Final structure and changes

Firmware/STM32H745 is the single active target project. It contains the
authoritative ZIT6 .ioc, both CubeIDE core projects, ZITX startup and linker
pairs, generated initialization, HAL/CMSIS/FreeRTOS and application composition.
Original Windows project hashes were verified unchanged.

Added/imported: ZIT6 generator/startup/linker configuration; target application
sources and headers; UART HAL dependencies; Common transport headers/sources;
build_stm32.py, check_firmware_integration.py, check_portable_protocol.py,
CM4 policy fixtures and a target CI workflow.

Modified: CM4 validation service/header and startup user block; both core
project metadata/preferences; SRAM4/transport identifier references; active
firmware checks; root CTest registration; README files and .gitignore.
The dedicated virtual-node CMake target now links its missing CiA402 source.
Two existing standalone CI commands now link their missing storage dependencies.

Removed as superseded: GP_STM32H745.ioc; both startup_stm32h745iitx.s files;
the four STM32H745IITX FLASH/RAM linker scripts. The old CM4 service was adapted,
not discarded. No old generated configuration was transplanted.

Eliminated divergent source copies: target IPC/trajectory transports and three
shared headers now live in Common; CM4 links the two M4 storage units from the
existing target implementation. Twenty-five portable math/drive/guidance C
units link to repository sources; forwarding headers use their single
declarations. The three math differences were only explicit C11 const casts
already present in the host implementation. Host APIs were not renamed.

Twenty-eight old staging/acceptance tools are preserved byte-for-byte in
Tests/historical. Historical milestone documents are marked as superseded.
The pre-existing local README and prior FIFO README were preserved in History.
Ignored migration candidates, archives, patches and unrelated tools remain.

## CM4 worker

The default task initializes IPC, attempts storage-worker startup and enters
the existing 1 ms IPC/HMI loop. The separate priority-1 worker steps validation
and playback with a 1 ms delay. Both real backend/committed-reader suppliers
are mandatory; none is implemented by default. Missing or invalid providers
leave worker_ready false; allocation failure revokes readiness. A repeated start
cannot create a second owner. Physical callbacks remain on CM4.

## Executed validation

ARM GCC 13.2.1 compiled/linked the CubeIDE source graphs using GNU nano/nosys
specifications and the checked-in target linker scripts. CubeIDE GUI execution
and its bundled compiler version were not tested.

| Image | Source units | Debug text/data/bss | Release text/data/bss |
| --- | --- | --- | --- |
| CM7 | 92 | 193144 / 112 / 105344 | 100528 / 112 / 105336 |
| CM4 | 48 | 33492 / 100 / 37416 | 19916 / 100 / 37416 |

Both configurations linked without compiler diagnostics. Actual ELF audits
verified Reset_Handler, scheduler, Supervisor, IPC and worker symbols; no M4
flash-step routines are retained in CM7. Project links and include directories
resolve locally for both configurations. Preference IDs match .cproject IDs.

CM7 actual NOBITS allocations: 2208 bytes at 0x38000000; 12352 at 0x38001000;
12288 at 0x38005000, all aligned to 32 bytes. CM4 has empty NOLOAD reservations
and defines none of these objects. Empty GNU ELF reservations can be PROGBITS
with zero size; linker scripts were not changed to accommodate this display.
All four FLASH/RAM synthetic linker checks passed. Shared ABI/capacity,
startup/HSEM order, EXTI ownership, MPU source configuration and absence of
legacy source identifiers passed.

Fresh host CMake builds passed with SIL Kit both enabled and disabled.
The original 77 host CTest cases all passed (431.61 s), including five genuine
Renode QSPI runs, IPC/FreeRTOS, CANopen, storage/power-loss, Supervisor, virtual
F/T and hand-guiding tests. Two newly registered target policy/protocol CTests
also passed: 79 cases verified in total, across the full and targeted runs.
Eleven target protocol files passed strict normalized token comparison; four
existing protocol fixtures passed against the renamed target implementations.
CM4 service policy passed static/dynamic allocation, missing/malformed providers,
bad periods, allocation rollback, duplicate starts and M7/M4 callback separation.

Fresh standalone SIL Kit 5.0.7 single-node and six-node integration tests and
the CAN backend transport test passed after fixing the missing CiA402 source
list entry. The corrected standalone CI path-port compile/link/run passed.
Production timer fault injection also passed wrap, notification loss/coalescing,
clock mismatch, partial startup rollback and cycle-overrun cases using the
ZIT6 100 MHz APB1 / divider-2 / prescaler-199 configuration.
All active target compile/static gates and git diff --check passed. The new
GitHub workflow was checked locally; it has not run on GitHub.

## Remaining physical work

No physical F/T provider, CM4 NOR backend/reader or commissioned board supplier
is present. Final pin mapping, CAN/transceiver/drive commissioning, interlocks,
E-stop/STO/fault tests, dual-core reset/cache/IPC verification, RTOS stack
watermarks and 500 Hz deadline measurements remain. Hardware safety validation
and explicit robot-motion authorization are still required. Simulation/build
results provide neither.

Build logs, initial Git inventory/diff, firmware backup, source hashes and the
complete changed-file list are under build/stm32-integration (ignored).
Current reproducible commands are in README.md. No migration helper is needed
to use the integrated repository.
