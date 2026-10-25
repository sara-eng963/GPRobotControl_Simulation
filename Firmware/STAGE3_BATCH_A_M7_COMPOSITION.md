> Historical engineering record: the ZIT6 integration supersedes these build/staging instructions. See [current firmware instructions](README.md). The original evidence below is retained.

# Stage 3 Batch A — M7 CANopen composition (2026-10-14)

**Scope:** compile-safe bridge from a real board-supplied commissioning configuration through the existing ST HAL1 FDCAN backend into the existing CANopen coordinator and Supervisor, without inventing board inputs or bypassing independent E-stop/STO. The active CubeMX FDCAN1 setup remains generated; no manually duplicated peripheral init or register accesses.

**Changes**
- `CM7/Board/Inc/board_commissioning.h`: explicit commissioning contract supplied by the *actual* physical board implementation; includes configured node IDs/timeouts, Supervisor dependencies, and interlock-check callback. No implementation or assumed node IDs are provided.
- `CM7/Core/Src/application_supervisor.c`: calls the absent-by-default **weakly referenced** `gp_board_supply_commissioned_config`. If absent or invalid, returns false **before opening FDCAN**. If provisioned, validates physical input callbacks, snapshot validity and stop inputs, starts the existing HAL adapter, initializes the existing CANopen master with explicit node IDs, and injects it into the Supervisor. Failed coordinator initialization closes the transport. State, backend and commissioning data use static storage rather than the small M7 default task stack.
- Supervisor runtime remains **intentionally disabled by default**. The existing `application_supervisor_board_configure` weak hook stays for compatibility. Real board I/O and physical safety architecture must be reviewed separately; passing a software interlock callback is not safety certification. Never provide a fake "healthy" supplier for production firmware.

**Pre-hardware acceptance test:** `bash Firmware/Tests/check_m7_board_canopen_composition.sh` compiles the actual HAL-active CM7 source with the ARM toolchain and checks the CANopen transport reference and weak-unresolved supplier. Existing host mock `bash Firmware/Tests/check_fdcan_hal_adapter_mock.sh` should also pass; then stage into a **separate** CubeIDE verifier and clean-build both cores. Pending actual user run results must be recorded. The real board provider, 2 ms hardware timer/clock, task WCET, bus load, physical FDCAN transceiver, CiA402 device EDS and safety paths remain **unverified**.

**Next design constraint:** the supervisor and CANopen TPDO timestamps are still derived from RTOS milliseconds. Do not claim deterministic 500 Hz until CubeMX-configured target microsecond timebase, bounded cycle and scope/logic-analyzer measurements exist. A 1 Mbit/s six-motor bus remains a feasibility question, not established fact.

### Verification-script correction and rollback hardening (2026-10-14)

First local operator run of `check_m7_board_canopen_composition.sh` **completed both ARM compilations and the linker-symbol checks**, then stopped at an overbroad Python source assertion: `'HAL_FDCAN_Init' not in a`. The C composition source mentions the API **in a comment** to identify the CubeMX ownership boundary; this was a **test false positive**, not evidence that HAL was initialized twice. Corrected check inspects `arm-none-eabi-nm -u` on the compiled application object to ensure there is **no actual HAL_FDCAN_Init reference** (`20b213b`). After reviewing startup failures, the initial composition now calls `canopen_master_close` on owned, unstarted CANopen backend when a later Supervisor/IPC/startup gate fails (`4b780b5`). The static assertion also checks rollback presence (`1a5d177`).

The initial test command had `set -e` and therefore **stopped before** running subsequent M7/M4 regression tests or staging files into the independent CubeIDE project. **New commits have not been locally rerun or clean-built yet**. Resume the consolidated verification gate; do not claim PASS until actual output is available.


### Local rerun stopped at malformed test script; restored (2026-10-14)

The second user-executed Batch A attempt (`set -euo pipefail`) stopped in `check_m7_board_canopen_composition.sh` at **Bash syntax error near line 54**, before any ARM/test assertions or staging in that attempt. Repository inspection found the preceding automated edit had accidentally copied a substantial suffix of the file twice and broken the `grep -Eq` quoting; this is a **test script corruption**, not an ARM firmware compilation result. The script was reconstructed from its known intact Git revision `f2763f9`, with the intended `nm`-based HAL reference check, absent board-supplier check and failure cleanup assertion reapplied in **`038be21`**. Next run must explicitly start with `bash -n Firmware/Tests/check_m7_board_canopen_composition.sh`; all ARM, host regression, CubeIDE and ELF results for the corrected Batch A revision are still pending operator evidence.


### Operator software acceptance: all source gates PASSED (2026-10-14)

Operator pulled commit `fc4dc63` into `~/freertos-pc-test` and executed the consolidated Batch A verification with `set -euo pipefail`. The following **actual reported outputs PASSED**:

```text
PASS: All five test scripts parse correctly
PASS: board-supplied CANopen composition, fail-closed startup invariants
PASS: HAL-active M7 CANopen composition ARM-compiled; adapter linked by reference
PASS: M7 board gating, IPC ownership, and CubeIDE settings
PASS: CM7 application ARM compile-only check (NOT target link)
PASS: HAL1 Classic CAN backend mock frame and failure-path tests
PASS: staged canonical CM7 application C and headers
PASS: CM7 application staging manifest and repeatability
PASS: CM4 task wiring and nonblocking shared transport contract
PASS: CM4 application IPC ARM compile-only gate (not a firmware link)
PASS: Batch A software checks and synchronization
Commit: fc4dc63
```

The checked-out M7 `application_supervisor.c` and new `board_commissioning.h` were copied into `D:\\University\\Graduation Project\\STM32\\GP_Stage3_Repro` and verified with `cmp`. This is **successful local source-level/ARM-object and host mock verification**, *not* new CubeIDE firmware ELF build results. Required next evidence: clean-build CM7 and CM4 in the independent reproduction workspace, inspect ELF for **strong** `gp_fdcan_classic_backend_open` and `canopen_master_init`, **weak** `application_supervisor_board_configure`, and no strong physical supplier. Physical bus, drive commissioning, microsecond timer and measured 500 Hz constraints remain pending.


### First complete target ELF evidence after Batch A integration (2026-10-14)

Operator independently reported **both clean STM32CubeIDE Cortex-M7/M4 builds completed with zero errors** in `GP_Stage3_Repro` after the Batch A source checks and synchronization. Both ELF files existed. Symbol and size output, copied verbatim from the operator's WSL console:

```text
CM7: 08019e04 W application_supervisor_board_configure
CM7: 08000b74 T canopen_master_init
CM7: 080199b0 T gp_fdcan_classic_backend_open
CM4: 08100a5c T application_ipc_service_run

text    data  bss    dec
CM7: 163656  112   92200  255968
CM4: 24644   100   21148   45892
```

**Verified conclusion:** The previously discarded `gp_fdcan_classic_backend_open` is now strongly retained in the real **CM7 ELF**, alongside `canopen_master_init`; the M7 board-provider fallback remains **weak** and the selected board-supplier symbol was not displayed, so no physical board configuration was linked/authorized. M4 IPC service is strongly retained. The recorded M7 `text` increased by 3864 B and `bss` by 664 B compared with the initial clean baseline; both cores link.

**Remaining issue in the one-off verification command:** It printed `=== Required symbol checks ===` but not the final `PASS` sentinel before returning to the shell prompt. Consequently do **not** claim the full strict shell gate completed. A plausible mechanism is `arm-none-eabi-nm ... | grep -Eq` combined with `set -o pipefail`, which can yield SIGPIPE when `grep -q` stops early; this is not yet proven. The already printed symbol table is independently convincing on the specific named functions; a follow-up robust check should capture `nm` to a file and verify with anchored patterns without producer-side SIGPIPE. Neither this ambiguity nor zero-error builds establish the 2 ms deadline or hardware operation.

**Next batch (timing):** The current `supervisor_now_us` derives time from the 1 kHz FreeRTOS tick, and Supervisor uses `vTaskDelayUntil`. Path execution already uses absolute sample deadlines and skips missed/expired samples, but physical microsecond resolution and loop timing remain unverified. A CubeMX-generated 32-bit hardware timer (e.g. TIM2/TIM5) and careful monotonic-wrap semantics are needed before claiming real 500 Hz timing.

### Reusable target ELF verification (2026-10-14)

A repository-owned `Firmware/Tests/check_stage3_batch_a_target_elfs.sh` gate was introduced in commit `f4f1cbf`. It checks the **already built** CM7/CM4 ELFs, captures their `arm-none-eabi-nm` symbol listings to files (avoiding the possible `nm | grep -q` + `pipefail` SIGPIPE early exit), and checks strong M7 FDCAN HAL/adapter/CANopen/Supervisor symbols, weak M7 fallback, no strong real board supplier, and M4 IPC symbols. Prints the actual ARM ELF size data and final PASS sentinel. This new script is source-controlled but **has not yet been executed by the operator**. It is a repeatable static-image gate, not hardware testing.


### Batch A target ELF gate PASS — verified final result (2026-10-14)

The operator ran the repository-owned `Firmware/Tests/check_stage3_batch_a_target_elfs.sh` against independently clean-built `GP_Stage3_Repro` firmware at the Batch A source baseline, after synchronizing and building both cores (operator reported both CubeIDE builds with **zero errors**). **The complete script exited through its final `PASS` sentinel**:

```text
=== M7 HAL + CANopen + fail-closed composition ===
PASS: T HAL_FDCAN_Init
PASS: T HAL_FDCAN_MspInit
PASS: T gp_fdcan_classic_backend_open
PASS: T canopen_master_init
PASS: T application_supervisor_start
PASS: W application_supervisor_board_configure
NOTE: no unresolved weak board supplier symbol displayed (check linker/map manually)
=== M4 IPC ===
PASS: T application_ipc_service_run
PASS: T gp_ipc_m4_receive
=== Firmware image sizes ===
CM7 text=163656 data=112 bss=92200 dec=255968
CM4 text=24644 data=100 bss=21148 dec=45892
PASS: Stage 3 Batch A target ELF symbol gate
```

**Acceptance: PASSED for software compile, link, and retained symbol inventory.** A missing unresolved weak `gp_board_supply_commissioned_config` entry in `nm` is **not** proof that the no-board condition has been exercised as a functional runtime test; the emitted `application_supervisor_board_configure` symbol remains weak, and no strong supplier was found by the gate. Record physical absent-provider startup behavior, safety diagnostics, and 500 Hz measurements as separate pending tasks.

**Next focus:** STM32CubeMX-generated **TIM2** microsecond monotonic timebase (no changes to existing TIM6 HAL tick), overflow-safe 64-bit extension and separate finite-deadline timing diagnostics. The M7 Supervisor currently derives `now_us` from a 1 kHz FreeRTOS tick; the path executor already uses absolute deadlines and missed-deadline failures. Do not invent timing precision or claim 500 Hz hardware determinism prior to actual cycle measurements.


### TIM2/TIM3 CubeMX configuration walkthrough — work in progress (2026-10-14)

The operator confirmed an identical **independent** backup of the separately clean-built project `GP_Stage3_Repro` at `GP_Stage3_Repro_Backup_Pre_TIM2_TIM3` before opening `GP_STM32H745.ioc` in STM32CubeMX 6.18.1. MCU shown: `STM32H745IITx`, LQFP176. APB1 peripheral and **APB1 timer clock = 64 MHz**, APB1 prescaler /1.

**TIM2 screenshot-confirmed:** assigned to **Cortex-M7** only, internal clock, channels off, PSC=63, up-counting, ARR=4294967295 (32-bit), clock division none, auto-reload preload disabled. The TIM2 global NVIC interrupt checkbox **was checked**, and **priority 5** was displayed but greyed out. Operator expressed a preference for priority 6; there is **no screenshot evidence TIM2's priority has changed to 6**. Priority 5 is permitted for the configured `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` if using FreeRTOS `FromISR` APIs (although TIM2 overflow handling need not call them).

**TIM3 screenshot-confirmed:** assigned to **Cortex-M7** only, internal clock, channels off, PSC=63, up-counting, ARR=1999 (16-bit), clock division none, auto-reload preload disabled. At the verified 64 MHz timer input this produces `64,000,000 / [(63 + 1) × (1999 + 1)] = 500 Hz` → **nominal 2 ms hardware update events**. The earlier TIM3 NVIC screenshot showed the global interrupt **unchecked** with a greyed priority 5; no later interrupt checkbox screenshot has yet been supplied.

**Current milestone = GUI configuration in progress only.** TIM2/TIM3 settings have **not yet been demonstrated saved in the authoritative repository `.ioc`**, no source generation/review has been verified, and TIM2/TIM3 are not started by current code. TIM6 M7 HAL timebase and TIM7 M4 assignments were seen in the timer overview and should remain untouched. Pending: TIM3 NVIC enablement + final NVIC review, CubeMX settings save and one controlled code-generation review, generated-source merge, validated overflow-safe microsecond-time adapter, FreeRTOS notification scheduling, missed-wakeup/deadline accounting and target clean builds; physical 500 Hz timing remains **unmeasured**.

See **[TIM2/TIM3 CubeMX configuration log](TIM2_TIM3_CUBEMX_CONFIGURATION.md)** for screenshot-derived settings, status, calculation and full pending acceptance gates.

