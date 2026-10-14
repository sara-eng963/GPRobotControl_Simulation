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
