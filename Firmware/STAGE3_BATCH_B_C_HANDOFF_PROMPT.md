> Historical engineering record: the ZIT6 integration supersedes these build/staging instructions. See [current firmware instructions](README.md). The original evidence below is retained.

# New-chat handoff — STM32H745 Stage 3 B and C

You are a senior embedded robotics architect and implementation engineer. Continue my graduation-project **STM32H745 dual-core (Cortex-M7 + Cortex-M4)** firmware on **GitHub repository https://github.com/sara-eng963/GPRobotControl_Simulation**, branch **sara/can-replacement**.

**Read the current branch and recent commits before proposing edits.** The repository, authoritative firmware configuration and running test results are the source of truth. The historical roadmap documents contain time-stamped states; do not treat their earlier "pending" statements as current without inspecting newer commits.

## Starting state / evidence

- Stage 1 dual-core project, Stage 2 SRAM4/HSEM/FreeRTOS Message Buffers and EXTI IPC, Stage 3A audit, Stage 3B M4 IPC and M7 Supervisor, and Stage 3C M7 CubeMX FDCAN/CANopen and timer integration have been implemented incrementally.
- M7 owns CANopen, motion authority, state machine and trajectory consumption. M4 owns NOR storage, trajectory production and HMI-facing IPC. Cross-core data uses fixed SRAM4 ABI plus ST FreeRTOS AMP/EXTI.
- M7 CubeMX TIM2: 32-bit, 1 MHz (64 MHz /64). TIM3: 500 Hz (2 ms), HAL NVIC priority 5. Existing TIM6 M7 and TIM7 M4 timebases preserved. Supervisor uses a hardware timing port on target; host simulation keeps the old RTOS-tick fallback. A host harness compiles actual extracted C blocks and tests missed notifications, wrap and overruns.
- FDCAN1: STM32CubeMX-generated ST HAL1, Classic CAN, normal, 1 Mbps theoretical bit timing using 43 MHz PLL2Q; the official HAL backend references the existing CANopen owner. Six drives and their real configuration are not present.
- Both independent CubeIDE verification ELFs **previously built and symbol-checked**: M7 text=167264, data=112, bss=92440; M4 text=24644, data=100, bss=21148. These are section totals, not per-RAM-bank budgets.
- Physical board provider `gp_board_supply_commissioned_config` is **absent by design**. No synthetic hardware-ready, motor-feedback, E-stop, STO or drive authorizations. Do not run real motion or claim hardware 500 Hz.
- **Stage 3 Batch A acceptance:** use the latest operator-run report `GP_Stage3_BatchA_Acceptance/.../BatchA_Acceptance.md` and `full_run.log`. Before treating A as closed, demand the actual final **PASS / 16 gates** output from `python3 Firmware/Tests/run_stage3_batch_a_acceptance.py <GP_Stage3_Repro>`. If that report has not been shared, ask for it. A script or an earlier partial test PASS is not final Batch A acceptance.

## Mission

Implement **Batch B**, then **Batch C**, in substantial cohesive changes rather than endless individual one-line fixes. You may inspect and update GitHub, add tests and documentation, and ask me for a single WSL/CubeIDE acceptance run per meaningful batch. Never claim tests ran on my PC unless I pasted actual output. GitHub edits should use atomic, reviewable commits without overwriting untracked work, unrelated branches or local modifications.

### Batch B — functional application composition (3C-3, 3D-1, 3D-2)

1. **3C-3 M7 trajectory execution:** Trace the existing `Intercore/trajectory_stream_path_port.c`, `StateMachine/States/state_path_execution.c`, `StateMachine/supervisor_task.c`, `PathExecutionServices` and CANopen drive APIs. Wire the canonical M7 stream consumer into execution with one owner, bounded 2 ms work, correct trajectory identity/CRC, expiry, underrun, missed deadline, following-error and safe release/acknowledgment policies. Do not fabricate target feedback or allow fault/pause/resume to silently resume an unsafe path.
2. **3D-1 cross-core validation:** Inspect `Intercore/validation_storage_bridge.*`, `GpValidationShared`, all CM7/CM4 linker scripts, MPU, caches and actual SRAM4 map. Reserve and verify the proposed 12 KiB validation slot `0x38005000..0x38007FFF` only if ABI sizes and alignment fit. Use genuine non-cacheable shared memory plus existing IPC; test ownership, cancellation, stale epochs, retries and worker failure under host/ARM mocks before claiming target completion.
3. **3D-2 M4 NOR / stream producer:** Audit the validated-trajectory and memory system already implemented in `Simulation/Storage`, `Simulation/Renode`, `Intercore/` and existing external-memory tests. These are **references**, not a production HAL flash driver. Use **STM32CubeMX + official STM32CubeH7 HAL** QSPI/OSPI infrastructure only when the actual peripheral, storage part and pin/clock resources are resolved. Implement a board-independent HAL adapter contract without inventing chip pins or passing fake flash results. M4 produces trajectory samples with bounded refill, durable commit/CRC, cancellation, underrun/overflow recovery, and correct power-fail semantics. For unresolved hardware, keep a compile-safe, non-operational port with explicit blockers instead of a simulated production success.
4. **HMI:** Implement IPC-based M4 command/status producer and freshness/error policy. Do not directly call M7 private FreeRTOS/task APIs from M4; do not assume a specific physical HMI transport until chosen.

Deliver Batch B code, tested source manifests, host failure-injection/ABI tests, ARM compilation, dual-core ELF/source requirements, a concise architecture/data-flow diagram, and a single WSL acceptance script. Request fresh CubeIDE builds only when target sources changed.

### Batch C — integrated software release qualification (3E)

Run a whole-system regression of M7 state machine + CANopen + timer + stream consumer ↔ shared SRAM4 + M4 validation/storage/HMI production; cover boot/reset and independent-core restart, stale status, bus-off, lost IPC wakeups, invalid drive frames, CRC corruption, queue saturation, flash busy/failure, late samples and controlled stop. Produce realistic **measured host/mock results**, not invented hardware metrics.

Audit **CM7/CM4 ELF linker-map memory per bank**, stack/high-water/heap assumptions, ISR priority and task WCET budget, CAN 1 Mbps / six-node 500 Hz message budget, and CPU contention; label analytical upper bounds distinctly from physical measurements. Create reproducible non-destructive all-tests scripts, require final independent clean CubeIDE Debug builds, record actual symbols/sizes/reports and unresolved hardware commissioning gates. Finish with an evidence-based Stage 3 software acceptance report and precise hardware-arrival test plan.

## Rules and workflow

- Read `Firmware/STAGE3_APPLICATION_INTEGRATION.md`, `Firmware/STAGE3_BATCH_A_M7_COMPOSITION.md`, `Firmware/TIM2_TIM3_CUBEMX_CONFIGURATION.md`, `Firmware/HARDWARE_ARRIVAL_VERIFICATION_PLAN.md`, actual code, linker scripts, hardware `.ioc`, and test outputs. Treat documentation history as historical, not proof of current runtime.
- Reuse official ST HAL/LL, CMSIS, CubeMX-generated project and FreeRTOS; don't create a parallel custom IPC scheduler, hand-written peripheral driver or duplicate CANopen master. Compare the existing coordinator with CANopenNode and document the decision before a rewrite.
- Protect the **independent** `D:\University\Graduation Project\STM32\GP_Stage3_Repro` CubeIDE verifier and its pre-timer backup. Do not delete or overwrite unrelated untracked local files.
- No physical STM32/robot, CAN drives, flash or safety system exists for tests today. Distinguish **static**, **host simulated**, **ARM compile**, **target ELF**, and **physical runtime** evidence with explicit PASS/FAIL/DEFERRED statuses.
- Avoid arbitrary design substitutions, optimistic placeholders, broad unverified claims, and repeated tiny corrections. Fix actual bugs with regression tests.

**Start by auditing the current repository, identifying exactly which Batch B interfaces are already implemented versus still unwired, and proposing a dependency-ordered finite work breakdown. Then implement and test Batch B before beginning Batch C.**
