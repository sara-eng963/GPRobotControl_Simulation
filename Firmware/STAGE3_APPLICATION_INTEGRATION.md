> Historical engineering record: the ZIT6 integration supersedes these build/staging instructions. See [current firmware instructions](README.md). The original evidence below is retained.

---
title: "Stage 3 — STM32H745 Application Integration"
stage: 3
status: "Stage 3B ARM/ELF verified; FDCAN HAL built; board composition, CM4 storage and final regressions pending"
date: 2026-10-14
branch: "sara/can-replacement"
tags: [stm32h745, freertos, robotics, firmware]
---

# Stage 3 — STM32H745 Application Integration

**Audit baseline:** `d18bdec71abc62fa835afe588398f9aec074e649` (2026-10-12).  
**Historical Stage 3A audit scope (2026-10-12):** initial repository/source review and ARM compile-only reconnaissance. The historical assessments and pending states **below refer to their original checkpoints**; use the current-status section immediately below for up-to-date verification. No hardware runtime verification has been performed.


## Current Stage 3 status — 2026-10-14 (software vs. hardware)

**Authoritative baseline:** branch `sara/can-replacement`; FDCAN-enabled `.ioc` committed as `c3d07a2`; generated dual-core firmware / HAL committed as `d8a2b32`; HAL source file modes corrected in `809882f`. Historical sections below retain their original *as-of* timestamps; a prior phrase such as "not yet generated" must not be mistaken for current status.

| Milestone | Current evidence | Still open to complete Stage 3 |
| --- | --- | --- |
| **3A.1 architecture audit** | Documented application ownership and ARM reconnaissance checks | No redo needed |
| **3A.2 resource/CANopen decision** | M7 motion/CANopen and M4 storage/HMI ownership proposed | Freeze task/priority/memory budgets; select and justify existing CANopen master vs CANopenNode; finalize hardware-dependent allocations |
| **3B M4 IPC + M7 Supervisor composition** | Both modules ARM-compiled, included in existing CM4/CM7 Debug ELFs; 0 errors / 0 warnings on both final local builds | Real IPC, boot/reset recovery, stack watermarks, HMI producer, safety I/O; board provider currently refuses M7 Supervisor startup |
| **3C-1 robot application composition** | StateMachine/CANopen/ServoDrive sources staged and statically linked into CM7; `application_supervisor_start` present | Implement board-provided real adapters with **fail-closed startup** (no fake healthy inputs); operational drive and state tests |
| **3C-2 FDCAN and cycle timing** | CubeMX .ioc + vendor HAL source/header + init + linked HAL symbols verified; HAL-enabled portable adapter compiled in `.o`; 43 MHz PLL2Q, 1 Mbit/s config | Connect `gp_fdcan_classic_backend_open` to the CANopen owner through a **strong board provider**; define safe startup/error policy, actual motor dictionary; monotonic microsecond clock and bounded 2 ms scheduling; physical CAN & bus-load test later. Adapter is **discarded from ELF** until referenced. |
| **3C-3 M7 path execution** | Portable trajectory/IPC contracts, host simulation/streaming work | Connect and exercise real firmware trajectory consumer/PathExecution port; bounded sample clock, starvation, following-error handling and ARM/map evidence |
| **3D-1 cross-core validation memory** | Stage 2 SRAM4 base slots/MPU/linkers verified | Reserve, initialize, cross-check `GpValidationShared` ABI and link map; integrate client/worker and failure tests |
| **3D-2 M4 NOR/HMI** | Separate simulated/file/Renode memory work and M4 IPC task | CubeMX/HAL external flash port and M4 producer/validation, refill/cancel/CRC tests; implement chosen physical HMI transport |
| **3E regression/reproducibility** | Current Windows verifier builds CM7: text 159792/data 112/bss 91536, CM4: text 24644/data 100/bss 21148; both 0 errors and 0 warnings. HAL1 test mocks passed earlier. | ****Independent checkout + clean CubeIDE builds PASSED 2026-10-14**; follow-up symbol/map + host runtime regressions**, symbol/map verification, host integration/fault-path regressions and release evidence pack. Physical tests separately governed by `HARDWARE_ARRIVAL_VERIFICATION_PLAN.md`. |

**Known board-provider result:** CM7 ELF contains `080198a2 W application_supervisor_board_configure`, a weak fail-closed fallback. `gp_fdcan_classic_backend_open` is present in `CM7/Debug/Board/Src/fdcan_classic_backend.o` but discarded in final ELF as unreferenced. Thus "STM32 HAL FDCAN builds" **does not** mean the motor CANopen stack is active.

**Reproducibility progress:** an independent checkout/workspace clean-build **PASSED** at `a331dea` as detailed below; remaining major work is 3C board/CANopen and motion scheduling, 3D SRAM4 validation/NOR/HMI and 3E host/target failure regression. Do not classify any hardware gate as passed without board evidence.


## Stage 3E — Independent dual-core clean-build reproducibility (2026-10-14)

**Evidence supplied by operator via separate STM32CubeIDE screenshot for each target.** Source checkout: `git worktree add --detach ~/gp-stage3-repro-src origin/sara/can-replacement`, resolved **`a331dea`**. Independent project: `D:\\University\\Graduation Project\\STM32\\GP_Stage3_Repro` with its own `CubeIDE_Stage3_Repro_Workspace`. The canonical `Firmware/STM32H745` tree was copied from the fresh worktree, and the tracked `stage_intercore_headers.sh`, `stage_intercore_sources.sh`, `stage_m7_application_sources.sh` commands completed with all reported PASS messages. No preexisting Debug objects, legacy verification project, or manual ST HAL fixes were copied. Root/Cortex-M7/Cortex-M4 Eclipse projects imported successfully with "Copy projects into workspace" disabled, and M7 FDCAN relative `PARENT-1-PROJECT_LOC` link was source-checked.

The operator selected **Clean Project → Build Project** for each core, with the following clean-build console evidence:

| Target | text | data | bss | dec | IDE result |
| --- | ---: | ---: | ---: | ---: | --- |
| Cortex-M7 | 159792 B | 112 B | 91536 B | 251440 B | **0 errors, 0 warnings** (15:28:05) |
| Cortex-M4 | 24644 B | 100 B | 21148 B | 45892 B | **0 errors, 0 warnings** (15:29:52) |

Both `GP_STM32H745_CM7.elf` and `GP_STM32H745_CM4.elf` linked; sizes **exactly match** the previously working standalone verifier. **Stage 3E's fresh-checkout dual-build compile/link gate is PASSED** for the pinned toolchain/project and generated source configuration. A full reproducibility artifact should still include ELF symbol/map audit and host regressions at the final Stage 3 integration baseline. Identical sizes alone do **not** prove byte-identical output, live IPC/CANopen operation, 500 Hz motion timing, or safety validation. The HAL ↔ CANopen adapter is still discarded from M7 ELF, and the board provider is still weak fail-closed until integrated.

## Planned larger-batch execution cadence (2026-10-14)

To avoid incremental one-file-at-a-time reviews, group the remaining software work into cohesive **end-to-end batches** with one acceptance report per batch:

1. **Batch A — M7 CANopen board composition + timing (3C-1/2):** close the HAL FDCAN-to-CANopen owner call path; define initialization/fault policy and transport diagnostics; implement a verified monotonic timebase and bounded 2 ms software-cycle contract using CubeMX/ST/FreeRTOS primitives. No fake drive-ready/safety signals or unqualified motion authorization. Preserve hardware-absent fail-closed operation. Acceptance: HAL-active adapter retained and referenced in image, both ELF links, host mocks/fault tests, and explicit unsupported-hardware gates.
2. **Batch B — Motion execution + cross-core validation/storage (3C-3/3D-1/3D-2):** connect M7 trajectory consumer and path-execution services, M4 validation shared ABI/linker allocation, official HAL storage port abstraction and producer with predictable refill/CRC/cancel semantics. Bind physical HMI only when transport is actually specified. Acceptance: source-backed ARM/linker/section checks, boundary/underrun/cancel/recovery host tests and both firmware builds. Keep physical flash tests pending hardware.
3. **Batch C — Software release qualification (3E):** whole-system integration regressions, target stack/heap/memory budgets, scheduling and CAN load feasibility models, fault injection, deterministic test scripts, reproducible clean source rebuild and documentation sign-off. Acceptance must explicitly separate **simulation/static verification** from physical measurements. The existing separate `HARDWARE_ARRIVAL_VERIFICATION_PLAN.md` remains the gate for powered board, bus, real-time and safety testing.

**Standards-first constraints:** use STM32CubeMX-generated low-level FDCAN/QSPI/timers; official ST HAL/LL/CMSIS/FreeRTOS interfaces; verify CANopen CiA-301/402 behavior against established stacks and vendor EDS; only use custom glue for robot-specific application protocols. Reject placeholder motion-enabling board hooks, ad-hoc drivers, invented memory/stack budgets or declaring hardware validation from simulation. Track actual test commands/results by commit. 


### Batch A — M7 board commissioning composition (code committed; tests pending)

Implemented in **`2577f93`**, with M7 startup stack pressure reduced in **`7123734`**. Rather than fabricating physical I/O or motor node IDs, `CM7/Board/Inc/board_commissioning.h` defines a required **externally supplied** `gp_board_supply_commissioned_config` contract. The existing **weak fail-closed** `application_supervisor_board_configure` now validates physical interlock callback, physical supervisor I/O availability, coherent non-stop input snapshot and explicit six-node CANopen IDs/timeouts before calling `gp_fdcan_classic_backend_open` and `canopen_master_init`. Static storage avoids moving another large commissioning/config object onto the provisional 512-byte CubeMX defaultTask stack. **No commissioned physical supplier is defined; there is no active motion permission.**

The code establishes a real HAL ↔ CANopen composition call path, while retaining source-of-truth ST HAL and existing CANopen coordinator. **Tests/builds for these new commits have not yet been executed by the operator.** Required next local checks: `bash Firmware/Tests/check_m7_board_canopen_composition.sh`, `bash Firmware/Tests/check_m7_application_integration.sh`, `bash Firmware/Tests/check_fdcan_hal_adapter_mock.sh`; stage changed M7 application file and new board header into the **separate** CubeIDE reproduction project; clean-build both targets and inspect `gp_fdcan_classic_backend_open` in M7 ELF. Conditional hardware logic cannot be claimed operational without real board supplier + safety design + timing qualification. Precise 2 ms hardware timing and motor bus saturation analysis are still open.

See **[STAGE3_BATCH_A_M7_COMPOSITION.md](STAGE3_BATCH_A_M7_COMPOSITION.md)** for the board port contract, scope and limitations.

## 3A. Source-backed architecture decisions

1. **Keep the Stage 2 boot, ABI, linker and EXTI infrastructure.** The tracked CM7 and CM4 `main.c` files call Stage 2 initializers before their independent FreeRTOS schedulers. Both still create only the CubeMX `defaultTask`; neither composes the real robot application. The Stage 2 CubeIDE builds were performed in a staged verification workspace; compiling the source in isolation or seeing a symbol in a host binary is not target-image integration.
2. **M7 owns motion authority, CANopen network state and the trajectory consumer.** Integrate the existing `StateMachine/`, `CANComm/CANopen/`, `ServoDrive/` and `Intercore/trajectory_stream_path_port.c` modules through a target composition root. Do not permit M4 commands, flash completion or HMI messages to authorize motion independently. Dedicated hardware safety inputs and outputs still lack a board adapter.
3. **M4 owns NOR transactions and the trajectory producer.** Reuse `Intercore/trajectory_stream.c`, `validation_storage_bridge.c`, and the existing validated-storage interfaces. `Simulation/Storage/file_validated_storage.c` is file/POSIX-backed; `Simulation/Renode/Storage/stm32h7_w25q_bus.c` accesses emulated/register-level QSPI, **not a qualified STM32CubeH7 HAL production flash port**. Do not link these simulation backends into the final firmware.
4. **Single-owner inter-core channels remain nonblocking.** Stage 2 uses native FreeRTOS 10.6.2 static Message Buffers in SRAM4 and ST HAL EXTI software notifications. `StateMachine/supervisor_task.c` already consumes up to eight M4 HMI messages per cycle and publishes best-effort M7 status. It is not yet instantiated in the target. M4 needs one owner task that drains M7 status, handles local service requests and retries without blocking either kernel. EXTI coalescing must be handled by draining plus bounded periodic polling, not by assuming one IRQ per message.
5. **Do not port the host HMI task to M4 unchanged.** `HMI/hmi_task.c` invokes `supervisor_task_post_hmi()` and `supervisor_task_get_diagnostics()` directly. These are local M7 FreeRTOS APIs. M4 must instead translate transport events into `GP_IPC_HMI_COMMAND` messages and derive display state from `GP_IPC_CONTROLLER_STATUS`. The GUI/SDL, UDP, SIL Kit and MATLAB tooling remain host-only. An actual HMI physical transport is not yet specified.
6. **Keep firmware/host/simulation boundaries explicit.** `CANComm/can_backend.h` is hardware-independent and suitable for an STM32 FDCAN adapter; `CANComm/SILKit/` is C++ host code. The existing `CanopenMaster`, `CiA402` and AVATAR drive implementations may be ARM-portable but their CANopen compliance and completeness are not thereby established.
7. **Do not claim a 500 Hz execution clock.** The checked-in `.ioc` still uses ~64 MHz startup clocks and TIM6/TIM7 for HAL ticks; it configures no FDCAN, QSPI or dedicated motion-cycle timer. `supervisor_task.c` uses the 1 ms FreeRTOS tick as a microsecond time source and `vTaskDelayUntil`, which cannot establish microsecond deadline quality. The 2 ms trajectory sample period is a data contract, not measured physical timing.
8. **Preserve reserved SRAM4 and allocation rules.** Stage 2 maps 4 KiB IPC at `0x38000000`, 16 KiB trajectory at `0x38001000`, and 12 KiB reserved at `0x38005000` inside the M7 non-cacheable 32 KiB region. `GpValidationShared` is **not yet placed, initialized, or linked** into a matching shared linker slot. Check ABI sizes, metadata alignment, both linker scripts and the MPU window before enabling the flash validation bridge; never place kernel task objects or local producer descriptors in shared SRAM.

## Existing module disposition

| Component | Evidence / integration disposition |
| --- | --- |
| `StateMachine/state_machine.{c,h}` and `States/` | **Reuse:** core logic in C, no required Linux runtime for the pure state machine; source-level ARM compile and dependency closure remain to be checked. |
| `StateMachine/supervisor_task.{c,h}` | **Adapt / reuse:** genuine FreeRTOS supervisor with single CANopen ownership, bounded queue polling and IPC hooks; needs target composition, correct timestamp source, task/memory budgets and safe hardware I/O. |
| `ControlCore/` numerical modules | **Reuse subject to ARM compile, math-library linkage and stack/WCET budgets.** IK/validation runs below motion task priority; no long calculation in the 2 ms action path. |
| `CANComm/CANopen/`, `ServoDrive/` | **Evaluate before replacement:** CANopen CiA 301/402 behavior exists, including NMT, SDO, heartbeat, PDO and SYNC. Compare against CANopenNode's existing NMT/SDO client/PDO/SYNC/heartbeat modules before choosing a long-term protocol implementation. Use the AVR/AVATAR motor manual and actual EDS/object dictionary for device-specific definitions, not assumptions. |
| `CANComm/can_backend.h` | **Reuse:** inject target STM32CubeH7 HAL FDCAN implementation (bounded receive and send, FIFO, bus-off, IRQ priority, timeout/error propagation). |
| `CANComm/SILKit/`, `Simulation/supervisor_silkit_main.c` | **Retain for host tests only:** not firmware sources. |
| `Intercore/ipc_mailbox_freertos.c`, `trajectory_stream.c` | **Reuse:** shared transport already introduced at Stage 2; no production application endpoints/tasks connected yet. |
| `Intercore/trajectory_stream_path_port.c` | **Adapt:** consumer callback surface exists; composition must bind the same `PathExecutionServices.context` consistently. Readiness, artifact identity and release/ack remain mandatory. |
| `Intercore/validation_storage_bridge.c` | **Defer linking until SRAM4 allocation/ABI and storage transactions are integrated**; host tests are not evidence of target coherence or power-fail safety. |
| `Simulation/Storage/`, `Simulation/Renode/` | **Keep as test/reference implementations**; use CubeMX/HAL QSPI and official Winbond command/timing specifications for production hardware driver, then adapt the existing storage semantics. |
| `HMI/hmi_task.c`, GUI/app modules | **Adapt command/status semantics only;** direct supervisor linkage is incorrect on M4. Physical HMI transport remains unassigned. |

## Standards/reference decision

- **STM32 platform:** the checked-in `GP_STM32H745.ioc`, HAL/LL, CMSIS, FreeRTOS 10.6.2 and STM32CubeIDE project remain authoritative. [ST STM32CubeH7 FreeRTOS_AMP_Dual_RTOS](https://github.com/STMicroelectronics/STM32CubeH7/tree/master/Projects/STM32H745I-DISCO/Applications/FreeRTOS/FreeRTOS_AMP_Dual_RTOS); [ST RM0399 documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32h745-755/documentation.html).
- **Cross-core IPC:** reuse the Stage 2 adaptation of ST's FreeRTOS AMP example. Target-specific application dispatching is custom only because robot commands, statuses, fail-closed semantics and epoch identity are application data; no replacement queue framework.
- **CANopen:** [CANopenNode](https://github.com/CANopenNode/CANopenNode) includes master NMT, SDO client, PDO, SYNC and heartbeat functionality. [CanOpenSTM32](https://github.com/CANopenNode/CanOpenSTM32) provides a HAL (FD)CAN port candidate. **Decision pending:** compare actual integration cost, licensing, supported target HAL version, message ordering, timing and AVATAR object mapping with the existing tested coordinator. Do not install an overlapping second CANopen master before that decision.
- **Storage:** use the bundled ST HAL QSPI driver and STM32CubeH7 examples as applicable, and the exact purchased Winbond part datasheet. Retain application-specific commit/CRC/cancellation rules; generic HAL does not provide durable robot-trajectory transactions.

## Integration roadmap (dependencies and acceptance gates)

| Step | Minimal deliverable | Evidence required |
| --- | --- | --- |
| **3A.1** | Document audit and add a compile-only source compatibility check without changing firmware runtime. | Check source paths and run `bash Firmware/Tests/check_application_arm_compilation.sh`; report errors as audit findings, not silent patches. |
| **3A.2** | Freeze the M7/M4 resource plan and provisional task/priority/memory table after compile results; choose CANopenNode adoption vs existing-coordinator retention based on an explicit comparison. | No unknown ownership of FDCAN/QSPI, task producer/consumer, or wake source. Record unresolved PCB/pin decisions. |
| **3B** | Integrate M4 IPC service and M7 supervisor endpoints using existing buffers; use bounded zero-timeout dispatch, overflow reporting, periodic drain and correct initialization. | Real ARM compilation, dual-image CubeIDE builds, linked symbols, host fault-path tests. No motion-enabled fallback. |
| **3C-1** | Compile/link pure state machine, drive models and supervisor on M7 with a target composition root; keep motion disabled until genuine hardware feedback/safety inputs exist. | Both builds and map/ELF checks; state tests including invalid/late messages. |
| **3C-2** | Generate FDCAN and motion timer via CubeMX, integrate verified HAL backend, real monotonic timestamps and drive commissioning; establish CANopen protocol strategy. | Real builds, CANopen host regressions, logic/bus measurements where available; final motor dictionary/pinout outstanding. |
| **3C-3** | Connect M7 trajectory consumer and path execution adapter; bounded one-sample cycle, starvation/deadline/following-error handling. | Host tests, ARM image links, timestamped cycle data; cannot mark 500 Hz hardware-verified without instrumented hardware. |
| **3D-1** | Reserve/initialize `GpValidationShared` on both linker maps and verify ABI; connect validation client/worker only after the shared-memory budget check. | SRAM4 linker probes, ARM build, full ELF symbol/section audit, transaction-failure tests. |
| **3D-2** | Generate QSPI via CubeMX, connect official HAL flash driver to existing storage abstraction and M4 stream producer; implement physical HMI transport only when specified. | Independent CM4 build, flash command tests on relevant device, refill/cancel/CRC tests; physical results separately identified. |
| **3E** | Regression and dual-image integration verification. | Both actual target builds, linked app symbols, host CTest and defined failure paths; hardware-only items explicitly open. |

Order of 3C/3D substeps may change when hardware pinout or storage record dependencies become concrete. Stage 3 **must not** be declared statically verified on an isolated compile test.

## First verification command

```bash
cd ~/freertos-pc-test
git fetch origin
git switch sara/can-replacement
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/check_application_arm_compilation.sh
```

This compiles a selected source inventory for actual CM7/CM4 ARM ABIs with pinned project headers; it does **not** build a complete firmware image. Existing Stage 2 checks should still pass:

```bash
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_intercore_startup_compile.sh
bash Firmware/Tests/check_intercore_notifications.sh
```

**Not performed during this GitHub-only Stage 3A change:** WSL shell execution, ARM compilation, STM32CubeIDE image builds, ELF/map inspection, SIL Kit runtime regression, or hardware tests. Stage 3A's portability gate is pending local results.

## Open architecture blockers

- Final target FDCAN and QSPI peripheral ownership, physical transceiver/pins and external NOR model are unconfigured.
- Real microsecond monotonic clock, 2 ms scheduler coupling, exception/IRQ budgets and maximum CAN load are unmeasured.
- The M4 HMI direct-calls-M7 design must be replaced by an IPC adapter; physical HMI transport is unspecified.
- Validation shared SRAM4 allocation, canonical source linking into both CubeIDE projects and linker/MPU checks are incomplete.
- Physical safety hardware, motor drive EDS/firmware and independent-core reset recovery are unverified; software fault latches are not safety-certified.

## Change log

- **2026-10-14, Stage 3A initial record:** audited branch baseline and relevant implementation interfaces; classified reuse/adaptation decisions; added ARM compile-only source reconnaissance and firmware README pointer. No runtime behavior changed; no tests executed from GitHub.

## Stage 3B-1 — CM4 application IPC service (2026-10-14)

**Source change committed; ARM/CubeIDE validation pending.** The existing CM4 CubeMX `defaultTask` now runs `application_ipc_service_run()` through its preserved `USER CODE` section. A CM4-local module in `CM4/Core/Src/application_ipc_service.c` uses **native FreeRTOS static queues** to serialize HMI commands before the established `gp_ipc_m4_send()` transport and to cache the most recent M7 `GP_IPC_CONTROLLER_STATUS`. No second reader/writer of the shared Message Buffers is introduced; M7 source is untouched.

- Drain at most 8 M7 messages per 1 ms service iteration, tolerate coalesced EXTI wakeups through periodic polling; filter invalid type, duplicate/older sequence, and malformed boolean fields.
- Publish a local single-slot status snapshot with `xQueueOverwrite`; check age with the CM4 clock since receipt rather than unsynchronized M7 timestamps.
- Reject HMI submission when no M7 status arrived or the last status is older than 100 ms; use a zero-wait 16-entry **CM4-local** FreeRTOS queue; keep at most one shared-send retry pending. Drop locally queued commands on lost/stale M7 status. These gates are additional application logic, **not** certified robot safety.
- Canonical `supervisor_hmi.h` is staged for the CM4 CubeIDE verifier alongside Stage 2 IPC headers. No HMI physical driver is added and no application currently calls `application_ipc_service_submit_hmi()`; no remote traffic can be expected until the M7 Supervisor is actually integrated.
- The default CM4 thread has a CubeMX-generated 512-byte stack; **measure stack watermark and configure final task stack through CubeMX before hardware release**. Do not interpret compile success as runtime stack adequacy.
- **Important pending risk:** IPC sequences cannot independently detect an M7-only reset; the current monotonic check deliberately fails closed after a remote sequence restart. Explicit shared generation/epoch and reset recovery are later gates, not assumed in this change.
- Sources: ST AN5617 section 4.2, ST's `FreeRTOS_AMP_Dual_RTOS` example, FreeRTOS static queue APIs, and the checked-in FreeRTOS 10.6.2 `message_buffer.h`. Application-specific queue/status serialization is necessary; the shared transport remains standard FreeRTOS.

**Changed sources:** `Firmware/STM32H745/CM4/Core/Inc/application_ipc_service.h`, `Firmware/STM32H745/CM4/Core/Src/application_ipc_service.c`, CM4 `main.c` (only `USER CODE`), `Firmware/Tests/stage_intercore_headers.sh`, `Firmware/Tests/check_m4_application_ipc.sh`, this record and `Firmware/README.md`.

**Local gate:**

```bash
cd ~/freertos-pc-test
git fetch origin
git switch sara/can-replacement
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/check_m4_application_ipc.sh
bash Firmware/Tests/check_intercore_startup_compile.sh
bash Firmware/Tests/check_intercore_notifications.sh
```

Then sync the complete tracked firmware tree into the existing standalone STM32CubeIDE verification directory, **stage** canonical headers/sources using the Stage 2 scripts, and clean-build both CM7/CM4 images. Verify `application_ipc_service_initialize`, `application_ipc_service_run` and `gp_ipc_m4_receive` appear in the CM4 ELF before calling 3B-1 statically integrated. No such builds/tests were executed via GitHub.

**Stage 3B-2 dependency:** integrate the M7 Supervisor composition using real task initialization and explicit fail-closed I/O. Application IPC itself does not permit robot motion, and the new CM4 service must not be treated as proof of functional bidirectional messaging without the M7 task and physical hardware.

### Stage 3B-1 ARM compile warning correction (2026-10-14)

User-executed local verification on commit `870e575` failed `check_m4_application_ipc.sh` with `-Werror=type-limits` at `application_ipc_service.c:92`: comparing nonnegative enum `SupervisorHmiCommand` against the first zero-valued enumerator was provably false under the GCC ARM enum representation. Replaced the two-sided comparison with the **unsigned upper-bound** check `(uint32_t)command >= (uint32_t)SUP_HMI_COMMAND_COUNT`. This also rejects negative out-of-range enum casts. No IPC ABI or task behavior changed for legitimate commands.

The user separately ran `check_intercore_startup_compile.sh` and `check_intercore_notifications.sh`; both **passed at ARM/source-check level**. The new corrected application IPC compilation check and full dual-image STM32CubeIDE builds **have not yet been executed**; leave 3B-1 verification pending.

### Stage 3B-1 compile-only check: local user verification (2026-10-14)

The initial ARM test found and corrected an enum range warning. The next local attempt showed a missing include search path for `cmsis_os2.h` **in the standalone test script**, not a missing firmware middleware header. After adding the vendored `Drivers/CMSIS/RTOS2/Include` search path locally, the user reported:

```
PASS: CM4 task wiring and nonblocking shared transport contract
PASS: CM4 application IPC ARM compile-only gate (not a firmware link)
```

This confirms that the tested CM4 application service and generated `main.c` compile with the ARM toolchain and that the source assertions passed. The corrected include path is now committed for reproducibility. **No full CubeIDE link, application symbol/stack budget inspection, live IPC test or hardware test is evidenced.** Stage 3B-1 remains **compile-only verified; dual-image linking pending**.

### Stage 3B-1 CubeIDE ELF evidence (operator-provided, 2026-10-14)

After synchronizing/staging the source files and using STM32CubeIDE, the operator reported the following `arm-none-eabi-size` output from the Debug ELFs at `GP_Repo_Verification/CM7/Debug/GP_STM32H745_CM7.elf` and `GP_Repo_Verification/CM4/Debug/GP_STM32H745_CM4.elf`:

| Image | text | data | bss | dec |
| --- | ---: | ---: | ---: | ---: |
| CM7 | 23316 | 100 | 35476 | 58892 |
| CM4 | 24644 | 100 | 21148 | 45892 |

The operator's `arm-none-eabi-nm --defined-only` inspection of the **CM4 ELF** confirms the actual strong, linked functions:

```
08100868 T application_ipc_service_initialize
08100914 T application_ipc_service_read_status
08100a5c T application_ipc_service_run
```

**Verification assessment:** CM4 service translation unit is present in the linked ELF, not just compilable in isolation; source-level ARM checks were independently reported passing. `application_ipc_service_submit_hmi` was **not** present in this defined-symbol listing: no M4 transport currently invokes it, so link-time garbage collection is a plausible explanation, **not independently verified**. A code path exercising HMI submission must be checked once a real M4 producer is integrated. The reported ELF existence does not itself prove a fresh clean build; exact IDE console success/warning totals and ELF timestamps were not provided in the report. M7 does not yet include the supervisor application.

**Stage 3B-1 status:** CM4 application service **ARM compiled and ELF-linked**; full verified dual-image clean-build gate, runtime task stack budget, software integration/fault tests, and physical dual-core traffic are still pending. Do not claim functional bidirectional application communication or certified safety.

## Stage 3B-2 — M7 supervisor composition (2026-10-14)

**GitHub source committed; local ARM/CubeIDE verification pending.** The M7 generated default task now calls `application_supervisor_board_configure` and, only on valid configuration, `application_supervisor_start`. The latter invokes the **existing** `supervisor_task_init/start`, injecting the actual Stage 2 SRAM4 IPC pointer. No second mailbox reader, replacement RTOS, or simulated supervisor exists. It rejects missing/uninitialized CANopen transport, missing input/output adapters, bad IPC pointer, and zero period.

**Fail-closed hardware gate:** the board configuration hook is a weak implementation returning false. No physical FDCAN, motor device, emergency-stop or I/O adapter is selected yet; the production supervisor therefore **does not currently start**, publish status or send motor commands. M4 will reject HMI input when M7 status is absent. A board-provided strong hook must supply genuine dependencies, not mock health flags.

The existing CubeIDE CM7 `.cproject` now adds the Application source folder, portable app header include paths, and `GP_IPC_FREERTOS_BACKEND=1` for Debug/Release. `stage_m7_application_sources.sh` deliberately mirrors canonical StateMachine, ControlCore, CANopen, ServoDrive and the trajectory callback adapter **only inside the separate verifier**, preserving relative headers and avoiding duplicate IPC implementations. No vendor source, HAL, FreeRTOS, CubeMX `.ioc` or linker memory map was replaced.

**Tests not yet executed:** `bash Firmware/Tests/check_m7_application_integration.sh`, M4 prior check, both full CubeIDE clean builds, ARM ELF symbol inspection and resource/stack budgets. Native math symbols may require enabling `m` via standard CubeIDE linker settings once the first actual link output is known. Hardware-dependent behavior remains untested. Do not call 3B-2 application-operational until the board hook is implemented and verified.

### Stage 3B-2 staging regression correction (2026-10-14)

The operator reported **PASS** for `check_m7_application_integration.sh` and `check_m4_application_ipc.sh`, then **FAIL** at `stage_m7_application_sources.sh` during preparation of the CubeIDE verifier. The cause was a Bash `case` pattern `CANComm/*.c` that also matched nested `CANComm/CANopen/*.c`; Bash `case` wildcards can span directory separators. The test correctly identified the missing `canopen_master.c`, but its generic message hid the exact missing dependency.

The script now matches on the exact **directory** rather than a combined path, verifies each required CANopen translation unit with a named error, and excludes only the intended top-level CANComm C files and extra Intercore backend sources. The additional `check_m7_application_staging.sh` exercises the staging operation in a throwaway verifier, compares mirrored files against their canonical source, asserts forbidden duplicate units remain absent, and tests repeatability. No firmware runtime code, shared IPC ABI, or ST-generated source has changed.

**Tests after fix pending operator execution:** `bash Firmware/Tests/check_m7_application_staging.sh`, `bash Firmware/Tests/check_m7_application_integration.sh`, and full STM32CubeIDE clean builds. The earlier ARM test pass is evidence only for the previous source set; the M7 full application build remains unverified.

### Stage 3B-2 build and linked-symbol evidence (operator-provided, 2026-10-14)

After synchronizing the verifier and applying the Bash nested-CANopen staging correction, the operator reported that staging **passed** and both Cortex-M7 and Cortex-M4 STM32CubeIDE firmware builds completed. The full IDE build console/warning totals were not supplied.

The operator's subsequent `arm-none-eabi-size` and `arm-none-eabi-nm --defined-only` inspection of the actual Debug ELFs established:

| Core | text | data | bss | dec |
| --- | ---: | ---: | ---: | ---: |
| CM7 | 150728 | 112 | 91376 | 242216 |
| CM4 | 24644 | 100 | 21148 | 45892 |

```text
CM7
080198a2 W application_supervisor_board_configure
080198b8 T application_supervisor_start
08019bf2 T gp_ipc_m7_receive
08019c2a T gp_ipc_m7_send
080192b8 T supervisor_task_init
08019464 T supervisor_task_start

CM4
08100868 T application_ipc_service_initialize
08100914 T application_ipc_service_read_status
08100a5c T application_ipc_service_run
08100dea T gp_ipc_m4_receive
08100db2 T gp_ipc_m4_send
```

**Acceptance:** Stage 3B-1 and 3B-2 **pass at ARM compile, source staging and target ELF link level**. The `W` on the M7 board hook is intended: the default weak provider refuses activation. The CM7 application is **linked but not operational**, and no bidirectional messages are produced by the physical firmware without a real board configuration. Both Debug ELFs exist; this alone does not establish measured task stack margin, dual-core boot, CANopen/FDCAN operation, 500 Hz timing, safety performance, or physical IPC traffic. Do not describe Stage 3B as hardware validated.

The operator also reported an earlier terminal exit code 1 without a failing diagnostic line; a subsequent non-exiting inspection found all the expected symbols. Do not assign a cause to the earlier shell exit without its captured stderr/first failure.

## Stage 3C-2a — ST HAL1 FDCAN Classic CAN backend (2026-10-14)

**Source prepared; target build and hardware checks pending.** The CM7 `Board/Src/fdcan_classic_backend.c` adapts the existing portable `CanBackend` interface to **official STM32CubeH7 HAL1** FDCAN functions, using a generated `FDCAN_HandleTypeDef` supplied by CubeMX, not custom register programming. It supports standard 11-bit Classic CAN (DLC 0..8), nonblocking TX/RX FIFO usage, and fails on bus-off or an invalid frame. It configures the official global filter for standard data frames and rejects extended/remote frames before starting. Startup rejects incorrect frame mode, receive/transmit element sizing, FIFO mode and capacity fewer than 16 TX/16 RX entries.

**No board configuration is guessed.** FDCAN remains **disabled** in the canonical `.ioc`; ST's FDCAN HAL header and driver are absent from the current generated vendor snapshot. The new board translation unit is therefore conditionally inactive under the canonical configuration. The Cortex-M7 CubeIDE project has an additional managed `Board` source folder and include paths in Debug/Release, but does not start a CAN peripheral, configure bit timing, select pins or change the weak fail-closed Supervisor provider. This is deliberately **not** a physical FDCAN driver test and not a CANopen integration result.

**Official ST references:**
- STM32Cube_FW_H7 V1.13.0 as selected by `GP_STM32H745.ioc` (generate from the installed matching CubeMX package).
- [STMicroelectronics stm32h7xx-hal-driver HAL1 FDCAN header](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/blob/v1.11.4/Inc/stm32h7xx_hal_fdcan.h) — `HAL_FDCAN_ConfigGlobalFilter`, `HAL_FDCAN_Start/Stop`, `HAL_FDCAN_AddMessageToTxFifoQ`, `HAL_FDCAN_GetRxMessage`, `HAL_FDCAN_GetProtocolStatus`, and FIFO fill/free-level APIs.
- [ST STM32CubeH7 FDCAN_Loopback example](https://github.com/STMicroelectronics/STM32CubeH7/tree/master/Projects/STM32H7B3I-EVAL/Examples/FDCAN/FDCAN_Loopback).

**Software-only gate:**
```bash
bash Firmware/Tests/check_fdcan_board_disabled.sh
```
This compiles the **inactive branch only**, because the current project has not generated HAL FDCAN. Once CubeMX enables FDCAN, this check is expected to refuse to act as evidence for the enabled branch; a new ARM/IDE link-and-test gate must compile the **HAL-enabled** branch against the exact generated ST HAL1 package.

**Next required GUI configuration (CubeMX, before actual bus operation):** configure FDCAN1 under CM7 in Classic CAN normal mode, nominal bit timing for the intended 1 Mbps bus from the **actual peripheral kernel clock**, 8-byte TX/RX elements, at least 16 TX FIFO and 16 RX FIFO0 elements, FDCAN filters/message RAM, a physically valid AF pin pair, transceiver/termination and optional IRQ choice. Do not manually change generated driver headers or ISR ownership; regenerate with CubeMX, then review/diff generated output and build both cores in CubeIDE. The final physical node dictionary, transceiver pins and PCB mapping are outstanding. **500 Hz bus throughput and 6-RPDO4-plus-SYNC cyclic atomicity remain unverified** and must be measured before motion activation.

### Stage 3C-2a HAL1 adapter host test

The optional host-only mock gate `bash Firmware/Tests/check_fdcan_hal_adapter_mock.sh` compiles the **HAL-enabled** adapter branch with a minimal mock of the HAL1 types/functions and tests strict Classic CAN framing, receive FIFO, transmit backpressure, extended/FD rejection, bus-off and close behavior. It does not substitute for compiling against the official HAL1 driver from the regenerated STM32CubeH7 package or measuring CAN on hardware. The separate `check_fdcan_board_disabled.sh` checks the currently compiled inactive branch only.

### Stage 3C-2a local verification (operator-provided, 2026-10-14)

The operator synchronized the latest `sara/can-replacement` changes, reported both native Cortex-M7 and Cortex-M4 STM32CubeIDE firmware **builds completed**, and supplied exact local test output:

```text
PASS: FDCAN Board source registration and CubeMX-disabled guard
PASS: conditional CM7 board source compiles without unconfigured FDCAN
NOTE: HAL-enabled FDCAN branch, bus timing and physical transceiver are NOT verified.
PASS: HAL1 Classic CAN backend mock frame and failure-path tests
NOTE: mocked HAL1, not the STM32CubeMX-generated ST driver or real CAN.
PASS: M7 board gating, IPC ownership, and CubeIDE settings
PASS: CM7 application ARM compile-only check (NOT target link)
PASS: CM4 task wiring and nonblocking shared transport contract
PASS: CM4 application IPC ARM compile-only gate (not a firmware link)
PASS: staged canonical Intercore headers for CM7
PASS: staged canonical Intercore headers for CM4
PASS: staged canonical CM7 source ipc_mailbox_freertos.c
PASS: staged canonical CM7 source trajectory_stream.c
PASS: staged canonical CM4 source ipc_mailbox_freertos.c
PASS: staged canonical CM4 source trajectory_stream.c
PASS: staged canonical CM7 application C and headers
```

**Stage 3C-2a outcome:** the disabled-mode ARM compilation, HAL1 mock adapter contract, prior application ARM checks, and source staging **passed** on the operator's system; both CubeIDE builds are operator-reported successful. No updated ELF hash/timestamp, full build log, or symbol inventory was submitted. **Active FDCAN is not compiled into the current image**, because the tracked CubeMX configuration has no FDCAN1 and the vendored firmware tree contains no HAL FDCAN source/header. The weak board provider still refuses M7 supervisor startup.

**Next gate:** configure FDCAN1 from the authoritative STM32CubeMX 6.18.1 / STM32Cube FW_H7 V1.13.0 project, not by hand-writing HAL/peripheral initialization. The tracked clock tree currently reports FDCAN kernel 129 MHz, but verify this remains true after generation before selecting nominal 1 Mbit/s timing. The STM32H745IITx LQFP176 pinout selection is a project placeholder pending physical PCB decisions. Preserve Stage 2 AMP startup/MPU/IPC USER CODE and review regenerated project settings before synchronizing back to GitHub. Both active HAL adapter compilation and actual bus measurements remain pending.

### Stage 3C — GUI configuration tracking (2026-10-14)

The operator enabled **FDCAN1 on CM7**, selected **Classic CAN / Normal mode**, switched FDCAN from PLL1Q 129 MHz to PLL2Q 43 MHz by changing **DIVQ2 /2 → /6**, and entered **Nominal Prescaler 1, SJW 8, Seg1 34, Seg2 8**. CubeMX displayed **1,000,000 bit/s** and no clock errors. Full rationale, unfinalized parameters and exact GUI-vs-repo status are recorded in **[FDCAN1_CUBEMX_CONFIGURATION.md](FDCAN1_CUBEMX_CONFIGURATION.md)**. The repository `.ioc` is **not yet updated**, and no active HAL FDCAN generation/build has been reported. Do not promote this screenshot-level evidence into a hardware-enabled milestone.

### Stage 3C-2: CubeMX FDCAN generation milestone (2026-10-14)

Local saved `.ioc` committed as **`c3d07a2`** and vendor HAL FDCAN files subsequently generated in the separate CubeIDE verifier. User's dry-run checksum inventory reported HAL configuration/MSP/main and project-metadata changes; SRAM4 setup, M7 Supervisor, M4 startup and M4 IPC wiring remained detectable. This is **not** an active HAL firmware build or hardware communication pass. Full per-file generation audit and both core builds remain pending. See [FDCAN1_CUBEMX_CONFIGURATION.md](FDCAN1_CUBEMX_CONFIGURATION.md).
