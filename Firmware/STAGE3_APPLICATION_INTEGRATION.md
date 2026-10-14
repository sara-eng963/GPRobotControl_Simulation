---
title: "Stage 3 — STM32H745 Application Integration"
stage: 3
status: "3A architecture audit documented; portability and firmware gates pending"
date: 2026-10-14
branch: "sara/can-replacement"
tags: [stm32h745, freertos, robotics, firmware]
---

# Stage 3 — STM32H745 Application Integration

**Audit baseline:** `d18bdec71abc62fa835afe588398f9aec074e649` (2026-10-12).  
**Scope:** repository/source review and an ARM compile-only reconnaissance check. **No Stage 3 application integration or hardware runtime test has passed yet.** Continue using the Stage 1 and Stage 2 records for historical build evidence.

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
