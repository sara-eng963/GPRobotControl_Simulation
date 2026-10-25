> Historical engineering record: the ZIT6 integration supersedes these build/staging instructions. See [current firmware instructions](README.md). The original evidence below is retained.

---
title: "Stage 2 — Shared SRAM4, Linker, MPU and IPC ABI"
stage: 2
status: "2A/2B pass; 2C link-and-clean-build pass; 2D CubeMX MPU generated and builds pass; 2E pending"
date: 2026-10-12
tags: [stm32h745, shared-memory, freertos, firmware]
---

# Stage 2 — Shared SRAM4 / MPU integration

**Current substage: 2E, shared-memory initialization and startup sequencing.**
Stages 2A–2D established the object sizes, address allocation, linker
sections and generated M7 MPU settings. No production shared objects have
yet been linked into both firmware images or exercised across real cores.

> [!IMPORTANT]
> Size and synthetic linker tests, and independent CubeIDE firmware builds,
> are **software-only evidence**. A linkable MPU configuration is now present,
> but production IPC/trajectory objects are not yet in those firmware ELFs,
> and no hardware cache-coherency or two-kernel FreeRTOS IPC test has occurred.

## 1. Established memory map

These are **current generated linker script** declarations, not new decisions.

| Address range | Region | Current CM7 linker | Current CM4 linker |
| --- | --- | --- | --- |
| `0x38000000–0x3800FFFF` | SRAM4, 64 KiB | Partitioned into shared IPC/trajectory/reserved (lower 32 KiB) and `RAM_D3` (upper 32 KiB) | Matching lower shared partitions; `GP_SRAM4_REMAINING` upper 32 KiB |
| `0x24000000` | M7 AXI SRAM, 512 KiB | `RAM_D1`, default data/BSS/stack | Not used as default |
| `0x10000000` | Generated M4 RAM, 288 KiB | Not used as default | `RAM`, default data/BSS/stack |
| `0x08000000` | CM7 program Flash bank, 1024 KiB | CM7 `FLASH` | — |
| `0x08100000` | CM4 program Flash bank, 1024 KiB | — | CM4 `FLASH` |

The shared range's end address is inclusive. Its end-exclusive value is
`0x38010000`. The two independent linker scripts must agree on an **absolute
physical address**, and neither CPU's ordinary stacks/heaps may overlap it.

## 2. Shared vs private ownership

**Shared in SRAM4:**
- `GpTrajectoryShared`: M4 produces 512 × six-axis (24-byte) samples; M7
  consumes; includes generation, identity and release handshake fields.
- `GpIpcShared`: two 16-message FreeRTOS IPC channels in opposite directions
  (target-backend layout only; not the host SPSC layout).

**Never store in shared SRAM4:**
- `GpTrajectoryProducer` and its 256-sample staging buffer and callback.
- `GpTrajectoryConsumer` descriptor, task handles, task stacks, OS heaps,
  core-local mutexes, pointers to arbitrary private objects.
- HAL peripheral state or local supervisor algorithms absent an explicit
  sharing design.

**Why:** the shared physical region is limited, and carrying private function
pointers/task handles across independently executing FreeRTOS kernels would
create invalid ownership and synchronization assumptions.

## 3. Exact layout audit

Run from the repository:

```bash
bash Firmware/Tests/check_shared_layout.sh
```

This script **uses the actual committed STM32CubeMX FreeRTOS 10.6.2 kernel
headers** and each core's **actual generated** `FreeRTOSConfig.h`. It builds
a small compile-only ARM object for each CPU, interrogates the symbol sizes
using `arm-none-eabi-readelf` and fails if:

1. Either ARM compilation fails or a required SDK directory is missing.
2. The 60-byte IPC payload, 24-byte trajectory sample, 512-entry ring, or
   32-bit ARM ABI changes.
3. **Any exported type footprint, alignment or important field offset** differs
   between the CM7 and CM4 builds.
4. The two principal shared objects alone exceed the 64 KiB gross SRAM4 capacity.

The probe reports actual object sizes, sample/storage offsets and alignment
requirements. That is **not yet a finalized layout**: pad to suitable
alignment, include shared init/version metadata, and reserve any future
shared blocks only after a full allocation review.

**FreeRTOS 10.6.2 compatibility gate (first Stage 2E code change):**
The vendor's `message_buffer.h` includes `stream_buffer.h` directly and
does **not** define or require `configUSE_STREAM_BUFFERS`. Our own
nonstandard check was removed from `Intercore/ipc_mailbox.h`; both ARM
probes now compile against the actual generated core configs **without
defining an artificial switch**. The firmware must still compile/link
the vendor's `stream_buffer.c` and verify its real two-core behavior.
Re-run both source-level ARM scripts after fetching this change.

## 3A. Stage 2A measured result and Stage 2B memory layout

**Measured using the real firmware FreeRTOS 10.6.2 headers on both ARM cores:**

| Object | Measured footprint | Alignment |
| --- | ---: | ---: |
| `GpIpcShared` | 2,208 bytes | 32 bytes |
| `GpTrajectoryShared` | 12,352 bytes | 32 bytes |
| Combined | **14,560 bytes** | — |

Both generated core configurations produced identical sizes and field offsets.
The M4-private flash staging/producer descriptor (6,172 bytes) must **not**
be placed in SRAM4; nor may RTOS stacks or private task handles be shared.

**Proposed fixed physical allocation, not yet enabled in linkers or MPU:**

| Inclusive SRAM4 address range | Slot size | Design purpose |
| --- | ---: | --- |
| `0x38000000–0x38000FFF` | 4 KiB | IPC buffers and metadata (2,208 B used) |
| `0x38001000–0x38004FFF` | 16 KiB | Trajectory ring (12,352 B used) |
| `0x38005000–0x38007FFF` | 12 KiB | Reserved for future shared metadata only |
| `0x38008000–0x3800FFFF` | 32 KiB | Unallocated upper SRAM4; not inside shared MPU window |

**Why these sizes?** The lower **32 KiB** is a power-of-two, 32 KiB-aligned
region suitable for one future Cortex-M7 MPU region. The two fixed slots
provide growth margin and prevent one message-buffer change from silently
moving the trajectory ring. Both bases are 32-byte aligned. The upper
32 KiB remains uncommitted to avoid unnecessary restrictions on later
hardware and DMA requirements.

The remaining 50,976 bytes reported by the Stage 2A gross-capacity test
should **not** be interpreted as one contiguous free region. Some bytes
are inside fixed slots or are deliberately reserved.

The single C-side address contract is
[`Common/Inc/sram4_memory_map.h`](STM32H745/Common/Inc/sram4_memory_map.h).
The stage 2 compile probe asserts both actual object sizes fit their
respective slots and checks adjacency and MPU alignment.

**Status:** physical reservations are now encoded in both linker scripts
and the corresponding M7 MPU Region 1 configuration is generated. The
production objects are not yet linked, nor is the startup/ready protocol
implemented. A passing size test is not a cache-coherency or inter-kernel
runtime validation.


## 3B. Stage 2C — shared SRAM4 linker sections (verified in software)

Both Cortex-M7 and Cortex-M4 linker scripts now declare **nonoverlapping
SRAM4 partitions**:

| Region name | Start address | Length | Role |
| --- | --- | --- | --- |
| `GP_SRAM4_IPC` | `0x38000000` | 4 KiB | Message Buffer descriptor and data storage |
| `GP_SRAM4_TRAJECTORY` | `0x38001000` | 16 KiB | Trajectory ring |
| `GP_SRAM4_RESERVED` | `0x38005000` | 12 KiB | Deliberately unused future shared slot |
| CM7 `RAM_D3` / CM4 `GP_SRAM4_REMAINING` | `0x38008000` | 32 KiB | Upper SRAM4, not yet assigned |

These partitions replace CM7's previous overlapping 64 KiB `RAM_D3`
declaration; CM4 previously had **no SRAM4 region**. The changes apply to
**both `*_FLASH.ld` and `*_RAM.ld` for both cores**, preventing different
build modes from assigning incompatible physical memory.

The two named output sections are:

- `.gp_shared_ipc (NOLOAD)` → `GP_SRAM4_IPC`
- `.gp_shared_trajectory (NOLOAD)` → `GP_SRAM4_TRAJECTORY`

Each uses 32-byte alignment, `KEEP` input-section retention, explicit
start/end symbols and GNU linker `ASSERT` checks for the exact slot base
and capacity. **`NOLOAD` means these regions are not initialized by the
firmware's normal copied-data/zeroed-BSS startup.** That is deliberate:
the eventual M7-only initialization requires a *separate, coordinated
ready handshake* with the M4; neither kernel may independently clear
the live shared objects.

### Verification

Run the Stage 2A/B audit, then **the new Stage 2C ARM link test**:

```bash
bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
```

The latter compiles a *synthetic test-only* instance of each actual shared
struct with the committed FreeRTOS 10.6.2 headers. It links that probe
independently with all four generated linker scripts and examines the
resulting ELF symbol positions and `NOBITS` section types with
`arm-none-eabi-nm` and `arm-none-eabi-readelf`. The expected physical
addresses are `0x38000000` (2,208-byte IPC) and `0x38001000`
(12,352-byte trajectory).

**Recorded results (2026-10-12):** all four combinations (CM7/CM4 ×
FLASH/RAM linker) passed synthetic ARM linking, symbol-address checks,
object-size checks and `NOLOAD`/`NOBITS` checks. The repository-derived
verification workspace then produced successful clean firmware builds for
both cores (operator-reported 0 errors / 0 warnings). These firmware ELFs
still lack the production shared objects and do not prove runtime IPC.

### Regeneration and safety

These sections are maintained **in generated linker scripts** because
CubeMX does not currently own this application-specific allocation.
A future CubeMX regeneration can overwrite them. Re-run the Stage 2C
script after every regeneration and review the linker diff. Do not place
these shared structures in ordinary `.bss`, duplicate-initialize them
from both cores, or use startup HSEM ID 0 for application notifications.

**Deferred:** production section-annotated objects, their single-owner
initialization/ready/version protocol, IPC HSEM ISR plumbing, application
integration and hardware execution/cache-coherency validation.



## 3C. Stage 2D — Cortex-M7 MPU configuration (source-verified)

**Commit:** `184147f` — `feat(firmware): configure M7 MPU for non-cacheable shared SRAM4`.

Using STM32CubeMX 6.18.1, the operator enabled **MPU Region 1** for the
lower 32 KiB of shared SRAM4. CubeMX regenerated the MCU `.ioc` and
`CM7/Core/Src/main.c`; both were copied from the verification workspace
into the tracked firmware with the original CRLF line endings preserved.
The reviewed Git diff was limited to **11 added lines in M7 `main.c`**
and the **8 additions / 1 replacement** required by the `.ioc` file.
The unrelated CubeMX-generated `rtos.0.ip` change was excluded.

| M7 MPU attribute | Generated value | Why |
| --- | --- | --- |
| Region | `MPU_REGION_NUMBER1` | Higher-priority override of existing Region 0 |
| Base address | `0x38000000` | Beginning of our shared SRAM4 reservation |
| Size | `MPU_REGION_SIZE_32KB` | One power-of-two region covers the lower 32 KiB |
| Subregions disabled | `0x00` | Include all eight subregions |
| TEX | `MPU_TEX_LEVEL1` | Normal memory when combined with C=0, B=0 |
| Access | `MPU_REGION_FULL_ACCESS` | Access from M7 firmware |
| Execute never | `MPU_INSTRUCTION_ACCESS_DISABLE` | Shared data is not executable |
| Shareable | `MPU_ACCESS_SHAREABLE` | Shared-memory attributes |
| Cacheable | `MPU_ACCESS_NOT_CACHEABLE` | Prevent stale M7 data-cache reads |
| Bufferable | `MPU_ACCESS_NOT_BUFFERABLE` | Chosen normal/noncacheable encoding |

**CubeMX code-generation detail:** Region 1 writes only its changed
`MPU_InitStruct` fields before calling `HAL_MPU_ConfigRegion()`.
The last four memory attributes listed above are set for Region 0 and
retained in the same struct when Region 1 is configured. The generated
`MPU_Config()` enables MPU operation after both region registrations.
There is no hand-written MPU register code.

**Verification:** CubeMX-generated source reviewed; both CM7 and CM4
firmware builds reported successful (0 errors, 0 warnings). Prior to
generation, the Stage 2C linker sections remained present in the
verification copy. Only the regenerated `.ioc` and M7 `main.c` were
transferred into Git, keeping the committed linker changes untouched.

**Limits:** both M7 I-Cache and D-Cache were **disabled** in the baseline
CubeMX setup. The explicit MPU mapping prepares for later cache use;
it does not itself demonstrate caching performance or cross-core data
visibility. No actual STM32H745 was flashed or tested.

## 3D. Stage 2E — reference implementation adoption

### Official ST baseline — not a new custom transport

- **[ST AN5617: Introduction to inter-processor communications for
  STM32H745/755 and STM32H747/757](https://www.st.com/resource/en/application_note/an5617-introduction-to-interprocessor-communications-for-stm32h745755-and-stm32h747757-mcus-stmicroelectronics.pdf)**:
  dual-core IPC mechanisms and FreeRTOS Message Buffer reference (section
  4.2), HSEM startup/notification discussion (section 4.3).
- **[ST STM32CubeH7 / STM32H745I-DISCO /
  FreeRTOS_AMP_Dual_RTOS](https://github.com/STMicroelectronics/STM32CubeH7/tree/master/Projects/STM32H745I-DISCO/Applications/FreeRTOS/FreeRTOS_AMP_Dual_RTOS)**:
  vendor C implementation of *separate FreeRTOS kernels* with statically
  created Message Buffers in shared SRAM4 and inter-core notification.
- **[ST AN5361](https://www.st.com/resource/en/application_note/an5361-getting-started-with-projects-based-on-dualcore-stm32h7-microcontrollers-in-stm32cubeide-stmicroelectronics.pdf)**:
  generated dual-core boot/build workflow.
- **[ST AN4838](https://www.st.com/resource/en/application_note/an4838-managing-memory-protection-unit-mpu-in-stm32-mcus-stmicroelectronics.pdf)**:
  MCU MPU regions and memory attributes.

**ST's actual AMP example (verified against upstream sources):**

1. M4 enables HSEM-0 wake notification, enters D2 STOP and waits.
2. M7 configures MPU/cache, initializes the clock and HAL, sets up EXTI0
   software interrupt routing, then calls `xMessageBufferCreateStatic()`
   for shared control/data buffers **before releasing M4**.
3. M7 releases HSEM ID 0; M4 wakes and validates shared buffer handles.
4. M7's `sbSEND_COMPLETED()` FreeRTOS hook sends a notification through a
   control Message Buffer and `HAL_EXTI_GenerateSWInterrupt(EXTI_LINE0)`.
   M4's EXTI handler calls `xMessageBufferSendCompletedFromISR()` to
   wake a local receiving task. This is **EXTI-based data notification**;
   HSEM-0 is a distinct **boot/wakeup** mechanism.
5. ST's demonstration primarily transmits **M7→M4**, not our full
   bidirectional HMI/status/storage and real-time trajectory traffic.
   We must qualify bidirectional adaptations and IRQ priorities rather
   than imply the example implements our robot-specific data flow.

### Required implementation order

1. **Use vendor primitives:** FreeRTOS 10.6.2's native
   `xMessageBufferCreateStatic`, `xMessageBufferSend/Receive`,
   `sbSEND_COMPLETED`, ISR completion API, CubeH7 HAL HSEM/EXTI and
   CubeMX-generated boot. Do not write a second queue, message framing
   layer, peripheral HAL or custom FreeRTOS scheduler.
2. **Actual physical allocation:** define one M7-owned, section-annotated
   `GpIpcShared` at `0x38000000` and `GpTrajectoryShared` at
   `0x38001000`, using existing linker sections and exact type sizes.
   M4 uses the same physical addresses and **does not reinitialize**.
   Verify symbol maps in BOTH firmware ELFs; do not silently add shared
   blocks in ordinary `.bss`.
3. **Initialize-before-release:** follow ST's AMP example: initialize
   both shared resources using the actual `gp_ipc_initialize()` and
   `gp_traj_shared_initialize()` before M7 releases HSEM-0. Preserve
   generated clock/MPU/ST boot prerequisites and fail closed if setup
   fails. M4 validates the native Message Buffer handles and trajectory
   structure after waking but before starting IPC work.
4. **Notifications:** port the ST EXTI/software-interrupt completion
   pattern, first in a small one-directional test, and only then adapt
   for the second direction. Our existing custom HSEM notifier callback
   is **not** identical to ST's published EXTI example: do not present it
   as vendor-derived until replaced or explicitly justified.
5. **Kernel compatibility:** current firmware ships FreeRTOS 10.6.2
   with static allocation enabled and `stream_buffer.c` linked.
   The nonstandard `configUSE_STREAM_BUFFERS` requirement is now
   removed from the target IPC header and from the ARM probes.
   Re-run both firmware ABI/linker tests before integrating sources.
6. **Application-only requirements:** fixed message IDs and payloads,
   trajectory producer/consumer ownership, motion gating, timeouts and
   reset/recovery are particular to this robot, **not standardized by
   ST's demonstration**. Define the minimum needed requirements,
   documenting exactly where the vendor pattern does not cover them.
   Do **not** introduce a custom bootstrap magic/epoch state machine
   merely because the earlier design note suggested it.
7. **Acceptance:** independent full CM7/CM4 CubeIDE clean builds;
   map-file proof that shared objects occupy the reserved SRAM4
   addresses; startup/notification fault-injection tests; eventual
   hardware interrupt, cache visibility, power-up and partial-reset
   measurements. A build or simulator does not prove dual-core runtime
   correctness or functional safety.

**Production naming:** the new firmware storage source is
`CM7/Core/Src/intercore_shared_memory.c`, with symbols
`intercore_ipc_shared` and `intercore_trajectory_shared`. The layout
header is `Common/Inc/sram4_memory_map.h`. The existing `.gp_shared_*`
linker-section names and historical `Gp*` / `GP_*` application ABI are
preserved temporarily; renaming those requires one coordinated,
ABI-validated cross-core migration, not a cosmetic text substitution.
CubeMX's `GP_STM32H745.ioc` identity is also separate from these
application-file changes.

**Current implementation state (2026-10-12):**
- **Stage 2E-1 — verified:** `CM7/Core/Src/intercore_shared_memory.c`
  defines the physical objects in existing `NOLOAD` linker sections.
  A clean CM7 CubeIDE build reported 0 errors / 0 warnings. The
  user inspected its actual ELF: `intercore_ipc_shared` at
  `0x38000000` (0x8a0 B) and `intercore_trajectory_shared` at
  `0x38001000` (0x3040 B), both aligned and `NOBITS`.
- **Stage 2E-2 — verified at CM7 build level:** M7
  `intercore_shared_memory_initialize()` calls the existing native
  FreeRTOS `gp_ipc_initialize()` and trajectory initializer before
  releasing HSEM-0. The user ran
  `check_intercore_startup_compile.sh` successfully, then completed
  a full CM7 CubeIDE build with 0 errors and 0 warnings. The resulting
  ELF contains `intercore_shared_memory_initialize`,
  `gp_ipc_initialize`, `gp_traj_shared_initialize` and
  `xStreamBufferGenericCreateStatic`. Final IPC and trajectory
  objects remain at `0x38000000` / `0x38001000` with `NOBITS`.
- **Stage 2E-3 — verified at CM4 build level:** The user ran the M7/M4
  ARM compile and startup-order tests and rebuilt the full CM4 CubeIDE
  project with 0 errors and 0 warnings. The resulting CM4 ELF contains
  `intercore_startup_validate`, `gp_ipc_valid` and
  `gp_traj_shared_valid`, and defines **no** M7-owned shared objects.
  Its empty `.gp_shared_*` sections have 0-byte sizes and are not
  memory allocations. The M4 checks after HSEM-0 wake and before the
  scheduler are therefore verified **at build level**.
- **Stage 2E-4 — verified at ARM and CubeIDE build level
  (2026-10-12):** The user ran `check_intercore_notifications.sh`
  successfully for CM7 and CM4 against the vendored HAL/FreeRTOS
  headers (including `-Werror`). It checked the device-defined
  `DUAL_CORE` macro, both IRQ translation units, per-core EXTI
  routing and generated startup order. After syncing
  `Common/Src/intercore_notifications.c`, both `.cproject`
  settings, both core `main.c` and `stm32h7xx_it.c` sources,
  the user independently built **CM4 and CM7 in STM32CubeIDE**:
  CM4 reported 0 errors, 0 warnings (5.389 s; text 18012 B);
  CM7 reported 0 errors, 0 warnings (3.151 s; text 23316 B).
  These are operator-provided IDE build results, not physical execution.
- **Stage 2E-4 — final static ELF symbol inspection PASS (2026-10-12):**
  The user inspected both CM7/CM4 Debug ELFs with
  `arm-none-eabi-nm --defined-only`. Both contain strong (`T`)
  definitions of `intercore_notifications_initialize`,
  `intercore_notifications_handle_irq`, and the vendor FreeRTOS
  `xStreamBufferSendCompletedFromISR`.
  CM7: `EXTI1_IRQHandler` is strong (`T`) at `0x08000B2C`;
  `EXTI0_IRQHandler` is a weak (`W`) default.
  CM4: `EXTI0_IRQHandler` is strong (`T`) at `0x081007A8`;
  `EXTI1_IRQHandler` is a weak (`W`) default.
  These opposite-core weak handlers are expected startup defaults,
  not active receiver implementations.
- **Still pending:** physical two-core interrupt delivery and latency;
  application scheduling and message draining; partial-core reset
  recovery and machine safety validation.

**Important:** ST's example covers cold-boot sequencing and one-way
Message Buffer demonstration. It does not establish safe partial-core
reset recovery, bidirectional ISR completion, or robot functional safety.

**Stage 2E-1 verification commands:**

```bash
bash Firmware/Tests/check_production_shared_objects.sh
```

The new test cross-compiles the actual production source and checks its
symbol addresses and NOBITS sections using the existing M7 Flash/RAM
linker scripts. The actual M7 CubeIDE ELF passed these same addresses
in the user's verification workspace.

**Stage 2E-2 ARM source and startup-order test:**

```bash
bash Firmware/Tests/check_intercore_startup_compile.sh
```

This compiles the M7 allocation/init module and the canonical
`Intercore/ipc_mailbox_freertos.c` and `Intercore/trajectory_stream.c`
against the vendored FreeRTOS 10.6.2 and asserts that M7 initialization
is called in the CubeMX boot user-code section before HSEM-0 release.
It does not replace the real CubeIDE build or a two-core runtime test.

**Separate CubeIDE verifier:** the canonical
`Intercore/ipc_mailbox.h` and `Intercore/trajectory_stream.h` live
outside `Firmware/STM32H745`. Before compiling the new M7 source in
the standalone Windows verifier, stage exact copies using:

```bash
bash Firmware/Tests/stage_intercore_headers.sh "<verification-project-root>"
```

For Stage 2E-2 and 2E-3, stage the canonical sources for both cores:

```bash
bash Firmware/Tests/stage_intercore_sources.sh "<verification-project-root>"
```

This copies exactly `ipc_mailbox_freertos.c` and
`trajectory_stream.c` into **each core** of the separate CubeIDE
verification workspace without modifying canonical `Intercore/` code. The FreeRTOS
backend compilation unit now selects its own compile-time backend mode;
the existing `ipc_mailbox.c` host-only implementation must **not** be
compiled into CM7. Neither staging script copies HAL startup or edits
ST-generated RTOS/driver files. For Stage 2E-3, copy the tracked M4 files
`CM4/Core/Src/main.c`, `CM4/Core/Src/intercore_startup_validation.c`
and `CM4/Core/Inc/intercore_startup_validation.h` into the verifier
before staging. Run `check_intercore_startup_compile.sh`, then build
M4 in CubeIDE and inspect its ELF. The M4 ELF should have its validator
and native ABI readers but **must not** define the M7-owned
`intercore_ipc_shared` or `intercore_trajectory_shared` symbols.
Do not describe Stage 2E as complete.


## 3E. Stage 2E-4 — native ST EXTI Message Buffer notifications

**Official implementation baseline:** ST's STM32H745I-DISCO
`FreeRTOS_AMP_Dual_RTOS` example (linked in 3D) uses EXTI software
interrupts to signal another core and FreeRTOS's
`xMessageBufferSendCompletedFromISR()` to notify the receiving
kernel. We adapted this **notification mechanism** to the ST HAL APIs
actually bundled with our pinned STM32CubeH7 V1.13.0:
`HAL_EXTI_SetConfigLine`, `HAL_EXTI_GenerateSWI`,
`HAL_EXTI_RegisterCallback` and `HAL_EXTI_IRQHandler`.
This is **not a copy** of the example's additional control Message Buffer
and `sbSEND_COMPLETED` hook.

| Flow | Software EXTI line | Recipient IRQ | Priority |
| --- | --- | --- | --- |
| M7 → M4 IPC message | EXTI0, CPU2 interrupt | CM4 `EXTI0_IRQHandler` | 6 |
| M4 → M7 IPC message | EXTI1, CPU1 interrupt | CM7 `EXTI1_IRQHandler` | 6 |

- M7 configures **both** EXTI routes and the SYSCFG clock before releasing
  M4 through ST HSEM-0. M4 validates the M7-created IPC buffer before
  enabling its own receiving IRQ. Each core registers the backend's
  `gp_ipc_set_notifier()` only after successful configuration.
- A successful **nonblocking** send in
  `Intercore/ipc_mailbox_freertos.c` triggers
  `HAL_EXTI_GenerateSWI()` for the opposite core; unsuccessful sends
  and receipt/space notifications do not generate a peer-data interrupt.
- The recipient's HAL EXTI callback clears its own pending flag and calls
  native `xMessageBufferSendCompletedFromISR()` **only when**
  `xTaskGetSchedulerState()==taskSCHEDULER_RUNNING`. This handles the
  possibility of M4 sending before M7 has started its own FreeRTOS
  scheduler; the buffered message remains available to drain later.
  Interrupt priority 6 is no more urgent than the generated FreeRTOS
  `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`.
- **Dual-core device macro:** ST's HAL EXTI source uses
  `#if defined(DUAL_CORE)` to select the M4-specific `C2IMR` /
  `C2PR` registers. STM32H745's CMSIS header already provides
  `#define DUAL_CORE`. Do **not** also pass `-DDUAL_CORE` through
  either CubeIDE `.cproject` or ARM test compiler arguments:
  redefinition causes a warning that fails under `-Werror`.
  No vendor HAL/CMSIS files are modified. The test checks both
  this automatic macro and absence of the manual flag.
- **Reservation:** EXTI0 and EXTI1 are allocated to IPC and cannot also
  serve independent GPIO interrupt sources. Confirm actual board pin /
  EXTI assignments before final PCB/peripheral configuration. ST's
  HSEM-0 boot channel remains separate. The earlier HSEM 24–27 mock
  adapter is NOT linked to either production firmware image.
- **Kernel limitation:** the target IPC wrapper always uses 0-timeout
  FreeRTOS Message Buffer send and receive. This prevents a sender from
  using a task waiter belonging to the other kernel. The ISR-side
  completion API is included following ST's pattern, but there are no
  blocking shared-message receivers or registered IPC service-task
  waiters yet; **the interrupt does not currently imply a task was
  awakened**, and app-level wake latency has not been measured. Do not
  change the cross-core IPC to blocking operations without explicitly
  replacing the default completion hook with a validated AMP scheme.
- The notification signal is a wakeup hint and may coalesce; consumers
  must drain all pending messages and handle ring-full conditions. This
  channel is not an E-stop, fail-safe, robot motion authorization or
  guaranteed-delivery service.

**Source and ARM verification (passed on 2026-10-12):**

```bash
bash Firmware/Tests/check_intercore_notifications.sh
```

This cross-compiles the shared `intercore_notifications.c` adapter and
each generated core IRQ file against the **real** STM32 HAL/FreeRTOS
headers, checks ST's device-defined `DUAL_CORE` macro and the
absence of duplicate project flags, and checks startup ordering.
Both CM7 and CM4 builds subsequently succeeded with 0 errors /
0 warnings. The test does not verify electrical EXTI delivery.

**CubeIDE include-path integration:** both core `.cproject` files
now include `../../Common/Inc` for the linked `Common` folder, in
Debug and Release tool settings. This lets `main.c`,
`stm32h7xx_it.c` and `Common/Src/intercore_notifications.c`
include `intercore_notifications.h` and `sram4_memory_map.h`
without duplicating the canonical headers in `Core/Inc`. The Stage
2E-4 test enforces this configuration.

**CubeIDE synchronization gate:** with the IDE closed, synchronize
`Firmware/STM32H745/Common/Inc/intercore_notifications.h`,
`Common/Src/intercore_notifications.c`, both core `main.c` and
`stm32h7xx_it.c`, and **both** core `.cproject` files into the
Windows verification project. Restart CubeIDE, refresh each project,
clean/build both, then inspect ELF symbols. Re-stage canonical
`Intercore` headers/sources using the existing scripts if changed.

**Stage 2E-4 acceptance status: all static software gates PASS.**
Both ARM compilation/startup checks, both CubeIDE builds (0 errors /
0 warnings), and both final ELF symbol/IRQ ownership inspections
passed. **Stage 2E-4 is closed at static software-verification level**.
Physical bidirectional interrupt delivery, application-task response,
coalescing and reset recovery still require hardware testing.
A correct ELF does not prove end-to-end communication or safety.

**Reproduce final ELF symbol inspection** (already executed by
the user):

```bash
DEST="/mnt/d/University/Graduation Project/STM32/GP_Repo_Verification"
for core in CM7 CM4; do
    ELF="$DEST/$core/Debug/GP_STM32H745_${core}.elf"
    echo "=== $core ==="
    arm-none-eabi-nm --defined-only "$ELF" |
        grep -E 'intercore_notifications_initialize$|intercore_notifications_handle_irq$|EXTI[01]_IRQHandler$|xStreamBufferSendCompletedFromISR

- **2B — memory contract:** fixed address and alignment contract verified
  using core-specific FreeRTOS 10.6.2 headers. Bootstrap metadata remains.
- **2C — linker sections:** synthetic ARM ELF tests passed for all four
  linker scripts; independent CubeIDE clean builds passed for CM7 and CM4.
- **2D — M7 MPU attributes:** generated code and `.ioc` committed; independent
  CubeIDE builds passed. Hardware MPU behavior remains untested.
- **2E — vendor-derived initialization and notifications:** adopt
  ST's STM32H745 dual-FreeRTOS AMP sequence: M7 creates shared Message
  Buffers before HSEM-0 releases M4, and EXTI0/FreeRTOS ISR completion
  delivers inter-core notifications. Validate application-specific reset,
  ABI and trajectory-ownership requirements independently.

**No device-level claims** until both new firmware images execute on actual
hardware and pass visibility, reset and timing checks.

## 5. Source references

- [Verified Stage 1 report](STAGE1_DUAL_CORE_FOUNDATION.md)
- [Trajectory type definitions](../Intercore/trajectory_stream.h)
- [IPC target/host type definitions](../Intercore/ipc_mailbox.h)
- [IPC integration contract](../Intercore/README.md)
- [CM7 generated Flash linker](STM32H745/CM7/STM32H745IITX_FLASH.ld)
- [CM4 generated Flash linker](STM32H745/CM4/STM32H745IITX_FLASH.ld)

done
```

Symbol class `T` indicates the strong receiving handler; `W`
indicates the unused weak vector default on the other core.
This is a static link check, not execution evidence.

## 4. Follow-up implementation checkpoints

- **2B — memory contract:** fixed address and alignment contract verified
  using core-specific FreeRTOS 10.6.2 headers. Bootstrap metadata remains.
- **2C — linker sections:** synthetic ARM ELF tests passed for all four
  linker scripts; independent CubeIDE clean builds passed for CM7 and CM4.
- **2D — M7 MPU attributes:** generated code and `.ioc` committed; independent
  CubeIDE builds passed. Hardware MPU behavior remains untested.
- **2E — vendor-derived initialization and notifications:** adopt
  ST's STM32H745 dual-FreeRTOS AMP sequence: M7 creates shared Message
  Buffers before HSEM-0 releases M4, and EXTI0/FreeRTOS ISR completion
  delivers inter-core notifications. Validate application-specific reset,
  ABI and trajectory-ownership requirements independently.

**No device-level claims** until both new firmware images execute on actual
hardware and pass visibility, reset and timing checks.

## 5. Source references

- [Verified Stage 1 report](STAGE1_DUAL_CORE_FOUNDATION.md)
- [Trajectory type definitions](../Intercore/trajectory_stream.h)
- [IPC target/host type definitions](../Intercore/ipc_mailbox.h)
- [IPC integration contract](../Intercore/README.md)
- [CM7 generated Flash linker](STM32H745/CM7/STM32H745IITX_FLASH.ld)
- [CM4 generated Flash linker](STM32H745/CM4/STM32H745IITX_FLASH.ld)
