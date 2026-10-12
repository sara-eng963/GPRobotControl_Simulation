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

**Important compatibility gap:** the generated `FreeRTOSConfig.h` files do
not explicitly define `configUSE_STREAM_BUFFERS`, although the current
project IPC header requires it. The **probe only** defines this flag when
absent; no firmware source or production macro is changed. FreeRTOS 10.6.2's
stream buffer API is separate from that project guard. Resolve this gate
during real IPC integration without silently rewriting vendor code.

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
[`Common/Inc/gp_shared_sram4_layout.h`](STM32H745/Common/Inc/gp_shared_sram4_layout.h).
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

## 3D. Stage 2E — implementation plan and acceptance gates

**Goal:** make the production shared objects exist at the reserved
addresses, and define a deterministic, fault-detecting protocol so M4
cannot access the regions before they have been initialized by M7.

1. **Production placement:** define a **single physical instance** of
   `GpIpcShared` at `0x38000000` and `GpTrajectoryShared` at
   `0x38001000`, with dedicated linker section attributes on the M7
   owner side. M4 shall reference **the same absolute addresses**, not
   define a second physical object or attempt separate initialization.
   Inspect both final ELF maps and prevent accidental `.bss` ownership.
2. **Boot ordering:** generated ST HSEM **ID 0** wakes M4 **before**
   M7 reaches `osKernelInitialize()`. Therefore the HSEM-0 wakeup
   means *clocks are ready*, **not** *shared IPC is ready*. Preserve ST's
   boot choreography; add an independent, bounded application-level
   readiness gate on M4 before any IPC/trajectory access.
3. **Initialization ownership:** initialize both shared structures
   exactly once from M7, when its kernel prerequisites are satisfied.
   Use the existing `gp_ipc_initialize` and
   `gp_traj_shared_initialize` APIs; publish an explicit compatible
   ABI/ready state **only after** both return valid.
4. **Readiness metadata:** reserve a small fixed, aligned bootstrap
   record in the Stage 2B shared spare slot (not in either object's
   active payload), with magic, ABI revision, lifecycle epoch, flags
   and error indication. Define release/acquire publication and a
   response to stale or incompatible version.
5. **Failure handling:** implement bounded wait, startup-timeout and
   incompatible ABI branches. No robot motion or shared-memory use
   before both cores acknowledge readiness. Plan partial-core reset
   recovery separately; never simply clear a live `NOLOAD` block.
6. **Compatibility and tests:** pin the target IPC to both vendored
   FreeRTOS 10.6.2 configurations (including resolving the explicit
   `configUSE_STREAM_BUFFERS` guard). Test lifecycle protocol with
   deterministic host and ARM compile/link checks. Keep ST HAL/HSEM
   IRQ wiring and target-specific service-task notifications separate
   from HSEM-0 boot wakeup; reserve IPC HSEM IDs 24–27.
7. **Hardware gate:** physical reset sequences, both-cores data
   visibility, cache-enabled operation, HSEM interrupts, timing and
   recovery must eventually be verified on real silicon.

**Do not treat a linked mailbox as a validated cross-kernel transport.**
The existing target FreeRTOS Message Buffer implementation was tested
in isolation; running its control blocks across **two independent
schedulers** requires explicit verification, not merely matching sizes.
No current 500 Hz or functional-safety claim follows from these stages.


## 4. Follow-up implementation checkpoints

- **2B — memory contract:** fixed address and alignment contract verified
  using core-specific FreeRTOS 10.6.2 headers. Bootstrap metadata remains.
- **2C — linker sections:** synthetic ARM ELF tests passed for all four
  linker scripts; independent CubeIDE clean builds passed for CM7 and CM4.
- **2D — M7 MPU attributes:** generated code and `.ioc` committed; independent
  CubeIDE builds passed. Hardware MPU behavior remains untested.
- **2E — lifecycle & verification:** M7 initializes once while M4 is stopped,
  independent from ST's HSEM-0 boot sync; IPC HSEM IDs 24–27 remain reserved.
  Handle resets, loss of readiness and mismatched ABI deliberately.

**No device-level claims** until both new firmware images execute on actual
hardware and pass visibility, reset and timing checks.

## 5. Source references

- [Verified Stage 1 report](STAGE1_DUAL_CORE_FOUNDATION.md)
- [Trajectory type definitions](../Intercore/trajectory_stream.h)
- [IPC target/host type definitions](../Intercore/ipc_mailbox.h)
- [IPC integration contract](../Intercore/README.md)
- [CM7 generated Flash linker](STM32H745/CM7/STM32H745IITX_FLASH.ld)
- [CM4 generated Flash linker](STM32H745/CM4/STM32H745IITX_FLASH.ld)
