---
title: "Stage 2 — Shared SRAM4, Linker, MPU and IPC ABI"
stage: 2
status: "In progress — 2A inventory/probe committed; measurements pending"
date: 2026-10-12
tags: [stm32h745, shared-memory, freertos, firmware]
---

# Stage 2 — Shared SRAM4 / MPU integration

**Current substage: 2A, object footprint and ABI audit.** The purpose is to
design the physical memory boundary *before* touching the CubeMX-generated
linker scripts or MPU configuration.

> [!IMPORTANT]
> The object-size check is compile-only. It does **not** place the objects into
> target SRAM4, configure the MPU, run on actual dual-core hardware, or prove
> multi-kernel FreeRTOS Message Buffer correctness.

## 1. Established memory map

These are **current generated linker script** declarations, not new decisions.

| Address range | Region | Current CM7 linker | Current CM4 linker |
| --- | --- | --- | --- |
| `0x38000000–0x3800FFFF` | Shared SRAM4, 64 KiB | Declares `RAM_D3`, 64 KiB; does not place IPC | No SRAM4 region yet |
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

## 4. Follow-up implementation checkpoints

- **2B — memory contract:** choose physical block offsets and alignment,
  reserve an SRAM4 layout and define shared initialization/version metadata.
- **2C — linker sections:** add `NOLOAD`/equivalent sections for both images;
  use explicit linker assertions to reject overflows and overlapping
  allocations, then verify in both ELF map files.
- **2D — M7 MPU attributes:** make the selected physical shared region normal,
  shareable **non-cacheable** memory with the correct region size/alignment.
  Preserve unrelated MPU guards and other memory's cache configuration.
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
