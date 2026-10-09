# Inter-core communication boundary

This module defines a **nonblocking, fixed-size M7/M4 IPC contract**. It does
not pretend that the existing single-RTOS POSIX simulator is already dual-core.

## Ownership
- M4-to-M7: only the M4 IPC service task writes; only M7 Supervisor reads.
- M7-to-M4: only M7 Supervisor writes; only the M4 IPC service task reads.
- Other M4 tasks send to their M4 IPC service through local FreeRTOS queues.
- The Supervisor has an optional GpIpcShared pointer. NULL preserves
  the current SIL Kit/single-core runtime.
- M7 consumes bounded HMI commands with monotonic sequence rejection and
  publishes a small status packet (state, motion permission, active fault,
  transitions, estop, most recently received command sequence, rejections).
- IPC receipt sequence is **not** confirmation that a command was accepted.
  Status may be dropped when the status ring is full.
- Existing taskENTER_CRITICAL() blocks are exclusively for same-scheduler
  snapshots. M4 must never call those APIs or receive their pointers.
- No pointer, task handle, callback, mutex or FreeRTOS queue goes across cores.
- Future flash operations use typed request/result messages; they are NOT yet
  connected to the current host flash worker.
- The mailbox is not a physical emergency-stop mechanism.

## Transport
- Ring depth: 16 entries per direction; 60 bytes per message.
- Two single-producer/single-consumer rings, aligned 32-bit atomic counters,
  release/acquire publication, bounded polling, no blocking/mutex/waits.
- Initialize exactly once while the other core is held waiting. Never reset a
  live mailbox. Handle full queues and stale messages at the application layer.

## STM32H745 integration gate
- Allocate GpIpcShared in SRAM visible to both CPUs via BOTH linker scripts.
- Configure Cortex-M7 MPU mapping as **non-cacheable**, and confirm physical
  alignment, memory barriers, ownership, and boot-time handshake (HSEM).
- Verify ABI version, GPIO safety paths, reset/fault/recovery behavior and
  deadlines on hardware. Atomics alone do not provide cache coherence.
- Do not put mailbox in cached AXI SRAM without correct cache maintenance.

## Tests
- ipc_mailbox_test: four native threads, full/empty/backpressure, invalid
  messages, 100000 ordered messages in each direction.
- supervisor_interface_test: optional IPC HMI command routed to Supervisor,
  replay ignored, status publication checked while in E-stop.
- supervisor_silkit_sequence: must remain passing with optional IPC unused.
- Host tests are not STM32 memory-map, MPU/cache, timing or safety tests.
