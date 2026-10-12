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

## Trajectory streaming (Issue 3: M4 flash -> M7 motion)

Files:
- `trajectory_stream.{h,c}`: NEW, separate cross-core path. Does not replace
  the legacy single-core `Simulation/Storage/trajectory_prefetch.{h,c}`.
- `trajectory_stream_path_port.{h,c}`: M7-only callbacks matching the
  `PathExecutionServices` stream signatures. A target composition root
  must connect these callbacks and consistently adapt the other services
  sharing the same `PathExecutionServices.context`.
- `Tests/test_trajectory_stream.c`: concurrency, generation, cancellation,
  flash-error, starvation, replay, and six-axis consistency checks.
- `Tests/test_trajectory_stream_path_port.c`: verifies the execution
  callback surface consumes exactly the samples produced by M4.

### Ownership and lifecycle

Only `GpTrajectoryShared` lives in shared SRAM (12,288 bytes of sample
data, plus a small header). M4 owns the `GpTrajectoryProducer` descriptor,
private 256-sample staging buffer (6,144 bytes), and flash read callback.
M7 owns the `GpTrajectoryConsumer` descriptor and 500 Hz execution.

1. M7 and M4 boot with a hardware handshake; initialize the shared region
   exactly once with both cores quiescent.
2. M4 validates a committed trajectory's identity (program ID, revision,
   artifact CRC, count, 2 ms period), then calls `gp_traj_m4_prepare`.
   **Identity equality is not a replacement for verifying flash contents
   against the committed artifact CRC.** That belongs to the M4 storage layer.
3. M4 calls `gp_traj_m4_step` as the sole producer. Each call reads at most
   256 samples from storage into private staging; it publishes complete
   six-joint samples to non-cacheable shared SRAM.
4. M7 starts only after `min(sample_count,512)` samples are published, with
   `gp_traj_m7_begin`, then consumes one indexed sample every 2 ms.
   M7 never calls a flash read or waits on a mutex/HSEM.
5. STARVED or STORAGE_ERROR is an immediate failure, not a valid position.
   The path-execution state must disable wire feed and issue its existing
   controlled-stop/fault response. This is not certified machine safety.
6. Once motion has ceased reading, M7 calls `gp_traj_m7_finish` (or
   `gp_traj_m7_discard` before starting). M4 acknowledges the epoch only
   after any in-flight read returned. M4 cannot reset/reuse the buffer
   before that acknowledgement.
7. Replaying the same validated path still requires a new epoch and M4
   re-prefill; it must not reuse previously consumed slots.

The shared ring is an SPSC protocol. `__atomic` acquire/release operations
order buffer publication on coherent / explicitly non-cacheable memory; they
are **not** a substitute for MPU configuration. Do not place the sample
ring in cached M7 SRAM without a designed cache-maintenance protocol.

### Boundaries not yet implemented

- Two STM32 firmware images, M4 QSPI driver ownership, actual shared-SRAM
  linker placement, M7 MPU non-cacheable region and M7/M4 boot via HSEM.
- Production integration with the current SIL Kit `validated_prefetch`;
  intentionally kept unchanged to protect existing regression behavior.
- Hardware timing, cache, reset/recovery, DMA consistency and bus contention.
- Full storage RPC orchestration and durable-commit migration (Issue 4).
- Demonstrating a 500 Hz hard deadline on physical STM32 hardware (Issue 5).

### Host verification

After regenerating CMake, build `intercore_trajectory_stream_test` and
`intercore_trajectory_path_port_test`. Their passing results only show
protocol correctness under the tested host thread schedules. Re-run the
original `supervisor_silkit_sequence`, `trajectory_prefetch_concurrent_test`,
`path_execution_prefetch_test`, and `flash_prefetch_500hz_test` before
replacing any real control path.
