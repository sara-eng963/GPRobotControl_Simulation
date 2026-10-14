# Stage 3 Batch B1 — M7 Motion Composition

**Source integration:** `aa8628a` (2026-10-14). **Acceptance:** operator pending.

## Ownership and data flow

```mermaid
flowchart LR
    NOR["M4 persistent storage (Batch B4, absent)"] --> P["M4 producer / 256 sample refills"]
    P --> S["SRAM4 trajectory ring / 512 samples"]
    S --> C["M7 GpTrajectoryConsumer"]
    C --> X["PathExecution in Supervisor"]
    X --> J["CANopen JointDrivePort"]
    J --> F["STM32 HAL FDCAN1 (Batch A)"]
    B["Real board stop/wire/retract services (absent)"] --> X
```

The existing `PathExecution` owns sample deadlines, missed-period skip/expiry, feedback freshness and joint-drive command requests. It uses `canopen_joint_drive_port_make()` in the M7 Supervisor; no new CANopen master or motor task is introduced. `GpTrajectoryExecutionBinding` is a persistent, M7-private structure supplied to `PathExecutionServices`.

## B1 implementation

- The M7 composition calls `gp_traj_execution_bind` on the **already initialized** Stage 2 trajectory SRAM4 partition, before starting Supervisor.
- The adapter forwards physical wire-feed, retraction, clearance and controlled-stop callbacks to the original board context; it supplies only `read_sample`, `skip_samples`, `stream_ready` and `end_stream`.
- A board trying to supply a second stream provider, or lacking any physical non-stream callback, is rejected **before motion can be enabled**.
- Stream preflight uses the immutable validated-artifact program ID, revision, artifact CRC, sample count and 2000-us period. It refuses an unprepared, stale, mismatched or insufficiently buffered epoch.
- Reads/skips recheck the metadata, refuse expired ownership and do not block to refill from flash. Paused paths retain M7 stream ownership; exiting paths release or discard the M4 producer epoch.
- No real board supplier exists; Batch B1 **does not** commission drives, fabricate F/T or safety signals or fabricate a durable M4 trajectory.

## Acceptance: one local command after staging and clean CubeIDE builds

```bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/stage_stage3_batch_b1.sh "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
# Clean-build Cortex-M7 AND Cortex-M4 independently using CubeIDE.
bash Firmware/Tests/check_stage3_batch_b1_acceptance.sh "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

The gate checks canonical staging parity, ARM object compilation, host trajectory and CANopen tests, target image freshness, strong retained M7 binding symbols and Batch A symbols. Source changes are not a substitute for passed test output.

## Hardware and dependent software gates

- B3/B4 must provide validated M4 storage and new prepared stream epochs. A stream without a commissioned M4 producer is **not ready**.
- The actual board provider must supply physical CANopen node identity, scale, safety and stop/retraction services. Software hold/controlled-stop callbacks are not a substitute for independent safety hardware.
- Physical scope/logic-analyzer tests must quantify 500 Hz jitter, budget, bus load and reaction to stale/error TPDO, bus-off and power loss.
