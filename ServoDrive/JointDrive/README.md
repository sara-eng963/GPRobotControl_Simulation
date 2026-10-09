# Joint-drive boundary (application → actuator network)

## Responsibilities

```text
Supervisor / robot states
       ├─ JointDrivePort (IDLE / HOMING / APPROACH / TEACHING / EXECUTION)
       └─ DriveCommissioningPort (BOOT: identity, network, SDO)
             ↓ CANopen adapters
          CanopenMaster (CiA-402 + AVATAR PDO/NMT/SDO)
             ↓ CanBackend (SIL Kit today; STM32 FDCAN later)
          Six AVATAR M nodes
```

The **state machine** decides *when* and *why* to move. The **joint-drive
port** exposes six-axis commands and measurements without exposing CAN IDs,
PDOs, statusword bits, or the CANopen master's mutable internals. The **CANopen
master** still owns protocol transactions and the command ordering of six
RPDO4 frames followed by SYNC.

- `state_idle`: hold current raw measured units, watch all six fresh feedback
  counters, and check drive readiness.
- `state_homing`: create and verify joint-space trajectory using current raw
  measurements, conversions, per-axis operation-enabled status, and feedback.
- `state_approach`: move from measured pose to validated trajectory sample 0,
  retaining the original target units and following-error checks.
- `state_path_execution`: stream validated samples and require fresh feedback
  on all axes before advancing.
- `state_teaching`: **read-only** capture of joint feedback and FK-based TCP
  measurements; manual-guidance motion remains a separate capability.
- `state_boot`: uses the separate commissioning port for NMT/SDO setup,
  heartbeat state and CiA-402 enable sequencing, then uses the joint-drive
  port for cyclic six-axis hold commands and freshness verification.

## Deliberate boundaries

- The port is a **C function-pointer interface**, not a shared-memory
  cross-core ABI. A future M7/M4 implementation needs IPC, cache maintenance,
  and explicit mailbox ownership.
- Currently the supervisor is the only intended *task* owner of the CANopen
  coordinator. Calls from states and supervisor heartbeat service occur in its
  serialized execution context. Simulator helpers must not concurrently
  mutate the master from other tasks.
- `joint_drive_port_feedback_sequence()` returns validity separately from
  its output counter; a zero sequence value is legitimate.
- `joint_drive_port_all_enabled()` is a BOOT diagnostic checking decoded
  enable status only, whereas `canopen_master_all_drives_operation_enabled()`
  also requires feedback_valid. BOOT explicitly tests feedback separately;
  `joint_drive_port_ready()` remains canonical for motion permission.
- `JointDrivePort.send_targets` is one coordinated six-axis *transaction*.
  A successful return reports software/backend acceptance, **not** verified
  bus delivery nor physical motion completion. States separately require
  post-command fresh feedback and apply timeouts.
- A position snapshot is not automatically coherent across axes unless the
  backend/owner enforces that. TPDO counters remain per-axis.
- `state_boot` performs drive commissioning via `DriveCommissioningPort`
  rather than calling `CanopenMaster` directly. The port is local and
  nonblocking; it does not replace the concrete CANopen protocol coordinator.
- The supervisor is still the composition root owning `CanopenMaster` and
  service-heartbeat production; simulator-specific input and HMI helpers still
  access it directly. Protocol ownership remains one supervisor task in the
  intended firmware architecture.
- Hard real-time 500 Hz timing and safety-rated stopping remain **unverified**
  on STM32 hardware. Passing PC SIL Kit tests is not hardware qualification.

## Regression checks

The root CMake project registers standalone fake-drive tests for BOOT,
IDLE, HOMING, APPROACH, and TEACHING; `canopen_master_test` verifies the CANopen
adapter's RPDO4 + SYNC sequence; `supervisor_silkit_sequence` runs the
full simulated six-motor supervisory workflow.

Re-run all after changing the port contract or a robot state.
