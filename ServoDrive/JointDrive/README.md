# Joint-drive boundary (application → actuator network)

## Responsibilities

```text
Supervisor / robot states
       ↓ JointDrivePort
CANopenJointDrivePort (adapter)
       ↓ CanopenMaster (CiA-402 + AVATAR PDO/NMT/SDO)
CanBackend (SIL Kit today; STM32 FDCAN later)
       ↓ six AVATAR M nodes
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

## Deliberate boundaries

- The port is a **C function-pointer interface**, not a shared-memory
  cross-core ABI. A future M7/M4 implementation needs IPC, cache maintenance,
  and explicit mailbox ownership.
- Currently the supervisor is the only intended *task* owner of the CANopen
  coordinator. Calls from states and supervisor heartbeat service occur in its
  serialized execution context. Simulator helpers must not concurrently
  mutate the master from other tasks.
- `JointDrivePort.send_targets` is one coordinated six-axis *transaction*.
  A successful return reports software/backend acceptance, **not** verified
  bus delivery nor physical motion completion. States separately require
  post-command fresh feedback and apply timeouts.
- A position snapshot is not automatically coherent across axes unless the
  backend/owner enforces that. TPDO counters remain per-axis.
- `state_boot` still performs CANopen-specific drive commissioning (NMT,
  SDO, CiA-402). Do not add every commissioning operation to the motion port;
  introduce a separate commissioning/control interface when migrating BOOT.
- Hard real-time 500 Hz timing and safety-rated stopping remain **unverified**
  on STM32 hardware. Passing PC SIL Kit tests is not hardware qualification.

## Regression checks

The root CMake project registers standalone fake-drive tests for IDLE,
HOMING, APPROACH, and TEACHING; `canopen_master_test` verifies the CANopen
adapter's RPDO4 + SYNC sequence; `supervisor_silkit_sequence` runs the
full simulated six-motor supervisory workflow.

Re-run all after changing the port contract or a robot state.
