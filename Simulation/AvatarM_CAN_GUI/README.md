# AVATAR M-Series CANopen Test Bench

This is a standalone visual test bench for the project's CANopen + AVATAR M
communication layer.

It intentionally reuses the real project code:

- `CanopenMaster`
- CANopen NMT / SDO / heartbeat framing
- AVATAR M drive layer
- AVATAR RPDO4 / TPDO4 mapping
- `AvatarMNodeSim`

The only substituted piece is the physical CAN transport. The GUI uses a small
in-memory `CanBackend` so the six protocol models can run deterministically in
one process.

The existing SIL Kit integration tests remain the transport-level validation.

## Build

From the repository root:

```bash
cmake -S Simulation/AvatarM_CAN_GUI -B build-avatar-can-gui
cmake --build build-avatar-can-gui -j "$(nproc)"
./build-avatar-can-gui/avatar_m_can_testbench
```

## Recommended demo sequence

1. Start with all six nodes in **PRE-OP**.
2. Press **NMT START**.
3. Edit the six raw target-position values.
4. Press **SEND RPDO4**.
   - The new values appear under **Cached**.
   - **Active** is still unchanged.
5. Press **SYNC**.
   - All six cached targets move to **Active** together.
6. Enable **DEMO MOTION** if you want the visual actual-position trace.
7. Press **STREAM 20Hz**.
   - RPDO4 + SYNC cycles repeat.
   - TPDO4 feedback updates as the visualization follower moves.
8. Watch the CAN monitor:
   - `0x501..0x506` = RPDO4
   - `0x080` = SYNC
   - `0x481..0x486` = TPDO4
   - `0x701..0x706` = heartbeat

## Important simulation boundary

`AvatarMNodeSim` is a protocol model, not a motor-physics model.

In protocol-only mode:

- RPDO4 caches target position.
- SYNC releases the target.
- Actual position does not magically change.

The optional **DEMO MOTION** switch adds only a bounded visual follower so that
command/feedback evolution can be plotted. It is deliberately labeled as
visualization-only and must not be presented as identified AVATAR dynamics.

## Test-fixture assumptions

The GUI initializes each virtual node with:

- interpolation mode `7`
- statusword `0x0437`
- actual position `0`

These are explicit simulation fixtures reused from the existing AVATAR tests;
they are not claimed motor power-on defaults.
