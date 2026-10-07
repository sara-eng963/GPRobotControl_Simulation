# Operator HMI + Teaching Simulation

This folder contains the operator-facing HMI used with the CANopen Supervisor
simulation.

## Two-window workflow

1. **Supervisor panel** — robot state, program selection, Record,
   Validate/Preview, Start/Replay, Pause, Reset and Home.
2. **Teaching Simulation** — a PC-only substitute for physical hand guiding.

The Teaching window does not inject taught points directly. It requests a
simulated guidance pose; the controller-side simulator solves IK, updates the
AVATAR simulated joints, and `state_teaching.c` records the resulting AVATAR
feedback through the normal FK path.

```
Teaching target
    -> ControlCore IK
    -> AVATAR simulated joint feedback
    -> CanopenMaster / TPDO4
    -> state_teaching.c
    -> ControlCore FK
    -> TaughtPoint
```

The operational panel displays CAN node readiness as `CAN: 6/6 nodes`.

## Controller-facing API

`HMI/hmi_api.h` defines the stable operator events. The PC transport is
implemented by `HMI/hmi_protocol.c`; the FreeRTOS controller integration is
implemented by `HMI/hmi_task.c` and `StateMachine/supervisor_task.c`.

The desktop software E-stop and simulated guidance packets are simulation-only.
A physical E-stop belongs to the independent hardware safety chain.

## Build

```bash
cmake -S . -B build-can-supervisor

cmake --build build-can-supervisor \
  --target supervisor_mock_sim supervisor_panel teaching_sim \
  -j "$(nproc)"
```

## Run

In separate terminals:

```bash
./build-can-supervisor/supervisor_mock_sim
./build-can-supervisor/supervisor_panel
./build-can-supervisor/teaching_sim
```

No virtual Ethernet pair, SOEM or external EtherCAT slave simulator is
required.

For the automated full sequence, run:

```bash
ctest --test-dir build-can-supervisor \
  -R '^supervisor_mock_sequence$' \
  --output-on-failure -V
```
