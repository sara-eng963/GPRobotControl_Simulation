# CANopen Supervisor + HMI PC integration

This guide describes the active integration on `sara/can-replacement`.

## Architecture

The PC simulation executes the real robot state modules under the FreeRTOS
POSIX port. The physical CAN controller and motors are replaced by the
in-memory AVATAR CANopen bus/node simulator.

```
HMI / automated test
        |
        v
HMI RTOS task
        |
        v
Supervisor task
        |
        v
StateMachine + real state modules
        |
        v
CanopenMaster
        |
        +-- NMT
        +-- heartbeat
        +-- expedited SDO
        +-- RPDO4 x 6
        +-- SYNC
        +-- TPDO4 x 6
        |
        v
AVATAR M simulated nodes
```

The test driver never forces `activeState` and does not manufacture state
completion events. State transitions occur through the real Supervisor policy.

## Main files

| File | Responsibility |
|---|---|
| `StateMachine/state_machine.c` | Pure transition policy, program lifecycle and recovery guards. |
| `StateMachine/supervisor_task.c` | FreeRTOS queue/task integration and real state dispatch. |
| `StateMachine/States/state_boot.c` | CANopen startup, identity, heartbeat, mode and CiA-402 commissioning. |
| `StateMachine/States/state_homing.c` | 500 Hz homing commands through RPDO4 + SYNC with TPDO4 feedback. |
| `StateMachine/States/state_idle.c` | Holds captured AVATAR raw positions on the CAN cyclic path. |
| `StateMachine/States/state_teaching.c` | Reads AVATAR feedback, converts to joint radians and records FK poses. |
| `StateMachine/States/state_path_validation.c` | Creates immutable 2 ms AVATAR raw-position execution artifacts. |
| `StateMachine/States/state_approach.c` | Executes the approach trajectory over CANopen at 500 Hz. |
| `StateMachine/States/state_path_execution.c` | Executes validated Preview/Production samples over CANopen at 500 Hz. |
| `CANComm/CANopen/canopen_master.c` | Six-axis CANopen master used by the state layer. |
| `ServoDrive/AvatarM/` | AVATAR protocol, drive and position conversion layer. |
| `Simulation/AvatarM_CAN_GUI/sim_can_bus.c` | In-memory CAN backend and six AVATAR simulated nodes. |
| `Simulation/supervisor_mock_main.c` | PC integration wiring, simulated guidance, RAM storage and HMI transport. |
| `Simulation/Tests/test_supervisor_mock_sequence.py` | End-to-end operator/recovery sequence. |

## Build

```bash
cmake -S . -B build-can-supervisor

cmake --build build-can-supervisor \
  --target supervisor_mock_sim supervisor_panel teaching_sim \
  -j "$(nproc)"
```

The active Supervisor simulator has no SOEM, KickCAT, EtherCAT or A6EC build
dependency.

## Run interactively

Controller:

```bash
./build-can-supervisor/supervisor_mock_sim
```

Operational panel:

```bash
./build-can-supervisor/supervisor_panel
```

Teaching window:

```bash
./build-can-supervisor/teaching_sim
```

UDP ports are 5010 for commands, 5011 for the operational panel status and
5012 for teaching status.

## Automated end-to-end test

```bash
ctest --test-dir build-can-supervisor \
  -R '^supervisor_mock_sequence$' \
  --output-on-failure -V
```

The demonstrated sequence covers startup, Teaching, guided movement, recording,
Path Validation, Preview, Production, Pause/Resume, E-stop recovery and
external-fault recovery without forced state completions.

## What the PC test establishes

It establishes the software architecture and integration contract: six-node
CANopen commissioning, CiA-402 readiness, heartbeat monitoring, synchronized
RPDO4/SYNC commands, TPDO4 feedback freshness and the complete supervisory
workflow on a 2 ms application grid.

It does **not** prove a physical 1 Mbit/s CAN bus or the real AVATAR actuators
meet the 500 Hz requirement. Physical validation still has to measure bus
utilization, arbitration latency, jitter, missed frames and real actuator
response.

## Archived EtherCAT implementation

The removed EtherCAT/A6EC implementation remains available in Git history and
on branch `sara/can-system-integration`. It is not part of the active
`sara/can-replacement` build.
