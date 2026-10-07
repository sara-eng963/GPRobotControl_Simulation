# SIL Kit Supervisor + HMI integration

This is the active PC integration on `sara/can-replacement`.

## Data path

```
Operator HMI / Teaching window
            |
            v
        HMI RTOS task
            |
            v
        Supervisor
            |
            v
      CanopenMaster
            |
            v
   SIL Kit CAN backend
            |
       CAN1 @ 1 Mbit/s
   +---+---+---+---+---+---+
   |   |   |   |   |   |
  N1  N2  N3  N4  N5  N6
   \___ six AVATAR SIL Kit participants ___/
            |
         TPDO4
            |
            v
        Supervisor
        /        \
      HMI       MATLAB
```

The controller and motor processes communicate only through SIL Kit CAN.
The Supervisor does not directly modify a virtual motor's protocol state.

## Main components

| File | Responsibility |
|---|---|
| `CANComm/SILKit/silkit_can_backend.cpp` | Generic `CanBackend` implementation over SIL Kit CAN. |
| `Simulation/AvatarM_CAN/avatar_m_silkit_node.cpp` | One AVATAR CANopen node attached to SIL Kit. |
| `Simulation/AvatarM_CAN/avatar_m_silkit_motor_bank.cpp` | Six AVATAR SIL Kit participants and demo-only motion follower. |
| `Simulation/supervisor_silkit_main.c` | FreeRTOS Supervisor integration, HMI transport and MATLAB status output. |
| `CANComm/CANopen/canopen_master.c` | NMT, heartbeat, SDO, RPDO4/SYNC and TPDO4 coordination. |
| `ServoDrive/AvatarM/` | AVATAR object/PDO/position conversion layer. |
| `Simulation/Tests/test_supervisor_silkit_sequence.py` | Whole-pipeline automated sequence over SIL Kit CAN1. |

## Build

```bash
SILKIT_ROOT="$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"

cmake -S . -B build-can-silkit \
  -DENABLE_SILKIT=ON \
  -DSILKIT_ROOT="$SILKIT_ROOT"

cmake --build build-can-silkit \
  --target supervisor_silkit_sim avatar_m_silkit_motor_bank supervisor_panel teaching_sim \
  -j "$(nproc)"
```

## Manual run order

Start the SIL Kit registry first:

```bash
"$SILKIT_ROOT/bin/sil-kit-registry" \
  --listen-uri silkit://localhost:8500
```

Then start the six AVATAR participants:

```bash
./build-can-silkit/avatar_m_silkit_motor_bank
```

Then start the Supervisor:

```bash
MATLAB_IP="<Windows-host-IP>" \
  ./build-can-silkit/supervisor_silkit_sim
```

Finally start:

```bash
./build-can-silkit/supervisor_panel
./build-can-silkit/teaching_sim
```

Run `MATLAB/can_robot_visualizer.m` on Windows before or after the
Supervisor starts.

## Automated run

```bash
ctest --test-dir build-can-silkit \
  -R '^supervisor_silkit_sequence$' \
  --output-on-failure -V
```

This launches the registry, six virtual motors and Supervisor automatically.

## Interpretation

A passing test establishes that the full software path works over SIL Kit CAN:
the real Supervisor performs CANopen commissioning and coordinated cyclic
motion against six separate AVATAR participants.

It does not establish physical CAN timing on STM32 hardware. The simple motor
follower is visualization/integration behavior only, not AVATAR dynamics.

The previous EtherCAT/A6EC implementation remains on
`sara/can-system-integration`.
