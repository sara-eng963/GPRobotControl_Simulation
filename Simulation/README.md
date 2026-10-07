# SIL Kit CANopen / AVATAR simulation

The canonical PC controller simulation is:

```
Simulation/supervisor_silkit_main.c
```

It runs the real FreeRTOS Supervisor and robot state modules through the
project's established SIL Kit CAN backend.

```
HMI / MATLAB
      |
      v
FreeRTOS Supervisor / StateMachine
      |
      v
CanopenMaster
      |
      v
CANComm/SILKit/silkit_can_backend.cpp
      |
      v
SIL Kit CAN1 @ 1 Mbit/s
      |
      +--> AVATAR node 1
      +--> AVATAR node 2
      +--> AVATAR node 3
      +--> AVATAR node 4
      +--> AVATAR node 5
      +--> AVATAR node 6
```

The six virtual actuators are hosted by:

```
Simulation/AvatarM_CAN/avatar_m_silkit_motor_bank.cpp
```

Each actuator is an independent SIL Kit participant using the existing
AVATAR protocol model. NMT, SDO, RPDO4, SYNC, TPDO4 and heartbeat traffic
therefore crosses the SIL Kit CAN network instead of an in-memory bus shim.

The motor bank includes a deliberately simple position follower only so the
HMI/MATLAB demo can show motion. It is not a model of AVATAR motor dynamics.

## Build

The established SIL Kit 5.0.7 installation is:

```
$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit
```

Configure a SIL Kit build:

```bash
cmake -S . -B build-can-silkit \
  -DENABLE_SILKIT=ON \
  -DSILKIT_ROOT="$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"

cmake --build build-can-silkit \
  --target supervisor_silkit_sim avatar_m_silkit_motor_bank supervisor_panel teaching_sim \
  -j "$(nproc)"
```

## Runtime processes

The complete PC pipeline uses four Linux-side processes plus MATLAB:

1. `sil-kit-registry`
2. `avatar_m_silkit_motor_bank`
3. `supervisor_silkit_sim`
4. `supervisor_panel` and `teaching_sim`
5. MATLAB `MATLAB/can_robot_visualizer.m`

The registry URI is `silkit://localhost:8500`. Both the controller and six
motor participants use SIL Kit network `CAN1` at 1 Mbit/s.

## Automated integration test

When configured with `ENABLE_SILKIT=ON`, CTest registers:

```
supervisor_silkit_sequence
```

The test launches its own SIL Kit registry, the six-axis AVATAR motor bank and
the Supervisor, then drives the HMI protocol through startup, Teaching, Path
Validation, Preview, Production, Pause/Resume, E-stop recovery and fault
recovery.

## MATLAB

The Supervisor sends the same versioned status packet used by the HMI to UDP
port **5005**. The packet contains Supervisor state, CAN node readiness, six
joint angles and TCP XYZ.

Run:

```
MATLAB/can_robot_visualizer.m
```

If MATLAB runs on Windows while the controller runs in WSL2, start the
Supervisor with `MATLAB_IP` set to the Windows host/gateway address.

## Scope of the simulation

This setup validates application integration over SIL Kit CAN communication.
It does not prove the physical STM32 transceiver path or real AVATAR actuator
timing. Detailed arbitration/bit-timing claims require a suitable SIL Kit
network-simulation layer or physical measurement; they are not inferred from
the protocol-only virtual nodes.

The removed EtherCAT/A6EC implementation remains recoverable from Git history
and branch `sara/can-system-integration`.
