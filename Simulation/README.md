# CANopen / AVATAR simulation

The canonical PC controller simulation is:

```
Simulation/supervisor_mock_main.c
```

It runs the real FreeRTOS Supervisor and robot state modules against the AVATAR
CANopen simulator.

```
FreeRTOS POSIX
    |
    v
Supervisor / StateMachine
    |
    +--> BOOT
    +--> HOMING
    +--> IDLE
    +--> TEACHING
    +--> PATH_VALIDATION
    +--> APPROACH
    +--> PATH_EXECUTION
    |
    v
CanopenMaster
    |
    +--> NMT / heartbeat / expedited SDO
    +--> 6 x RPDO4 target positions
    +--> SYNC
    +--> 6 x TPDO4 feedback
    |
    v
AVATAR M node simulator
```

Motion states use a 2 ms / 500 Hz application command grid. Each cyclic motion
command sends the six RPDO4 targets followed by SYNC, and the motion states
require fresh TPDO4 feedback before advancing.

Simulation-only boundaries are the AVATAR node model, desktop HMI UDP,
simulated hand guidance, RAM-backed validated-trajectory storage and software
fault/E-stop injection. They are not claims about physical motor dynamics or a
real 1 Mbit/s CAN bus.

The old EtherCAT/A6EC implementation is intentionally absent from
`sara/can-replacement`. It remains recoverable from Git history and from
`sara/can-system-integration`.

The future physical controller entry point is the root `main.c`.


## Live MATLAB visualization

The Supervisor sends the same versioned status packet used by the HMI to UDP
port **5005** for MATLAB. It contains Supervisor state, CAN node readiness,
six joint angles and TCP XYZ.

Run `MATLAB/can_robot_visualizer.m` in MATLAB.

If MATLAB runs on Windows while the controller runs inside WSL2, start the
controller with `MATLAB_IP` set to the Windows host/gateway address, for
example:

```bash
MATLAB_IP="$(ip route | awk '/default/ {print $3; exit}')" \
  ./build-can-clean/supervisor_mock_sim
```

If MATLAB and the controller run in the same Linux environment, the default
`127.0.0.1` destination is sufficient.
