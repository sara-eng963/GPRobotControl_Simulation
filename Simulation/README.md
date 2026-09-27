# Simulation entry point

The canonical PC simulator entry point is:

```
Simulation/simulator_main.c
```

It uses the **FreeRTOS POSIX port** and drives the same global StateMachine and
state modules intended for the real robot.

```
PC simulator
Simulation/simulator_main.c
        |
        v
FreeRTOS POSIX
        |
        v
StateMachine/state_machine.c
        |
        +--> BOOT
        +--> HOMING
        +--> IDLE
        +--> TEACHING
        +--> PATH_VALIDATION
        +--> APPROACH
```

Simulation-only boundaries around the shared robot logic are:

- KickCAT virtual EtherCAT slaves,
- SOEM on the PC,
- desktop HMI UDP,
- simulated hand guidance,
- RAM-backed validated-trajectory storage,
- MATLAB UDP telemetry.

The future real controller entry point is the root `main.c`. That file is
reserved for the teammate implementing the STM32/FreeRTOS target runtime.

The previous pre-global-FSM simulator was archived at:

```
Legacy/old_simulator_main.c
```

It is not part of the normal build.
