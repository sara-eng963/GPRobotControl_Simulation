# Legacy simulator entry point

`old_simulator_main.c` is the previous pre-global-FSM PC simulator that used
its own `MotionState`, direct Line/Arc/Circle planning, and the old HMI
protocol.

It is kept only as historical/reference code. It is **not** part of the normal
CMake build.

Current PC simulator entry point:

```
Simulation/simulator_main.c
```

Future real robot entry point:

```
main.c
```
