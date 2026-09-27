# Real HMI + Teaching Simulation

This folder is the new operator-facing HMI integration. The old planner/test UI
remains untouched under `HMI-Mock/`.

## Two-window workflow

The new simulation deliberately mirrors the intended physical workflow:

1. **Industrial Robot HMI** — the first window represents the real panel.
   It contains the physical-panel controls:
   - robot status screen,
   - E-Stop,
   - Line / Circular Arc / Circle selection,
   - Record,
   - Preview / Validate Path,
   - speed decrease / increase / default,
   - Start / Replay,
   - Pause,
   - Reset,
   - Home.
2. **Teaching Simulation** — the second window substitutes only for physical
   hand guiding. It displays one current TCP guide target, the actual TCP, and
   the points already recorded by the real Teaching state.

There is intentionally **no A/B/C batch waypoint editor** in the new HMI.
The operator moves the simulated robot to one pose, returns to the first HMI
window, presses **RECORD**, then moves to the next pose and presses **RECORD**
again.

## State-machine integration

`Simulation/hmi_state_controller.c` runs the actual state modules:

```
BOOT -> HOMING -> IDLE -> TEACHING -> PATH_VALIDATION -> APPROACH
```

The second window does not inject a fake Teaching point into
`state_teaching.c`. Instead it behaves like a simulation-only external
manual-guidance controller:

```
3D TCP target
    -> ControlCore ADLS IK
    -> short ControlCore JointTrajectory
    -> A6-EC CSP targets
    -> KickCAT simulated drives
    -> actual PDO feedback
    -> state_teaching.c RECORD
    -> ControlCore FK
    -> TaughtPoint
```

That keeps the existing Teaching logic unchanged.

## Path Validation and Approach

Path Validation uses a host RAM implementation of the existing
`PathValidationStorage` interface. Approach reads sample 0 from the same
committed artifact through its existing `ApproachServices` callback.

Collision callbacks remain disabled in this PC UI simulator because no final
cell collision model exists yet. The simulator does not pretend that this is a
production safety check.

## MATLAB

MATLAB telemetry is preserved on UDP port **5005** using the same six
`int32_t` A6 position-unit values sent by the existing `main.c`. The current
MATLAB visualizer can therefore remain unchanged.

## Build

```bash
cmake -S . -B build
cmake --build build --target hmi_state_controller robot_hmi teaching_sim
```

## Run

With KickCAT installed in `~/KickCAT`:

```bash
./scripts/run_hmi_state_simulation.sh
```

Wait for the first HMI window to show **IDLE**, select a program, press
**START / REPLAY**, move the robot from the Teaching Simulation window, and
press **RECORD** on the first window for each taught point.
