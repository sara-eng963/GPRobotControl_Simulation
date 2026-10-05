# Supervisor, HMI and real-state PC integration

This guide describes the supervisor mock simulation on `roaa/final-supervisor-integration`. It excludes the optional periodic TRACE/CSV logger. The controller still prints state transitions and guidance failures for debugging.

## Purpose and architecture

The PC simulation executes the project's C state modules under FreeRTOS. Only the physical hardware boundary and persistent trajectory storage are simulated. The test driver sends operator commands; it does not set activeState or manufacture STATE_COMPLETE or validation-success events.

The operational GUI and hand-guiding GUI are separate Linux processes. Commands travel by localhost UDP into the HMI RTOS task, through the supervisor message queue, and into the real state step functions. Status travels back to the GUIs.

The pure state_machine.c owns transition policy. supervisor_task.c integrates that policy with state entry/step calls and services. There is no separate competing coordinator and no task for each state.

## File map

| File | Responsibility |
|---|---|
| StateMachine/state_machine_types.h | Shared state IDs, events, safety and fault types. |
| StateMachine/state_machine.h / .c | Pure supervisory context, transition guards, program lifecycle and recovery policy. |
| StateMachine/supervisor_task.h / .c | FreeRTOS queue/task wrapper, state dispatch, HMI translation and diagnostic snapshot interface. |
| StateMachine/supervisor_hmi.h | Commands accepted at the supervisor HMI boundary. |
| StateMachine/supervisor_io.h / .c | Logical input/output snapshots and status-output mapping; board pins and polarity belong in a future adapter. |
| StateMachine/States/state_boot.c / .h | Real startup sequence through the simulated drive/bus API. |
| StateMachine/States/state_homing.c / .h | Actual joint-space homing execution and checks. |
| StateMachine/States/state_idle.c / .h | Actual Idle state behavior. |
| StateMachine/States/state_teaching.c / .h | Draft creation, recording, geometry selection and submission. Updated implementation provides job-wide speed and inherited segment starts. |
| StateMachine/States/state_path_validation.c / .h | Actual trajectory generation, IK, configured checks and artifact creation. |
| StateMachine/States/state_approach.c / .h | Actual approach from current robot position to the first validated joint target. |
| StateMachine/States/state_path_execution.c / .h | Preview/production sample execution, wire-feed command and retraction callbacks. |
| StateMachine/States/state_paused.c / .h | Hold confirmation and wire-feed OFF. |
| StateMachine/States/state_fault.c / .h | Latched fault behavior and safe-output services. |
| StateMachine/States/state_emergency_stop.c / .h | Safe outputs and operator-driven recovery. |
| HMI/hmi_api.h | Teammate's existing operator-event definitions. |
| HMI/hmi_protocol.c / .h, hmi_types.h | Existing UDP event/status transport and GUI types. |
| HMI/hmi_theme.c / .h | Existing GUI styling and drawing helpers. |
| HMI/hmi_task.c / .h | Added RTOS event receiver/translator and status publisher. Retains a pending event if the supervisor queue is full. |
| HMI/supervisor_panel_app.c | Derived operational panel with supervisor state names and separate Validate/Preview controls. Original HMI/hmi_app.c remains separate. |
| HMI/teaching_sim_app.c / .h | Existing virtual hand-guiding GUI; mouse projection was corrected in the .c file. |
| hmi.c, teaching_sim.c | Existing entry points for the GUI executables. |
| Simulation/supervisor_mock_main.c | PC startup, service wiring, RAM storage, mock input publication, real IK for simulated guidance, UDP callbacks. |
| Simulation/MockHardware/mock_backend.c / .h | Ideal simulated drive feedback and the temporary backend compatibility API. |
| Simulation/Tests/test_supervisor_mock_sequence.py | Automated real-state sequence using HMI protocol commands. |
| supervisor_mock_sim.cmake | Defines supervisor_mock_sim, supervisor_panel and automated sequence test registration. |
| supervisor_interface_test.cmake | Builds/registers the RTOS supervisor interface test. |
| StateMachine/Tests/test_supervisor_interfaces.c | RTOS interface checks. |
| StateMachine/Tests/test_state_machine.c | Pure policy and transition assertions. |
| StateMachine/Tests/test_runtime_states.c | Runtime state service/behavior tests. |
| CMakeLists.txt | Includes the integration target definitions. |

HMI-Mock/ is not used by these targets. Original main.c and older simulation targets are not replaced.

## Tasks instantiated by this simulator

| Task | Priority | Scheduling | Responsibility |
|---|---:|---|---|
| Supervisor | 4 | Requested 1 ms periodic wake | Queue handling and real active-state stepping. |
| MockInputs | 3 | 1 ms relative delay after each update | Mock safety/runtime feedback and guidance IK. No missed-cycle publication catch-up bursts. |
| HMI | 2 | Requested 10 ms periodic wake | GUI commands and status publication. |

FreeRTOS also creates its Idle task. GUI windows are not RTOS tasks. PC periods do not demonstrate physical 1 ms deadlines. STM32 deployment still requires communication/cyclic motion, real input monitoring and suitable storage integration; a separate buffered logger can be added later.

## Build and startup

Use an existing configured WSL checkout with the project's FreeRTOS, SOEM and raylib dependencies. Although physical EtherCAT is not used, the target retains legacy bus/servo compatibility layers and build dependencies. CAN integration is separate work.

```bash
cd "$HOME/freertos-pc-test"
"$HOME/.local/share/gprobot-tools/venv/bin/cmake" -S . -B build
"$HOME/.local/share/gprobot-tools/venv/bin/cmake" --build build \
  --target supervisor_mock_sim supervisor_panel teaching_sim -j "$(nproc)"
```

Controller terminal:

```bash
SIM_CPU="$(python3 -c 'import os; print(min(os.sched_getaffinity(0)))')"
taskset -c "$SIM_CPU" ./build/supervisor_mock_sim
```

Single-CPU affinity resolved an observed startup stall in this WSL setup. Its exact root cause has not been isolated; this workaround is not an STM32 requirement.

In two additional terminals, set the WSLg variables if missing and start one GUI per terminal:

```bash
cd "$HOME/freertos-pc-test"
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
# Terminal 2:
./build/supervisor_panel
# Terminal 3, separately:
./build/teaching_sim
```

UDP ports: commands 5010, operational-panel status 5011, teaching status 5012. No sudo, KickCAT or virtual Ethernet pair is needed. Do not run a second controller or the automated sequence alongside these GUIs.

## Operator sequence

1. Wait for BOOT -> HOMING -> IDLE.
2. Select LINE/ARC/CIRCLE to enter Teaching. Drag the blue target to simulate guidance. Recording remains on the operational panel.
3. Record the first segment: LINE requires two points; ARC/CIRCLE require three non-collinear geometry points.
4. For another segment, choose geometry before Validate. Its P1 is copied from the previous endpoint: LINE P2, ARC P3, CIRCLE P1. Record the remaining points. A full circle ends at P1 even though the operator last recorded P3.
5. Adjust job TCP speed during Teaching. Updates affect completed, current and future draft segments. Validation uses that speed. Submitted/validated trajectories are not retimed by changing a display value.
6. Press Validate. Actual validation returns to IDLE with a valid result or rejection details. An error name of NONE alone is not proof of validation success.
7. For a valid artifact, press Preview. Approach and trajectory execution run with wire feed OFF; completion returns home and to IDLE.
8. After accepted Preview, press Start. Production executes the validated samples with wire-feed command ON during trajectory execution and OFF for pause, stop and return.
9. E-stop is a simulator toggle. Release it, press Reset to acknowledge, then Home. Release or Reset alone does not start motion. There is no separate RESET global state.

No arc-start command, arc feedback or arc-stabilization phase is implemented. The relay command is currently defined as wire-feed control; actual welding-machine wiring and semantics require separate hardware integration.

Matching endpoint poses do not guarantee blended corners or continuous joint velocities. The validator still checks motion consistency. Reset recovery can clear program data; do not assume the saved artifact survives Reset.

## Automated testing

Close both GUIs and stop the controller first. The script starts its own controller:

```bash
SIM_CPU="$(python3 -c 'import os; print(min(os.sched_getaffinity(0)))')"
taskset -c "$SIM_CPU" python3 Simulation/Tests/test_supervisor_mock_sequence.py \
  ./build/supervisor_mock_sim
```

The demonstrated passing sequence covers startup, two-point LINE recording, real IK-guided movement, real validation, Preview, production, Pause, duplicate Pause, Resume, completion, E-stop release/Reset/Home recovery and external-fault clear/Reset/Home recovery. No synthetic state completion is supplied by the test.

Run the independent policy/module/interface regressions as well:

```bash
"$HOME/.local/share/gprobot-tools/venv/bin/cmake" --build build \
  --target state_machine_test runtime_states_test supervisor_interface_test -j "$(nproc)"
./build/state_machine_test
./build/runtime_states_test
./build/supervisor_interface_test
```

Passing assertion counts describe checks, not the number of independent scenarios or proof of complete coverage. After recent Teaching changes, rerun tests; do not assume earlier results cover the new code.

## Coverage and limitations

| Confirmed or implemented | Not established by current tests |
|---|---|
| Actual C state modules execute together under the FreeRTOS PC port. | STM32 startup, memory/stack sufficiency, dual-core synchronization, physical timing. |
| Demonstrated LINE preview and production with HMI protocol commands. | Exhaustive ARC/CIRCLE and mixed-program end-to-end coverage. |
| Job-wide speed and all nine geometry endpoint inheritance combinations had focused helper checks. | Full public-event/GUI regression of connected multi-segment programs and speed retiming. |
| Actual IK and configured validation run. | Physical calibration, collision safety or feasibility under real dynamics. |
| Simulated position feedback follows commanded motion. | Tracking lag, torque, brakes, missed CAN messages, real following-error behavior. |
| Joint position/velocity and configured continuity checks run. | Joint acceleration is disabled in this mock configuration; collision callbacks are not required. |
| Latched software fault/E-stop recovery sequence is exercised. | Certified hardware safety, independent stop-chain response, all in-motion safety scenarios. |
| Wire-feed ON/OFF commands are observable. | Actual relay operation, wire motion, welding quality, arc status. |
| RAM-backed artifact read/write services run. | NOR flash/SD persistence, power-loss recovery, concurrent logging and storage. |

The approach/retraction callback currently reuses Homing. The supervisor may subsequently enter its own Homing state, resulting in redundant home passes; dedicated safe retraction planning remains future work. BOOT has an existing safety-verification placeholder. Queue-overflow handling in the mock adapter stops the process after safe-output requests; durable event/snapshot backpressure design remains work for production integration.

## Diagnostics without the optional logger

Keep transition prints and IK failure diagnostics. The optional 10 Hz TRACE/CSV implementation is not included. Guidance error 1 denotes solver failure; 2 denotes a blocked guidance request. IK failure can mean non-convergence and does not alone prove geometric unreachability.

## Sources

The repository source files listed above are authoritative for this integration:
https://github.com/sara-eng963/GPRobotControl_Simulation/tree/roaa/final-supervisor-integration

FreeRTOS queues: https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/02-Queues-mutexes-and-semaphores/01-Queues

Linux CPU affinity: https://man7.org/linux/man-pages/man1/taskset.1.html
