# HMI and Simulator Changes

Date: 2026-10-14

## Operator behavior

- Replace incremental teaching speed controls with Low, Medium, High presets.
- Identify duplicate points using position and geometric orientation differences with separate tolerances. Preserve the last accepted point and recording stage on duplicate input; report orientation-only segment attempts.
- Protect validated paths from teaching selection until Reset clears the program and runtime status.
- Gate Pause/Resume by motion state and support pausing Home without restarting its homing state on resume.
- Unified path completion returns to Idle after its internal retraction. This removes an additional global homing cycle; internal retraction remains enabled.

## Execution feedback

The recorded failure code 0x60012 identifies PATH_EXECUTION / CYCLIC_FEEDBACK. This confirms a watchdog trip, but does not establish whether the underlying TPDO delay was host scheduling, transport, or another cause.

PathExecutionConfig now accepts feedback_max_age_us. Zero retains the existing 20,000-us default. The host SIL Kit simulator explicitly selects 100,000 us to tolerate host scheduling delays. This permits up to 80 ms more stale feedback before stopping in simulation. Physical hardware settings require measured latency and a separate decision.

The failure path still disables wire feed and requests a hold. No final-feedback requirement was removed. State 11 is Fault and state 12 is Emergency Stop in the current enum; the captured failure entered Fault.

## Verification

- Controller, supervisor panel, teaching simulator, and path execution regression target built successfully.
- state_machine_test passed.
- path_execution_prefetch_test passed, including the original 20-ms missing-axis case and the new 100-ms configured-window missing-axis case.
- joint_drive_teaching_test passed.
- Registry, six-node motor bank, controller, supervisor panel, and teaching simulator launched and remained alive through startup checks.

Manual end-to-end path replay under the original failing load is still required. Startup and regression tests do not prove the intermittent failure is eliminated. The optional CAN live monitor is omitted from this launch because of its previously observed participant startup failure.

## Packaging

This checkout includes substantive tracked changes from the working checkout. Line-ending-only differences, generated logs, build artifacts, and downloaded dependencies are excluded. The original checkout is preserved.
