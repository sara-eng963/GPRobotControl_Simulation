# Stage 3B — Headless SIL Kit Integration Regression

**Status:** Test implementation prepared; Python syntax and analyzer/registration
self-tests verified. **No SIL Kit execution has been performed by the authoring
session.** The existing controller, command gate, CANopen master, virtual motor
model and Stage 1/2 fixes are unmodified.

## Coverage

| CTest suffix | Inputs / acceptance |
|---|---|
| `matrix` | ±Fx, ±Fy, ±Fz, ±Tx, ±Ty, ±Tz: signed twist, actual motor feedback displacement, zero-force velocity decay |
| `mixed` | Gradually increasing Fx plus Tz, signed reversal, real motor feedback, finite bounded joint commands |
| `sustained` | Twenty-second alternating Fx/Fy and zero input: thousands of accepted samples, strictly monotonic timestamps, measured reference/feedback tracking, no unexpected FAULT |
| `duplicate`, `out_of_order` | One invalid virtual-F/T sequence from the sole sender; Supervisor must fault and reject later guidance |
| `nonfinite`, `range`, `truncated` | One invalid input datagram; fail-closed Supervisor state, no further valid guidance |
| `disabled`, `dropout` | Explicitly disabled source or no new packets for >150ms; existing watchdog/fail-closed path must fault |

The harness uses `127.0.0.1:5013` with the existing atomic VFT1 protocol and
existing 5010/5012 Supervisor command/status services. It does not control
motors directly. The current GUI exposes only Fx buttons; the headless sender
can exercise all six available virtual protocol components.

**Quantitative checks:** positive/negative Cartesian velocity response, measured
joint movement, 20% or greater decay in the observed velocity over the release
window, time monotonicity, finite signals, feedback age <=100ms, consecutive
joint-target increments <=0.008rad and reference-feedback difference <=0.1rad.
Those are the current HOST TEST FIXTURE thresholds as verified in
`Simulation/supervisor_silkit_main.c`; they are not physical robot approvals.

## Installation and execution (WSL)

1. Stage 3B tests are already registered in `supervisor_silkit_sim.cmake`.
   No installation helper or historical commit checkout is required.
2. Run:

```sh
cd ~/freertos-pc-test
python3 Simulation/Tests/test_supervisor_silkit_virtual_ft_3b.py --selftest
export SILKIT_ROOT="$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"
cmake -S . -B /tmp/virtual-ft-regression -DENABLE_SILKIT=ON -DSILKIT_ROOT="$SILKIT_ROOT"
cmake --build /tmp/virtual-ft-regression --target supervisor_silkit_sim avatar_m_silkit_motor_bank -j2
ctest --test-dir /tmp/virtual-ft-regression -N -R '^supervisor_silkit_virtual_ft_3b_'
set -o pipefail
ctest --test-dir /tmp/virtual-ft-regression -R '^supervisor_silkit_virtual_ft_3b_' -j1 --output-on-failure 2>&1 | tee /tmp/virtual-ft-regression-results.log
ctest --test-dir /tmp/virtual-ft-regression -R '^(supervisor_silkit_virtual_ft_input|supervisor_silkit_guidance_sim_motor_.*|canopen_master_test|canopen_bounded_rx_test)$' -j1 --output-on-failure
```

Each SIL Kit case launches its own registry, six-node bank and Supervisor.
CTest serializes all Stage 3B cases because they share ports 8500, 5010, 5012
and 5013. **Do not run the interactive GUI simultaneously.**

## Boundaries and unverified requirements

- A passed Python self-test validates only analyzer/installer logic; a passed
  CTest integration run is needed before claiming end-to-end behavior.
- Host wall-clock duration, received timestamps and bounded joint-reference
  increments are not measured STM32 cycle WCET or motor bus latency.
- Faults are injected at the virtual F/T UDP layer. Missing TPDOs, CAN frame
  partial-send failure and physical drive STO are **not injected** by this test.
  Existing `stale_ft`/`bad_feedback` synthetic fixtures and CANopen unit tests
  remain separate and must be rerun.
- Automated exit/re-entry to TEACHING during active guidance, physical fault
  recovery, GUI rendering and MATLAB visualization remain outside these tests.
- No physical dynamics, calibrated load, real encoder behavior, collision
  detection, certified stopping or 500 Hz hardware timing is established.

Do not commit these test additions as fully validated until WSL SIL Kit
execution and existing regression tests pass. If a scenario fails, capture
CTest output and full Supervisor diagnostic logs before modifying code or
adjusting fixture thresholds.
