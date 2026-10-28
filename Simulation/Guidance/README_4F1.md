# 4F1 — Host Timing and Hand-Guiding Tracking

**Status:** software-in-the-loop instrumentation implemented; full sustained run
requires WSL verification. Nothing here releases STM32H745 hardware motion.

## What is measured?

The existing **single M7 Supervisor** is still the sole owner of all CANopen
motor commands. The same hand-guiding pipeline and the Stage 4E3 gate run as
before. F1 adds only host instrumentation and offline analysis.

| Instrument / file | Observable | Interpretation |
|---|---|---|
| `TIMING_SUPERVISOR_2MS` | Host Supervisor task-body time and observed start interval; nominal metadata 2,000 us | **Not** true RTOS release-to-release jitter or STM32 deadline measurement |
| `TIMING_GUIDANCE_OBSERVER` | Time spent in the F/T-to-joint-reference callback | Includes read-only feedback and admittance pipeline work |
| `TIMING_GUIDANCE_DISPATCH` | Time for candidate validation, JointDrive CANopen dispatch request | Measures host operation; **not** acknowledged motor motion latency |
| `guidance.csv` | Same-cycle pre-dispatch computed `q1..q6` and measured `actual_q1..actual_q6` | `q_ref - q_actual` is a reference-minus-prior-feedback difference, **not** settled point tracking error |
| `guidance.csv` | Observed feedback age, valid/fault markers, cycle timestamp | Host/synthetic sample freshness, not physical sensor latency |

The pre-dispatch `q_ref` is a **candidate** that may still fail the gate and
might not be sent. Realistic response delay and stability need a more
representative physical plant model and physical sensor.

## File ownership / modularity

- `Simulation/Timing/host_timing_trace.[ch]` remains the existing
  lock-free-ish one-producer-per-source ring and background file writer.
  F1 extends its source enum; no additional RTOS task or blocking logging
  is added to the Supervisor.
- `StateMachine/supervisor_task.c` adds three small optional timing spans.
  The normal 1-ms SIL Kit demonstrations retain their existing 1,000-us
  metadata. The opt-in 2-ms guidance case now uses a separate 2,000-us
  source to avoid mixing incompatible timestamps.
- `Simulation/Guidance/host_guidance_demo.[ch]` extends **the existing**
  per-cycle trace. Encoder-feedback radians are converted with the
  existing `avatar_m_position_units_to_joint_rad()`. The CSV is still
  written only by the lower-priority HMI task, never inside M7.
- `scripts/analyze_guidance_4f1.py` is pure Python stdlib except
  **optional** matplotlib plotting; it reuses
  `scripts/analyze_timing_csv.py` for the existing timing format.
- `Simulation/Tests/test_guidance_4f1_analyzer.py` checks a deterministic
  *synthetic file fixture*, not a control-loop stability test.
- `Simulation/Tests/test_supervisor_guidance_4f1_sustained.py` runs the
  existing SIL Kit registry, motor bank, and Supervisor for ~4 seconds of
  TEACHING after HOMING. This catches unexpected FAULT/ESTOP, verifies
  a sustained stream of valid calculated references, checks that measured
  virtual joint positions move, and generates the report.

## Run from WSL

```bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement
export SILKIT_ROOT="$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"

cmake -S . -B /tmp/hand-guiding-f1-silkit \
    -DENABLE_SILKIT=ON -DSILKIT_ROOT="$SILKIT_ROOT"
cmake --build /tmp/hand-guiding-f1-silkit \
    --target supervisor_silkit_sim avatar_m_silkit_motor_bank -j2

# Preserve the raw CSV and generated reports; otherwise tests use temp files.
export HAND_GUIDING_EXPERIMENT_OUTPUT_DIR="$PWD/build-can-silkit/logs/stage4f1"
ctest --test-dir /tmp/hand-guiding-f1-silkit \
    -R '^(guidance_4f1_analyzer_fixture|supervisor_silkit_guidance_4f1_sustained)$' \
    --output-on-failure
```

Read `build-can-silkit/logs/stage4f1/analysis/REPORT.md`, the CSV data
`guidance.csv` / `timing.csv`, and optional comparison plots. If
matplotlib is not installed, numerical reports are still produced.

To rerun a report on an existing CSV pair without starting the simulator:

```bash
python3 scripts/analyze_guidance_4f1.py \
  --guidance build-can-silkit/logs/stage4f1/guidance.csv \
  --timing build-can-silkit/logs/stage4f1/timing.csv \
  --output-dir build-can-silkit/logs/stage4f1/analysis
```

## Acceptance / interpretation

Passing F1 proves only that a sustained *host software simulation* ran and
the observed joint references and feedback can be quantitatively evaluated.
Inspect actual P95/P99 host timing, intervals, observed body overruns,
trace drops, gaps and tracking errors **before** making performance claims.
A test success does not impose a numerical 500Hz/2ms host deadline limit.

No real F/T sensor, hand-guiding handle compliance, physical robot inertia,
manufacturer drive latency, real STM32H745 execution time or
independently-rated E-stop/STO has been validated. Simulated +0.15 rad
observation-only kinematic model remains a deliberate mathematical fixture.
