# Issue 5 — Measured host timing (not STM32 WCET)

The PC SIL Kit controller can record event-level timing into an opt-in CSV
with a monotonic, microsecond-resolution POSIX clock. Measured loops do not
write files or wait for the logger. Each source has a dedicated single-producer/
single-consumer ring, drained by one background pthread. On ring saturation,
measurement rows are dropped rather than delaying the control loop.

## One-command experiment (WSL)

From the repository root, with the existing SIL Kit build configuration:

```bash
bash scripts/run_timing_benchmark.sh
# optionally add cancellation and flash program failure:
bash scripts/run_timing_benchmark.sh --include-faults

# Resume only the flash scenario after baseline has completed:
bash scripts/run_timing_benchmark.sh --flash-only --no-plots

# Analyze a saved baseline without rerunning SIL Kit:
python3 scripts/analyze_timing_csv.py \
  .simulation-logs/timing-benchmark/baseline.csv --no-plots
```

The script reconfigures/builds the binaries, checks the recorder and analyzer,
then runs the existing full SIL Kit scenario and the flash scenario.
Allow roughly 2-3 minutes for the two full-simulation runs.

Results are under \`.simulation-logs/timing-benchmark/\`:

- \`baseline.csv\`, \`flash.csv\`: raw measured event timing, one row per event.
- \`baseline-analysis/summary_metrics.csv\`, \`flash-analysis/summary_metrics.csv\`: per-source P50/P95/P99/observed max and misses.
- \`*-analysis/timing_report.md\`: plain-language interpretation.
- \`*-analysis/*_execution_time.png\` and \`*_start_lateness.png\`: plots when matplotlib is installed.

Or collect manually while using the GUIs:

```bash
bash scripts/run_full_can_silkit_simulation.sh
python3 scripts/analyze_timing_csv.py .simulation-logs/timing-metrics.csv
```

The launcher sets GP_TIMING_CSV automatically, and overwrites the previous
\`timing-metrics.csv\` for each complete simulation run. To preserve multiple
runs, use the benchmark runner's separate scenario files or copy the CSV.

Optional plotting:
\`python3 -m pip install matplotlib\`. Without matplotlib the analyzer still
produces CSV/Markdown summaries. Use \`--no-plots\` to suppress PNGs.

## Nonredundant Issue 5 diagnosis: path-command hold reasons

Existing Renode QSPI tests already exercise Cortex-M7 flash/streaming
functionality, but the repo has no Renode image running the actual
Supervisor on a faithful dual-core STM32H745 with accurate cycle timing.
Another Renode virtual-time 1 ms benchmark would not verify the observed
500 Hz CAN irregularity. We therefore leave the Renode suite unchanged.

The current SIL Kit simulator records additional HOST-ONLY sources on the
real `PathExecutionState` code path:

- `path_commands`: trajectory-specific six-axis send attempts (excludes
  teaching, homing and other CAN transactions).
- `path_feedback_wait`: feedback from at least one of the six motors
  has not advanced, but the next 2 ms target is not yet due.
- `path_feedback_late`: feedback has not advanced and the next target
  is ALREADY due. This directly indicates a feedback-gated delay.
- `path_period_wait`: feedback gate is satisfied; next target is
  not due yet by the 2 ms sample clock.

The feedback check precedes the period check in the existing state machine,
so counters are exclusive first-blocking-reason **polls** on each Supervisor
iteration, not numbers of distinct commands missed. These counters do not
measure per-axis TPDO wire arrival, nor does any result justify removing the
all-six-axis feedback safety gate.

Only ONE new flash-backed end-to-end run is necessary. Previous long-run
baseline.csv/flash.csv remain unchanged:

```bash
git fetch origin
git switch fix/path-execution-timing-diagnostics
cmake -S . -B build-can-silkit
bash scripts/run_timing_benchmark.sh --path-only --no-plots
```

Results:
`.simulation-logs/timing-benchmark/path_diagnostics.csv` and
`.simulation-logs/timing-benchmark/path_diagnostics-analysis/timing_report.md`.
If `path_commands` has no samples, do not claim cadence was measured:
verify that the scenario actually entered path-follow/preview execution.

Decision guide:
- Many `path_feedback_late` polls suggest all-axis TPDO freshness is a
  substantial contributor to command delay (still investigate CAN receive
  processing, motor response and scheduling).
- Few feedback-late polls but long `path_commands` intervals suggest
  missed Supervisor wakeups, other control gating, or test-driver pacing.
- Zero path-specific events means this test did not reach trajectory-follow
  logic; no feedback attribution is possible.

## Raw CSV contracts

**Current recorder (schema v2):**

`schema_version,source,kind,sequence,period_us,start_us,finish_us,duration_us,interval_us,execution_budget_overrun,success,dropped_total`

- `start_us` / `finish_us`: timestamps from CLOCK_MONOTONIC in microseconds
  since instrumentation startup.
- `duration_us`: elapsed wall time spent executing the measured function.
  This includes time preempted by WSL/POSIX scheduling.
- `interval_us`: elapsed wall time since the previous execution started.
- `execution_budget_overrun`: for *periodic* tasks only, 1 if
  `duration_us > period_us`. This is NOT an actual release-to-deadline miss
  and says nothing about when the FreeRTOS task was released.
- `success`: whether the instrumented callback returned success.
- `dropped_total`: number of ring-buffer logging records lost due to
  saturation; a nonzero count makes observed percentiles incomplete.
- `supervisor`: nominal 1000 us execution budget; `hmi`: nominal
  10000 us execution budget. `sim_inputs`, `prefetch`,
  `validation`, `flash_storage` and `can_targets` have no periodic
  execution-deadline classification in this host instrumentation.

**Old schema v1:** previous logs include `scheduled_us`,
`start_lateness_us`, `finish_lateness_us`, and `missed_deadline`.
The old deadline calculation compared every cycle to
`first_start + n * nominal_period`, so it accumulated POSIX/WSL
scheduler/tick drift and produced misleading near-100%-miss counts.
**Do not cite or interpret those four old values as actual misses.**
The updated Python analyzer ignores them and recomputes trustworthy
descriptive statistics from `start_us`, `duration_us`, and `period_us`.
You do not need to rerun your two ~67-second scenarios.

## Reading the output

- `P99 execution time`: 99% of **observed** calls were this fast or
  faster. Not a worst-case execution time proof.
- `Max execution time`: largest measured wall duration in the run.
- `Execution-budget overruns`: number of periodic task bodies that
  actually took longer than their 1 ms / 10 ms budget.
- `Mean/P99 start interval`: actual spacing between consecutive recorded
  invocations; a better host wake/pacing diagnostic than accumulating
  lateness from the beginning of the test.
- `Intervals over 1.5x nominal period`: **heuristic gap counter**,
  not a proven hard deadline-miss count. For the Supervisor, this is
  a start gap exceeding 1500 us; for HMI, exceeding 15000 us.
- `CAN target-command bursts`: gaps between host API calls sending
  six RPDO4 frames + SYNC; this does NOT measure physical CAN wire timing.

Compare `baseline.csv` and `flash.csv` on execution P99/max,
observed start spacing, and overruns. Any deterioration is descriptive
under the measured PC load; repeat runs are recommended. The recorder
does **not** have the FreeRTOS tick ISR wall-clock timestamp needed to
attribute a delayed wake exactly to RTOS release lateness.

## Limitations / sign-off

This is **measurement instrumentation**, not a 1 ms deadline certification.
Logging itself has a cost, WSL and POSIX scheduling can introduce artifacts,
and both M7/M4 roles currently share one host RTOS scheduler. The independent
background logger may lose a few last rows if the harness SIGTERMs the
process; it flushes roughly every 5 ms while running.

Before MCU sign-off: capture DWT cycle-counter timing on the 480/240 MHz
(or actual configured) Cortex-M7/M4 clocks, use GPIO/logic analyzer traces,
measure real CAN frame arrival/worst-case jitter, confirm SRAM/MPU/HSEM
coherency and verify priority inversion/interrupt masking/flash contention.
Use the same metric field definitions, but do not import host numbers as
hardware execution deadlines.
