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

## Raw CSV contract

\`schema_version,source,kind,sequence,period_us,start_us,finish_us,duration_us,interval_us,scheduled_us,start_lateness_us,finish_lateness_us,missed_deadline,success,dropped_total\`

- \`start_us\` and \`finish_us\`: microseconds since instrumentation startup
  (CLOCK_MONOTONIC); duration is time spent within the instrumented routine.
  It includes scheduling preemption and instrumentation overhead.
- \`supervisor\`: complete 1 ms control-loop body, excluding vTaskDelayUntil.
  Period and nominal deadline both 1000 us.
- \`hmi\`: 10 ms period and deadline.
- \`sim_inputs\`, \`prefetch\`: best-effort RTOS tasks (their vTaskDelay(1)
  does **not** imply a fixed absolute 1 ms periodic release).
- \`validation\`: one numerical validation step in the lower-priority worker.
- \`flash_storage\`: one M4-role task step, including actual simulated NOR
  callback time when an operation is queued. This is still a POSIX single-
  scheduler M4-role simulation, not the physical STM32 M4.
- \`can_targets\`: call duration of SIX RPDO4 target frames plus SYNC.
  The events can be absent when no motion command is sent.
  No pretend 2 ms deadline is assigned to intermittent motion episodes.
- \`scheduled_us\` = first observed release + sequence * nominal period.
- \`start_lateness_us\` = actual start - scheduled start, signed.
- \`finish_lateness_us\` = actual finish - (scheduled start + period).
- \`missed_deadline\` is 1 only if a periodic task finishes after that
  nominal period boundary. Nonperiodic rows leave those columns empty.
- \`interval_us\` = spacing between two recorded starts from the same source.
- \`success\` = whether the measured call returned success (for CAN/flash
  operations); not necessarily robot operation success.
- \`dropped_total\` = cumulative logger ring saturation count. A nonzero
  count means the observed percentiles may be biased/incomplete.

## Reading the output

- \`P99 execution time\`: 99% of *recorded* calls finished no slower than
  this duration; it is not a true worst-case execution-time proof.
- \`max execution time\`: maximum **observed** wall time in this run.
- \`P99 start lateness\`: 99th percentile of release delay relative to the
  first cycle's nominal FreeRTOS schedule.
- \`deadline misses\`: observed completion later than its nominal next release.
  Even zero misses on a host run is not an STM32 timing guarantee.
- \`CAN burst jitter\`: differences between successive CAN target-command
  **host calls** inside runs of >=10 commands with gaps <=5 ms.
  Do not equate these with CAN bus frame delivery or command-to-drive latency.
  Hardware CAN 2.0 bit timing is analyzed independently.
- Compare \`baseline\` and \`flash\` to see whether background storage changes
  Supervisor execution duration, lateness, or misses. Compare under the
  same WSL/host load, clock and logging configuration.

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
