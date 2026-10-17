# Stage 4F1 — Hand-guiding host timing & tracking

**Scope:** SIL Kit / POSIX host measurements only. Not a 500 Hz STM32 proof, not a physical safety assessment or human interaction stability test.

Guidance trace: `/home/sara/freertos-pc-test/.simulation-logs/stage4f1/guidance.csv`  
Host timing trace: `/home/sara/freertos-pc-test/.simulation-logs/stage4f1/timing.csv`

Observed duration: 3.886 s; valid pre-dispatch feedback/reference pairs: 1943; rejected: 0; fault rows: 0.
Host trace missing cycle numbers: 0; nonmonotonic pairs: 0; max feedback age: 4000 us.

## Joint tracking observations

| Axis | RMS qref-qactual (rad) | P95 abs (rad) | Max abs (rad) | qref range (rad) | actual range (rad) |
|---|---:|---:|---:|---:|---:|
| J1 | 0.0014545 | 0.0032040 | 0.0033583 | 1.0038196 | 1.0004623 |
| J2 | 0.0007651 | 0.0010192 | 0.0010230 | 0.6577161 | 0.6577173 |
| J3 | 0.0005184 | 0.0007800 | 0.0007853 | 0.4293978 | 0.4293996 |
| J4 | 0.0002968 | 0.0004750 | 0.0004795 | 0.2284907 | 0.2284903 |
| J5 | 0.0014533 | 0.0032037 | 0.0033550 | 1.0030645 | 0.9997106 |
| J6 | 0.0000022 | 0.0000044 | 0.0000058 | 0.0007907 | 0.0007900 |

These comparisons use feedback sampled **before** the corresponding joint reference was dispatched. The sample may come from a prior motor update and the command may subsequently be rejected. Do not interpret as settled accuracy or controller bandwidth.

## Timing observations

| Source | Rows | P95 body (us) | P99 body (us) | Max body (us) | P99 interval (us) | Overruns | Dropped log rows |
|---|---:|---:|---:|---:|---:|---:|
| supervisor_2ms | 4763 | 55.0 | 79.4 | 224.0 | 2563.8 | n/a | 0 |
| guidance_observer | 1946 | 21.0 | 28.0 | 54.0 | 2443.4 | n/a | 0 |
| guidance_dispatch | 1945 | 23.0 | 37.6 | 148.0 | 2443.8 | n/a | 0 |

The host clock measures observed task-body duration and start intervals; it cannot identify true RTOS release times or MCU deadline misses. Logger drops and missing sequences must be reviewed before using percentiles as representative.

## Plots
![J1_reference_vs_feedback](J1_reference_vs_feedback.png)
![J1_pre_dispatch_error](J1_pre_dispatch_error.png)
