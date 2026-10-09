# SPSC trajectory prefetch: staged CAN/SIL Kit integration

This patch introduces a 512-sample SRAM SPSC ring and a 256-sample staging area.
The supervisor's execution `read_sample` callback only reads from the SRAM ring; a priority-2
FreeRTOS task calls the bulk source-reader callback when occupancy drops.

## Verify component in isolation

```bash
./scripts/test_trajectory_prefetch.sh
```

## Wire into PC SIL Kit simulator

Back up the two touched files first:

```bash
cp Simulation/supervisor_silkit_main.c Simulation/supervisor_silkit_main.c.before-prefetch
cp supervisor_silkit_sim.cmake supervisor_silkit_sim.cmake.before-prefetch
python3 scripts/apply_trajectory_prefetch_sim.py
cmake -S . -B build-can-silkit
cmake --build build-can-silkit --target supervisor_silkit_sim -j "$(nproc)"
ctest --test-dir build-can-silkit -R '^supervisor_silkit_sequence$' --output-on-failure
```

A possible cleanup is restoring backups and deleting `Simulation/Storage/trajectory_prefetch.{c,h}`.
The script refuses to edit source if the expected baseline markers have changed.

## Semantics and limitations

- The existing SIL Kit simulator uses large `RamValidatedStorage`, *not* flash.
  The worker is connected to a RAM bulk-reader in the simulator only.
- Validation commits prefill up to 512 samples *before execution*. This does a
  bounded RAM copy, not a flash read, and is carried out in the validation path.
- Worker owns batch refills; supervisor owns sample consumption, with lock-free
  atomically published counters. `take` never calls a storage callback.
- A source failure or buffer underrun causes `read_sample` to fail and triggers
  the existing `PATH_EXEC_ERR_STORAGE_READ` path. Verify the higher-level drive
  stop response separately before any physical robot application.
- The producer is suspended during validation `begin`/`abort` and resumed only
  after commit/preload. The prepare API REQUIRES the worker and consumer quiescent.
- The worker automatically re-arms once it observes all samples consumed for
  subsequent executions of the same committed trajectory.
- Production FLASH: STM32 QSPI bulk-read adapter, DMA/cache coherency if used,
  real FreeRTOS timing, MCU/M4 coordination, hardware safety response, watchdog,
  and dual-core ownership still require separate integration and tests.
- There is no performance guarantee from PC SIL Kit timings or synthetic clocks.
