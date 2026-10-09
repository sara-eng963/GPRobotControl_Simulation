# Batch 4 — automated Renode STM32H7/QSPI functional tests

## Scope and observed run

The five existing `.resc` firmware tests ran on Renode **1.17.0** in the project WSL Ubuntu environment on 2026-10-09. The user's headless run of `scripts/run_renode_qspi_tests.py` reported **5/5 PASS, 54 asserted 32-bit result words**. The assistant reviewed the implementation and assertion mapping but did not independently execute that WSL run.

| Firmware/test | Checks | Functional interpretation |
|---|---:|---|
| `qspi_jedec_test` | 3 | STM32 QUADSPI indirect JEDEC transfer yields `EF 40 20` and firmware reports PASS |
| `qspi_rw_test` | 9 | Legacy 24-bit `02h/03h/20h` erase/program/readback sequence and final test pattern |
| `qspi_pv_sample_test` | 13 | Legacy 24-bit transfers round-trip four 24-byte six-joint `PvExecutionSample` records |
| `qspi_storage_backend_test` | 13 | Production storage/flash API writes, commits, reloads and validates 12 samples in simulated NOR |
| `qspi_streaming_test` | 16 | Production storage API to **sequential** `ValidatedStreamBuffer`, 1536 samples, 512-sample FIFO, 256-sample refills, no underruns in unpaced sequential execution |

### Explicit exclusions

- Tests 7.2 and 7.3 use **legacy three-byte addresses and opcodes**, not the W25Q512JV production driver. They test the Renode QSPI peripheral and record representation.
- Test 7.5 uses `ValidatedStreamBuffer`, **not** the concurrent `TrajectoryPrefetch` worker or the FreeRTOS Supervisor. Its reported zero underruns does not establish a 500 Hz deadline.
- The supplied `SPI.GenericSpiFlash` is not a Winbond W25Q512JV behavioral model: earlier probes showed immediate program/erase completion, NOR 0-to-1 violations, no page wrapping, and unsupported `66h/99h` reset. Therefore **Renode 5/5 PASS does not supersede the independent protocol/power-cut host tests**.
- Synthetic virtual-time polling, emulated FIFO behavior, memory-mapped status and built firmware do not validate physical pin configuration, the 50 MHz read limit, VCC/reset/chip select waveforms, real latency or power-loss robustness.
- Flash-backed CANopen/SIL Kit integration, deterministic 500 Hz refill/consumer timing, board measurements, and the storage-fault actuator-stop policy remain separate work.

## How tests run

The Python runner compiles each freestanding Cortex-M7 firmware through its existing Makefile. It places ELF, BIN, scripts and logs under an isolated `/tmp/batch4-renode-*` directory (unless `--workdir` is explicitly supplied), copies the original Renode setup commands, substitutes the generated ELF path and then appends labeled `sysbus ReadDoubleWord` probes. It verifies each required 32-bit value, including magic and `PASS`, and rejects missing/duplicate/mismatched results, timeouts and nonzero Renode exit codes. It never edits the repository's `.resc` files or the preexisting untracked build directories.

Execute manually from repository root:

```bash
python3 scripts/run_renode_qspi_tests.py --self-test
python3 scripts/run_renode_qspi_tests.py
python3 scripts/run_renode_qspi_tests.py --test qspi_storage_backend_test
```

Configure with CMake option `ENABLE_RENODE_TESTS=ON` (the default). When Python3, `arm-none-eabi-gcc`, `arm-none-eabi-objcopy`, `make` and Renode are available, five tests named `renode_qspi_*` and a runner self-test are registered with CTest. The Renode executable is searched on PATH and in `$HOME/renode_portable`, and can be supplied via `-DRENODE_EXECUTABLE=/path/to/renode`.

```bash
ctest --test-dir /tmp/your-isolated-build -R '^renode_' --output-on-failure
```

If dependencies are missing, the five actual Renode firmware tests are **not registered**; the CMake configure summary states that explicitly. No fake or vacuous PASS tests are added. `ENABLE_RENODE_TESTS=OFF` disables registration without preventing other builds/tests. Each Renode case uses a separate temporary directory and `RUN_SERIAL` under CTest to reduce emulation contention.

## Acceptance gate

Batch 4 is considered fully integrated when a **fresh CMake configure/build**, complete CTest suite including registered `renode_qspi_*`, and previous flash/prefetch/FIFO scripts pass on the same tested Git revision. Passing the stand-alone runner alone proves the first functional gate, not the CTest-integrated build gate.
