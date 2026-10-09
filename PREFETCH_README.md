# Trajectory prefetch ownership and execution readiness

The CAN/SIL Kit Supervisor consumes six signed 32-bit joint targets per sample
at the established 2 ms period. A priority-2 worker refills the 512-sample SRAM
ring in batches of at most 256 through a separate staging array. The current
source is `RamValidatedStorage`; flash-backed integration remains separate work.
Execution and preflight callbacks never read storage, take mutexes or wait.

## Ownership and publication

One task owns control and consumption: the Supervisor. One worker owns refills.
`written` release publishes sample bytes; the consumer acquires it. `consumed`
release publishes reusable slots; the producer acquires it. Fault and armed flags
also use atomic access consistently. Compile-time assertions require lock-free
32-bit and flag atomics. This is a single-core contract; STM32H745 M4/M7 memory
and cache coordination is not established here.

The worker owns `next_flash_index` and `refill_batches` while running. The consumer
owns `underruns`; the worker resets it only after consumer release. Inspect these
non-atomic statistics only under their ownership or while STOPPED. Configuration
and source data are immutable until acknowledged worker quiescence.

## Lifecycle

| State | Allowed behavior |
|---|---|
| STOPPED | Worker accesses no source/configuration. Control may prepare/disarm and synchronously prefill. |
| REFILLING | Worker refills. Fresh consumer acquires only after startup prefill. |
| EXECUTING | Consumer owns the stream through final feedback, retraction and PAUSED. Worker cannot reset. |
| REPLAY_PENDING | Consumer explicitly released; worker resets and prepares automatic replay. |
| STOP_PENDING | Worker acknowledges STOPPED after its previous read/publication returns. |

Initialization starts STOPPED. Validation calls `request_stop`, waits for
`stopped`, changes storage, calls `prepare`, and performs initial RAM prefill
before `resume_worker`. Keeping prefill STOPPED avoids two producers. The wait
is confined to validation, never trajectory execution.

The worker calls `worker_step`, never resets from `consumed == total_samples`,
and never inspects stream fields directly. Stop during replay reset takes priority:
compare/exchange prevents overwriting a pending stop. There is no separate stale
pause acknowledgement to confuse consecutive stops.

`begin_execution` acquires ownership after `min(total_samples,512)` samples are
buffered at index zero. Short trajectories require only their actual sample count.
Rechecks/resume require the exact current index and the next sample, if one remains.
A failed source read blocks readiness even when older samples remain buffered.
Rejected takes never advance consumption.

Optional `PathExecutionServices.stream_ready` checks SRAM during prerequisites
and immediately before wire-feed configuration. Missing/failed prefill reports
`PATH_EXEC_ERR_TRAJECTORY_NOT_READY`; runtime read failure/underrun uses the
existing `PATH_EXEC_ERR_STORAGE_READ` path. NULL callbacks retain the existing
behavior for other execution-service users.

The Supervisor retains ownership across PAUSED. On execution exit, including
external fault/E-stop and abort transitions, the same task stops dispatching
execution before calling `end_stream`. The worker automatically re-arms. No
manual re-arm or blocking prefill is needed in the consumer. Interrupted
executions restart at sample zero; PAUSED resumes at the same index.

## Verification

Scripts use fresh `/tmp` directories by default, preserving existing builds:

```bash
./scripts/run_host_flash_tests.sh
./scripts/run_w25q_fifo_mock_tests.sh
./scripts/test_trajectory_prefetch.sh
```

Output overrides are `W25Q_TEST_BUILD_DIR`, `W25Q_FIFO_TEST_BUILD_DIR`, and
`PREFETCH_TEST_BUILD_DIR`; use fresh directories. `PREFETCH_SANITIZERS` defaults
to `address,undefined` and can select `thread`.

In the managed WSL environment, LeakSanitizer fails under ptrace; verified
ASan/UBSan runs use `ASAN_OPTIONS=detect_leaks=0`. TSan initially fails with
`unexpected memory mapping`, including non-PIE builds. Disabling ASLR only for
the test process allows TSan to run:

```bash
setarch "$(uname -m)" -R env PREFETCH_SANITIZERS=thread \
  TSAN_OPTIONS=halt_on_error=1 ./scripts/test_trajectory_prefetch.sh
```

CMake/CTest includes existing memory tests and new concurrency/execution tests.
File-backed tests have separate build-tree working directories. Configure a full
SIL Kit build without overwriting existing build files:

```bash
prefetch_build_dir=$(mktemp -d /tmp/prefetch-regression.XXXXXX)
cmake -S . -B "$prefetch_build_dir" -DENABLE_SILKIT=ON \
  -DSILKIT_ROOT="$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"
cmake --build "$prefetch_build_dir" -j "$(nproc)"
ctest --test-dir "$prefetch_build_dir" --output-on-failure
```

SIL Kit needs local TCP/Unix/UDP socket access. Sandbox socket denial is an
environment failure, not a passing simulation or a protocol defect.

Deterministic pthread tests hold reads in flight across stop/restart, retain
final-sample ownership, inject read failure/starvation and reject wrong indices.
Concurrent stress checks all six joint values and indices over 392476 samples,
12 long executions and 32 stop/reconfigure cycles. Execution tests cover final
feedback, short prefill, preview/production, pause/resume, HOME abort/restart,
runtime faults and wire-feed gating. The six-motor sequence also aborts active
production with HOME and repeats production with the same validated trajectory.

## Remaining limits and separate policy review

Batch 1 does not establish physical flash timing, 500 Hz scheduling deadlines,
electrical behavior or power-loss integrity. Timing, flash-backed CANopen
integration, persistence and Renode validation remain later batches.

The current storage-fault policy disables wire feed and transitions to FAULT.
The simulator's FAULT safe-output callback only disables wire feed; it does not
issue drive hold/stop, and Supervisor heartbeat continues. This behavior is
preserved and explicitly tested, not certified as a physical stopping policy.
Any change requires separate control-policy review.
