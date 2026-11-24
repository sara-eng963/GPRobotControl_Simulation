# STM32 memory benchmark

Run in WSL/Linux from this checkout (Python standard library, ARM GCC/newlib,
native C compiler, CMake/CTest, make, and the installed portable Renode):

```sh
python3 scripts/memory_benchmark.py
```

Every run creates a new `build/memory-benchmark/<UTC timestamp>/` directory.
`report.md` and `results.json` summarize the evidence. `commands.json` records
the runner's exact argv/cwd/timeout/exit code and log for each subprocess.
Production `commands.txt` and diagnostic `variant_commands.txt` retain compiler
commands. `inputs.json` hashes the working-tree code/header/build inputs;
`artifacts.json` hashes generated binaries, maps, dumps and logs. Dirty-tree
builds are intentional and identified; the tool never resets or cleans Git.

```sh
# A CI purchase-qualification gate must remain nonzero without full coverage.
python3 scripts/memory_benchmark.py --require-full-runtime
# Build and linker audit only; no runtime claim.
python3 scripts/memory_benchmark.py --static-only
# Independent parser/accounting/evidence regressions.
python3 -m unittest discover -s scripts/tests -p test_memory_benchmark.py -v
```

Exit codes: 0 means the requested ARM application evidence contract passed (or explicitly
requested static-only processing completed); 1 means an execution, build, test,
parser or accounting failure; 2 means the full-runtime gate remains unmet.
The default outcome is `PASS_ARM_APPLICATION_WORKLOADS` only after real ARM
application assertions pass. The JSON retains `full_runtime_status: UNVERIFIED`
for the original dual-core physical composition. Unknown requirements are never zeros.
An error after output creation still generates a failure report and retained logs.

## What the default command does

1. Records identity and executes tool regression/negative tests.
2. Builds both real Release source graphs with `Firmware/Tests/build_stm32.py`
   into the run directory, then audits each ELF and its linker map. Existing
   `build/stm32` products are neither reused nor overwritten.
3. Attempts bounded execution of both unmodified production ELFs, separately,
   retaining exact final PCs/source locations and raw counters. Expected missing
   platform startup functionality is classified `STARTUP_BLOCKED`; monitor or
   process failures fail the benchmark rather than becoming startup evidence.
4. Replays the production compile commands into separate diagnostic directories.
   CM7 omits only its second D2 ready poll in a generated `main.c`. CM4 omits
   STOP entry in its generated `main.c`; it loads a 32 KiB SRAM4 snapshot made
   by the paused CM7 diagnostic. CM4 still executes the real shared-memory ABI
   validation before scheduling. Its D2 0x10000000 aliases map the same emulated
   SRAM objects as 0x30000000, and its core is Cortex-M4.
5. Runs both diagnostic ELFs, collecting three paused snapshots, task creation
   records, actual task switch-ins, heap counters and stack fill patterns.
6. Builds a separate CM7 negative image: one arena-sized allocation and one
   oversized task allocation must fail. Exactly two allocator failures and one
   task-creation failure are expected while task scheduling continues.
7. Builds and executes the asserted ARM application composition described below,
   followed by a separate reader-failure negative image. Executes existing
   production-state, Supervisor, CANopen, CM4 storage/playback,
   fault, F/T and guidance fixtures, real POSIX FreeRTOS IPC, and five standalone
   Renode QSPI/storage/streaming tests. Each is labeled with its actual execution
   domain. Native test memory is never presented as STM32 memory.
   The established connected P05 runner also executes its virtual six-axis
   teaching/validation/durable-playback/fault chain. Its outside-checkout build
   policy is honored using a temporary directory; all outputs/logs are copied
   into the benchmark's `workloads/connected-evidence` before that directory is
   removed. Its own existing evidence gate checks sample comparisons and faults.
8. Checks source immutability, validates evidence, and writes reports/artifact hashes.

## Static accounting

The audit uses ARM `objdump -h`, `nm -S -a`, and the exact generated linker maps.
Only nonempty ALLOC sections count toward RAM. LOAD section LMAs in Flash count
toward Flash, including `.data` initializers; NOLOAD runtime RAM does not become
Flash consumption. Flash payload is an interval union, image span includes gaps,
and the combined fixed-address span includes the gap before CM4 at 0x08100000.
The two 1 MiB Flash linker partitions are retained in JSON.

CM4's D2 aliases normalize to physical addresses. SRAM1, SRAM2 and SRAM3 are
reported independently, even when a section crosses their contiguous boundaries.
ITCM, DTCM, AXI and SRAM4 are also distinct. Every section must fit exactly one
linker region and valid physical banks. Cross-core storage overlaps fail.
CM7 owns the SRAM4 objects; nonempty CM4 allocations in those shared partitions
fail. Partition lengths come from the maps and count once in the reservation
union, independent of their currently occupied bytes. Occupied linker-region
spans also retain inter-section placement padding (including CM4's four-byte
gap); unused space after the last section remains free unless reserved.

The FreeRTOS `ucHeap` arena already resides in `.bss`; runtime heap consumption
is a subset of the static section allocation. Static task stacks are also already
included. Neither is added again to SRAM totals. `.bss` also contains storage for
inactive services; linker allocation is not proof that their tasks ran.

The linker reserves `.\_user_heap_stack` at the low end of each core's main RAM,
but startup MSP actually points to `_estack`. The audit separately reserves
`_Min_Stack_Size` below `_estack` and rejects collision with section storage.
Committed bytes therefore conservatively include both linker obligations. The
bank span exposes the current high-address layout. It is not a prediction of
minimum capacity after redesigning the linker layout. Neither `_Min_Stack_Size`
nor `_Min_Heap_Size` is a measured runtime maximum. C-library malloc and interrupt
MSP peaks remain unmeasured.

## Test-only instrumentation

`Simulation/Renode/MemoryBenchmark/trace.c` and `trace.h` enter only the derived
images. Production firmware, configuration and startup source files are unchanged.
The generated FreeRTOS configuration enables bounded trace hooks. A generated
copy of `tasks.c` adds one observer at the end of `prvInitialiseNewTask` to retain
the exact requested stack depth. This avoids inferring capacity from an aligned
`pxEndOfStack`, which can lose a word. The TCB layout is unchanged.

The observer uses a fixed 32-record buffer, no heap, transport, logger, or extra
task. It captures task name, TCB, stack base, requested bytes, static/dynamic
ownership, switch-ins, and deletions. External `xTaskCreate`/`xTaskCreateStatic`
calls are linked through failure-counting wrappers. This scope matches both
current core graphs; restricted task APIs and future internal creation APIs would
need additional wrappers. Trace overflow or unexpected failure counts fail.

Heap values come from the real target `heap_4.c` counters. `pxEnd` must confirm
heap initialization before reporting a peak. Arena size minus minimum-ever-free
includes heap headers, alignment and initial sentinel loss. A failed request does
not count as successfully allocated memory. The negative test verifies this
instrumentation independently of the normal zero-failure acceptance criterion.

Stack scans use exactly the downward-stack FreeRTOS 0xA5-fill convention, rounded
down to 32-bit `StackType_t` units, over the entire requested stack allocation.
Renode pauses the CPU before dumping RAM, so reading consumes no target stack or
heap. Deleted tasks remain in the creation history but have no claimed final
watermark. Fill scanning cannot bound unwritten stack frames or unexecuted paths.
Instrumentation itself adds storage/code and execution costs; the report identifies
each diagnostic ELF and its memory difference from production.

## Dual-core feasibility and coverage boundary

The installed Renode 1.17 STM32H7 platform describes one Cortex-M7 and one NVIC.
It includes QSPI, MCAN, SRAM banks and a hardware semaphore peripheral, but its
presence is not sufficient evidence of STM32H745 dual-core fidelity. The observed
unmodified CM7 boot fails the CPU2 wake-up handshake; unmodified CM4 does not
establish initialized shared-memory startup or a running scheduler by itself.
The installed model/platform hash and the actual boot logs are retained per run.

The upstream RCC implementation declares D1CKRDY/D2CKRDY as tagged fields, rather
than modeling H745 D2 STOP/wake transitions. The upstream semaphore model has a
single IRQ and current-CPU identification; that needs verification/extension for
the H745 per-core interrupt masks and EXTI wake routing. A trustworthy coupled
model also needs per-core NVIC/System Control Space registration, shared physical
SRAM/aliases, consistent RCC/HAL timebases, peripheral interrupt ownership and
repeatable IPC round trips. Merely adding another CPU or changing a ready bit
would not validate this chain. This implementation therefore makes no claim of
complete H745 emulation, and does not introduce an unvalidated dual-core model.

References inspected for feasibility (upstream may differ from installed binary):

- [Renode platform registration and core-specific bus registrations](https://renode.readthedocs.io/en/latest/advanced/platform_description_format.html)
- [STM32H7 RCC implementation](https://github.com/renode/renode-infrastructure/blob/master/src/Emulator/Peripherals/Peripherals/Miscellaneous/STM32H7_RCC.cs)
- [STM32H7 hardware semaphore implementation](https://github.com/renode/renode-infrastructure/blob/master/src/Emulator/Peripherals/Peripherals/Miscellaneous/STM32H7_HardwareSemaphore.cs)

There is a second independent limit in the original firmware composition: no
`board_supply_commissioned_config` implementation is linked, so the production
Supervisor/CANopen startup remains inactive. F/T transport also lacks its physical
provider, and CM4 validation/playback lacks its two durable storage providers.
Those closed gates are intentional production behavior and are not overridden.
The isolated application composition below supplies explicit deterministic
providers without changing those production definitions or gates.

Startup-only ARM task memory evidence remains separate from application results.
Renode cannot establish hardware timing or guarantee worst-case memory. MCU
purchasing still needs remaining workload coverage, interrupt-stack
characterization and an engineering margin policy.

## Asserted ARM application composition

```sh
# Fresh production builds, tool tests, ARM application and ARM negative test:
python3 scripts/memory_benchmark.py --application-only
# Bound simulated execution and wall execution independently:
python3 scripts/memory_benchmark.py --application-only --application-seconds 60 --application-timeout 600
```

The same application runs in the default command. `report.md` is a concise
application report; `diagnostics.md` retains detailed static/startup evidence.
`application/application-results.json` retains partial functional evidence even
if a workload fails. Only `ARM_APPLICATION_WORKLOADS_MEASURED` can satisfy the
application gate; scheduler flags alone cannot do so. The unchanged original
dual-core topology still has a separate unmet `--require-full-runtime` gate.

The original optional providers excluded the following production tasks:

| Task/module | Original exclusion | Isolated application action |
|---|---|---|
| Supervisor / CANopen / control states | No commissioned board configuration | Start real `supervisor_task_init/start` with explicit virtual-device configuration |
| StorageValidate / geometry / IK | Supervisor inactive; no backend | Enable real lower-priority worker, fixed geometry workspace and async storage proxy |
| FT_RX / serial / guidance | No vendor decoder or UART provider | Real service task, actual F/T parser/pipeline and test-only wire decoder; wrap only DMA transport functions |
| NorValidate / durable storage / playback | Missing CM4 backend and reader | Recompile real CM4 service for M7; existing NOR model plus real W25 driver and validated-storage implementation |
| CM4 IPC and M4Progress | Separate-core startup unavailable | Real service loop, real message buffers, real HELLO/challenge/HMI checks, existing progress task |

`application_providers.c` and `application_scenarios.c` are test-only. They enter
only a derived image; production source, projects and configuration stay intact.
No private Supervisor state is injected. No host Supervisor fixture, `longjmp`,
fake FreeRTOS queues, replacement state machines, or simplified numerical engine
is linked. `compiled_sources` records exact source paths/hashes in results.json.

The actual CM7 tasks execute with their original stack reservations and `-Os` ARM
flags. The CM4 services are recompiled as Cortex-M7 and run on the same kernel;
their measurements are explicitly **relocated service measurements**, not CM4
machine-code or dual-kernel stack guarantees. The task trace includes all ten
created tasks. The generated defaultTask, BenchScenario and M4IPC entry wrapper
are identified as test composition overhead; the original CM7 defaultTask
progress-observer loop is preserved and runs with the active Supervisor.
The five production application
entries and kernel idle/timer entries run unchanged.

Hardware providers use the existing AVATAR CANopen node simulator, real six-axis
production CANopen coordinator, and deterministic perfect-following feedback
after SYNC. They do not simulate mechanics or create physical authorization.
The finite virtual CAN FIFO drops arriving frames once FAULT has stopped normal
CAN polling, as a bounded device would. Overflow during normal work still fails.
The safety/session/approval callbacks supply explicit synthetic evidence; all
production freshness, ownership, challenge and release/press checks remain active.
The harness waits for actual CM7 HMI acknowledgements, retrying requests dropped
by rotating challenge generations. Queue acceptance alone is never execution.

The application executes full CANopen commissioning from BOOT, HOMING and IDLE,
two teaching/validation cycles, preview playback twice, production playback once,
pause/resume, RESET and repeated state transitions. The real observation-only
hand-guiding adapter runs inside Supervisor using samples published by FT_RX.
It never dispatches candidates, as production explicitly prohibits that path.
Taught endpoints come from deterministic virtual manual movement.

Geometry/IK produces actual trajectory samples. StorageValidate fills the real
64-entry async queue; NorValidate writes and commits those bytes using the real
W25Q and validated-storage code. Playback CRC-verifies the committed artifact
before prefilling the real 512-sample ring. Observation wrappers count successful
sample takes and skips independently of metadata. Completion asserts the actual
count matches the expected artifact samples. Both repeated cycles must restore
the same free-heap baseline.
The scenario caches sample zero from that same committed artifact between
operations for the Approach board callback; bulk validation/playback storage
remains owned by the real NorValidate task.

Capacity scenarios fill the 512-point raw geometry workspace, the actual FT_RX
512-byte ring (with overflow rejection and service shutdown), the 32-segment
Teaching draft (attempt 33 must be rejected), and the actual 32-entry Supervisor
queue. Queue overload must execute the real safe-output/fault logic; RESUME must
not restart motion. E-stop must take precedence. A separate ARM image injects a
reader failure only at the storage-provider boundary; the real playback must
fault after successful numerical validation/commit. That image is expected to
fail functional completion and cannot be reported as a successful workload.

NOR device bytes use the existing Renode external memory at 0x90000000 (480 KiB
for two test slots); they are not MCU SRAM or `.bss`. A generated-image-only MPU
region grants access to that backing store while preserving production regions.
The model's own metadata/unused inline buffer is still in benchmark `.bss` and
is honestly included as test overhead. The virtual NOR capacity is 10000 samples;
full artifact capacity and power-loss recovery remain unexercised by this ARM run.

Three paused snapshots read the real heap counters and all task stack fill
patterns. A bounded Renode-side helper advances in one-second quanta until
terminal status, then samples again; it records the **actual** simulated elapsed
time, not the maximum requested budget. It cannot create a target pass status.
Allocator failures, wrong task sets/reservations, zero stack margins, incomplete
phase bits, missing actual counters or reader failures reject application success.
The separate `_sbrk` observer records C-library peak break growth.

Unverified ARM paths include circular/arc geometry, other IK cases, heartbeat and
feedback loss, stale/forged safety samples, peer reboot/loss, hardware deadlines,
full artifact capacity, storage power loss and exhaustive fault interleavings.
The report lists these explicitly. Native tests remain supporting evidence only.
No measured value is converted into a minimum MCU requirement or a worst-case
bound. See `scripts/membench/application.py` for the executable acceptance contract.
