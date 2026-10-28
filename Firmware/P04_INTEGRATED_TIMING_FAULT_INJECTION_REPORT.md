# P04 — Integrated Timing and Fault-Injection Pre-Hardware Review

**Disposition: INCOMPLETE / UNVERIFIED.** This is a source-reviewed **candidate** patch package, not a verified production integration. No ARM device, WSL checkout, FreeRTOS POSIX kernel checkout, or toolchain was available in the authoring container. No target timing measurements or physical stopping tests were performed. The baseline branch and files were read from the connected GitHub repository. All tests described below are supplied as executable source, but their execution against the complete P03 checkout is **NOT RUN** until applied and verified in WSL.

## 1. Baseline and environment

- Canonical repository: `sara-eng963/GPRobotControl_Simulation`, only branch `sara/can-replacement`.
- Verified canonical HEAD: `916e7a85fd2d20f445fe513901f8d149260c09cb`, message `P03: implement runtime safety freshness and CM4 liveness`; remote tree `bc552fe74fd6ebe50d5785338181ebaf62a20280`.
- Source files inspected from that exact commit: production Supervisor `.c/.h`, `supervisor_timing_quality.h`, CM7 timer composition, production CM4 IPC service, `runtime_evidence.h`, `supervision_progress.h`, both IPC transport copies, STM32 CANopen master/header, desktop CANopen master, current runtime/timer/CAN/CM4 harnesses and CMake registrations.
- Local environment: Linux container with C compiler and CMake, **no GitHub network/DNS access from its shell**, **no repository checkout** and **no `arm-none-eabi-gcc`**. GitHub connector supplied read-only baseline data. No source-level test below has yet compiled against the complete P03 checkout. The source from GitHub was not mirrored to a local Git repository.
- Current CTest baseline: **55 as reported for P03, not independently re-executed**. P04 proposes one new CTest (`p04_integrated_production_scenarios`), conditional on Python3 discovery. The actual total must be obtained from `ctest -N` in WSL; do not assume 56.

## 2. Production timing and execution trace

Source: `Firmware/STM32H745/CM7/Core/Src/application_supervisor.c` owns TIM2/TIM3, 1 MHz TIM2/2 ms TIM3 settings, sequence-matched ring of **8 serviced TIM3 IRQ timestamps**. `timer_wait` uses a counting FreeRTOS notification, `pdTRUE` to drain count, deadline wait 6 ms. TIM2 is extended to 64-bit by an overflow ISR; a pending UIF correction covers one pending wrap but cannot prove arbitrary multi-wrap IRQ masking. Only CM7 TIM2 events and CM7 task timestamps are directly comparable. CM4 progress timestamps and host wall clocks are different domains.

Production task `supervisor_task.c` executes:

1. TIM3 `wait_next`, missed-count/timeout E003 detection, optional `observe_wake`;
2. capture `cycle_start_us` from TIM2, and **new P04 pre-dispatch TIM3 timing validation**;
3. `supervisor_pre_dispatch_cycle`: external inputs, at most **16** queued messages (special Teaching-event boundary, overload E016), at most **8** CM4 IPC frames, safety freshness E017 / peer response freshness E018, state-specific CAN/drive containment;
4. controller CANopen heartbeat service, retry deferred state entry, restart/authorization gate and `canopen_master_set_drive_permission`, active-state step or FAULT service, observation-only guidance, drive snapshot, CM7 IPC status, output diagnostic adapter;
5. task execution elapsed check: `>= 2000 us` E004; `supervisor_timing_assess` on serviced ISR-to-wake/completion and previous wake gap, E003 on severe >=4000 us service/wake gaps, E004 on ISR-to-completion >=2000 us; record per-cycle progress, but an unsuccessful cycle is not eligible for an independent watchdog feed.

**Identified defect at P03 baseline:** the original Supervisor evaluated `supervisor_timing_assess` *after* state step. Consequently a single late wake (including `isr_to_wake >= 2000 us` or `isr_to_task_pre_dispatch >= 2000 us`), inconsistent timing observation or severe observed gap could be processed as a motion cycle before the fault. Existing tests explicitly expected invalid observations to increment a counter but *not* fault. The patch adds a guarded pre-dispatch assessment and a distinct E019 for absent/invalid timestamps; severe observed gaps use E003 and observed ISR-to-current-dispatch age reaching 2 ms E004 (including scheduling preemption after an on-time wake). Completion overrun cannot be known until work finishes and is still evaluated post-cycle. This change is safety-oriented and requires WSL regression.

A target notification count >1 is distinct from a serviced ISR gap >2 ms; an interval between serviced interrupts does **not** independently establish missed hardware update count. E003 represents source-defined release/cadence faults, E004 the source-defined deadline faults, E016 Supervisor queue overflow, E017 safety freshness, E018 CM4 peer freshness, and candidate **E019** invalid or missing timing evidence. E019 is proposed, not yet integration-validated for external fault-code consumers.

### Timing ownership and provisional budget

| Subsystem | Owner/clock | Existing bound | Safety evidence / unresolved budget |
|---|---|---|---|
| TIM3 release/IRQ timestamp | CM7 timer ISR, TIM2 extended us | 8-entry history, notification accumulation | IRQ masking/latency and DWT time pending physical test |
| Supervisor message processing | CM7 task, local tick/time | 16 items/cycle; exact 16 boundary depth query | E016 fail-closed backlog; queue send may block external producers, not this bounded drain |
| CM7 IPC consumption | CM7 task / CM7 local ms | 8 messages/cycle; zero-timeout receive | Under continuous old traffic, challenge can starve and peer expires; fresh responses do not override safety faults |
| CM4 IPC service | CM4 defaultTask / CM4 tick | 1 ms nominal task, 8 status frames per pass, 16 queued HMI | Task scheduling and CM4 load not hardware measured |
| Actual FreeRTOS Message Buffers | shared SRAM4; each core's kernel | zero send/receive timeout; bounded storage | 60-byte app wire frame, 4-byte frame length on ARM; HSEM/EXTI transport ordering not physically measured |
| P03 challenge | CM7 issue/receipt clock | 30 ms provisional maximum RTT | CM4 service must finish before response, no CM4-to-CM7 timestamp subtraction |
| P03 peer freshness | CM7 receipt ms | 50 ms provisional maximum age | `runtime_peer_fresh` also enforces pending challenge deadline |
| P03 safety freshness | CM7 verified producer time + receipt ms | 100 ms provisional | Requires non-null verifier, valid same-clock source timestamp/sequence |
| CANopen master RX | CM7 owner, HAL FDCAN backend | max 32 frames/poll | Excess deferred, **no explicit saturation fault / high-water mark** in baseline; risk pending hardware measurement |
| RPDO target + SYNC | CM7 CANopen owner | 6 sends + 1 SYNC per target cycle | TX nonblocking HAL backend must be proven; any failure revokes permission |
| Supervisor execution | CM7 TIM2 us | 2000 us **deadline**, not a proven WCET | worst-case interrupt preemption, DMA/cache, memory and bus contention unknown |

Loops with maximum counts do not establish a time bound for each HAL/backend call. `run_active_state` includes state-specific stop/trajectory/validation work; lower-priority validation worker can defer. Read/write GPIO provider callbacks and CAN backend completion are not known-WCET components.

## 3. CAN load model (analytical; one shared Classic CAN bus at 1 Mbit/s)

The **actual** project PDO mapper uses one `RPDO4` **DLC 4** per axis, one `TPDO4` **DLC 6** per axis, and a SYNC frame **DLC 0**. Six axes at 500 Hz imply **6 RPDO4 + 6 TPDO4 + 1 SYNC = 13 standard 11-bit CAN frames every 2 ms**, assuming each drive returns one TPDO every SYNC. Standard 11-bit Classical CAN 2.0 frames plus intermission require approximately `47 + 8 × DLC` bits, excluding stuffing. Lower-bound serialization is `6 × 79 + 6 × 95 + 47 = 1091 bit / 2 ms`, or **54.55%** of a 1 Mbit/s link. A conservative bit-stuffing envelope bounded by about `floor((33 + 8×DLC)/4)` extra bits per frame yields about **65.75%** (1315 bit/2 ms), before additional heartbeats and diagnostics. This maximum-stuffing model is not an exact exhaustive CAN bit-pattern proof.

The controller sends a periodic 100 ms heartbeat (11-bit DLC 1). Drive heartbeat periods/phase must be established from BOOT's commissioned SDO mapping, not assumed always 100 ms. Each 100 ms heartbeat source adds nominal 550 bit/s excluding stuffing. Arbitration order, FIFO backlog, repeated arbitration loss, error frames, retransmissions, NMT/SDO diagnostics and extended drive timing are not represented by average utilization. Critically, 1091 µs serialized per 2 ms **does not imply completion within a 2000 µs command-to-feedback deadline**, since RPDO and TPDO transmit phases and the application processing must also fit.

## 4. Candidate integrated fault matrix

The supplied `test_p04_integrated.c` uses the repository's **actual Supervisor TU through the existing alignment fixture** with its task/IPC/fake-drive seams. Scenarios are injected, not observed on hardware. All are **NOT RUN in current environment**.

| Case | Injected conditions | Required assertion / observed only after WSL execution |
|---|---|---|
| P04-01 | Fresh P03 evidence, on-time TIM3, valid drive and Approach path | No spurious fault / permission remains granted under fixture assumptions |
| P04-02 | Peer age 51 ms during Approach | E018, FAULT, immediate software permission revocation |
| P04-03 | Wake at 2000 µs, severe service gap, invalid ordering | E004/E003/E019 pre-dispatch, revocation and fault response; PathExecution on-time acceptance |
| P04-04 | >16 normal Supervisor items plus expired safety | E016 queue precedence, no motion permission; separate non-overloaded E017 safety expiry assertion |
| P04-05 | Missing drive axis 2 feedback, CM4 fresh | E202 and FAULT independently of CM4 |
| P04-06 | Simultaneous peer expiry and drive feedback loss | FAULT, revocation, fault service |
| P04-07 | Renew communications after peer FAULT | No automatic authorization or motion recovery |
| P04-08 | Repeated queued overload in PathExecution | Continued FAULT service and no restored permission |
| P04-09 | CM7 uint32 ms wrap, delayed/replay peer response, stale safety | Modular age and sequence checks reject stale/replay |
| P04-10 | Post-FAULT timing disturbance | No forced motor dispatch; fault response still reachable |
| P04-BOUNDARY | Wake at 1999/2000/2001 µs, post-wake preemption to 2000 µs, and missing observation | Strict boundary fault behavior; E019 for absent observation |

**Gaps:** True CAN bus-off/TX saturation, exact-full RX boundary and post-recovery NMT traffic, high-water traces, CM4 stalled after independent IPC buffer exhaustion, simultaneous two-core preemption under actual FreeRTOS, real physical stop and hardware watchdog feed are **not implemented/verified by this candidate**. Existing P03 real-POSIX IPC type 6 HELLO/type 7 PROGRESS tests must be run in addition; the new integrated Supervisor fixture uses fake IPC and does not replace them.

## 5. Source changes and rationale

- `Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.c`: add `supervisor_timing_pre_dispatch_check`, invoke before `supervisor_pre_dispatch_cycle`. Do not alter P01/P02/P03 safety ordering or 2000 us threshold. E019 proposed for invalid timing evidence.
- `Firmware/Tests/test_p04_integrated.c`: new source-owned Supervisor fixture sharing all production C and test dependencies; 10 adversarial sequences + timing boundaries. No new supervisor/IPC/CANopen architecture.
- `Firmware/Tests/run_state_machine_alignment_tests.py`: register P04 fixture and source-only fast mode for mutation execution. Existing mutations remain intact.
- `Firmware/Tests/run_p04_integrated.py`: run test, parse exact scenario markers, optionally emit JSON trace; all timing values explicitly **injected**.
- `Firmware/Tests/run_p04_mutations.py`: temporary symlink mirror compiling mutated production source and requiring behavioral assertion failure, not a compiler error. Candidate 8 Supervisor + 2 timer mutations.
- `CMakeLists.txt`: register P04 CTest under existing Python availability conditional.
- This report and exact-blob patch builder. **No SRAM4, IPC ABI, flash partitions or CANopen owner code modified** in this candidate.

## 6. Provenance and diagnostics

The `P04_TRACE_FILE=/path/p04.json` output records per-scenario PASS/FAIL and the exact synthetic wake deadline test inputs. Source: production C evaluated with *deterministically injected fake CM7 timestamps* in host mocks. It is not a production cycle trace or benchmark. Statistical min, mean, p95, p99, max of actual task time are **NOT AVAILABLE**; no hardware samples were collected. No telemetry, dynamic memory or file I/O was inserted into CM7's real-time cycle.

## 7. Verification matrix

| Verification | Status in authoring environment | Reproducibility requirement |
|---|---|---|
| Canonical branch HEAD/P03 metadata and source inspection | PASS (GitHub read-only) | `git rev-parse HEAD`, `git show --no-patch -s` |
| New package Python syntax | PASS (local `python -m py_compile`) | `python3 -m py_compile ...` |
| Isolated candidate pre-dispatch C smoke + exact P03 timing header | PASS (local host only) | `python3 local_source_check/run_candidate_smoke.py` |
| Exact P03 timing header boundaries + one compiled negative mutation | PASS (local host only) | `local_source_check/test_timing_quality_exact.c`; one wake-boundary mutant compiled and failed assertion |
| New P04 production scenario fixture | **NOT RUN** | `python3 Firmware/Tests/run_p04_integrated.py` |
| New P04 10 negative mutations | **NOT RUN** | `python3 Firmware/Tests/run_p04_mutations.py` |
| Production extracted TIM2/TIM3 injection regression | **NOT RUN** | `python3 Firmware/Tests/run_m7_timer_fault_injection.py` |
| Existing P01/P02 state alignment and mutations | **NOT RUN** | `python3 Firmware/Tests/run_state_machine_alignment_tests.py` |
| Existing P03 runtime/supervisor/CM4 suites | **NOT RUN** | `run_p03_...` in Firmware/Tests |
| Actual FreeRTOS POSIX transport, types 6/7 | **NOT RUN** | `bash Intercore/Tests/run_ipc_freertos_runtime.sh` |
| CANopen C2B master ABI/permission tests | **NOT RUN** | `python3 Firmware/Tests/run_c2b_canopen_master_tests.py` |
| CMake configure, build, CTests | **NOT RUN** | `cmake -S . -B build-p04 ...`, `ctest --test-dir build-p04` |
| CM7/CM4 Debug/Release ARM builds | **NOT RUN** | `python3 Firmware/Tests/build_stm32.py --help` then documented options |
| Firmware integration Debug/Release, `git diff --check` | **NOT RUN** | Repository scripts and Git checks |
| Actual 500 Hz / safe physical stopping | **HARDWARE-PENDING** | Oscilloscope/logic analyzer and drive commissioning required |

## 8. Mutation evidence

`run_p04_mutations.py` defines 10 mutants (late dispatch, invalid timestamp, severe gap, permission containment, peer expiry, safety expiry, queue overload, CAN feedback, missed-release condition, execution-overrun condition). **0/10 integrated source mutations executed** here; one separate native timing-header boundary mutant compiled and failed an assertion. A mutation only counts rejected after the executable compiles and emits `P04_ASSERT_FAIL` (integrated case) or a native assertion failure (existing timer-source extraction). A failure during compilation is recorded as a verification failure, not a killed mutation. Some candidate mutants may fail to detect if the fixture does not exercise the specific path; this must be resolved before P04 acceptance. Existing mutation suites remain unchanged.

## 9. Outstanding risks and defects

- CM7 prior to P04 may dispatch a target after an already late or invalid wake. Proposed fix is **unverified** until real production fixture and ARM builds pass.
- Invalid timing evidence has a newly proposed E019 code; confirm fault-code consumers, dashboards, documentation and operator handling.
- The preflight check cannot predict an overrun caused by later CAN operations. E004 completion faults can only prohibit **subsequent** cycles, not retroactively prevent transmission already attempted in the overrunning cycle.
- CAN RX consumes at most 32 frames per poll but saturated backlog is not separately reported; no claim of complete CAN outage coverage. Source-grounded overload reporting requires a suitable backend pending-watermark contract to avoid equating an exactly-full batch with true queue overflow.
- CAN send return-success is HAL enqueue, not bus arbitration completion or physical motor execution.
- P03 30/50/100 ms thresholds are provisional; measured producer jitter, independent board verifier and latency margin remain unavailable.
- Shared SRAM4 FreeRTOS message-buffer ABI works in the existing POSIX harness but true 2-core cache/ISR/HSEM scheduling needs hardware.
- FAULT routine itself may fail hardware I/O. Software attempts to revoke permission do not prove drive torque removed.

## 10. Future STM32 hardware measurement procedure

1. Confirm board TIM2 is 1 MHz, TIM3 generates the expected 2 ms releases, interrupts priority 5, 64-bit TIM2 overflow handling. Run a sustained 10+ minute stress test and a separate 72+ minute rollover test (TIM2 wraps after approximately 71.6 minutes at 1 MHz) before accepting hardware rollover claims; injected rollover cases are not physical evidence.
2. Toggle dedicated low-intrusion GPIO markers at TIM3 serviced ISR entry, Supervisor immediately after `wait_next`, immediately before first RPDO, after last SYNC enqueue, after fault permission revocation, and at progress completion. Capture with a synchronized multi-channel logic analyzer/oscilloscope. Avoid intrusive printing, allocation and unbounded instrument calls.
3. Use DWT CYCCNT for cycle counts with 64-bit software extension and an independent TIM2 correlation; at 400 MHz 32-bit CYCCNT wraps after about 10.7 s. Confirm actual CPU clock and target interrupt overhead.
4. Record 100,000+ continuous cycles in SRAM circular buffers, export **outside** the 500 Hz loop, and compute min/mean/p95/p99/max, misses, worst wake delay, earliest/last RPDO/SYNC bus time and software fault/revocation latency. Include intentional ISR contention, CM4 QSPI/IPC traffic, full message queues, flash wait states and CAN error/retransmission campaigns. Capture CAN controller's bus-off/error counters and physical CAN analyzer timestamps.
5. Acceptance is **zero missed releases and zero >=2000 us task execution overruns under the qualified test envelope**; report any measured failure, source-to-bus-end latency and the actual fault-service tail. These are qualification targets, not current pass claims.
6. Independently validate physical safe stop against actual drives including STO/relay design, fault-injection stop latency, watchdog configuration and E-stop channel; do not infer from firmware-only tests.

## 11. Historical executor disposition (original authoring environment)

**INCOMPLETE / UNVERIFIED.** A source-grounded pre-dispatch correction, deterministic integration tests, mutation runner, JSON scenario output and exact-baseline patch builder have been authored but production compilation, transport stress, full regressions, ARM builds and physical deadlines are not verified. Engineering Director must not authorize a P04 commit on this evidence alone.

## Independent WSL verification — 2026-10-29

Canonical baseline:
916e7a85fd2d20f445fe513901f8d149260c09cb

Branch: sara/can-replacement

Results:
- P04 integrated scenarios: 11/11 PASS.
- P04 compiled source mutations: 10/10 rejected.
- Existing production Supervisor regressions: PASS.
- Full CMake build: PASS.
- Desktop CTests: 56/56 PASS.
- STM32 CM7 Debug and Release builds: PASS.
- STM32 CM4 Debug and Release builds: PASS.
- Debug and Release firmware integration: PASS.
- git diff --check: PASS.

Local verification correction:
- P04-04 safety-expiry fixture changed to 101 ms.
- The P03 safety freshness limit of 100 ms is inclusive.
- No production safety threshold was relaxed.

Verification scope:
- Timing scenarios use deterministic injected timestamps.
- Host execution does not establish STM32 worst-case execution time.
- Physical 500 Hz timing and motor stopping remain unverified.

Final source review and commit authorization remain pending.

## P04 director correction — production task-loop invocation and late pre-dispatch window

**Independent WSL verification, supplied by the Engineering Director (not executed by this package author):** 56/56 CTests, 11/11 P04 scenario groups, 10/10 original compiled P04 mutations rejected, all four CM7/CM4 ARM Debug/Release builds and both firmware integration checks PASS. Retain the historical authoring-environment **NOT RUN** entries above; these are separate test runs and are not evidence of author execution. The following incremental correction requires its own WSL execution.

**Production change:** The existing entry preflight check is retained. A second `supervisor_timing_pre_dispatch_check(context, supervisor_now_us())` now runs **after** input, queue, IPC, heartbeat, and drive authorization work, **immediately before** `run_active_state`. If the task-local TIM2 observation reaches 2000 us since serviced TIM3 ISR, E004 is entered and software permission is revoked before the motion-state call. Invalid observation still uses E019 and severe gap/missed releases E003. The FAULT/EMERGENCY_STOP servicing condition remains unchanged; no automatic recovery and no deadline relaxation. The second check does not predict future state-step runtime, CAN transmit completion or physical stop latency.

**Real task-path executable tests:** `test_p04_integrated.c` now calls the actual static `supervisor_task_entry` for exactly one deterministic release, exiting at the next synthetic wait via `setjmp`. For missing initial timing evidence, an input-acquisition callback requires E019 already latched **before normal work**; bypassing the early production invocation fails the callback assertion even if the second check could eventually catch the same fault. For a valid initial wake, input acquisition advances injected CM7 time across the 2000 us limit, then tests require E004, inhibited drive permission, continued fault service, untouched Approach-request sentinel and zero target sends. No ISR/MCU execution time is measured.

**New compiled source mutations:** `task_entry_early_callsite_bypass` and `task_entry_predispatch_recheck_bypass` are added to the existing ten. Expected total: **12**, subject to WSL execution (NOT RUN here). Both must compile, execute the actual task entry, and produce `P04_ASSERT_FAIL`, rather than pass on compiler errors. The existing 11 P04 output scenario groups and 56 CTests are intentionally unchanged.

**Residual limitations:** Potential preemption or execution overrun *inside* `run_active_state` cannot be retroactively prevented by this guard; physical scheduling, CAN TX arbitration/feedback under saturation, genuine CM7/CM4 IPC contention, real timing WCET and hardware stop assurance remain **NOT VERIFIED / HARDWARE-PENDING**. No GPIO instrumentation or independent watchdog feeding was enabled.

**Post-correction WSL verification: NOT RUN by executor in this environment.** Engineering Director must run the exact commands supplied in the incremental package README, capture compiled mutation results, CTest totals, ARM builds and `git diff --check` before accepting the incremental change.

**Incremental artifact disposition: READY FOR DIRECTOR REVIEW.** This is a review readiness label, not an assertion that the two new mutations or production task-loop test passed WSL. No commit or push is authorized by this label.

## Final independent WSL acceptance — 2026-10-29

Post-correction verification:
- 56/56 desktop CTests PASS.
- 11/11 P04 scenario groups PASS.
- 12/12 compiled P04 source mutations rejected.
- Production task-entry timing checks verified through executable assertions.
- P01/P02/P03, CANopen and IPC regressions PASS.
- CM7/CM4 Debug and Release ARM builds PASS.
- Both firmware integration checks PASS.
- git diff --check PASS.

Final fixture correction:
The task-entry test now checks persistent FAULT service evidence
because the next simulated cycle resets fault_response_step_ok.

Scope: software-only acceptance. STM32 physical WCET and motor stopping
remain unverified.
