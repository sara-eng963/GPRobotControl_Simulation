# P03 — Runtime Safety Evidence, CM4 Work Freshness, and Fail-Closed Recovery

**Status: CANDIDATE / ENGINEERING DIRECTOR REVIEW REQUIRED.** No hardware commissioning.

## Baseline and provenance

- Canonical branch: `sara/can-replacement`; GitHub HEAD verified as `620da1bbdf7b33304c2040bfb518292ff256dc43` on 2026-10-29 (repository read only).
- Source review: production CM7 `supervisor_task.c/.h`, `state_machine.c`, `application_supervisor.c`, `main.c`; production CM4 `application_ipc_service.c`, `application_validation_service.c`, CM4 startup; shared `ipc_mailbox.h`, `supervision_progress.h`, `drive_restart_gate.h`, HMI lifecycle and alignment tests. Source obtained via commit-pinned GitHub reads. No complete executable repository checkout was available to this environment.
- All modifications in the candidate are generated locally from exact baseline anchors by `p03_build_patch.py`. No repository commit, push, merge, force-reset, branch creation or deletion was performed.

## Source ownership and pre-P03 failures

| Evidence/path | Actual owner and pre-P03 behavior | P03 change |
|---|---|---|
| `RobotSafetySnapshot` in `CM7/.../supervisor_task.c::handle_message` | Upstream safety producer is **not identified/commissioned** by baseline; `state_machine_update_safety()` copies it unconditionally; no proven timestamp epoch, authentication, sequence or timeout | `RuntimeSafetyPort.verify` explicitly authenticates source ID/generation/strict completed sequence and asserts timestamp shares CM7 monotonic-ms epoch; missing port denies motion. Replayed/expired samples never renew expiry. |
| C2B `supervisor_restart_system_sample()` | `ipc_valid` previously checked structural shared-memory validity only; `statusValid` alone could stay true | Requires both valid source evidence and peer challenge-completion freshness before BOOT or continued motion. |
| CM4 `application_ipc_service_run()` | sole CM4 M4→M7 producer, sole M7→M4 consumer; service completion increments CM4-local `SupervisionProgressSample` but CM7 never sees its work completion | CM4 publishes a single HELLO per service initialization; responds to currently observed CM7 challenges **only after end-of-service completion**, nonblocking, at the pace set by CM7. No remote timestamp subtraction. |
| CM7 `consume_m4_ipc()` | sole consumer M4→M7; accepted sequence-forward HMI with no binding to controller boot | HELLO-before-progress, challenge match, completed sequence and CM7-local issue/receipt age; HMI checks current CM7 session and last accepted challenge. |
| `handle_policy_event()` | P02 revokes CANopen and inhibits C2B restart gate on FAULT/E-stop/protective stop | Reused unchanged as the only drive-command revoker; fresh evidence failure is injected into the existing synchronous containment path. |
| `main.c` and shared IPC | CM7 initializes SRAM4 before first CM4 release by HSEM; initial mailbox header alone does not prove CM4 is running | Shared header unchanged; both images need protocol v2 application frames. Retained/partial mailboxes and cross-core reset hazards remain subject to actual HSEM/startup architecture. |

## Source timestamp and commissioning contracts

The commissioned `RuntimeSafetyPort.verify()` must establish BOTH authenticated sample provenance and **CM7-monotonic clock identity** for `timestampMs`, and return a nonzero producer generation plus a distinct completed sample sequence. The firmware does not infer epoch equivalence from a field name. Without a verified source, it denies. A completed sample must have `statusValid`; contradictory positive motion permission with E-stop, protective stop or global fault is rejected. Raw E-stop/protective assertions may cause an immediate state-machine stop, but may never clear it without accepted evidence.

Initial sample absence, duplicate/old producer sequence, non-forward sample time, future time, ambiguous wrap interval, session change, expired source time and expired CM7 receipt all deny; no rebooting producer can silently replace an authorized generation. Exact threshold is accepted; just-expired is denied. Millisecond modular comparisons require every accepted interval to remain strictly below `2^31` ms and verified identical epochs. Source has no actual commissioned verifier in this baseline. `RuntimeSafetyPort.max_age_ms` is supplied explicitly; test baseline 100 ms is **provisional**.

The installed `RuntimeIpcSessionPort.issue()` must give CM7 a nonzero boot-unique token that cannot be recovered from retained SRAM4 or unchanged ticks. An absent/invalid issuer returns default deny (rather than inventing a token). The baseline has **no such commissioned provider**. A future strong board port must be reviewed; tests use a synthetic issuer solely as a deterministic fixture.

## CM4 challenge/response and peer reset contract (protocol v2)

CM7 publishes its unique boot session in `IPC_CONTROLLER_STATUS.data[7]`, version 2 in `[8]`, the currently outstanding challenge in `[9]` and last successfully validated response challenge in `[10]`. A challenge counter is monotonically incremented once per round-trip; it is **not** incremented on retransmitting the same outstanding challenge. One outstanding challenge is retained, with the **CM7-local** issue timestamp. If no response arrives within the provisional **30 ms** issue-to-accept bound, that challenge cannot establish freshness (even if an old progress sequence advances). CM7 never uses CM4's unsynchronized time in age calculations.

The first response from CM4 for a given CM7 session MUST be `IPC_CM4_HELLO=7` (sequence=1, matching controller session, exact version, initialized-service marker). CM4 does not publish progress/HMI before HELLO successfully enters its single-producer ring. CM7 accepts only one HELLO per session; any later HELLO is treated as an application restart, invalidates evidence and FAULTs an authorized motion session regardless of CM4 sequence counter value. CM7 issues its first challenge *after* receiving HELLO; stale pre-HELLO status cannot authorize progress.

`IPC_CM4_PROGRESS=6` is published only at end of actual CM4 IPC-service work (after `supervision_record_completion`), once per observed challenge, when sample.valid, completed_ok, response_ok and SUP_PROGRESS_SERVICE are true. It returns the current session in `data[0]`, version in `[1]`, mode/completion flags `[2..5]` and the exact challenge in `[6]`. The frame's `sequence` is CM4 completion sequence. `timestamp_ms` is CM4-local diagnostic only. CM7 checks exact format, hello-seen, uncompromised peer, current outstanding nonce, completion sequence strict modular forward progress, CM7-local round-trip time and CM7-local last-response age (provisional **50 ms**). Sequence zero may be valid **at wrap**, but not replay. No challenge is reused within a session, including at counter wrap (fail closed).

HMI commands include CM7 session `data[1]` and last *confirmed* challenge `data[2]`; CM7 rejects old/missing session, missing/old challenge, or expired peer evidence, in addition to existing HMI ordering and P02 safety checks. CM4 drops queued commands when M7 status goes stale or the controller session changes. An absent status, full mailbox or stalled CM4 cannot generate progress acknowledgments; neither ring introduces a second consumer, a blocking operation or free-running progress flood.

## CM4 reset detection: precise guarantee and residual limitation

The single-HELLO-first protocol catches **new CM4 application startup after CM4 RAM reinitialization** even if its quickly increasing post-reset completion sequence overtakes the previous value; the second HELLO faults before accepting new post-reset progress in the SPSC FIFO. A HELLO blocked by a full ring prevents new CM4 progress, eventually timing out. The CM7 session token identifies the CM7 boot, **not a CM4 hardware boot generation**. A previously buffered, already completed response matching a still-valid outstanding CM7 challenge can be read after CM4 has reset. Challenge issue-to-receipt bounding limits, but cannot eliminate, the interval in which CM7 lacks independent reset knowledge. Similarly, retained old HMI frames *ahead* of a new HELLO that still match the most recently accepted challenge cannot be categorically distinguished from live messages without independent peer reset evidence or per-command handshake. This cannot be marketed as guaranteed reset detection. A commissioned, independently observed CM4 reset/boot generation or synchronized reset-and-mailbox invalidation procedure is required for a stronger claim. No fictitious hardware token was introduced.

## Revocation, stop precedence and recovery

New checks execute every 500-Hz CM7 Supervisor cycle after bounded queue/IPC reads and before any new active motion-state command. After authorization, loss of fresh safety evidence causes fault `E017`; loss of fresh peer challenge evidence causes `E018`. The established `handle_policy_event()` immediately inhibits C2B restart authorization, clears BOOT enable authorization and revokes CANopen drive-command permission. E-stop/protective-stop events have precedence. FAULT servicing remains active even if CM4 stops publishing. Expired evidence in Approach/PathExecution **does not assert verified controlled deceleration**: it takes the established fail-closed FAULT/safe-output path; physical stop performance and independent STO/E-stop remain UNVERIFIED.

Resumed progress alone cannot clear `DriveRestartGate.inhibited`; normal RESET cannot bypass the latch, and re-BOOT requires fresh safety evidence, peer challenge progress, the commissioned board input and a new physical release/rising edge. BOOT still must not require `drivesReady` before CiA-402 enable (C2B non-circularity). P02 16-message exact-boundary handling, queue overload E016, feedback watchdog and guided-target default denial remain unchanged by the candidate.

## IPC compatibility and layout matrix

`IpcMessage` remains 60 bytes, twelve `uint32_t` payload words; STM32 `IpcShared` object and `IPC_ABI_VERSION=2` unchanged. This is **application protocol v2**, not a shared-memory ABI migration.

| CM7 firmware | CM4 firmware | Expected behavior | Safe to commission as P03? |
|---|---|---|---|
| old | old | Legacy C2B/P02 rules; **neither image implements P03 protocol enforcement** | No |
| new | old | New CM7 never sees required HELLO/challenge completion. New CM7 BOOT default denies; legacy HMI frames lack valid session/challenge. | No |
| old | new | New CM4 refuses v1/unversioned controller status, sends no versioned progress/HMI; **old CM7 does NOT enforce P03 and may still authorize motors under its older C2B conditions**. This combination is not fail-safe as a system migration. | No |
| new | new | P03 negotiation possible **only** with real commissioned safety verifier, unique-session issuer and peer evidence; otherwise default denies. | No, pending commissioning evidence |

A pairwise firmware version/upgrade management procedure, flash synchronization and rollback controls have **not** been implemented; mixed-firmware deployment is prohibited. Keeping the mailbox ABI at version 2 does not prove negotiated P03 application-protocol support in an old CM7.

## Verification matrix (only executed results are PASS)

| Check | Result | Evidence |
|---|---|---|
| GitHub canonical branch SHA | PASS | Commit-pinned GitHub branch metadata |
| Isolated host C policy with temporary shape-equivalent IPC/safety headers | PASS | GCC executed 2026-10-29. **Not compiled against full repo headers.** |
| Fourteen defective production-policy-header variants | PASS: 14/14 rejected | GCC compile and runtime assertion rejection; stand-in header dependencies as above |
| Zero-context patch applied to synthetic anchor mock | PASS, STRUCTURAL ONLY | `git apply --unidiff-zero --check` on mock text; **not an actual baseline patch check** |
| `p03_build_patch.py` on complete baseline checkout | NOT RUN | Full Git checkout unavailable; builder verifies exact HEAD/branch, overlapping dirty paths and anchors |
| `test_p03_cm4_service.c` with actual CM4 service | NOT RUN | Runnable host fixture included for WSL; no complete CM4 source tree locally |
| `test_production_supervisor.c` updated CM7 pre-dispatch cycle / IPC / revocation / BOOT scenarios | NOT RUN | Production test fixture executes the same pre-dispatch function as the 500 Hz production task; full alignment runner unavailable locally |
| Four source-level Supervisor negative mutations | NOT RUN | WSL runner includes temporary symlink-based mirror; behavior must be proved by real fixture assertions, not compile failures |
| P01/P02 production regression and mutation suites | NOT RUN | No full checkout |
| CMake / CTest suite (including historical 55) | NOT RUN | No full checkout |
| CM7 Debug, CM7 Release, CM4 Debug, CM4 Release linked ARM ELFs | NOT RUN | No ARM toolchain or checkout |
| A02.1 ABI, IPC SRAM4 integration checks | NOT RUN | No checkout |
| Physical execution times / CAN latency / motor outputs / STO / safety deadlines | UNVERIFIED | Hardware uncommissioned |

## Changed-files inventory

`Intercore/ipc_mailbox.h`, STM32 Common IPC mirror, new STM32 `Common/Inc/runtime_evidence.h`, CM7 `supervisor_task.h/.c`, CM4 `application_ipc_service.c`, production Supervisor regression fixture, new P03 runtime evidence host policy test and mutation runner, new CM4 service test and mocks, source-level Supervisor mutation runner, this report. The zip also contains an exact-baseline patch builder, a **synthetic-only/unverified** static unified diff, and WSL commands.

## Timing / remaining blockers

CM7: bounded ≤8 received M4 frames/cycle, O(1) checks and challenge issue per 10 ms status publication; 16-message Supervisor queue drain unchanged. CM4: ≤8 M7 receive operations per 1 ms task iteration, one HELLO on startup, then one progress response for each CM7-issued challenge (nominal 10 ms), zero-timeout send. No heap allocation or blocking cross-core synchronization introduced. Actual execution time, scheduler gaps, cache coherency, independent reset observation, and physical stop behavior need target evidence. The provisional 100/50/30 ms thresholds **are not certified stop-time requirements**. The safety verifier, reset-unique CM7 session provider, hardware interlocks and peer reset provenance remain explicit commissioning blockers.

## Independent WSL verification — 2026-10-29

Baseline: 620da1bbdf7b33304c2040bfb518292ff256dc43
Branch: sara/can-replacement

Verified results:
- Exact-baseline patch generation/application: PASS
- P03 runtime-evidence policy: PASS
- P03 policy mutations: 14/14 rejected
- Production CM4 service harness: PASS
- Production and desktop Supervisor regressions: PASS
- P01/P02 Supervisor mutations: 19/19 rejected
- P03 production Supervisor mutations: 4/4 rejected
- CANopen authorization, ABI, restart and wiring checks: PASS
- Desktop CMake build: PASS
- Desktop CTests: 55/55 PASS
- CM7 Debug and Release ARM builds: PASS
- CM4 Debug and Release ARM builds: PASS
- Firmware integration checks, Debug and Release: PASS
- git diff --check: PASS

Local verification corrections:
- Isolated P03 test fixture from seeded P02 peer evidence.
- Initialized desktop Supervisor IPC receive message.
- Updated P02 mutation anchor for the new authorization expression.
- Corrected P03 HMI session mutation to compile without warning.
- Corrected runtime_evidence.h include dependency.

Remaining limitations:
- Board safety-evidence provider is not commissioned.
- Reset-unique CM7 session provider is not commissioned.
- CM4 hardware-reset detection is not independently guaranteed.
- Physical stopping, watchdog timing, and 500 Hz execution
  have not been demonstrated on real hardware.

This constitutes software verification, not physical safety approval.

## Final Director review — IPC transport correction

The final source review identified an integration defect: the IPC
message validators rejected P03 frame types IPC_CM4_PROGRESS (6)
and IPC_CM4_HELLO (7), preventing real CM4 liveness publication.

Corrections:
- Updated the STM32 FreeRTOS IPC transport validator.
- Updated the host FreeRTOS IPC transport mirror.
- Updated the desktop IPC transport validator.
- Extended real IPC transport tests to verify P03 HELLO/progress
  message transmission, reception, and invalid-type rejection.

Independent WSL verification after correction:
- Actual desktop IPC mailbox test: PASS.
- Actual FreeRTOS POSIX message-buffer test: PASS.
- Both FreeRTOS transport sources synchronized: PASS.
- Full desktop CTests: 55/55 PASS.
- P03 policy mutations: 14/14 rejected.
- P03 Supervisor mutations: 4/4 rejected.
- P01/P02 Supervisor mutations: 19/19 rejected.
- STM32 CM7 Debug and Release: PASS.
- STM32 CM4 Debug and Release: PASS.
- Firmware integration, Debug and Release: PASS.
- Staged and unstaged whitespace checks: PASS.

Acceptance scope: software implementation and host-based verification.
Physical motion authorization, real hardware timing, independent CM4
reset observation, and safety-system commissioning remain unverified.
