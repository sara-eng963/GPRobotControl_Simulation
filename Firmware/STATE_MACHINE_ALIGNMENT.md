# CM7 / desktop behavior agreement

Review baseline: `de8f7d77c965bc2cb8d453b2f5f9f5165ec98f86` on
`feature/admittance-tustin-500hz`. Working branch:
`fix/cm7-host-semantic-alignment`. Prepared 2026-10-28. No commit, push, merge,
flashing, or robot operation was performed.

The production tree remains authoritative for target composition and safety.
Software evidence below does **not** establish physical safety or authorize
motion. Existing untracked patch bundles were left alone.

The [P01 review follow-up](#p01-review-follow-up) records the subsequent deadline,
pending-RESET and long-pause findings and the latest verification results.

## Independent comparison and decisions

Compared every C/header under root `StateMachine` with
`STM32H745/CM7/Application/StateMachine`, then traced the live root HMI task,
CM7 message routing, CM4 IPC service and CM4 HMI lifecycle policy. Files not
listed as differences were identical at baseline (excluding tests/docs).

| Baseline difference or boundary | Classification | Decision / implemented contract |
| --- | --- | --- |
| HMI speed command slots 9/10/11 meant Low/Medium/High on host, Up/Down/Default on CM7 | Required shared behavior | Slots now mean Low/Medium/High everywhere. Root `StateMachine/supervisor_hmi.h` is canonical; target header forwards to it. Numeric slots, command count and IPC message layout remain unchanged. |
| Teaching config/events used increments on target and presets on host | Required shared behavior | Target uses the reviewed host preset fields, validation, events and segment/global speed update behavior. Presets must lie within configured limits and be ordered. No physical board configuration is fabricated. |
| Teaching duplicate handling used position alone on target | Required shared behavior | Position plus shortest quaternion rotation, including q/-q equivalence. Identical records are accepted no-ops with a message; orientation-only points are rejected without advancing or overwriting the recorded point. Other geometric validity checks remain. |
| Idle selection destroyed validated-path authority on target | Required shared behavior | Line/Arc/Circle are rejected while a validated trajectory is available. RESET explicitly clears volatile program authority. |
| Homing absent from target operator-pausable states; reentered on resume | Required shared behavior | Homing pauses/resumes with its state, sample cursor and command bookkeeping preserved. Wrapper also waits for the pause service's hold acknowledgement before resume. This is not a claim of measured physical stopping. |
| Target unified PATH_EXECUTION completion entered global Homing | Required shared behavior | Completion enters Idle, preserving the validated path. Internal wire-off/retraction/clearance phases still run before completion; Preview acceptance follows successful execution. Legacy execution branches remain unchanged. |
| Target worker published a batch after cancellation could arrive | Required shared behavior | Cancellation is rechecked in the same critical section as result/DONE publication. A cancelled result is never published; the worker cancels its own state before acknowledging IDLE. |
| Target wrapper lacked host runtime-program cleanup on RESET | Required shared behavior | Accepted nonmoving RESET and acknowledged ABORTED_RESET clear draft/submitted program and runtime outputs. During active motion the context is retained until stop acknowledgement. No Supervisor mutation of active worker state/workspace/frozen program is added. |
| Both wrappers routed Teaching HMI RESET only to a draft event | Required shared behavior, clarified in both | HMI RESET now takes the policy RESET path: Teaching -> Idle, clearing the whole volatile runtime context. Direct module TEACH_EVENT_RESET still exists for module clients. |
| Both Approach modules reported ABORTED immediately on HOME/RESET | Shared safety defect, fixed in both | Added a nonblocking controlled-stop phase with explicit acknowledgement and a bounded timeout. Missing stop provider/zero timeout prevents Approach prerequisites from passing. No arrival at Idle is inferred from merely receiving RESET. |
| Later snapshots/HOME could overwrite queued RESET; Pause could abandon a stop | Shared safety defect, fixed in both | Pending abort requests survive snapshot replacement; RESET takes priority over HOME without restarting the stop deadline. Pause is rejected during a pending/active abort. Stale/queued motion-result events must agree with the motion module's actual result/reason. |
| Host path feedback config can override 20 ms with 100 ms | Host-only behavior | Target retains its fixed 20,000 us limit and has no override field. The new regression faults at 21 ms and rejects a 100 ms mutation. Existing Approach/Homing feedback limits are not changed by importing the host override. |
| Host tracing macros, guidance dispatch/candidate gate, legacy SIL Kit opt-in and scheduler catch-up handling | Host-only behavior | Remain on host. Target guidance port remains observation-only; target rejects a take_candidate callback. No host candidate dispatch or POSIX scheduling tolerance is imported. |
| Target BOOT pre-enable status reads, WAIT_AUTHORIZATION and repeated enable veto | Firmware-specific safety logic | Preserved. No boot or restart-gate implementation change. Time passing and HMI RESET do not grant drive restart permission. |
| Target owner-bound CANopen permission, same-cycle fault/safety revocation and reset rejection after inhibition | Firmware-specific safety logic | Preserved, including default-deny, pending-SDO cancellation and NMT-reset inhibition. New stop timeouts use the existing failure/revocation path. |
| Target TIM2/TIM3 observations, deadline checks, completion checkpoints and diagnostics | Firmware-specific safety logic | Preserved. No cycle/feedback deadline is widened. No new allocations or polling waits are added. The configured production control task retains asynchronous storage separation. Stop callbacks must be nonblocking board services. |
| Target task header forces IPC ABI v2; native FreeRTOS shared transport | Firmware-specific integration/safety | Preserved. Host ring-buffer ABI is not substituted. SRAM4 allocations, MPU policy and message/status fields are unchanged. |
| CM4 local HMI queues/status age, sequence checks, invalidation on state change/loss, progress monitor | Firmware-specific integration/safety | Preserved. CM4 calls the shared command enum; status acknowledges receipt, never execution/authorization. Native CM4 lifecycle behavior is executed; the service is also ARM-compiled and linked. |
| Root screen HmiEvent speed IDs are 6/7/8, distinct from Supervisor IDs 9/10/11 | Required translation boundary | Existing host `HMI/hmi_task.c` explicitly translates events. CM4's public submit API accepts SupervisorHmiCommand, not raw screen event numbers. Physical screen mapping is not commissioned by this change. |
| Root raylib UI/UDP protocol and direct host HMI task queue retry | Host-only behavior | Not copied into the target. CM4 retains its stricter status-freshness/pending-command rules. |
| `HMI/hmi_state_bridge.*` references an older state interface | Host-only inactive legacy source | Not selected by the current host/firmware source graphs; not used as an implementation reference or repaired in this change. It is not a verified production interface. |
| Host tests/runtime documentation versus target timing-quality header | Host-only evidence / firmware-specific policy respectively | No requirement to copy these files across trees. |

## Source and ownership architecture

One canonical command header prevents another silent enum reinterpretation.
Other reviewed shared state units remain in their existing source locations;
`run_state_machine_alignment_tests.py` enforces token parity for policy, Teaching,
Approach, Homing and their types/headers, and for CM4 HMI lifecycle behavior.
This keeps the existing CubeIDE graph intact instead of replacing target Boot,
Supervisor or timing modules with desktop copies. Target-specific path feedback
config and authorization code are deliberately outside that equality rule.
Existing portable numerical/drive source links and canonical CANopen type
contract remain in use.

Supervisor may change only worker mailbox state/cancel flags and copied outputs
under the same-core critical section. The lower-priority M7 worker alone enters,
steps and cancels its mutable numerical state. A new frozen program is not
copied until the previous worker is quiescent. Target entry additionally waits,
without blocking the Supervisor, for M4 storage/playback ownership release.
M4 alone performs physical storage callbacks; in-flight I/O completes before
its cancellation acknowledgement. The synchronous host path now also cancels
an open transaction before clearing its state.

Cancellation after a durable commit suppresses volatile validation authority;
it does not promise erasure of an already committed flash artifact. Recovery
must not infer execution authorization solely from persisted bytes.

## Board API change to review

`ApproachServices` appends `controlled_stop`, `stop_context` and
`controlled_stop_timeout_ms`. A board provider must supply a nonblocking callback
with an explicit `*stopped` result and a commissioned nonzero timeout. A successful
callback return with `*stopped == false` keeps Approach in its stop phase.
Communication failure or timeout reports a fault; it does not complete RESET.
Existing phase/error numeric values are preserved by appending the new values.
This changes an application service type, not the CANopen/IPC ABI.

The host SIL Kit composition supplies its existing simulated hold callback and
1000 ms abort timeout. That callback's acknowledgement is not physical stop
evidence. Real target board/storage suppliers remain absent/default-denied.
Do not treat the placeholder host teaching presets (5/10/25 mm/s), duplicate
tolerances or mock stop acknowledgements as commissioned robot settings.

## Initial P01 executed software verification

The local evidence directory is `build/state_machine_alignment/` (ignored build
artifacts), including command outcomes, logs and file hashes for review.

| Check | Result / scope |
| --- | --- |
| Existing desktop suite | Final run: **55/55 passed** (all original 54 plus the new production test), 49.30 s. SIL Kit disabled, Renode enabled. |
| New `stm32_state_machine_alignment` CTest | Production state policy, Teaching, Homing, Approach, numerical validation/storage and CM4 lifecycle fixtures; exact production Supervisor included without copying/extracting its implementation. |
| Production scheduling seam | Cancellation before start, between batches, and during RUNNING/COMPLETE/FAILED publication; normal DONE publication also tested. Supervisor reset leaves worker mutable state and frozen input untouched. A deterministic batch seam models preemption; it is not a real concurrent RTOS stress test. |
| Actual numerical cancellation | Real validator produces samples through the asynchronous bridge, cancellation revokes VALID, M7 does not abort M4 storage, and M4 acknowledges abort before idle. Executed against both root and production units. |
| RESET/stop and completion cases | Incomplete stop retains context in PathExecution and Approach; later snapshots/HOME/Pause cannot cancel RESET; stale result envelopes are rejected; acknowledged stop clears context; HOME -> RESET preserves the deadline; stop timeout faults and revokes drive permission; RESET cannot recover the revoked fault session. Successful path completion enters Idle. |
| Negative mutations | 11 variants must compile and fail executed assertions: stale cancelled publication, missing cleanup, Homing reinitialization, wrong Low mapping, 100 ms physical feedback, premature RESET cleanup, immediate Approach abort, lost path protection, extra Homing on completion, duplicate rejection and wrong High speed. |
| ARM builds | CM7 Debug/Release: 100 translation units each; CM4 Debug/Release: 48 each. ARM GCC equivalent builds from checked-in CubeIDE source graphs, with real linked ELFs. |
| Firmware integration | Both Debug and Release ELF/source-graph checks passed. Shared layout, SRAM4 linker placement, actual shared objects, startup and EXTI notification checks passed. |
| Application integration | CM4 IPC, CM4 validation-service policy, CM7 application/board CANopen composition and FDCAN HAL mock checks passed. |
| A02 | Production master with genuine shared adapters passed layout/include tracing, denied/granted/revoked behavior and authorization veto checks. |
| C2A | Production supervision policy plus six negative variants; wiring checks passed. |
| C2B | Restart policy plus five negative variants, production Boot plus enable-veto mutant, production CANopen master plus seven negative variants, and wiring checks passed. |
| Timing/configuration | Stage A configuration self-test, TIM2/TIM3 static checks and executable production-C timer fault injection passed. |

Initial sandbox execution attempted all 54 original tests: 48 passed; UDP socket
creation and Renode's per-user config lock were blocked. The run with the required
local runtime access passed those original fixtures. An intermediate new-test
harness failure was corrected before the final run; neither failure was hidden
by disabling tests or weakening assertions.

Reproduce:

```sh
cmake --build build/a02_2_desktop -j8
ctest --test-dir build/a02_2_desktop --output-on-failure -j4
python3 Firmware/Tests/run_state_machine_alignment_tests.py
python3 Firmware/Tests/build_stm32.py --core CM7
python3 Firmware/Tests/build_stm32.py --core CM4
python3 Firmware/Tests/build_stm32.py --core CM7 --config Release
python3 Firmware/Tests/build_stm32.py --core CM4 --config Release
python3 Firmware/Tests/check_firmware_integration.py
python3 Firmware/Tests/check_firmware_integration.py --build-root build/stm32/Release
python3 Firmware/Tests/run_a02_1_canopen_adapter_abi.py
python3 Firmware/Tests/run_supervision_policy_tests.py
python3 Firmware/Tests/check_c2a_wiring.py
python3 Firmware/Tests/run_drive_restart_gate_tests.py
python3 Firmware/Tests/run_c2b_canopen_master_tests.py
python3 Firmware/Tests/check_c2b_wiring.py
```

`Firmware/README.md` lists the additional integration scripts. The firmware CI
workflow now includes both build configurations and the production alignment
regression, and triggers for root StateMachine changes. The remote workflow
itself has not been run from this uncommitted branch.

## Unverified assertions and remaining hardware work

- No measured deceleration, stationary robot, physical holding behavior,
  collision avoidance, STO/E-stop response, welding relay response or safe
  Homing/retraction claim follows from these tests. Stop/hold acknowledgement
  correctness depends on an independently verified board provider.
- Actual 500 Hz WCET, interrupt latency/coalescing, jitter, priority behavior,
  stack/heap margins (including RESET clearing and the new stop callback), and
  callback nonblocking bounds remain unmeasured on STM32. Existing deadline
  detection remains enabled, but does not prove deadlines are met.
- The 20 ms path-feedback limit is preserved and regression-tested, not
  independently justified for the physical CAN bus. Actual PDO latency/loss,
  drive status/enable sequencing and authorization input freshness require
  commissioning. Existing Approach/Homing timeouts are not newly validated.
- Dual-core startup/reset races, MPU/cache coherence, HSEM/EXTI delivery,
  shared FreeRTOS ABI behavior on two independently scheduled cores, and physical flash power-loss
  behavior remain hardware work. Native scheduling stubs do not test those.
- Physical F/T acquisition, kinematics/scales, speed/tolerance choices,
  mechanical limits, sensor freshness and clearance remain uncommissioned.
- Physical HMI/RS485 wiring, screen IDs, translation into Supervisor commands,
  bounce/repeat behavior and operator UX have not been exercised. CM4 service
  runtime with the actual panel is unverified; native lifecycle tests and ARM
  linking cover software only.
- STM32CubeIDE GUI builds and its packaged compiler were not executed; the
  repository ARM GCC equivalent builds were. Physical board/storage providers
  are still absent, so these default ELFs cannot demonstrate a running robot
  application. No hardware was connected or programmed.
- The optional SIL Kit scenarios, graphical UI interaction and legacy inactive
  HMI bridge were not exercised by the 54-test desktop configuration. The small
  SIL Kit composition and HMI task C sources passed a syntax check with the
  actual host kernel headers and opt-in macros; the SDK link and scenarios were
  not executed. That end-to-end path remains unverified.
- Software tests and source review establish only the enumerated observations;
  no functional-safety certification, complete race proof, hardware readiness,
  or authorization to restart physical drives is asserted.

## P01 review follow-up

Continued on the same uncommitted branch. Changes are limited to eight existing
implementation files (the host/CM7 copies of Approach, Homing, PathExecution and
Supervisor), three existing test/runner files, and this report. No public API,
build target, CMake/CI change or new test framework was added in this follow-up.
The pre-existing `P01_review_bundle.zip` and patch bundles were not modified.

### Approach deadline policy

Expiry is inclusive: a controlled-stop step whose supplied `now_ms` has
`(uint32_t)(now_ms - stop_started_ms) >= controlled_stop_timeout_ms` fails with
`APPROACH_ERR_STOP_TIMEOUT` before polling the callback. An acknowledgement
observed at that step can therefore succeed only strictly before expiry.
At/after expiry no callback is polled, a late acknowledgement cannot complete
RESET/HOME, and subsequent steps remain failed. Callback failure before expiry
reports `APPROACH_ERR_COMMUNICATION`, even if it also writes `stopped = true`.

The existing Approach fixture now executes 12 boundary combinations: RESET and
HOME, ordinary and wrapping tick starts, and first acknowledgement at 19/20/21 ms
for a 20 ms timeout. It also checks both abort reasons with a failed callback
before expiry and no callback retry after expiry. The old implementation fails
16 assertions in this fixture; the reproduction log is retained.

### Reachability of completion after pending RESET

This is reachable through the exported message API in **both** wrappers:

1. `PATH_EXEC_PHASE_VERIFY_CLEARANCE` can set phase/result COMPLETE while
   `state_path_execution_step` still returns RUNNING. The global state therefore
   remains PATH_EXECUTION until a later dispatch.
2. `supervisor_task_post` accepts `SUPERVISOR_MESSAGE_EVENT`; the task drains
   messages through `handle_message` before running the active module. It does
   not restrict STATE_COMPLETE envelopes to internal synchronous emitters.
3. A RESET HMI message followed by a matching-source STATE_COMPLETE in that
   message batch previously passed the result guard using the already-latched
   COMPLETE, entered Idle and bypassed the requested controlled stop.
4. Merely guarding the pending request flag was insufficient: after entering
   the stop, that flag is consumed. The old `begin_abort` left the earlier
   COMPLETE result intact, allowing another completion to pass during stopping.

Both wrappers now require that neither RESET nor HOME is pending before accepting
a motion STATE_COMPLETE. Both PathExecution implementations set result RUNNING
when beginning an abort, invalidating any old terminal completion until the
controlled-stop acknowledgement produces ABORTED_RESET/HOME.

The regression obtains the terminal latch from a real PathExecution step, then
dispatches the actual message envelopes in order, checks rejection both before
and after stop entry, verifies retained program/Preview authority, and confirms
cleanup only after acknowledgement. The same completion without RESET remains
accepted. The same fixture runs against each actual Supervisor and its own
state/CANopen translation units. Target-only C2B assertions remain enabled in
the firmware run.

Current in-tree motion modules emit their result events synchronously; no stock
producer was found queueing STATE_COMPLETE. This finding is API reachability,
not an assertion that an installed panel or board currently generates that
sequence. Approach completes synchronously without PathExecution's deferred
terminal latch; its existing result/reason guard remains and its completion
predicate now also respects pending abort requests.

### Homing after a long pause

The additional tests exposed a confirmed defect: a newer sequence received early
in a pause remained acceptable even when its timestamp was stale at resume.
Homing now checks each axis's timestamp, validity and existing enabled-state
checks before every command-capable phase. Feedback age must be strictly below
the existing `HOMING_CYCLIC_FEEDBACK_TIMEOUT_MS` (100 ms), at the millisecond API's
resolution. Tick subtraction remains unsigned; future timestamps at that
resolution fail closed. The firmware PathExecution limit remains **20,000 us**;
the host-only 100 ms path configuration was not imported.

At a 250 ms pause, tests exercise missing feedback, a newer but stale sequence,
one stale axis, invalid feedback, a future timestamp and fresh all-axis feedback.
Invalid/stale cases fault without sending another target or consuming a sample.
Fresh feedback resumes with exactly the next trajectory target, increments the
cursor once, and produces no same-timestamp catch-up command. Additional real
steps cover pauses before the first Homing sample and before a verification
hold, when no command acknowledgement is pending, with stale/fresh controls.
The tests run against both actual wrappers. No trajectory reset, sample skipping,
timeout relaxation or timestamp fabrication was added.

### Follow-up verification results

| Executed check | Result |
| --- | --- |
| `run_state_machine_alignment_tests.py` | Passed; actual production fixtures plus both Supervisor wrappers, and **15/15 compiled negative mutations rejected** (11 original, four new). |
| `cmake --build build/a02_2_desktop -j8` | Passed. |
| `ctest --test-dir build/a02_2_desktop --output-on-failure -j4` | **55/55 passed**, zero failures, 55.66 s; loopback and Renode runtime access enabled as in P01. No test was disabled. |
| CM7 Debug / Release | Both passed, 100 translation units each. |
| CM4 Debug / Release | Both passed, 48 translation units each. |
| `check_firmware_integration.py` for Debug and Release | Both passed. |
| A02 adapter/ABI, restart policy/Boot, C2B CANopen coordinator mutations and C2B wiring | All passed again. |
| C2A wiring and production-C timer fault injection | Passed again. |
| Shared behavior parity / unchanged protections | Approach/Homing token parity passed. CANopen production coordinator, canonical type ABI, Boot, restart policy and supervision policy files remain unchanged from `de8f7d7`; PathExecution feedback stays at 20 ms. |

The four new negative variants independently remove the Approach expiry veto,
pending-RESET completion veto, abort result invalidation, or Homing feedback-age
check; each compiles and fails an executed assertion. The existing test harness
is reused. `build/state_machine_alignment/` contains `p01-followup.patch`, updated
full `review.patch`, follow-up logs and source/ELF hashes; initial P01 artifacts
are retained separately.

Deadline enforcement uses the Supervisor-supplied millisecond time for the step,
not a new clock read after the callback. The callback must satisfy its existing
nonblocking/timing contract; blocking callback duration and physical stop
acknowledgement truthfulness remain unverified on hardware. Homing age checks
likewise depend on the drive provider publishing correct monotonic timestamps.
Queue scenarios use deterministic ordered dispatch of real message handlers,
not concurrent two-core queue stress or a physical panel. Actual WCET, callback
latency, timestamp correctness, physical feedback/holding/stopping and optional
SIL Kit end-to-end scenarios remain unverified as detailed above. No physical
safety claim, commit or push was made.
