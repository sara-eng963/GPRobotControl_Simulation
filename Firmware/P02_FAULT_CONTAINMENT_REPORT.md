# P02 — Final Director Corrections: Queue Saturation and Guided-Target Quarantine

**Status — VERIFIED IN WSL (2026-10-28):** P02 implemented and locally committed on `fix/p02-fault-containment`, based on `cdd908e`. Production regressions, mutations, CTests, ARM builds, and firmware integration PASSED. No physical stopping or 2-ms WCET verification.

## Actual production source ownership

- CM7 Supervisor and state dispatch: `Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.c`, `state_machine.c`, and `States/*.c`.
- CM7 CANopen coordinator: `Firmware/STM32H745/CM7/Application/CANComm/CANopen/canopen_master.c`, with shared `ServoDrive/JointDrive/canopen_joint_drive_port.c` adapter. The top-level `StateMachine` and `CANComm` copies are desktop implementations, not the firmware source-of-truth.
- CM7 board/timers: `Firmware/STM32H745/CM7/Core/Src/application_supervisor.c`, `main.c`; CM4 IPC/storage: `Firmware/STM32H745/CM4/Core/Src/application_ipc_service.c`, `application_validation_service.c`.
- CM7/CM4 linked ARM ELFs, source ownership, SRAM4 sections, and firmware integration were checked locally.

## Corrected containment matrix

| Priority | Source/detection in production | Intended containment | Recovery | Remaining dependencies |
|---|---|---|---|---|
| P0 | BOOT C2B permission: `DriveRestartSystemSample` and `drive_restart_gate_tick` | Require valid motion permission, external communication health, fresh motor heartbeats, pre-enable CiA-402 status, board release/rising edge, IPC, and safety input validity; **do not** require `drivesReady` until BOOT has finished enabling | Original C2B inhibit latch/new BOOT, not ordinary RESET | Safety producer freshness, actual drive enable state uncommissioned |
| P0 | Partial RPDO4 send failure / failed SYNC in production master | Revoke software CANopen permission in the same call; no later RPDO/SYNC allowed merely because CAN transport recovers | Explicit C2B authorization | Already-sent RPDO cannot be recalled; no physical stopping guarantee |
| P0 | Unsafe SAFETY snapshot / denied external guided command | Immediate E-stop/protective/fault event and C2B inhibition; capture state-coded error; guided command rejected without any CAN dispatch | New operator session, verified conditions | Physical independent safety chain required |
| P1 | Guided target queued before unsafe safety update | Retain the bounded deferral/ordering barrier; reject all external guided targets because the physical source is uncommissioned and the message carries no authenticated freshness or origin | No automatic replay of discarded targets | Concurrent post after empty observation is not a hardware safety interlock |
| P1 | Authorized session, failed `io.read_inputs()` acquisition or an invalid returned snapshot | Fault and revoke before queue handling; no stale raw input reuse | C2B reauthorization | Board adapter is observation-only, not certified E-stop input |
| P1 | Six-axis feedback/heartbeat lost while active | Poll CAN before software preflight, check health, CiA-402/feedback validity and per-axis age for HOMING/APPROACH; external guided-target commands are denied | Fault with stored state/axis code | Double bounded poll in some states; measure worst-case cycle time |
| P1 | PATH_EXECUTION feedback freshness | Preserve **existing state-owned 20,000 us** watchdog, with poll before watchdog evaluation; no added approximate 20 ms competing guard | Fault; no automatic restart | TIM2/TPDO timestamps require commissioning |
| P1 | PAUSED/PATH_VALIDATION/IDLE intervals without SYNC | Continue bounded RX poll and heartbeat/drive validity checks; *do not* fault just because TPDO age exceeds 100 ms where cyclic feedback is not required | Revalidate fresh data before active motion resumes | Servo PDO scheduling configuration unverified |
| P1 | RESET/HOME during Approach or PathExecution stop | Preserve P01 state-owned controlled-stop acknowledgment and reset arbitration | Fault cannot exit through RESET without C2B gate | Stop callback not physical safety rated |
| P1 open | CM4 progress stall, IPC session timeout, stale external safety producer | Existing structural `ipc_valid` and `statusValid` checks are insufficient to prove liveness/freshness | Do not automatically reauthorize | Requires validated producer-specific age/session contract |
| P2 open | First-fault diagnostic overwritten by later events | Future latched first-fault record and trace; unchanged here | Engineering review | No durable log yet |

## BOOT dependency and fix

`state_boot.c` phases `WAIT_OPERATIONAL_HEARTBEAT` -> `PRE_ENABLE_STATUS` -> `WAIT_AUTHORIZATION` -> `ENABLE_DRIVES` -> `READ_POSITION_FEEDBACK` -> `INITIAL_PDO_EXCHANGE` -> `VERIFY_CYCLIC_COMMUNICATION`. The original `DriveRestartSystemSample` only assigned `safety_valid = s->statusValid` and used the operational-heartbeat state without checking the independent snapshot's `motionPermitted` and `communicationHealthy` or heartbeat freshness in that C2B sample. The revision constructs `.safety_valid = statusValid && motionPermitted`; `.communication_ready = canopen_master_healthy(now_ms) && communicationHealthy`; `.state_ready` requires `drivesReady` only **after BOOT**, not during authorization/drive enabling. The BOOT SAFETY message handler also excludes `drivesReady=false` from fault triggers until leaving BOOT; loss of motion permission, communication, or snapshot validity after authorization still faults. Actual drive feedback/CiA-402 readiness is still checked in BOOT and motion states. The existing C2B board release/rising edge and reset evidence remain unchanged.

## CAN monitoring and queue ordering

`supervisor_motion_containment_check` polls the real `JointDrivePort` (production adapter -> CANopen coordinator) before reading heartbeat/axis status. It does not call flash/storage, wait, retry or change physical pins. Existing state modules may also call poll; each call is bounded by `CANOPEN_MASTER_MAX_RX_PER_POLL`. This is an operation-count bound, **not a demonstrated 2-ms WCET**. No new polling occurs before a BOOT authorization is established. PAUSED/PATH_VALIDATION do not require cyclic feedback age without an explicit SYNC producer. On transition back into HOMING/APPROACH, the active freshness check must pass before new state-issued motion targets. TEACHING, like PAUSED/PATH_VALIDATION, may record without SYNC; external guided-target dispatch is uncommissioned and categorically denied. PathExecution performs its own 20,000-us watchdog after its poll; the preflight checks communication/validity but does not change this deadline.

`supervisor_drain_messages` processes up to 16 messages, defers untrusted GUIDED_TARGET until a genuinely complete drain (but **never dispatches it**, only records rejection), and preserves FIFO order for all other messages. `uxQueueMessagesWaiting` distinguishes an **exactly full 16-message batch** with no remainder from genuine overload. A pending Teaching Record causes a distinct `SUPERVISOR_DRAIN_TEACHING_EVENT` break so the existing single event slot cannot be overwritten. On overload, `supervisor_process_message_batch` transitions to FAULT with E016, immediately revokes authorization and allows `run_active_state()` to execute the FAULT safe-output service on that very cycle and subsequent cycles. It does **not** try to run an Approach/PathExecution controlled-stop callback before reading pending safety, because that could require an unvalidated drive command. This is a software containment trade-off, **not proof of physical stop**, and the unsafe message remaining queued is not relied upon for detection.

## Source changes and test coverage (locally executed)

1. `supervisor_task.c`: modified containment poll/age policy; extracted board-input acquisition for production test; corrected C2B sample; bounded queue overload faulting, guided-target quarantine; preserved C2B `was_authorized` revocation and P01 reset/stop logic.
2. `canopen_master.c`: permission revocation when RPDO frame build, RPDO send, or SYNC fails.
3. `test_production_supervisor.c`: BOOT negative prerequisite and legitimate `drivesReady=false` cases; held-button recovery; stale heartbeat; PAUSED/PATH_VALIDATION at 501 ms; stale active TPDO, modulo-time wrap, PathExecution delayed feedback at 20 ms and stale at 21 ms later; actual queue `GUIDED_TARGET -> unsafe SAFETY`; sustained Approach/PathExecution backlog; exact-full-batch and Teaching-event boundary; actual board-input callback returning false **or returning true with an invalid snapshot**.
4. `test_c2b_canopen_master.c`: partial RPDO 3/6 failure, SYNC failure, and no restart on simulated transport recovery.
5. `run_state_machine_alignment_tests.py`: four P02 negative mutations (watchdog, BOOT motion permission, overload containment, forbidden guided send) extending P01 checks.
6. `run_c2b_canopen_master_tests.py`: two additional TX-latch mutations extending prior C2B tests. Prior P01/C2B mutations are retained.

## WSL software verification — 2026-10-28

Executed on the actual WSL checkout at `~/freertos-pc-test`.

| Verification | Result |
|---|---|
| P02 Supervisor production regressions | PASS |
| Supervisor negative mutations | 19/19 rejected |
| Production CANopen authorization | PASS |
| CANopen negative mutations | 9/9 rejected |
| Desktop CMake and CTest | 55/55 PASS |
| STM32 CM7 Debug and Release | PASS |
| STM32 CM4 Debug and Release | PASS |
| Debug and Release firmware integration | PASS |
| A02.1 CANopen ABI and adapters | PASS |
| C2B restart authorization and BOOT gate | PASS |
| C2A supervision and wiring checks | PASS |

### Corrections during verification

The test fixture was corrected to include the canonical CiA-402
header, model successful controlled-stop acknowledgment for the
21-ms feedback-watchdog test, and accommodate cumulative queue
writes. These corrections did not alter the production watchdog.

### Remaining limitations

Software verification does not establish physical stopping,
safe-torque-off behavior, E-stop performance, actual CAN hardware
behavior, external safety-producer freshness, or the worst-case
2-ms MCU execution deadline.

## Historical WSL reproduction commands

```sh
# With a clean checkout at the actual cdd908e baseline and no commits:
git rev-parse HEAD
git status --short
git apply --check P02_fixed.patch
git apply P02_fixed.patch
python3 Firmware/Tests/run_state_machine_alignment_tests.py
python3 Firmware/Tests/run_c2b_canopen_master_tests.py
python3 Firmware/Tests/run_a02_1_canopen_adapter_abi.py
python3 Firmware/Tests/run_drive_restart_gate_tests.py
python3 Firmware/Tests/run_supervision_policy_tests.py
python3 Firmware/Tests/check_c2a_wiring.py
python3 Firmware/Tests/check_c2b_wiring.py
cmake -S . -B build/p02 && cmake --build build/p02 -j4
ctest --test-dir build/p02 --output-on-failure
for core in CM7 CM4; do
  for cfg in Debug Release; do
    python3 Firmware/Tests/build_stm32.py --core "$core" --config "$cfg"
  done
done
python3 Firmware/Tests/check_firmware_integration.py --build-root build/stm32/Debug
# Inspect CM7 .project source graph and ARM ELF objects for the changed units.
git diff --check
git diff --stat
```

## Explicit untested assumptions and open issues

**Software assumptions requiring targeted validation:** `canopen_master_healthy` heartbeat timestamps use the same monotonic `now_ms` epoch; all six `CanopenMaster` nodes are configured before motion; HAL CAN RX poll is nonblocking; each state-owned polling call remains bounded; 500 Hz budget tolerates at most one new preflight poll plus existing state polls; the previously reviewed Teaching state does not issue motor targets; successful `io.read_inputs()` means a valid coherent board snapshot, not an actual E-stop chain; the application is the only CANopen runtime owner; HMI messages in a full queue cannot bypass the deferred target policy; and late raw safety producers cannot override current authoritative interlocks.

**Confirmed software gaps left open:** CM4 progress liveness not fully motion-gated; unsafe external snapshot age/sequence not independently validated; runtime fault diagnostic retains only current code rather than durable first cause; forced fault entry does not independently confirm motor deceleration; there is no board-provided physical stop confirmation handshake; SYNC-delivered cached RPDO behavior after an interrupted send remains a commissioning unknown.

**Hardware requirements:** test physical E-stop, protective stop, STO, drive bus-off reaction, CiA-402 quick-stop semantics, watchdog architecture, FDCAN error callbacks, CAN isolation/transceiver fail-safe and servo heartbeat supervision in an engineered commissioning procedure. This patch is **not** functional-safety certification or proof of physical stopping.

## Final director corrections: external target authorization contract

`SupervisorMessage` defines `SUPERVISOR_MESSAGE_GUIDED_TARGET` only as an array of six `int32_t` motor positions (`supervisor_task.h`). It has no authoritative producer identity, timestamp, sequence, replay/session marker, or verified source authorization. `TeachingRuntimeInputs` *separately* has `manual_guidance_active`, `motion_permitted`, `robot_homed`, and `timestamp_ms`, but they cannot securely authenticate an unrelated queued position envelope or demonstrate source freshness. `application_supervisor.c` configures an observation-only guidance port; it explicitly does not commission target dispatch. Consequently the production guided-target case **always rejects** and increments the saturated rejected-event counter. Neither current TEACHING state nor homing/motion flags can override this. Enabling future physical hand-guiding needs a separately reviewed source-authentication and freshness contract, not a hidden conditional in this patch.

## Final director corrections: bounded overload containment

- `SUPERVISOR_MAX_MESSAGES_CYCLE` is 16; the production FreeRTOS queue capacity is 32. After 16 dequeues, `uxQueueMessagesWaiting(g_queue)==0` is a complete batch; nonzero is overload. A producer enqueue after that observation belongs to the next cycle and is **not** a real-time atomic safety guarantee. External guided targets stay disabled irrespective of timing.
- `SUPERVISOR_DRAIN_TEACHING_EVENT` is distinct from `SUPERVISOR_DRAIN_OVERLOAD`. Only TEACHING may intentionally stop early for a pending Record/teaching event; an event in any other state cannot suppress state servicing.
- Overload forces recoverable FAULT E016 through the existing fault transition, including restart-gate inhibit and CAN drive permission revocation. The Supervisor continues into FAULT servicing *on the same 2-ms cycle*, rather than waiting for queue exhaustion. This avoids backlog starvation of FAULT handling. It intentionally does not claim completed controlled-stop acknowledgment during that fault: physical drive containment requires independently commissioned STO/quick-stop and hardware testing.
- The executable production Supervisor fixture includes sustained backlog during Approach and PathExecution controlled-stop phases, no-resume attempts, exact-full batch, adverse SAFETY behind queued target, Teaching Record break, and default-deny negative combinations (manual guiding, homed, runtime permission, timestamp, wrong state). These cases passed production-targeted WSL tests; physical stopping and timing remain unverified.

## Additional specific unverified assumptions

`uxQueueMessagesWaiting` is available as the ordinary FreeRTOS task-context API and has bounded cost; queue producers can race the zero-depth observation; `run_active_state()` FAULT service is safe and nonblocking; the high-rate command sender does not bypass this Supervisor owner; and the physical E-stop/STO/quick-stop behavior remains outside this software-only patch. A FAULT on queue congestion aborts any in-progress application-level controlled-stop acknowledgment; this is explicitly the fail-closed policy, not validation of physical halt.
