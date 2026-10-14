# Stage C2 — Integrated Fault Injection and Timing Feasibility

**Status:** PASS — C2 software acceptance based on the operator's WSL output on 2026-10-14, at source revision `9c0dd0d3f02240c663f4fa415121be7c75c12edd`. Refer to `STAGE3_C2_ACCEPTANCE.md` for the gate-by-gate evidence. Physical qualification and C3 remain pending.
Branch: sara/can-replacement. Evidence scope is software-only.

## Concrete issue corrected

CANopen master previously drained incoming frames in an unbounded loop.
Under endless RX, that can prevent a bounded 500Hz Supervisor tick.
C2 caps the poll to **32 frames** (two 16-frame CubeMX FIFO0 depths)
per call. This is a **work-count bound**, not proof of the 2ms WCET.
A new host test supplies endless valid TPDO4 feedback, asserts the
limit, and exercises empty/error backends.

## Stage C2 tests

- Re-run accepted B1-B5 + C1 HAL/IPC/FDCAN and linked memory-bank gate.
- New exact-source CANopen continuous-RX bound; existing CANopen suite.
- Existing extracted REAL M7 timer C fault harness: TIM2 overflow,
  invalid clock, lost/coalesced TIM3 notifications and 2ms overrun.
- Source-anchored 1Mbps Classic CAN 2ms load model for six 4-byte RPDO4,
  six 6-byte TPDO4 and one SYNC, including bit-stuffing and heartbeat
  burst sensitivity. This is NOT measured controller latency.
- Static source audit of FreeRTOS 1000Hz clocks, configured heaps and
  stack arrays, interrupt priority ceilings, TIM2/TIM3 NVIC5, EXTI NVIC6
  and task priority. Hardware stack watermarks remain unmeasured.
- Optional HSEM FreeRTOS POSIX mock only when its external dependency exists.

## Run

~~~bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/stage_stage3_c2_canopen.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
~~~

**In STM32CubeIDE:** Refresh and CLEAN-BUILD CM7 Debug only.
CM4 source/build unchanged.

~~~bash
bash Firmware/Tests/check_stage3_c2_acceptance.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
~~~

Expected final line:

~~~text
PASS: Stage C2 bounded CANopen RX, timer faults, CAN budget, RTOS contracts and C1 regression
~~~

Individual logs, CAN/RTOS JSON+markdown reports, C1 nested evidence and
ELF/source hashes are stored in the printed Stage C2 run directory.

## Remaining release blockers

- Independent M7/M4 reset under live IPC/storage: no safe restart
  protocol has been physically implemented or qualified.
- M4 NOR write/read/commit timing and power-fail atomicity not validated
  on selected actual flash with official HAL.
- 500Hz WCET, CAN bus errors and jitter, memory/task stack watermarks
  and true queueing behavior not measured on STM32 or six motors.
- Both CubeMX defaultTask stacks are provisionally **512 bytes**; they
  must be checked under actual system load before commissioning.
- M7 defaultTask nominal priority may preempt Supervisor priority4;
  study scheduling traces on target, do not assume a timing guarantee.
- Independent physical E-stop/STO and safety circuits not yet certified.

C2 is a finite software gate, not physical motor release authorization.
