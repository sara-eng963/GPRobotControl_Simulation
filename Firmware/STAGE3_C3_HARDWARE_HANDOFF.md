# Stage C3 — Hardware Commissioning Handoff & Open Gates

**2026-10-14 · engineering status:** PRE-HARDWARE.  
**Motion enable authorization:** **NO-GO** until independent safety and hardware
tests are passed. This checklist is subordinate to
`HARDWARE_ARRIVAL_VERIFICATION_PLAN.md` (H0–H10).

## Responsibility and evidence

Assign an owner, qualified reviewer, board revision, device serials,
test date, acceptance thresholds and saved raw measurement links at
actual commissioning time. None are assigned or fabricated now.

| ID | Required work or evidence | Gate | Status |
|---|---|---|---|
| HW-01 | Actual MCU/PCB schematic, datasheet, errata, clock and voltage cross-check against saved CubeMX `.ioc` | H0–H2 | **BLOCKED: no board** |
| HW-02 | Current-limited power, CM7 and CM4 startup, HSEM wake/EXTI, MPU and cache policy, reset causes | H2–H3 | **BLOCKED** |
| HW-03 | Defined safe independent-core reset/resynchronization and stale-SRAM4 rejection under actual dual-core operation | H3 | **OPEN DESIGN + HARDWARE** |
| HW-04 | Real CAN transceiver, termination, arbitration, RX FIFO overflow/errors, bus-off, six-node identities/EDS and PDO mapping | H4–H5 | **BLOCKED** |
| HW-05 | 1 Mbps/500 Hz six-motor latency, per-cycle WCET, CPU/interrupt jitter, feedback age and error/retransmit margins | H6 | **BLOCKED; analytical only** |
| HW-06 | Both 512-byte CubeMX defaultTask stacks, FreeRTOS worker stacks and heap/ISR high-water measurements; verify M7 priority interference | H3/H6 | **BLOCKED** |
| HW-07 | Final real NOR part/wiring and STM32CubeMX-generated QSPI/HAL provider; same-artifact durable write/readback; power-interruption and restart tests | H7 | **BLOCKED; provider absent** |
| HW-08 | Physical external hand-guiding handle 6-axis F/T sensor: load path/mounting, calibration, RS-485, timing, failure/overload and admittance integration | H8 | **NOT YET COMMISSIONED** |
| HW-09 | Physical HMI transport, stop and diagnostic behavior during communication loss or core reset | H8 | **BLOCKED** |
| HW-10 | Independently engineered E-stop/protective stop, drive STO/inhibit, brake and power safety wiring, documented risk assessment | H8 | **BLOCKED; safety critical** |
| HW-11 | Controlled one-axis then multi-axis limit/encoder direction/brake/stop experiments, no uncontrolled motion | H9 | **BLOCKED by H0–H8** |
| HW-12 | Tool/frame calibration, industrial repeatability, thermal/dust/noise conditions and integrated failure/trace testing | H10 | **BLOCKED** |

## Verified in software (not substitutes for open gates)

B1–B5: integrated code, SRAM4 linker and host fault tests.
C1: linked ELF memory banks, FDCAN and IPC HAL mock contracts.
C2: CANopen RX now bounded to 32 frames per owner poll; host
continuous-backlog tests, production-source TIM2/TIM3 fault mock, and
static RTOS/CAN wire model.

At 1 Mbps with the assumed 6 RPDO + 6 TPDO + SYNC frame set,
the C2 model estimated **1,091–1,315 µs** per 2 ms without/with
conservative stuffing, and **1,770 µs** during a hypothetical aligned
seven-heartbeat burst. These are not physical communication guarantees.

## Non-negotiable no-go rules

- A software-only status, generic camera human detector or FreeRTOS
  task does **not** replace a properly engineered safety chain.
- Missing M4 NOR worker/provider is not silently replaced by a RAM
  fake in firmware; trajectory validation remains unavailable.
- Missing physical CM7 board config must keep drive startup disabled.
- Never reset the M7-owned SRAM4 FreeRTOS Message Buffer behind an
  active CM4 kernel; independent reset recovery is unqualified.
- Run physical H0–H8 with motor power/inhibit secured before H9 motion,
  only with competent electrical/mechanical safety supervision.
- Hardware failures must be labelled FAIL/BLOCKED, not reclassified
  PASS because the host or target-ELF tests succeed.

## Required hardware test records

For each H-gate, fill: `test ID | build SHA | board/drive IDs | setup |
requirement | actual trace/measurement | PASS/FAIL/BLOCKED | owner |
reviewer | date | follow-up commit`.

Retain raw CAN traces, timer GPIO/logic-analyzer captures,
fault logs, power/flash interruption events, relevant MCU register
snapshots, high-water task stacks, and actual stop-energy evidence.
Explicitly document whether 500 Hz with six drives passes before
choosing one versus two FDCAN buses. Do not invent runtime thresholds
without a derived requirement and acceptance methodology.

## Release decision

**C3 can release a reproducible *pre-hardware software candidate*.**
Physical robot energization, protective motion, hand-guiding, and
industrial operation remain **NOT RELEASED** until all safety-critical
hardware and firmware qualification gates are signed off.
