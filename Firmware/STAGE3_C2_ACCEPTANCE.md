# Stage 3 — Stage C2 Software Acceptance

**Date:** 2026-10-14  
**Repository:** `sara-eng963/GPRobotControl_Simulation`  
**Branch:** `sara/can-replacement`  
**Tested source revision:** `9c0dd0d3f02240c663f4fa415121be7c75c12edd`  
**Decision:** **PASS — Stage C2 software gates. C3 and physical qualification remain OPEN.**

## Operator test provenance

The operator supplied the full terminal output after a successful
`git pull --ff-only origin sara/can-replacement` and staging:

```bash
bash Firmware/Tests/stage_stage3_c2_canopen.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
bash Firmware/Tests/check_stage3_c2_acceptance.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

The stage script reported both M7 CANopen master `.c` and `.h` files
staged successfully. The acceptance script checked source parity and
newness of the independent M7 ELF relative to the staged CANopen sources.
The pasted output **does not independently include the CubeIDE build
console**, so this record does not claim possession of that console.

## Passed software checks

| Stage C2 acceptance step | Reported result |
|---|---|
| Source-based CAN DLC and conservative bit-stuffing self-test | PASS |
| FreeRTOS stack/priority/IRQ audit self-test | PASS |
| Nested C1 acceptance (including B1–B5, HAL, FDCAN and M4 IPC) | PASS |
| Host CMake configure and build | PASS |
| Continuous-RX bound and original CANopen master regression | PASS: 2/2 CTest tests |
| M7 production-C timer fault injection | PASS |
| HSEM adapter mock on both core configurations | PASS |
| Source-anchored CAN model generation | PASS |
| Source-based RTOS task/interrupt audit generation | PASS |
| Full C2 acceptance script | **PASS** |

The timer host harness reported actual C code extracted and exercised
with HAL/FreeRTOS mocks, covering invalid timer clock, partial startup
rollback, TIM3 counting notification behavior, missed/coalesced periods,
2 ms overrun and TIM2 overflow behavior.

## Analytical CAN wire budget — NOT measured latency

At a nominal 1 Mbps, 2 ms cycle with six 4-byte RPDO4, six 6-byte TPDO4
and one zero-byte SYNC frame, the model reported:

| Frame scenario | Modelled wire time per 2 ms |
|---|---:|
| No bit stuffing | 1,091 µs |
| Conservative stuffing envelope | 1,315 µs |
| Envelope plus seven coincident heartbeats | 1,770 µs |

The burst scenario assumes aligned heartbeats. The model omits SDO,
EMCY, retransmissions, traffic from other nodes, software and motor
response latency and other controller queueing. It is **not** proof of
actual 500 Hz closed-loop scheduling.

## Report location (operator WSL, not GitHub)

`/home/sara/freertos-pc-test/.simulation-logs/stage-c2/run-zYRkpvH3/`

The script generates per-gate logs, an embedded C1 evidence run, CAN
budget JSON/Markdown, RTOS static audit JSON/Markdown, and ELF/source
SHA256 manifest. Only the terminal summary was supplied here; the
individual local files have **not** been independently inspected or
committed. No unprovided actual stack figures or hashes are inferred.

## Unverified physical and timing risks

- Neither a real six-drive 1 Mbps CAN bus nor physical 500 Hz command
  + feedback deadlines have been demonstrated.
- Actual STM32 CPU WCET, interrupt latency, clock configuration under
  load and stack high-water/heap peaks remain unmeasured.
- Provisional CubeMX defaultTask stacks remain 512 B each.
- M7 task-priority scheduling interference remains to be measured.
- Independent-core restart with shared IPC/storage is not qualified.
- Physical NOR atomic commit/persistence and safety-rated E-stop/STO
  are outside this software acceptance.
- No physical board provider is commissioned; the firmware must remain
  fail-closed until board-specific safety, drive and storage qualification.

**Next:** Stage C3 — final software evidence/reproducibility, blocker
matrix, handoff and explicit on-target commissioning plan.
