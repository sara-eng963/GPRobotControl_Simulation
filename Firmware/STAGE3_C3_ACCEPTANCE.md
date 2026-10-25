> Historical engineering record: the ZIT6 integration supersedes these build/staging instructions. See [current firmware instructions](README.md). The original evidence below is retained.

# Stage 3 C3 — Final Pre-Hardware Software Acceptance

**Date:** 2026-10-14  
**Repository:** `sara-eng963/GPRobotControl_Simulation`  
**Branch:** `sara/can-replacement`  
**Accepted candidate source revision:** `8c387c9cb09ede9c00ed07896d8d9ddc71844677`  
**Decision:** **PASS — Stage 3 pre-hardware software qualification (B1–B5, C1–C3).**  
**Hardware/motion release decision:** **NO-GO — not commissioned**.

## Operator-reported evidence

The operator executed:

```bash
bash Firmware/Tests/check_stage3_c3_release.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

The resulting WSL transcript reports:

| Qualification gate | Observed result |
|---|---|
| CM7 and CM4 freshly linked after frozen source checkpoint | `PASS` |
| Source ancestry from previously accepted B5/C1/C2 revisions | `PASS` |
| Nested C2 acceptance, including C1 and B1–B5 | `PASS` |
| Production-C TIM2/TIM3 timer fault mocks, HSEM adapter and CAN/RTOS models | `PASS` through C2 |
| Portable host CMake configure and build | `PASS` |
| Curated portable CTests | **19/19 passed, 0 failed** |
| No unexpected commissioned motor/NOR board provider linked | `PASS` |
| C3 candidate archive/manifest generation | `PASS` |
| Source-locked CM7/CM4 ELF evidence and SHA256 manifest | `PASS` |
| Final C3 gate | `PASS: Stage C3 final pre-hardware software qualification and dual-core candidate` |

C3 includes fresh ELF timestamps relative to the C3 build marker,
revision consistency, frozen `.ioc` / linker / `.cproject` hashes, and
the previous source/target memory and ARM compilation acceptance tests.
It does **not** independently document the operator clicking
`Project > Clean` or include CubeIDE console output.

## Local evidence location

`/home/sara/freertos-pc-test/.simulation-logs/stage-c3/run-0Jm9dMlz/`

The output reports `PASS: candidate_manifest` and a source-locked
candidate containing two linked ELF images, SHA256 manifest and copies
of C1/C2 and C3 software reports. The raw ZIP/file manifests and
actual hashes remain **on the operator's WSL machine** and were not
uploaded to or independently inspected in this GitHub documentation
commit. Do not invent them from the summary console output.

## What this acceptance means

**Complete at software qualification level:** the Stage 3 dual-core
firmware integration and its tested portable inter-core, trajectory,
validation, storage abstractions, HMI policy, CANopen and timing
contracts. The terminal PASS covers the exact scripted gates and
accepted branch source at the time of preparation.

**Still not completed/qualified:**

- Commissioned physical M7 safety I/O, six motor/drive configuration,
  CAN transceiver and real bus timing; 500 Hz hardware WCET/jitter.
- Commissioned M4 external NOR interface, durable persistence and
  recovery after actual power interruption.
- Actual simultaneous/independent dual-core reset/IPC recovery,
  MPU/cache/HSEM behavior under powered execution.
- Stack high-water/heap/interrupt timing measured on the board.
- Physical HMI electrical interface and external handle F/T sensor
  integration, including associated control behavior.
- Independent robot E-stop, safety-rated STO/brakes, protective-stop
  architecture, controlled motion, and full engineering risk review.

**Handoff:** `STAGE3_C3_HARDWARE_HANDOFF.md` and
`HARDWARE_ARRIVAL_VERIFICATION_PLAN.md` define H0–H10. The candidate
is **NOT FOR ROBOT MOTION** until the safety-relevant hardware and
commissioning gates are independently signed off.

**Stage 3 software phase: CLOSED (qualified pre-hardware candidate).**
