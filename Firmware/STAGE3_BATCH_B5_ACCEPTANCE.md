# Stage 3 — Batch B5 and Consolidated Batch B Acceptance

**Date:** 2026-10-14  
**Branch:** `sara/can-replacement`  
**Accepted source revision:** `26217a6ce44bdd1b941edefcdb1bee98ae54e003`  
**Status:** **PASS — B5 and B1–B5 consolidated software gates**

## Provenance

Operator supplied the complete WSL transcript of
`Firmware/Tests/check_stage3_batch_b5_acceptance.sh` run against
`/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro`, following
`git pull --ff-only origin sara/can-replacement`. They had separately
reported successful **both-core STM32CubeIDE IDE builds**. The current
transcript includes compiled ARM probes and linked Debug ELF checks,
but does not contain the IDE console or ELF hashes.

## Operator-reported acceptance results

| Check | Evidence/result |
|---|---|
| B5 HMI lifecycle host regression | PASS: ABI, invalid frames, stale sequences, counter rollover, freshness, FAULT/ESTOP, state-change discard |
| B1 motion | PASS: CANopen, trajectory consumer, PathExecution/hold/feedback/fault regressions; 4/4 designated CTests |
| B2 SRAM4 | PASS: IPC + trajectory + validation partition; synthetic and production object linker gates and M7 ELF/map gate |
| B3 storage validation | PASS: queue backpressure, cancellation, offline M4 backend, worker/ELF symbols |
| B4 playback | PASS: fake-NOR CRC readback, refill/replay, identity mismatch and read failure, cancellation and M7/M4 ELF symbols |
| B5 source/ARM | PASS: staged source parity and ARM compile of 20 CM7 + 7 CM4 application translation units |
| B5 target images | PASS: linked target M7/M4 IPC symbols; CM4 HMI policy symbols; source-to-ELF freshness |
| Overall | `PASS: Stage 3 Batch B5 consolidated B1-B5 software and dual-core ELF gates` |

Last script-reported image sizes (aggregate sections, NOT SRAM bank budgets):

| Core | text | data | bss |
| --- | ---: | ---: | ---: |
| CM7 | 172480 | 112 | 104944 |
| CM4 | 29660 | 100 | 37248 |

The nonfatal raylib `CMAKE_BUILD_TYPE` warning is from the host test
configuration, and the run continues through full acceptance.

## Acceptance boundary

This **does close Stage 3 Batch B software integration** as defined
in the current test scripts. It is not end-to-end physical robot acceptance.
No official or actual NOR flash write/read/persistence was measured;
the physical M4 storage provider remains absent, so deployment must
remain fail-closed. The source-level tests and linked ELFs do not establish:

- Independent-core reset coherency, HSEM/MPU/cache operation under powered hardware or STO/E-stop behavior
- Timing jitter and 500 Hz closed-loop execution on the STM32 or real six-node CAN bus
- Per-memory-bank RAM and flash utilization, runtime stack high-water and WCET margin
- Physical display/HMI protocol, power loss recovery or storage endurance
- Certification or safety-rated protective stop

## Next stage

**Stage 3C — Integrated software release qualification:** produce
reproducible regression evidence across boot/reset, stale IPC, bus-off,
invalid CAN feedback, storage faults, deadline misses, and physical
provider boundaries; audit target memory regions, task/ISR priorities
and CAN 1 Mbps × six-node 500 Hz budget. Separately plan on-target
commissioning with available board, drives and NOR hardware.

Do not convert mock/host PASS into physical PASS.
