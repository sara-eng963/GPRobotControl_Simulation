# Stage 3 — C1 Release Baseline & Linked-Memory Audit

**2026-10-14 · branch:** sara/can-replacement  
**Status:** **PASS — C1 software release acceptance**, based on operator-supplied WSL command output for revision `ee6783f57fd7c9fa18c32dcba664fa4297d1bd1e`. Detailed gate results: `STAGE3_C1_ACCEPTANCE.md`. Full report files remain in the operator's local WSL evidence directory, not committed to GitHub.

## Purpose

Stage C1 (first of three Stage C batches) freezes the already-accepted
B1–B5 baseline, verifies the actual independently linked **CM7/CM4 ELF
section placement by every linker MEMORY region**, and captures hashes,
host fault-test evidence and HAL/IPC contracts.

This makes **no changes to MCU target source**, generated CubeMX code,
peripheral assignments or linker scripts. No new IDE rebuild is needed.

## Software evidence collected

- Rerun canonical B5 acceptance, including B1, B2, B3 and B4.
- Test official ST HAL/FreeRTOS EXTI source and startup-order contracts.
- Run host FDCAN HAL1 fault injection including bus-off; not physical CAN.
- ARM-compile M4 IPC API and verify nonblocking FreeRTOS message transport.
- Parse actual ALLOC ELF sections against CM7/CM4 official FLASH linker
  MEMORY declarations; reject out-of-range or overlapping allocations.
- Verify exclusive M7 allocation of the 4 KiB IPC, 16 KiB trajectory
  and 12 KiB validation SRAM4 reservations (NOLOAD/NOBITS).
- Read actual ELF PT_LOAD file-backed flash ranges, including initialized
  .data contents; report occupied bytes and end-of-region offsets.
- Record both binary SHA256 hashes and linker/map hashes when present;
  collect all gate stdout/stderr and machine-readable memory JSON.

## One operator command

~~~bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/check_stage3_c1_release.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
~~~

Final PASS means **software-only C1 accepted**, not complete Stage C:

~~~text
PASS: Stage C1 software release baseline, HAL fault mocks and actual ELF memory audit
~~~

Logs go into a unique folder printed by the script under
.simulation-logs/stage-c1/run-XXXXXXXX. Do not edit or restage the
independent CubeIDE project for this source-only verification batch.

## Stage C remaining phases

**C2:** targeted missing integrated failure scenarios (reset and stale
cross-core state, CAN loss, flash corruption and queue backpressure);
CAN bus 1 Mbps/6-node/500 Hz analytical capacity; FreeRTOS priorities,
stack/heap provisioning, and task/ISR scheduling budget.

**C3:** final software release and independent clean-build evidence,
fault matrix, signed-off open blockers and the existing hardware-arrival
commissioning plan.

**Not qualified by C1:** physical STM32 startup, powered HSEM/MPU/cache
coherence, independent-core restart policy, physical NOR persistence,
six-drive CAN timing, actual 2 ms deadline/WCET, task watermarks, or
certified safety chain. Fail closed until the associated tests and
physical providers exist.
