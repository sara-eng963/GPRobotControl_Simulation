# Stage 3 — C1 Release Qualification Acceptance

**Date:** 2026-10-14  
**Repository:** `sara-eng963/GPRobotControl_Simulation`  
**Branch:** `sara/can-replacement`  
**Accepted source revision:** `ee6783f57fd7c9fa18c32dcba664fa4297d1bd1e`  
**Decision:** **PASS — C1 software gates. Stage C overall remains OPEN.**

## Exact operator evidence

The operator executed `Firmware/Tests/check_stage3_c1_release.sh`
against the independent STM32CubeIDE project
`/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro`.
Their pasted terminal output records all of the following:

| C1 gate | Outcome |
|---|---|
| Memory-audit Python self-test | `PASS: Stage C1 parser unit tests (bank limits, ELF sections and LOAD ranges)` |
| B1–B5 consolidated host/ARM/dual-core ELF regressions | `PASS: batch_b_consolidated` |
| HAL/FreeRTOS EXTI notification compile/startup checks | `PASS: hal_exti_contract` |
| ST HAL1 FDCAN host fault-injection mocks | `PASS: fdcan_hal1_fault_mocks` |
| M4 IPC application ARM compile and nonblocking API | `PASS: m4_ipc_api` |
| Real linked ELF section placement and shared-SRAM4 ownership audit | `PASS: Stage C1 bank-by-bank linked ELF occupancy and SRAM4 ownership audit` |
| Full C1 script | `PASS: Stage C1 software release baseline, HAL fault mocks and actual ELF memory audit` |

Programmed-image *file-backed FLASH PT_LOAD occupancy* reported by the audit:

- **CM7: 172600 bytes** (168.55 KiB).
- **CM4: 29768 bytes** (29.07 KiB).

These quantities are **not** total RAM usage, task stack high watermarks,
NOR storage size, or measured physical flash programming. The local
`memory_regions.md` and `memory_regions.json` contain actual per-bank
addresses, sections and remaining linker headroom; those details were not
included in the pasted terminal excerpt.

## Local evidence directory

`/home/sara/freertos-pc-test/.simulation-logs/stage-c1/run-c89Rjfv7/`

The script reported:

- `memory_regions.md`
- `memory_regions.json`
- `evidence_manifest.txt`
- `qualification_status.md`
- Separate per-gate logs.

This record cites the **operator's stated output**, not independent access
to those local files. Do not insert inferred hashes or RAM bank occupancy.

## Qualification boundary

C1 proves the source-backed, host/mock, ARM compile, dual-core ELF
presence and static memory-allocation gates executed by the scripts.
It does **not** establish real-world execution or a deployable system:

- Independent M7/M4 restart coherency while shared SRAM4 is active.
- Powered ST HAL HSEM/EXTI or MPU/cache behavior.
- Real STM32 500 Hz timing, CAN 1 Mbps utilization, bus-off and PDO jitter.
- NOR persistence or power-loss safety on the selected physical device.
- Per-task stack/heap **runtime** high-water measurements.
- External safety-rated E-stop/STO wiring and robot motion.

**Next:** Stage C2 integrated fault-injection and timing/resource analysis,
then Stage C3 release-signoff evidence and the hardware commissioning gates.
