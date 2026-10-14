# Stage 3 — Batch B4 Software Acceptance

- **Date:** 2026-10-14
- **Branch:** `sara/can-replacement`
- **Source revision under test:** `4ac15f3f10d5c08985c74c14b7e40a8ac874f6b4` (reported Git pull fast-forward to `4ac15f3`).
- **Evidence:** operator-pasted WSL session containing `stage_stage3_batch_b4.sh` followed by the complete output of `check_stage3_batch_b4_acceptance.sh` using `GP_Stage3_Repro`.
- **Decision:** **PASS — Batch B4 software acceptance**, at the explicitly defined source-parity, ARM compile, host fault/regression and independent target ELF symbol gates.

## Acceptance evidence

1. `PASS: Stage B4 canonical M7 request/M4 producer staged; ready to CLEAN-BUILD both cores.` and `PASS: B4 canonical staging parity for 18 firmware sources`.
2. `PASS: Stage 2A CM7/CM4 ABI equality and SRAM4 gross-capacity check`.
3. `PASS: Stage 2C synthetic linker placement on both cores and both link modes`, including the 12 KiB validation reservation at `0x38005000`.
4. `PASS: production shared allocations at correct M7 SRAM4 addresses`; IPC at `0x38000000`, trajectory at `0x38001000`, validation at `0x38005000` in FLASH/RAM tests.
5. Portable ARM compile-only: **20 CM7 translation units** and **5 CM4 translation units** passed.
6. Host durable-playback tests: M7 write → M4 CRC scan → ring prefill/refill → replay; CRC mismatch and NOR-read rejection; cancellation during blocking read; identity mismatch and absent worker all **PASS**. These tests use a **host-only fake NOR**.
7. M7 PathExecution tests: board callbacks and stream identity, prefill before wire/motor enable, feedback-age and following-error holds, missed deadlines, restart and starvation handling all **PASS**.
8. Linked M7 target symbols: `gp_playback_m7_request`, `gp_playback_m7_cancel`, `gp_traj_execution_attach_playback`, `application_supervisor_start`: **PASS**.
9. Linked M4 target symbols: `gp_playback_m4_init`, `gp_playback_m4_step`, `gp_traj_m4_prepare`, `gp_traj_m4_step`, `application_validation_service_start`: **PASS**.
10. Final output: `PASS: Stage 3 Batch B4 M4 durable-playback software and dual-core ELF gates`.

## Scope and limitations

The console output establishes software acceptance and freshness/retention checks for both pre-existing target ELFs. It **does not include an independent CubeIDE build-console transcript** showing each clean build, or the actual target-binary checksums.

**Not qualified:** real external NOR physical writes/readback and persistence; STM32CubeMX peripheral pins/clock/flash provider; independent M4/M7 restart or brownout; board stack-watermarks, real 500 Hz control scheduling, CAN buses/motors or certified protective stop.

No physical storage backend provider has been added. The M4 worker remains fail-closed without a commissioned HAL-backed NOR interface.

**B4 software gate accepted.** Next work: remaining M4 HMI/lifecycle composition, consolidated Batch B verification and Stage C hardware qualification.
