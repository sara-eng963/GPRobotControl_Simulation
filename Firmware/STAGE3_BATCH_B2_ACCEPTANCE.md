> Historical engineering record: the ZIT6 integration supersedes these build/staging instructions. See [current firmware instructions](README.md). The original evidence below is retained.

# Stage 3 Batch B2 — SRAM4 Validation Reservation: Software Acceptance

- Recorded: 2026-10-14
- Branch: `sara/can-replacement`
- Tested source revision: `f863c46c1ed91748ba14d838462c908c1ee23e16`
- Baseline B2 allocation: `8bbfede276c9e898f2adeb30b50a7c1deb9db128`
- Evidence: operator-pasted WSL command and complete acceptance output in project conversation on 2026-10-14.
- Decision: **PASS — B2 static/ARM/linker/target-image allocation**, with the limitations below.

## Operator commands

```bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/stage_stage3_batch_b2.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
bash Firmware/Tests/check_stage3_batch_b2_acceptance.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

Staging output confirmed all six canonical source/linker inputs copied into the separate CubeIDE verifier. Acceptance output, in order:

- `PASS: six Batch B2 verifier source/linker inputs match canonical Git files`
- `PASS: Stage 2A CM7/CM4 ABI equality and SRAM4 gross-capacity check`
- `PASS: Stage 2C synthetic linker placement on both cores and both link modes`; all four cases explicitly include `gp_stage3_linker_validation` at `0x38005000`, size `0x3000`, NOLOAD.
- `PASS: production shared allocations at correct M7 SRAM4 addresses`; both FLASH and RAM cases explicitly include `intercore_validation_storage` at `0x38005000`, size `0x3000`, NOLOAD.
- `PASS: Stage 3 Batch B2 source/ABI, four linker probes and independent target ELF/map`
- `DEFERRED: typed validation protocol, M4 flash worker, storage device, hardware runtime.`

The runner checked source parity, file freshness, required CM7/CM4 ELFs, the M7 symbol table, NOBITS section and linker map. The pasted terminal output does **not independently include a fresh STM32CubeIDE Clean/Build console log**; do not silently upgrade this result to an independently witnessed clean build.

## Physical memory contract

| Slot | Base | Size | Ownership and status |
| --- | --- | ---: | --- |
| M4/M7 FreeRTOS IPC | `0x38000000` | 4 KiB | Already implemented |
| Trajectory ring | `0x38001000` | 16 KiB | Already implemented |
| Validation reservation | `0x38005000` | 12 KiB | **B2 accepted** as NOLOAD M7-owned reservation; deliberately inactive |
| Remaining SRAM4 | `0x38008000` | 32 KiB | Unallocated |

The previous Stage 2 text `Shared spare` is a historical label; the current B2 production linker reservation is explicitly allocated.

## Open Batch B/C gates

1. B1: M7 trajectory consumer ↔ PathExecution ↔ CANopen command composition and failure tests.
2. B3: Typed `GpValidationShared` cold-boot initialization/ABI; M7 client, M4 single-owner worker, cancellation, stale-epoch and reboot handling.
3. B4: M4 production storage/readback/refill with a physical-part-specific CubeMX/HAL adapter. The real bus/pins/flash part remain unconfigured; no simulated or successful physical operations may be substituted.
4. B5: M4 HMI command/status lifecycle and target imaging; consolidate Batch B acceptance.
5. Batch C: full software fault qualification, per-region memory budgets, stack/heap and timing/CAN load analysis, final clean dual-core release evidence.

**No physical board, drive, NOR, motion, safety or 500 Hz runtime verification is claimed.**
