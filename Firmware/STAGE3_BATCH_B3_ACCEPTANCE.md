# Stage 3 — Batch B3 Software Acceptance

- **Date:** 2026-10-14
- **Branch:** `sara/can-replacement`
- **Accepted implementation revision:** `734d8f498c928a9aaaa12ecb327025d00fefd698`
- **Source of evidence:** operator-pasted WSL output of `Firmware/Tests/check_stage3_batch_b3_acceptance.sh`, after `git pull --ff-only origin sara/can-replacement`.
- **Decision:** **PASS — source parity, shared SRAM4 ABI and linker placements, portable ARM compilation, host validation protocol tests, both target ELF symbol gates.**

## Command

```bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/check_stage3_batch_b3_acceptance.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

## Evidence (as reported by operator)

- `PASS: canonical B3 M7/M4 sources and CubeIDE include/source settings`
- `PASS: B3 staging/acceptance Bash syntax`
- `PASS: Stage 2A CM7/CM4 ABI equality and SRAM4 gross-capacity check`
- `PASS: Stage 2C synthetic linker placement on both cores and both link modes` (validation at `0x38005000`, 12 KiB, NOLOAD in all four synthetic probes)
- `PASS: production shared allocations at correct M7 SRAM4 addresses` (FLASH/RAM production object checks)
- `PASS: CM7 compiled 19 application translation units`
- `PASS: CM4 compiled 4 application translation units`
- Host regression: absent/shut-down M4 backend cannot authorize validation; full queue returns BUSY without lost samples; durable commit acknowledgement; flash failure and stale queue abort; cancellation waits for in-flight operation — **all passed**.
- Both target images: required M7 validation initialization/client/proxy/idle and Supervisor symbols, and required M4 startup/service/worker/shutdown symbols all found.
- Final script output: `PASS: Stage 3 Batch B3 source, host protocol, ARM and both target ELF gates`.

## Exact qualification boundary

Acceptance asserts target ELF existence, required source freshness and linked symbols. Operator output does not separately provide the full STM32CubeIDE clean-build console, linker maps by RAM bank, test-generated report files, binary hashes or physical M7/M4 execution. The positive result therefore applies **only to the declared B3 software gates**, not to physical system behavior.

- No actual NOR peripheral, flash part, pin/clock mapping or physical storage provider is commissioned.
- The no-provider default keeps M4 validation storage worker offline and prevents M7 from accepting new validation transactions.
- Independent M7/M4 restart and stale worker-ready bit recovery are not qualified. A coherent two-core restart strategy remains a Batch C gate.
- A host fake backend durable-commit response does not verify actual flash persistence, CRC/readback, write interruption or power loss.
- No 500 Hz board timing, motor operation or safety interlock is established.

**B3 is accepted. Stage 3 remains open: B4 M4 NOR storage/trajectory producer, M4 HMI lifecycle completion, consolidated Batch B qualification, Batch C release assessment.**
