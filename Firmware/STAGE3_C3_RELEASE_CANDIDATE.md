# Stage C3 — Final Pre-Hardware Software Qualification

**Date:** 2026-10-14  
**Branch:** `sara/can-replacement`  
**Status:** **C3 candidate pending independent dual-core clean-build and C3 acceptance.**  
**Release category:** pre-hardware *software-qualified candidate only*. **NOT commissioned / NOT authorized for robot motion.**

## Accepted historical software baselines

| Gate | Tested source revision | Evidence |
|---|---|---|
| B1–B5 software integration | `26217a6ce44bdd1b941edefcdb1bee98ae54e003` | `STAGE3_BATCH_B5_ACCEPTANCE.md` |
| C1 memory, IPC, HAL mock and link gates | `ee6783f57fd7c9fa18c32dcba664fa4297d1bd1e` | `STAGE3_C1_ACCEPTANCE.md` |
| C2 bounded CANopen, timer fault and CAN/RTOS audit | `9c0dd0d3f02240c663f4fa415121be7c75c12edd` | `STAGE3_C2_ACCEPTANCE.md` |
| C3 final candidate | Frozen at operator preparation | **PENDING** `check_stage3_c3_release.sh` |

The above results were supplied through operator-run WSL acceptance
transcripts. C3 must revalidate against *one frozen revision* and
independently refreshed target ELFs; historic PASS is not a substitute.

## C3 finite acceptance contract

1. Snapshot clean tracked source on the correct branch. Historical accepted
   revisions must be ancestors of the frozen source commit.
2. Check that the latest bounded-RX CANopen `.c/.h` are staged in the
   separate CubeIDE workspace. Create a build-start marker.
3. **Operator:** clean and rebuild *both* CM7 and CM4 Debug targets in
   STM32CubeIDE, rather than trusting prior build outputs. Save IDE
   compiler/linker console logs separately if possible.
4. Reject an ELF predating the build-start marker; verify both ELF
   identities, boot/HAL and source paths via C2 (which reruns C1/B1–B5).
5. Build and run the **complete curated portable firmware regression set**
   (18+ host CTest suites: IPC, streams, HMI, validation, storage, path,
   CANopen, and all existing portable states).
6. Capture SHA256 hashes, ELF/source/toolchain versions, build logs,
   map files when present, `.ioc`, linker scripts and all C2 report
   references. Package two ELFs marked **NOT FOR ROBOT MOTION**.
7. Publish every deferred physical/runtime item with gate owner and
   evidence requirement in `STAGE3_C3_HARDWARE_HANDOFF.md`.

**A C3 PASS closes only the defined software-qualification phase.** The
project is not a fully commissioned, physically safe robot controller.

## Operator sequence

```bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement

bash Firmware/Tests/prepare_stage3_c3_release.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

Keep the `PREP DIRECTORY` line. Refresh, **Project → Clean** (CM7
and CM4) and **Build both Debug** projects in STM32CubeIDE.

```bash
bash Firmware/Tests/check_stage3_c3_release.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

The last successful preparation checkpoint is located automatically.
The two-argument form also accepts a specific `PREP DIRECTORY` if
you want to qualify an older release snapshot explicitly.

Only after a final script PASS may the operator record C3 as accepted.
The successful run prints an evidence root and a software-only ZIP
candidate. No script flashes the MCU or permits actual motor operation.

## Hardware-safety statement

The canonical board-supplier functions are intentionally absent.
Physical CAN drive configuration, E-stop/STO, M4 NOR HAL provider,
and external handle F/T interface remain uncommissioned. No
physical CAN jitter, CPU WCET, stack watermarks, independent-reset
coherency, motion limits or safety-category certification exists.
See `HARDWARE_ARRIVAL_VERIFICATION_PLAN.md` for H0–H10 procedures.

**Never substitute host mocks/ELF symbols for physical validation.**
