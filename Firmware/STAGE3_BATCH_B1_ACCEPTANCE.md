# Stage 3 — Batch B1 Software Acceptance

Date: **2026-10-14**  
Branch: `sara/can-replacement`  
Source revision at test: `a82eab77f180692098602cd66ef433d46c07d24c`  
Evidence: operator-reported successful independent STM32CubeIDE CM7/CM4 builds and full acceptance-script output pasted to the project conversation on 2026-10-14.

## Decision

**PASS: Batch B1 M7 motion source integration, host regressions, ARM compilation and target ELF symbol gates.**

Verified console output from `Firmware/Tests/check_stage3_batch_b1_acceptance.sh`:

- CM7 Debug/Release assembler and C compiler `Application/Intercore` include paths and canonical B1 source parity: PASS.
- M7 board gating, IPC ownership, fail-closed CANopen composition: PASS.
- 16 CM7 and 2 CM4 application ARM translation units compile: PASS.
- Four CTest tests (trajectory stream, trajectory path adapter, Avatar M CANopen, CANopen master): **4/4 passed**.
- PathExecution prefetch host suite: PASS, including absolute deadline skip, following error and motion permission controlled-hold request, delayed TPDO, 20 ms feedback expiry, missed final target, startup prefill failure, pause/replay/abort, runtime storage starvation and rejected controlled hold.
- M7 strong target ELF symbols: `gp_traj_execution_bind`, `gp_traj_m7_begin`, `gp_traj_m7_take`, `gp_traj_m7_finish`, `gp_traj_m7_discard`, `state_path_execution_step`, `canopen_master_init`, `application_supervisor_start`: PASS.
- Batch A ELF regression: PASS; `HAL_FDCAN_Init`, `HAL_FDCAN_MspInit`, HAL CAN adapter, CANopen master, supervisor and CM4 IPC symbol checks.
- Script final line: `PASS: Stage 3 Batch B1 composed M7 motion software gates and ELF inventory`.

Operator separately stated both target projects built successfully. The supplied transcript does not include complete CubeIDE build logs or binary hashes, so the independent clean-build process was **user-reported**, not personally witnessed.

## Reported firmware aggregate sizes

| Core | .text (bytes) | .data (bytes) | .bss (bytes) |
| --- | ---: | ---: | ---: |
| CM7 | 170272 | 112 | 104880 |
| CM4 | 24644 | 100 | 21148 |

These **aggregate** numbers are not a bank-by-bank flash/SRAM utilization report and do not replace Batch C memory, stack, heap and timing qualification.

## Residual restrictions

- The target test printed: `NOTE: no unresolved weak board supplier symbol displayed (check linker/map manually)`. Source-level board gating passed, but exact weak-symbol elimination/retention requires linker-map review; this note is **not** proof that physical board configuration exists.
- No physical STM32H745, CAN motor drives, NOR flash, E-stop/STO chain, or actual 2 ms timing were tested.
- M4 persistent trajectory producer / durable storage remain to be integrated in subsequent Batch B work.
- One acknowledged software-controlled hold request is **not** independently verified motor deceleration or a certified protective stop.

**B1 is closed at the software-verification level. The overall Stage 3 remains open (B3/B4, M4 HMI lifecycle, consolidated Batch B, Batch C).**
