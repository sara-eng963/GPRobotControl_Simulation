# Stage 3 Batch B3 — Typed validation memory and M4 worker composition

Date: 2026-10-14. Branch `sara/can-replacement`.

**Status:** SOURCE IMPLEMENTED, LOCAL ACCEPTANCE PENDING. This report does
not claim target builds or test runs that have not been supplied.

## Integration

- SRAM4 `0x38005000` remains one M7-owned `0x3000` NOLOAD symbol.
  The typed `GpValidationShared` begins at offset zero inside this
  reservation; M7 initializes it before HSEM-0 wakes M4.
- ABI version advanced to 2. A lightweight M4 boot wire header verifies
  magic/version with no dependence on M7 kinematics or robot validation code.
- M4 sets `worker_ready=1` **only after** a complete real storage backend
  is accepted by `gp_validation_m4_init`. M7 rejects new validation
  transactions until the worker is online and the shared queue is idle.
- M7 Supervisor uses the existing asynchronous
  `gp_validation_m7_storage` in its lower-priority validation task;
  physical callbacks never execute on the M7 control cycle.
- M4 optional FreeRTOS `NorValidate` task calls the established
  `gp_validation_m4_step`; it does not share M4 HMI private queues.
  Without a physical board module implementing
  `gp_board_m4_supply_validation_backend`, the task is not created,
  `worker_ready` remains zero, and IPC/HMI continues separately.
- The M4 worker stack is **provisional 768 words**, subject to Stage C
  stack-watermark and resource qualification. No physical NOR bus/pin
  settings or successful flash operation have been invented.

## Dependencies and restrictions

M4 hardware-part-specific HAL NOR provider remains **Batch B4**.
Independent core restarts during active storage IO are **not** approved:
a supervised two-core quiescent restart/reset policy remains mandatory
for Stage C. A stale M4-ready bit cannot be used as evidence that an
independently rebooted core is healthy; this gate requires hardware runtime
fault injection later.

## Operator staging and acceptance

```bash
cd ~/freertos-pc-test
git pull --ff-only origin sara/can-replacement
bash Firmware/Tests/stage_stage3_batch_b3.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
# Clean-build CM7 and CM4 in STM32CubeIDE.
bash Firmware/Tests/check_stage3_batch_b3_acceptance.sh \
  "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"
```

The test runner is software/static/host/ARM/ELF qualification only; it
never accesses real flash, CAN drives, safety IO or a powered STM32.
