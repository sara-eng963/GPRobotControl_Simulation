# STM32 migration utility v4 — narrowly corrected rename check

The v3 checker incorrectly categorized `gp_traj_shared_initialize` -> `trajectory_shared_initialize` as a structural difference. v4 recognizes the exact `gp_traj_` -> `trajectory_` prefix while retaining strict comparison of all other C tokens. It does not waive ABI checks, allow actual structural changes, or allow dropping the CM4 validation worker.

## Install / run (WSL)

Extract `stm32_migration.py` into `~/freertos-pc-test`, replacing the existing script only. The staged candidate is preserved.

```bash
cd ~/freertos-pc-test
python3 stm32_migration.py inspect > build/stm32-migration/inspect-v4.log
rg 'SHARED HEADER|BLOCKER|First token mismatch|LEGACY SOURCE' build/stm32-migration/inspect-v4.log
```

**Do not run `stage` again now:** an existing candidate is already present. Do not invoke `activate` to bypass unresolved worker integration.

## Inspect missing CM4 service dependencies (read-only)

```bash
cd ~/freertos-pc-test
NEW='/mnt/d/University/Graduation Project/STM32/GP_STM32H745ZIT6_BoardConfig'
rg -n 'application_validation_service|validation_m4_|playback_m4_|worker_ready|CommittedTrajectoryReader|board_m4_supply' "$NEW/CM4" "$NEW/CM7/Application/Intercore" Firmware/STM32H745/CM4/Core
rg -n 'application_ipc_service_run|application_validation_service_start' "$NEW/CM4/Core/Src/main.c"
```

The CM4 service must be ported against the new `validation_storage_bridge` and `trajectory_storage_service` interfaces, linked into the CM4 CubeIDE project, and called only where the storage-worker design requires it. Its board backend checks and fail-closed path must remain intact. Do not change pin mapping or link a fake flash backend.

## Existing working tree

The reported 86 dirty entries are untracked files/directories, including build output, backup archives and temporary scripts. Do not run `git clean -fd`. The ignored staged tree is under `build/stm32-migration/candidate`.

No tool-created commits, Git pushes, or firmware deletion occur in `inspect`.
