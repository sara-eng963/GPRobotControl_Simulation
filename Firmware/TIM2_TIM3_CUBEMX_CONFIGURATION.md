# STM32H745 — TIM2/TIM3 timing configuration record

**Status:** STM32CubeMX GUI configuration **partially completed**, **no generated code, target timer operation or measured 500 Hz performance claimed**.  
**Date:** 2026-10-14  
**Branch:** `sara/can-replacement`  
**GUI working project:** `D:\University\Graduation Project\STM32\GP_Stage3_Repro\GP_STM32H745.ioc`  
**Verified independent backup:** `GP_Stage3_Repro_Backup_Pre_TIM2_TIM3` (operator-reported `diff -qr` = identical before edits)

## Purpose and architecture

- **TIM2:** free-running 32-bit 1 MHz counter on Cortex-M7 for **1 µs resolution** timestamp source. Counter wraps every `2^32 µs` ≈ **71.58 minutes**. Firmware must implement an overflow-safe monotonic 64-bit timestamp extension and demonstrate edge-case behavior. An interrupt on overflow is planned; merely configuring the interrupt does not implement the time extension.
- **TIM3:** 16-bit timer generating nominal **500 Hz / 2 ms** update interrupts to signal a Cortex-M7 FreeRTOS Supervisor. **No robotic control code belongs in the ISR**. The existing task currently uses `vTaskDelayUntil` and RTOS-tick-derived `supervisor_now_us`; timing integration is **not yet committed**.
- **TIM6 / TIM7:** preserve existing generated HAL timebase arrangements and CM4 assignment. Do not repurpose as the motion timer.
- Reuse **STM32CubeMX/HAL/FreeRTOS** for peripheral and RTOS infrastructure; no direct-register timer rewrite, no artificial motor enable, and no software-only safety claim.

## CubeMX evidence and values

Operator screenshots (Pinout & Configuration, Clock Configuration, TIM2 and TIM3 Parameter Settings), **2026-10-14**:

| Configuration | TIM2 — timestamp timer | TIM3 — cycle timer |
| --- | --- | --- |
| Processor/runtime context | Cortex-M7 checked; M4 unchecked | Cortex-M7 checked; M4 unchecked |
| Clock source | Internal clock | Internal clock |
| APB1 timer clock | 64 MHz, prescaler /1 (Clock Configuration screenshot) | Same source |
| Prescaler (PSC) | `63` | `63` |
| Counter mode | Up | Up |
| Auto-reload (ARR) | `4294967295` / 0xFFFFFFFF | `1999` |
| Internal clock division | No Division | No Division |
| ARR preload | Disable | Disable |
| Channels | Disabled | Disabled |
| Nominal count frequency | 1,000,000 counts/s | 1,000,000 counts/s |
| Nominal update frequency | 1 update per 2^32 counts | 500 updates/s |
| Global NVIC interrupt | **Checked** in screenshot | **Unchecked** in screenshot; pending |
| Screenshot-displayed NVIC priority | **5, greyed out** | **5, greyed out** (when unchecked) |
| Operator preference for priority 6 | Requested, **not confirmed applied** | May use if GUI allows; **not confirmed** |

The HAL-/NVIC-related FreeRTOS constraint is the configured `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` on M7. On Cortex-M, numerically larger priority values are lower urgency; `FromISR` operations require interrupt priority numerically **≥ 5**. Priority **5 is already acceptable**; priority **6 is an optional choice, not an official STM32 mandate**. Check effective NVIC priority in generated source and use an actual RTOS ISR API only from an appropriate priority.

### Frequency derivation

For an upcounter timer driven at 64 MHz:

`f_counter = 64,000,000 / (PSC + 1) = 64,000,000 / 64 = 1,000,000 Hz`

For TIM3:

`f_update = 1,000,000 / (ARR + 1) = 1,000,000 / 2000 = 500 Hz`

`T_update = 1 / 500 = 0.002 s = 2 ms`

These are **theoretical configuration values**, not measured wake-up latency, command intervals, jitter, or fieldbus throughput.

## Chronological operator checkpoints

1. A separate full backup of the reproducer already existed. Operator compared `GP_Stage3_Repro` and `GP_Stage3_Repro_Backup_Pre_TIM2_TIM3` with `diff -qr`; **PASS: backup identical** prior to edits.
2. Opened the **working** `.ioc` in STM32CubeMX 6.18.1. Confirmed STM32H745IITx LQFP176 dual core.
3. Assigned **TIM2** exclusively to M7; enabled internal clock; confirmed APB1 timer clock **64 MHz** via Clock Configuration. Entered PSC=63, up, ARR=4294967295, clock division no division, preload disable.
4. Enabled TIM2 global interrupt. CubeMX showed priority **5** but made it read-only on its peripheral NVIC page. Discussed that FreeRTOS **does not mandate priority 6**; 5 is valid. **Do not report 6 as applied unless later evidence confirms it.**
5. Assigned **TIM3** exclusively to M7; selected internal clock with disabled channels. Entered PSC=63, up, ARR=1999, no division, preload disabled. Screenshot confirms parameter values; **TIM3 NVIC remains to be enabled/verified**.
6. **No final review, save, source generation, build or hardware measurement has yet been reported for these timer changes.**

## Remaining configuration/acceptance gate

- Enable **TIM3 global interrupt** and inspect actual configured priority; `5` or `6` meets the M7 configured FreeRTOS `FromISR` threshold, absent other priority-grouping/configuration changes.
- Recheck TIM2 enabled, TIM3 enabled, both M7-only; confirm **TIM6 M7** and **TIM7 M4** stay allocated and their HAL timebases unchanged.
- Capture and review Clock Configuration and final NVIC tables before generation.
- Save the **working** `.ioc`; verify `Project Manager → Code Generator → Keep User Code when re-generating`. Generate once in the working copy, not in the verified backup.
- Audit generated `.ioc`, HAL `tim.c` or `main.c`, `stm32h7xx_it.c`, `msp.c`, HAL configs, startup/MPU/IPC USER CODE, `.cproject` links, linker sections and M4 changes before accepting any generated files into canonical Git.
- Integrate TIM2 startup, overflow-safe 64-bit monotonic time and TIM3 interrupt-to-task notification with bounded ISR; protect against lost/coalesced notifications and missed 2 ms deadlines. Retain fail-closed safety startup.
- Run ARM compilation and host fault/timing tests; clean-build **both** cores independently and verify retained symbols/stack memory.
- **Hardware-deferred:** scope measured timer update intervals, task wake-up/dispatch latency, worst-case execution time, CAN 1 Mbps load, actual target-command intervals, physical safety interlocks, and end-to-end 500 Hz feasibility.

## Interpretation

A verified 500 Hz TIM3 **interrupt source** is not proof of 500 Hz synchronized motion. In particular, a timer ISR may fire every 2 ms while the Supervisor misses deadlines, CAN transmission stalls or motor feedback goes stale. Maintain separate metrics for ISR counts, missed notifications, Supervisor cycle time, CAN RX/TX diagnostics, target-send intervals and path-execution deadline failures.

## TIM3 NVIC enabled — screenshot update (2026-10-14)

After the earlier TIM3 parameter screenshot, the operator provided a **TIM3 NVIC Settings** screenshot showing **TIM3 global interrupt checked (Add enabled), Preemption Priority 5, Sub Priority 0**. The priority field appears read-only in the per-timer NVIC page. Both TIM2 and TIM3 have now been shown with their global interrupts enabled at **displayed priority 5**. The earlier preference for priority 6 was **not applied** in the visible GUI settings; priority 5 meets the currently configured M7 FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` threshold for any ISR that calls a FreeRTOS `FromISR` API.

**Remaining before code generation:** confirm settings in the left-side Timers overview, inspect Project Manager/Code Generator preservation options and project output path, verify no unintended TIM6/TIM7, FDCAN, dual-core or SRAM4 settings changes, then save and regenerate once in the working copy. Generated source, runtime timing and hardware tests remain pending.

## CubeMX Project Manager — Code Generator options verified (2026-10-14)

Operator screenshot shows the following GUI settings before any code generation:

- **Copy only the necessary library files** selected.
- **Generate peripheral initialization as a pair of `.c/.h` files per peripheral** **unchecked**; expected timer init code therefore remains in existing generated module such as `CM7/Core/Src/main.c` rather than `tim.c`.
- **Backup previously generated files when re-generating** checked.
- **Keep User Code when re-generating** checked.
- **Delete previously generated files when not re-generated** unchecked.
- User Actions: no before/after generation scripts supplied in visible fields.

Screenshot confirms **configuration options only**, not saved/generated output. USER CODE preservation is not a guarantee for application files outside marked blocks, project metadata, or manual linked resources. Existing independent backup remains the rollback baseline. Before clicking **Generate Code**, verify **Project Name, Project Location, Toolchain/IDE** under Project Manager → Project, and inspect the resulting files after generation.


## Project destination and firmware package verified — screenshot (2026-10-14)

The operator opened STM32CubeMX **Project Manager → Project**. The GUI screenshot confirms: **Project Name** `GP_STM32H745`; **Project Location** and **Toolchain Folder Location** `D:\University\Graduation Project\STM32\GP_Stage3_Repro`; **Toolchain / IDE** STM32CubeIDE; **Generate Under Root** checked; **Application Structure** Advanced; **Dual Core Boot Mode** Both CPUs booting at once; **MCU Reference** STM32H745IITx; **Firmware Package** STM32Cube FW_H7 V1.13.0. Displayed M7/M4 minimum heap `0x200` and stack `0x400` each; these are *linker baseline settings*, not FreeRTOS task-stack measurements.

The `.ioc` had an unsaved-changes asterisk in the screenshot and generation had **not yet been reported**. The independently verified pre-timer backup path is a different folder and was not selected. Next safe GUI gate: save the working `.ioc`, generate once with user-code preservation enabled, then **compare output against independent backup before building or syncing to canonical Git**. Inspect generated TIM2/TIM3 init, NVIC handlers, HSEM/IPC startup, CubeIDE resource links, FreeRTOS configuration, and CM4 source differences. This screenshot verifies **destination and options only**, not successful generation.


## CubeMX code generation reported complete — awaiting generated-diff audit (2026-10-14)

The operator reported **"generated done"** after confirming project destination, backup, and code-generator options. This establishes that **CubeMX reported completion to the operator**, but **no generated-file listing, source audit, clean-build, runtime test or new Git source commit has yet been supplied**. The next gate is a read-only compare of the separate working folder `GP_Stage3_Repro` against `GP_Stage3_Repro_Backup_Pre_TIM2_TIM3`, excluding only build output folders `Debug` and `Release`. In particular inspect CubeMX TIM2/TIM3 initialization and interrupt handlers plus ensure `application_supervisor.c`, `fdcan_classic_backend.c`, both Eclipse `.project` files, core MPU/IPC startup sections, and the M4 application are preserved. Do **not** infer correct scheduling or 500 Hz performance from successful code generation.


## Post-generation read-only audit — TIM3 auto-reload mismatch (2026-10-14)

The operator executed a read-only `diff -qr` of `GP_Stage3_Repro` against its independent `GP_Stage3_Repro_Backup_Pre_TIM2_TIM3`, excluding Debug/Release build directories. **14 differences** were reported: `.mxproject`, CM7/CM4 `.settings/language.settings.xml`, new CubeMX-generated `Backup` subfolders in core source/header directories, updated CM7 `Core/Inc/stm32h7xx_it.h`, `Core/Src/main.c`, `Core/Src/stm32h7xx_hal_msp.c`, `Core/Src/stm32h7xx_it.c`, new CMSIS Device ST `Source` directory and `Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_tim.h`, plus the changed `GP_STM32H745.ioc`. **Unchanged by byte-compare:** M7 `application_supervisor.c`, M7 `fdcan_classic_backend.c`, CM7 `.project` and CM4 `.project`. Further diff review (especially startup, MPU, FreeRTOS, CAN, CM4 settings and vendor-sourced files) remains pending; no builds reported after regeneration.

**Critical mismatch found in regenerated M7 `main.c`:**
- TIM2 handle, `MX_TIM2_Init`, PSC `63`, ARR `4294967295` generated correctly.
- TIM3 handle and `MX_TIM3_Init` generated, PSC `63`, **ARR `65535` instead of screenshot-confirmed GUI `1999`**. At APB1 timer 64 MHz the TIM3 counter input is 1 MHz, so generated `ARR=65535` gives an update **every 65.536 ms (about 15.26 Hz)** rather than 2 ms (500 Hz). This discrepancy blocks acceptance.
- `TIM2_IRQHandler` and `TIM3_IRQHandler` were emitted in M7 `stm32h7xx_it.c` and declared in `stm32h7xx_it.h`.

**Immediate next action:** inspect `TIM3.*` keys in the **working** `.ioc` and compare against generated `MX_TIM3_Init` (do not hand-edit CubeMX-generated `main.c`). If the `.ioc` contains `TIM3.Period=65535`, correct through CubeMX Parameter Settings, verify persisted value on Save and regenerate **once**, then recheck. If the `.ioc` actually contains `TIM3.Period=1999`, investigate generator state/version and source consistency before making any change. Preserve the verified pre-timer backup and avoid committing or testing timer integration until generated values match. Cause of mismatch **not established** merely from the difference listing; do not assume the user entered a wrong value.

Evidence is from the user's WSL console output; hardware timer behavior remains entirely untested.

## TIM3 mismatch localized to saved .ioc (2026-10-14)

User ran a targeted WSL check against the actual `GP_Stage3_Repro` files after code generation. **Saved .ioc** shows `NVIC1.TIM3_IRQn=true\\:5\\:0\\:false\\:false\\:true\\:true\\:true\\:true\\:true`, `TIM3.IPParameters=Prescaler` and `TIM3.Prescaler=63`; **no persisted TIM3 counter-period key**. Generated `CM7/Core/Src/main.c` has `htim3.Init.Prescaler = 63`, `htim3.Init.CounterMode = TIM_COUNTERMODE_UP`, and `htim3.Init.Period = 65535`. The mismatch is therefore attributable to the period **not being saved as a nondefault CubeMX parameter** rather than to a contradictory saved `1999` field. The precise UI mechanism (e.g. edit not committed) remains unproven.

**Corrective step pending operator result:** CubeMX TIM3 Parameter Settings → set Counter Period to `1999`, **confirm with Enter/Tab and focus change**, save `.ioc`, and *before regeneration* verify `TIM3.IPParameters` includes its period and `TIM3.Period=1999` or the equivalent generated CubeMX key. Only then regenerate once and verify `htim3.Init.Period = 1999`. Keep all FDCAN/IPC/dual-core configuration unchanged. No software/hardware timing acceptance until clean compile, link and measured runtime behavior.


## TIM3 .ioc persisted correction — verified (2026-10-14)

Operator explicitly verified the saved working `GP_Stage3_Repro/GP_STM32H745.ioc` with a read-only WSL grep after editing CubeMX:

```ini
TIM3.IPParameters=Prescaler,Period
TIM3.Period=1999
TIM3.Prescaler=63
```

**PASS: CubeMX project configuration now persists the desired 2 ms / 500 Hz TIM3 period setting** (given previously screenshot-confirmed APB1 timer input 64 MHz). This does **not yet** verify regenerated `CM7/Core/Src/main.c` or a firmware build. Operator's next gate: regenerate code from the saved working `.ioc`, verify `htim2.Init.{Prescaler,Period} = {63,4294967295}` and `htim3.Init.{Prescaler,Period} = {63,1999}` in generated M7 `main.c`, then audit generated changes against the independent pre-timer backup and target builds before software acceptance.


## TIM2/TIM3 regenerated source parameters verified — 2026-10-14

After persisting the TIM3 period in `GP_STM32H745.ioc`, the operator regenerated CubeMX output and inspected the **actual generated** `GP_Stage3_Repro/CM7/Core/Src/main.c` with WSL `grep`. Console evidence:

```text
=== TIM2 ===
  htim2.Init.Prescaler = 63;
  htim2.Init.Period = 4294967295;
=== TIM3 ===
  htim3.Init.Prescaler = 63;
  htim3.Init.Period = 1999;
```

**PASS: generated timer PSC/ARR match the verified GUI design.** With the previously displayed APB1 timer clock of 64 MHz, TIM2 has nominal 1 µs tick resolution and TIM3 update events are nominally 500 Hz / 2 ms. The corresponding IRQ handlers were already observed in `CM7/Core/Src/stm32h7xx_it.c` in the earlier generated-file audit.

**Not yet passed:** complete post-regeneration diff review of critical MPU/IPC, FreeRTOS, CAN/FDCAN and CM4 files; HAL timer module project links and clean dual-core CubeIDE firmware builds; explicit timer starts; overflow-safe 64-bit timestamp implementation; TIM3 task notification scheduling, cycle overrun detection, or physical timing measurements. Neither `HAL_TIM_Base_Init` nor a TIM IRQ handler definition starts a timer automatically. The independent pre-timer project backup remains the rollback reference. Do not imply 500 Hz trajectory execution from configuration alone.


## Post-regeneration preservation audit PASSED — 2026-10-14

The operator ran a read-only WSL check comparing the independent pre-timer backup to the regenerated `GP_Stage3_Repro` project. All six byte-for-byte checks passed: `CM7/.project`, `CM4/.project`, `CM7/Core/Src/application_supervisor.c`, `CM7/Board/Src/fdcan_classic_backend.c`, `CM4/Core/Src/main.c`, and `CM7/Core/Inc/FreeRTOSConfig.h`. Eight further source-presence checks passed on the regenerated M7 `main.c` and `stm32h7xx_it.c`: `intercore_shared_memory_initialize()`, `intercore_notifications_initialize()`, `application_supervisor_start(&config)`, `MX_FDCAN1_Init();`, `MX_TIM2_Init();`, `MX_TIM3_Init();`, `TIM2_IRQHandler(void)`, and `TIM3_IRQHandler(void)`. The operator reported the final sentinel **"PASS: Initial TIM2/TIM3 preservation audit"**.

**Scope:** This is a successful *initial file-equality and symbol-presence preservation audit*, not an exhaustive generated diff or HAL timer link test. In particular the timer initialization and IRQ handler functions merely exist; timer start, ISR notification, overflow-safe timestamp extension, and measured 500 Hz remain pending.

**Next gate:** Refresh and clean-build CM7 and CM4 using the existing STM32CubeIDE reproduction workspace. If M7 encounters undefined `HAL_TIM_Base_Init` or related timer HAL functions, audit the CubeMX-generated HAL TIM driver linkage and project resources rather than manually inventing a driver. Record compile/link output, warning count, ELF symbol inventory and source changes before moving to timer runtime integration.


## CubeIDE dual-core build milestone reported PASSED — 2026-10-14

After the operator verified the second CubeMX generation with TIM2 PSC=63/ARR=4294967295 and TIM3 PSC=63/ARR=1999, and the initial 14-point source preservation audit completed successfully, the operator reported that **both Cortex-M7 and Cortex-M4 builds succeeded** in the separate STM32CubeIDE reproduction workspace. **Detailed console warnings, ELF hashes and new sizes were not submitted**; do not assume zero warnings or measured runtime performance.

**Software gate:** compiled and linked target images were reported successful after timer regeneration, with core application/IPC symbols checked by source audit. This is not timer runtime validation or evidence of 500 Hz Supervisor execution. New CubeMX-generated timer files currently reside in the Windows `GP_Stage3_Repro` project; the GitHub `Firmware/STM32H745` snapshot remains **pre-TIM2/TIM3** until reviewed and synchronized. **Next dependency:** import the reviewed official generated diff into source control without overwriting application files or blind whole-directory copy. Then implement the timer-to-Supervisor composition using documented ST HAL/FreeRTOS ISR APIs and separately test deadline/rollover behavior.


## Generated baseline sync — diagnosed CRLF bug and fixed importer (2026-10-14)

On the user's Windows-produced `GP_STM32H745.ioc`, `TIM3.Period=1999`, `TIM3.Prescaler=63`, and `TIM2.Prescaler=63` each end in a **CRLF** line ending (Python readback showed trailing `\\r`). The first `Firmware/Tests/stage_tim2_tim3_generated.sh` used anchored `grep -Eq '...$'` and exited at its first check with status **1 and no explanation**, although the values themselves and both IRQ handlers were correct. Git branch synchronization, script syntax, and file-generation checks had passed independently.

**Remedy:** source-control commit `f66b490` replaces the three strict grep expressions with `awk` exact key/value matching after removing a possible trailing CR, with explicit PASS/FAIL diagnostics; it also gives a descriptive error for missing TIM2/TIM3 IRQ handlers. This is an importer robustness correction, **not a firmware/configuration change**. The AWK expression was locally tested with both LF and CRLF synthetic `.ioc` lines. **The end-to-end import against the user's actual Windows project is not yet verified**; the operator needs to pull and rerun the script and submit its output before committing any generated sources.


## CubeMX-generated sync diagnostic — Git CRLF whitespace gate (2026-10-14)

The operator attached a **1012-line terminal paste** ending in `Exit code: 2`. Visible lines are almost entirely `git diff --check` messages such as `Firmware/STM32H745/.mxproject:11: trailing whitespace`, with newly generated `CM7/Core/Src/main.c`, `stm32h7xx_hal_msp.c`, `stm32h7xx_it.c`, `stm32h7xx_it.h`, and `GP_STM32H745.ioc` lines ending in `^M` (Windows CRLF). The script performs `git diff --check` **after** copying the generated sources, so **it likely already copied them into the WSL working tree** before exiting. The paste begins mid-output: the initial PASS and SYNCED lines cannot be independently inspected from this attachment. The CRLF false-positive is not a compiler or CubeMX error, and the source timestamps/PSC/ARR were previously independently verified.

**Fix:** `Firmware/Tests/stage_tim2_tim3_generated.sh` commit `bb351e1` uses invocation-local `git -c core.whitespace=cr-at-eol diff --check`, preserving other Git whitespace checks while allowing official CubeMX CRLF files *without altering their generated content*. The script is now **idempotent**: a previously copied path is accepted iff its bytes equal the authoritative generated source, while modified tracked files that differ, and untracked files that differ, are refused. It validates all paths before writing any. The next operator run should capture only succinct PASS / ALREADY SYNCED / status to avoid a multi-page Git diagnostic scroll. Acceptance awaits a fresh script run ending in PASS; do not auto-commit or mass-add unrelated untracked build files.


## TIM2/TIM3 generated-source staging gate PASSED (2026-10-14)

Operator reran `Firmware/Tests/stage_tim2_tim3_generated.sh` after the CRLF and idempotency fixes. `git pull --ff-only origin sara/can-replacement` succeeded, and the staging script exited **0** and printed:

```text
ALREADY SYNCED: CM7/Core/Src/stm32h7xx_hal_msp.c
ALREADY SYNCED: CM7/Core/Src/stm32h7xx_it.c
ALREADY SYNCED: CM7/Core/Inc/stm32h7xx_it.h
ALREADY SYNCED: Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_tim.h
PASS: reviewed CubeMX-generated timing inputs copied into Git working tree.
NOTE: no git add, commit or push performed; review diff before publication.
```

The operator's `git status --short` within `Firmware/STM32H745` shows six **modified tracked files**: `.mxproject`, `CM7/Core/Inc/stm32h7xx_it.h`, `CM7/Core/Src/main.c`, `CM7/Core/Src/stm32h7xx_hal_msp.c`, `CM7/Core/Src/stm32h7xx_it.c`, `GP_STM32H745.ioc`; plus one **untracked official HAL** header `Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_tim.h`. Tracked-file diff stat: **6 files changed, 252 insertions and 30 deletions** (with generated metadata and CRLF effects in the raw count).

**Gate PASSED:** the independent, locally generated CubeMX inputs are now in the user's **local working tree** and were rechecked, including idempotent rerun and Git whitespace check allowing CRLF. **Not yet done:** source-diff review, staging/committing these seven generated files, pushing to GitHub, implementing any runtime timer start or Supervisor scheduling changes. The existing source-controlled GitHub firmware snapshot remains at its pre-TIM2/TIM3 state until this follow-up commit is completed. Keep the independent pre-timer backup untouched. No claim of hardware execution or motion validation.


## Submitted 445-line generated-code diff — review results (2026-10-14)

The operator uploaded `GP_TIM2_TIM3_Review.diff` (445 lines, 111,385 bytes), produced with `git diff --ignore-space-at-eol -- Firmware/STM32H745` after successful local CubeMX staging. A detailed hunk inspection identified **six changed tracked paths** and one **new untracked HAL header not included in the diff**.

**Generated functional content (reviewed):**
- `CM7/Core/Src/main.c`: M7 TIM2/TIM3 handles, init declarations, and `MX_TIM2_Init()`/`MX_TIM3_Init()` calls added after FDCAN; PSC=63 for both, TIM2 ARR=4294967295, TIM3 ARR=1999; proper STM32 HAL `HAL_TIM_Base_Init`, `HAL_TIM_ConfigClockSource`, and `HAL_TIMEx_MasterConfigSynchronization` calls. No changed startup/IPC user-code hunk.
- `CM7/Core/Src/stm32h7xx_hal_msp.c`: TIM2/TIM3 peripheral clock enable/disable, IRQ priority 5 and enable/disable generated. Existing FDCAN numeric spelling `PLL2FRACN=0.0` changed to `=0` (equivalent), no changed FDCAN bitrate/pin/PLL selection observed.
- `CM7/Core/Src/stm32h7xx_it.c` and `CM7/Core/Inc/stm32h7xx_it.h`: both timer IRQ handlers emitted and delegate to ST `HAL_TIM_IRQHandler`; TIM6 ISR unchanged.
- `GP_STM32H745.ioc`: TIM2/TIM3 registered on **Cortex-M7**, internal-clock virtual pins, expected PSC/ARR settings and NVIC priority 5; M7 TIM6 and M4 TIM7 remain. **Non-timer metadata change:** `rtos.0.ip=FREERTOS_M7` → `FREERTOS_M4`. Both `CortexM7.IPs` and `CortexM4.IPs` still enumerate their respective FreeRTOS IPs; both cores were reportedly built after regeneration. **Meaning of this single selector is not independently verified**; retain it in review record and compare on future regeneration rather than asserting it disables M7.
- `.mxproject`: large generated-history reorder plus inventory additions. Comparing list *multisets* found no removed source files, lib entries, include paths or C defines; added M7 FDCAN source listing, FDCAN-related lib entries and LL timer header. CM4 source list, header paths and CDefines unchanged. Do **not** treat this auto-generated history file as a replacement for actual Eclipse `.project`/`.cproject` validation.

**Scope and remaining gate:** Based on the visible diff, the TIM2/TIM3 *functional* changes are appropriate to stage as the **CubeMX-generated baseline**, pending inspection/provenance of new untracked `Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_tim.h` (not represented in ordinary unstaged `git diff`). The diff alone cannot prove exact running timing, ISR-to-FreeRTOS startup, or physical CAN safety. The operator reported that CM7 and CM4 clean builds succeeded in the independently regenerated project. No runtime timer start is introduced by this diff. Before commit, explicitly stage only the seven listed generated paths, check staged diff with `git -c core.whitespace=cr-at-eol diff --cached --check`, and inspect staged file names to ensure unrelated untracked simulations are excluded. Do not commit until the new HAL header's expected ST provenance is confirmed.


## New ST TIM LL header checked; no files staged (2026-10-14)

Operator confirmed the seventh, previously untracked generated dependency at `Firmware/STM32H745/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_tim.h`: approximately **234 KiB**; opening comments identify `@file stm32h7xx_ll_tim.h`, `@author MCD Application Team`, `@brief Header file of TIM LL module`, STMicroelectronics copyright (2017), include guard `__STM32H7xx_LL_TIM_H`, and `#include "stm32h7xx.h"`. This is consistent with the STM32Cube H7 LL TIM header and was copied from the user's independently CubeMX-generated directory by a byte-verified import script. **No comparison against an independently downloaded ST firmware release hash has been performed**, so official-release identity should not be asserted beyond these checks.

The user's `git diff --cached --name-only` returned **no staged files**. Next gate: explicitly `git add` only the **seven** reviewed paths, review staged paths and `git -c core.whitespace=cr-at-eol diff --cached --check`, and commit/push only if those gates pass. Unrelated untracked simulation artifacts must remain untouched.

## TIM2/TIM3 runtime integration — software implementation, target build pending (2026-10-14)

**Baseline:** User successfully rebased and pushed generated CubeMX timer firmware to `sara/can-replacement` as GitHub commit `1b90eed`. Branch local/remote synchronized with no tracked modifications. TIM2/TIM3 initialization, MSP, NVIC priorities and ST LL timer header are thus source-controlled.

**Implementation commit:** `ad9d2a0` adds M7 timing functionality on top of the generated baseline, preserving CubeMX code in USER CODE regions and in existing application sources:

1. `CM7/Core/Src/main.c` calls `application_supervisor_timer_period_elapsed_isr(htim)` from **USER CODE Callback 1**, after the existing **TIM6 HAL tick** handler; generated TIM2/TIM3 handlers remain ST HAL's `HAL_TIM_IRQHandler`.
2. `CM7/Core/Src/application_supervisor.c` implements TIM2's 32-bit 1 MHz counter extended with an interrupt-maintained high word. The Supervisor-only read masks priority-5 interrupts via a FreeRTOS task critical section and accounts for a pending update flag before the ISR executes. HAL-generated configuration remains authoritative. Timer startup uses `HAL_TIM_Base_Start_IT` for both TIM2 and TIM3 **only from the actual Supervisor task after CANopen runtime ownership is bound**, validates HAL clock and PSC/ARR contract, clears stale update flags, and returns failure rather than running with mismatched clock/timer settings. TIM3 ISR delivers only a FreeRTOS **counting direct-to-task notification** and yields if requested; it never executes motor/CANopen path logic.
3. `StateMachine/supervisor_task.h/.c` adds an **optional** `SupervisorTimingPort` (monotonic microseconds, start, wait for counted periods). The STM32 composition supplies it; PC simulation remains on FreeRTOS's `xTaskGetTickCount/vTaskDelayUntil`. The target runs its main state-machine loop upon TIM3 notifications and records missed counts (>1), notification timeouts (0), and elapsed cycles >=2000us; it emits state-failure events and publishes diagnostic counts and max cycle duration. The port requires a 2-tick config at 1 kHz. The existing application board supplier remains absent by default and fails closed, so **no timer-based motion starts without real physical board commissioning**.

**Verification limitations:** implementation is **committed but not independently clean-built or run on an STM32**. Need run `python3 Firmware/Tests/check_tim2_tim3_integration.py` (read-only static wiring/mathematical gate), then `bash Firmware/Tests/stage_m7_timer_integration.sh "/mnt/d/University/Graduation Project/STM32/GP_Stage3_Repro"`, refresh and clean-build **both** cores using CubeIDE. The gate does **not** measure interrupt latency, task wake-up jitter or bus throughput. Hardware tests, 71.58-minute TIM2 wrap stress, ISR priority, timestamp monotonicity under load, and real 500 Hz CAN target intervals remain pending. A TIM3 configuration of 500 Hz **does not guarantee** 500 Hz motion execution.
