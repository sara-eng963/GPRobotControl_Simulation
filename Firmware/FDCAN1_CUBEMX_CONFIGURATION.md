# STM32H745 FDCAN1 — CubeMX Configuration & Decision Log

> **Configuration session:** 2026-10-14  
> **Branch:** `sara/can-replacement`  
> **Stage:** 3C-2 — Classic CAN / physical interface preparation  
> **MCU/project:** STM32H745IITx, LQFP176; STM32CubeMX 6.18.1; STM32Cube FW_H7 V1.13.0  
> **Status:** **GUI selections evidenced by operator screenshots; NOT YET SAVED/REGENERATED INTO THE TRACKED .ioc OR LINKED ACTIVE FDCAN HAL.**  
> **Target:** one CANopen bus at **1 Mbit/s** for six drives; real transport and motion remain disabled

## 1. What changed in CubeMX (chronological)

1. Opened the *separate verification workspace's* `GP_STM32H745.ioc` in STM32CubeMX. This is **not** yet a change to the repository's tracked `Firmware/STM32H745/GP_STM32H745.ioc`.
2. **Connectivity → FDCAN1:** kept **Cortex-M7 selected** (Cortex-M4 unchecked) and checked **Activated**. Selected/observed **Classic mode** and **Normal mode**.
3. **Clock Configuration:** declined the automatic clock-issues solver to review changes manually. FDCAN's clock mux originally selected **PLL1Q**, with `To FDCAN = 129 MHz` (highlighted as invalid after FDCAN activation).
4. Selected **PLL2Q** in the **FDCAN Clock Mux**. This enabled the previously greyed-out PLL2 `DIVQ2` field.
5. Changed only **PLL2 DIVQ2 from /2 to /6**. The PLL2 VCO was displayed as **258 MHz**, so its Q output became **43 MHz**. `To FDCAN = 43 MHz`; the operator reported **no remaining clock warnings**.
6. **FDCAN1 → Parameter Settings:** configured the nominal timing fields below. CubeMX displayed **Nominal Baud Rate = 1,000,000 bit/s** and **Nominal Time Quantum ≈ 23.25581395 ns**.

## 2. Confirmed settings and engineering rationale

| CubeMX parameter | Observed GUI value | Reason / interpretation | Disposition |
| --- | --- | --- | --- |
| Runtime context | Cortex-M7 enabled; Cortex-M4 disabled | CM7 owns the existing CANopen Master and robot Supervisor | **Chosen** |
| FDCAN1 | Activated | Enable peripheral configuration and clock dependency | **Chosen (GUI only)** |
| Frame Format | **Classic mode** | The six-motor CANopen system is Classic CAN, not CAN FD | **Chosen** |
| Mode | **Normal mode** | Intended external physical CAN operation; loopback can later be used for tests | **Chosen for eventual bus operation** |
| FDCAN clock mux | **PLL2Q** (was PLL1Q) | Separate FDCAN clock from the 129 MHz PLL1Q output | **Chosen** |
| PLL2 Q divider / DIVQ2 | **/6** (was /2) | 258 MHz / 6 = 43 MHz | **Chosen** |
| FDCAN kernel clock | **43 MHz** (was 129 MHz) | Fits the current **VOS3 50 MHz FDCAN kernel-clock limit**; supports exact 1 Mbit/s | **Chosen; review VOS after generation** |
| Nominal Prescaler | **1** | No additional prescaling of the 43 MHz nominal time-quantum clock | **Chosen** |
| Nominal Sync Jump Width | **8 tq** (was 1 tq) | Maximum nominal resynchronization adjustment; not greater than Time Seg2 = 8 | **Initial selection; review with real bus** |
| Nominal Time Seg1 | **34 tq** | Establishes sample-point timing | **Chosen initially** |
| Nominal Time Seg2 | **8 tq** | Establishes time after sample point and meets selected SJW bound | **Chosen initially** |
| Nominal Baud Rate (CubeMX readout) | **1,000,000 bit/s** | Computed from kernel clock and nominal timing | **Confirmed in GUI** |
| Nominal Time Quantum (CubeMX readout) | **≈23.25581395 ns** | 1 / 43 MHz | **Confirmed in GUI** |

**Clock derivation, as seen in the CubeMX clock tree:** selected source **HSI 64 MHz**, PLL2 input divider **DIVM2 /32**, multiplier **DIVN2 ×129**, Q divider **DIVQ2 /6**. Thus:

```text
PLL2 input = 64 MHz / 32      = 2 MHz
PLL2 VCO   = 2 MHz × 129      = 258 MHz
PLL2Q      = 258 MHz / 6      = 43 MHz
FDCAN kernel clock           = 43 MHz
```

**Classic CAN nominal bit timing** (one synchronization time quantum plus the two configurable time segments):

```text
Nominal bit time quanta = 1 + 34 + 8 = 43 tq
Time quantum            = 1 / (43,000,000 / 1) = 23.2558 ns
Bit time                = 43 × 23.2558 ns = 1.000 µs
Nominal bit rate        = 43 MHz / [1 × (1 + 34 + 8)] = 1,000,000 bit/s
Sample point            = (1 + 34) / 43 × 100 = 81.3953%
```

This gives **exact nominal 1 Mbit/s** by calculation, with an **approximately 81.4% sample point**. It is **not proof of physical bus synchronization or reliability**. The 8-tq SJW and sample point are initial design choices, to be checked against oscillator tolerance, transceivers, cable length, network topology, and measured CAN errors.

## 3. Other visible settings — *not yet finalized*

These are recorded because they appeared in the user's CubeMX screenshots. They are **observations**, **not approvals**, and some must change before the HAL adapter can start.

| Setting | Currently displayed | Follow-up |
| --- | --- | --- |
| Auto Retransmission | **Enable** (was Disable) | Allow the CAN controller to retry frames following lost arbitration / eligible transmission errors; application must still detect feedback staleness, bus-off and missed deadlines |
| Transmit Pause | **Disable** | Review with final TX scheduling |
| Protocol Exception | **Disable** | Keep under review; Classic CAN mode |
| Clock Calibration | **Disable** | Confirm clock accuracy against actual oscillator |
| Data Prescaler | **1** | Not the nominal bus bit timing; data timing is not used for Classic CAN |
| Data Sync Jump Width | **1** | Unused in Classic CAN |
| Data Time Seg1 | **1** | Unused in Classic CAN |
| Data Time Seg2 | **1** | Unused in Classic CAN |
| Message RAM Offset | **0** | Review allocation with any additional FDCAN instance |
| Std Filters Nbr | **0** | Filter policy remains to be configured; the HAL adapter currently configures a *global* filter for standard IDs |
| Ext Filters Nbr | **0** | Extended identifiers are rejected by the adapter |
| Tx Events Nbr | **0** | No transmit-event FIFO allocated currently |
| Tx Buffers Nbr | **0** | Adapter uses TX FIFO, not dedicated TX buffers |
| Tx FIFO Queue Elements Nbr | **0** | **Must be ≥16** before enabling the existing adapter |
| Tx FIFO Queue Mode | **FIFO mode** | Required by the existing adapter |
| Tx Element Size | **8-byte data field** | Required for Classic CAN frames |

**Other values not yet evidenced or finalized:** Rx FIFO0 element count and element size; FDCAN1 RX/TX GPIO/alternate-function pin mapping; transceiver/termination; nominal clock divider within FDCAN; interrupt strategy and NVIC priority; physical node-ID/encoder/gear-ratio confirmation; board safe I/O. **Do not infer their values from the screenshots.**

### Why the FIFO requirements matter

The prepared driver `CM7/Board/Src/fdcan_classic_backend.c` explicitly refuses to start unless the HAL instance reports:

- Classic CAN normal mode and TX FIFO mode;
- at least **16 TX FIFO/Queue elements** and **16 Rx FIFO0 elements**;
- **8-byte TX and Rx FIFO0 elements**.

The visible default of **0 TX FIFO/Queue elements** therefore **does not satisfy the adapter yet**. This is expected at this intermediate stage; it is not a firmware build failure.

## 4. Repo vs. local configuration: crucial distinction

**Repository verified before this document commit:**

- `Firmware/STM32H745/GP_STM32H745.ioc` still listed `RCC.FDCANFreq_Value=129000000` and `RCC.DIVQ2Freq_Value=129000000`.
- There was **no FDCAN1 peripheral configuration** in its `CortexM7.IPs` or any `FDCAN1.*` block.
- The checked-in HAL snapshot did **not** contain `stm32h7xx_hal_fdcan.c/.h`, because CubeMX had not generated FDCAN for this project.
- The board adapter is conditionally **inactive** when `HAL_FDCAN_MODULE_ENABLED` is absent, and the existing Supervisor board configuration hook refuses startup.

**This commit only documents the GUI changes. It does not edit the tracked .ioc, generate HAL drivers, configure real pins, prove the active FDCAN branch compiles, or enable robot motion.**

Once the CubeMX configuration is complete and the operator saves/generates code, review and commit the actual `.ioc`, CubeMX-generated HAL/header/source changes and both project builds **as a separate, clearly traceable implementation commit**. Preserve all Stage 2 dual-core IPC, HSEM, MPU, USER CODE sections, and linker layouts when regenerating.

## 5. Outstanding verification gates

- [x] Screenshot confirmation of CM7 ownership, FDCAN1 activation, Classic/Normal mode
- [x] Screenshot confirmation of PLL2Q 43 MHz without reported CubeMX clock warnings
- [x] Screenshot confirmation of nominal timing and computed 1 Mbit/s
- [x] Select Auto Retransmission = Enable in CubeMX GUI (hardware behavior still unverified)
- [ ] Complete message RAM / filters / GPIO / NVIC configuration and review generated settings
- [ ] Save the updated `.ioc` and inspect the actual diff against this log
- [ ] Generate matching STM32Cube FW_H7 V1.13.0 HAL FDCAN source using CubeMX
- [ ] ARM-compile the **active** HAL adapter with the generated official headers
- [ ] Clean-build and inspect both actual CubeIDE Debug ELFs
- [ ] Integrate genuine physical CANopen backend into the CM7 board composition
- [ ] Hardware bus and transceiver validation (bitrate, errors, bus-off, RX/TX)
- [ ] Measure six-drive CANopen 500 Hz latency, occupancy and lost-cycle behavior
- [ ] Verify independent safety/stop hardware before enabling any robot motion

## 6. Authoritative references

- ST [RM0399 — STM32H745/755 and STM32H747/757 reference manual](https://www.st.com/resource/en/reference_manual/rm0399-stm32h745755-and-stm32h747757-advanced-armbased-32bit-mcus-stmicroelectronics.pdf): RCC clock distribution/voltage scaling; FDCAN nominal-bit timing and message RAM.
- ST [STM32CubeH7 firmware](https://github.com/STMicroelectronics/STM32CubeH7): official device HAL package and CubeMX-generated workflow; project pins package V1.13.0.
- ST [STM32H7 FDCAN HAL1 API header](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/blob/v1.11.4/Inc/stm32h7xx_hal_fdcan.h): `FDCAN_InitTypeDef`, nominal timing, FIFO configuration and official HAL functions. **Representative HAL1 header reference; the installed CubeH7 V1.13.0 source is authoritative for the real generated build.**
- [CAN in Automation](https://www.can-cia.org/): CANopen / CAN physical-layer standards ecosystem.

**Record policy:** distinguish **GUI-only selections → saved CubeMX project → generated/linked firmware → hardware measurement**. A software-reported bit rate is a *configuration calculation*, not a measured signal.


### FIFO settings screenshot (2026-10-14)

**Operator's CubeMX GUI evidence, not yet committed to the actual `.ioc` file:**

| Configuration parameter | Observed value | Reason / status |
| --- | --- | --- |
| Tx Fifo Queue Elmts Nbr | **16** | Explicitly selected; meets minimum accepted by `gp_fdcan_classic_backend_open` |
| Tx Fifo Queue Mode | **FIFO mode** | Preserves command insertion order; expected by board adapter |
| Tx Elmt Size | **8 bytes data field** | Maximum Classic CAN payload size |
| Tx Buffers Nbr | **0** | Only TX FIFO is used; no dedicated TX buffers |
| Tx Events Nbr | **0** | Current adapter does not use the transmit event FIFO |
| Rx Fifo1 Elmts Nbr | **0** | A single FIFO0 is planned; FIFO1 not required |
| Rx Fifo0 Elmt Size | **8 bytes data field** | Matches Classic CAN feedback payloads and adapter's size check |
| Rx Buffers Nbr | **0** | No dedicated receive buffers planned |
| Rx Buffers Size | **8 bytes data field** | Shown default; irrelevant with zero dedicated RX buffers |
| Clock Calibration | **Disable** | Displayed default, not yet evaluated for actual oscillator/transceiver |

**Now confirmed by subsequent operator screenshot (2026-10-14):** `Rx Fifo0 Elmts Nbr = 16`, meeting the prepared HAL adapter's `RxFifo0ElmtsNbr >= 16` requirement. No FIFO timing, occupancy, overflow, or delivery guarantee follows merely from allocation. Message RAM use and interaction with a second FDCAN instance, if selected later, must be checked against the STM32H745 reference manual. Automatic retransmission is now enabled in the CubeMX GUI; physical I/O and bus-error/deadline verification are pending.

**Status:** Both 16-element TX FIFO and 16-element Rx FIFO0 configurations have GUI screenshot evidence. They remain uncommitted to the canonical `.ioc` and unverified in active HAL firmware. No changed CubeMX `.ioc` has been fetched into the repository and no active HAL build has been claimed.

### Receive FIFO0 confirmation (2026-10-14)

A second screenshot explicitly shows `Rx Fifo0 Elmts Nbr = 16`, `Rx Fifo0 Elmt Size = 8 bytes data field`, `Rx Fifo1 Elmts Nbr = 0`, `Std Filters Nbr = 0`, `Ext Filters Nbr = 0`, and `Message RAM Offset = 0`. The standard-filter count of zero is an intermediate setting; the current board adapter uses HAL's global acceptance for standard frames and rejection for extended/remote frames. This permissive standard-frame policy must be revisited after actual node IDs/COB-IDs and bus traffic are confirmed; do not present it as a production-optimized acceptance filter. No CubeMX generation or physical bus test has happened yet.

### Automatic retransmission confirmed (2026-10-14)

Operator screenshot of **FDCAN1 → Parameter Settings → Basic Parameters** shows **Auto Retransmission = Enable**, **Classic mode**, **Normal mode**, **Transmit Pause = Disable**, **Protocol Exception = Disable**, **Nominal Sync Jump Width = 8**, **Data Prescaler = 1**, **Data Sync Jump Width = 1**, **Data Time Seg1 = 1**, **Data Time Seg2 = 1**, **Message RAM Offset = 0**, and **Std Filters Nbr = 0**. These are **GUI-observed values**, not yet committed configuration values in `Firmware/STM32H745/GP_STM32H745.ioc`.

**Rationale:** HAL FDCAN automatic retransmission is a normal reliability mechanism for Classic CAN/CANopen: after a transmission error or arbitration loss, the controller can retry subject to protocol rules. However, a queued motion command must not be treated as delivered on time simply because retries are enabled. The application needs explicit stale-command/feedback detection and measured worst-case latency; a retry can outlive the intended 2 ms cycle. Do not interpret retransmission as guaranteed delivery. CANopen bus-off recovery and safety behavior have **not** been validated.

**Next interface decision:** inspect CubeMX **FDCAN1 → GPIO Settings** and record the actual peripheral RX/TX pin selections. Do not freeze alternate-function pins until the MCU package, intended CAN transceiver, PCB routing and possible FDCAN2 conflicts are reviewed. The current firmware adapter polls the FDCAN RX FIFO directly; NVIC/interrupt configuration is not automatically required for the polling transport, although interrupt-driven error monitoring may be added after design review.


### FDCAN1 GPIO pin screenshot (2026-10-14)

The operator provided a CubeMX **Connectivity → FDCAN1 → GPIO Settings** screenshot showing the following provisional alternate-function selections for the currently configured **STM32H745IITx, LQFP176** device:

| MCU pin | CubeMX signal | Core context | GPIO mode | GPIO pull | Max GPIO output speed | MCU package pad (LQFP176) |
| --- | --- | --- | --- | --- | --- | --- |
| **PA11** | **FDCAN1_RX** | ARM Cortex-M7 | Alternate function | No pull-up/pull-down | Low | **130** |
| **PA12** | **FDCAN1_TX** | ARM Cortex-M7 | Alternate function | No pull-up/pull-down | Low | **131** |

**Source verification:** ST [STM32H745xI/G datasheet (DS12923), pin/ball definition and alternate functions](https://www.st.com/resource/en/datasheet/stm32h745xi.pdf) lists PA11 as FDCAN1_RX (also OTG_FS_DM) and PA12 as FDCAN1_TX (also OTG_FS_DP) for the package; pad numbers above refer specifically to **LQFP176**, not an arbitrary STM32H745 package. Exact final part number/board package remains to be confirmed.

**Rationale and decisions:**

- **Core ownership:** FDCAN1 remains on **M7**, matching the single-owner CANopen Master/Supervisor software architecture. This is a peripheral-core ownership choice; the physical pins themselves are chip pins.
- **PA11/PA12 are provisional**, selected by CubeMX. They are legal alternate functions, but share functions with **USB OTG FS data minus/plus**. Check future USB/other peripheral needs, PCB routing, transceiver location and any conflicts **before freezing the hardware pin map**.
- **No pull** was the shown GUI setting, **not a separately verified hardware recommendation**. The transceiver's RXD output drives the MCU's RX pin; use transceiver datasheet requirements for line idle state, power sequencing and bus failsafe.
- **GPIO maximum output speed = Low** was shown for both pins. This is the *GPIO slew/output-drive setting* on MCU pins, **not the FDCAN bit rate**; assess TXD edge integrity at the actual transceiver input, trace load and board design before finalizing. Do not blindly equate a "Low" GPIO speed selection with a 1 Mbit/s bus specification.
- PA12 (TX) and PA11 (RX) are **digital-side** signals: connect to a compatible external CAN transceiver's TXD/RXD. The physical bus uses transceiver CAN_H/CAN_L with appropriate termination and wiring. No transceiver, pin-level electrical design or safety function has been verified yet.

**Evidence/status:** These mappings are confirmed **only in the local CubeMX GUI screenshot**. The repository's authoritative `.ioc` has not been updated with the new FDCAN configuration or generated for active HAL code. Physical pinout is **provisional**, not approved for the final PCB.

**Next check:** review **FDCAN1 → NVIC Settings** and record selected interrupt lines/priorities. The present `fdcan_classic_backend.c` polls HAL FIFO0 and checks protocol status from task context rather than requiring RX callbacks, so do **not** enable arbitrary interrupt priorities or define ISR handlers before reviewing the final execution/diagnostics architecture.

### FDCAN1 NVIC settings screenshot (2026-10-14)

The operator's **FDCAN1 → NVIC Settings** screenshot shows all three checkboxes **unchecked** in the **NVIC1 Interrupt Table** (M7):

| Interrupt | NVIC enabled | Displayed preemption priority | Displayed sub-priority |
| --- | --- | ---: | ---: |
| FDCAN1 interrupt 0 | **No** | 5 | 0 |
| FDCAN1 interrupt 1 | **No** | 5 | 0 |
| FDCAN calibration unit interrupt | **No** | 5 | 0 |

**Decision:** Leave these IRQs disabled for the current **HAL FIFO polling** adapter. The existing `fdcan_classic_backend.c` uses `HAL_FDCAN_GetRxFifoFillLevel` / `HAL_FDCAN_GetRxMessage` from the owning task; it does not configure `HAL_FDCAN_ActivateNotification` or install `HAL_FDCAN_RxFifo0Callback`. Enabling an IRQ without defining its event subscriptions, correct ISR wiring and interrupt safety would not improve this implementation. The priority/sub-priority numbers are **inactive defaults**, not verified real-time priorities.

**Verification still required:** measure worst-case task polling interval, RX FIFO occupancy/loss, the rate of CAN reception, error handling and bus-off recovery. `HAL_FDCAN_GetProtocolStatus` is polled in the current adapter, but this is not a substitute for final fault-monitoring latency analysis; higher-rate or safety-relevant event handling may justify interrupts at a later stage, following FreeRTOS and STM32 NVIC priority rules. The calibration unit interrupt remains disabled since the clock calibration feature is not enabled.

**Evidence status:** screenshot of local CubeMX NVIC settings only. The new FDCAN1 `.ioc` and generated active HAL firmware have **not** been committed or verified. Do not interpret disabled RX interrupts as disabling the FDCAN peripheral itself.

**Next configuration gate:** inspect the complete FDCAN1 parameter summary and the CubeMX **Project Manager → Code Generator** options before the first regeneration, then save and compare the changed `.ioc`. Generation must preserve Stage 2 AMP boot, HSEM, SRAM4 MPU/IPC and USER CODE integration.


### CubeMX Code Generator settings (2026-10-14)

The operator's screenshot of **Project Manager → Code Generator** shows the following deliberate GUI choices:

| Option | Selected state | Engineering reason |
| --- | --- | --- |
| STM32Cube packages — copy only necessary library files | **Selected** | Keep the generated project self-contained without copying unrelated vendor modules |
| Generate peripheral initialization in separate `.c/.h` files | **Unchecked** | Maintain the existing main-file initialization layout and avoid unnecessary structural churn |
| Backup previously generated files when re-generating | **Checked** (changed from unchecked) | Ask CubeMX to retain its own backups of replaced generated files, in addition to the required independent project snapshot |
| Keep User Code when re-generating | **Checked** | Preserve recognized `USER CODE BEGIN/END` regions; this does **not** preserve all custom linker or IDE metadata |
| Delete previously generated files when not re-generated | **Unchecked** (changed from checked) | Reduce the chance of silently removing custom-staged files or previously generated build inputs |
| Set all free pins as analog | **Unchecked** | Avoid new pin changes without a PCB review |
| Enable Full Assert | **Unchecked** | Leave the prior HAL instrumentation setting unchanged during FDCAN-only regeneration |
| Before / After Code Generation actions | **Blank** | No custom external generation hooks installed |

**Screenshot evidence:** CubeMX GUI only. The title bar still displayed `GP_STM32H745.ioc*`, consistent with *unsaved* configuration edits at screenshot time. **Next:** take a separate full project snapshot **before saving/generating**, then save the .ioc and inspect its diff, then regenerate in a controlled operation. These Code Generator checkboxes **do not guarantee** preservation of custom SRAM4 linker maps, dual-core HSEM startup, MPU, FreeRTOS application source staging, or `.cproject` entries. Verify each explicitly after generation. No code generation or updated target build is claimed.


### Saved .ioc vs tracked repository diff (2026-10-14)

The operator ran `git diff --no-index` comparing the **saved local** `GP_Repo_Verification/GP_STM32H745.ioc` with the **original tracked repository** `Firmware/STM32H745/GP_STM32H745.ioc`. The textual diff shows:

- **Expected:** `FDCAN1` added to `CortexM7.IPs`; `FDCAN1.*` parameters for 1 Mbit/s / prescaler 1 / SJW 8 / Seg1 34 / Seg2 8 / 16-element TX and RX FIFO0 / automatic retransmission.
- **Expected:** `PA11=FDCAN1_RX`, `PA12=FDCAN1_TX`, both M7; `Mcu.IPNb=9→10`, `Mcu.PinsNb=4→6`, and generated function-list additions `MX_GPIO_Init`, `MX_FDCAN1_Init`.
- **Expected:** `RCC.DIVQ2=6`, `RCC.DIVQ2Freq_Value=43000000`, `RCC.FDCANCLockSelection=RCC_FDCANCLKSOURCE_PLL2`, `RCC.FDCANFreq_Value=43000000`. The other RCC output frequencies remain as before in the diff.
- **Expected:** `ProjectManager.BackupPrevious=false→true`; `ProjectManager.DeletePrevious=true→false`.
- **Needs explicit review before generation:** `rtos.0.ip=FREERTOS_M4` changed to `rtos.0.ip=FREERTOS_M7` even though the operator did not intentionally change RTOS ownership. The original file lists **both** `FREERTOS_M4` and `FREERTOS_M7` in their respective core IP lists with separate tasks. This metadata key's effect on CubeMX regeneration has **not been established**; verify both FreeRTOS middleware configurations are still active on their intended cores before code generation. **Do not blindly change this key by hand.**
- **Diff display/metadata:** `100644→100755` may be caused by WSL Windows-mount permission inference; `^M` are carriage-return indicators (CRLF). Neither itself implies an STM32 HAL/runtime change. Preserve ordinary non-executable `.ioc` mode and use a controlled newline policy when importing the file to Git.
- The earlier `diff` against backup `GP_Repo_Verification_Backup_20261014_140623` was empty, so that snapshot contains **the same .ioc content** as the saved version. Treat it as a complete filesystem rollback for the pre-regeneration sources, **not** as a guaranteed pre-FDCAN configuration snapshot. The original source-controlled .ioc remains available on GitHub.

**Status:** The .ioc contents were reviewed *from operator-provided diff*, but have **not been imported or committed** to GitHub, and no new code generation has occurred. To proceed, confirm CM4/CM7 FreeRTOS configuration in the CubeMX GUI and save a pre-generation copy of the FDCAN-enabled .ioc under version control.

### Dual-core FreeRTOS GUI confirmation (2026-10-14)

The operator supplied screenshots of **both** STM32CubeMX middleware configuration pages after saving the FDCAN-enabled local `.ioc`:

| Core | Middleware page | Runtime context | RTOS API | Kernel / CMSIS-RTOS | Tick |
| --- | --- | --- | --- | --- | --- |
| Cortex-M4 | `FREERTOS_M4` | **Cortex-M4 checked**, power domain D2 | CMSIS_V2 | 10.6.2 / 2.1.3 | 1000 Hz |
| Cortex-M7 | `FREERTOS_M7` | **Cortex-M7 checked**, power domain D1 | CMSIS_V2 | 10.6.2 / 2.1.3 | 1000 Hz |

Both configurations show **USE_PREEMPTION enabled**. Therefore the change from `rtos.0.ip=FREERTOS_M4` to `rtos.0.ip=FREERTOS_M7` in the local text diff did **not** remove either middleware instance in the GUI. **This does not prove it is semantically inert for generated files.** After regeneration, compare both core's `FreeRTOSConfig.h`, `freertos.c`, CMSIS RTOS startup, project metadata, USER CODE regions, and task initialization against the backup, and rebuild both. Do not hand-edit `rtos.0.ip` based solely on intuition.

**Next control:** import the **saved** FDCAN-enabled local `.ioc` into the tracked repository (normalizing Windows-mount file permissions and line endings), commit it separately from generated firmware, and then run CubeMX code generation in the backed-up verification workspace. The present GitHub documentation does not imply this import/generation has occurred.


### FDCAN1 .ioc committed and generated sources locally inspected (2026-10-14)

The operator committed the actual saved `GP_STM32H745.ioc` to `sara/can-replacement` as **`c3d07a2`** (`config(stm32): configure FDCAN1 for 1Mbps Classic CAN`). Git reported **46 additions / 21 deletions** after restoring its original CRLF line endings; `git -c core.whitespace=cr-at-eol diff --check` passed. The earlier source-controlled `.ioc` is no longer the canonical FDCAN-disabled version.

Next, the operator ran STM32CubeMX code generation **in the separate, backed-up Windows verification project**. They provided a WSL inspection confirming:

- Both official vendor files now exist in `GP_Repo_Verification/Drivers/STM32H7xx_HAL_Driver/`: `Inc/stm32h7xx_hal_fdcan.h` and `Src/stm32h7xx_hal_fdcan.c`.
- M7 generated `main.c` still contains `intercore_shared_memory_initialize` and `application_supervisor_start`.
- M4 generated `main.c` still contains `intercore_startup_validate` and `application_ipc_service_run`.
- M7 `.cproject` still contains source folders named `Application` and `Board`.
- The checksum-based dry-run comparison against the **20261014_140623 pre-generation snapshot** reported changes to both core `.cproject` metadata, CM7 `.project`, CM7 `Core/Inc/stm32h7xx_hal_conf.h`, CM7 `Core/Src/main.c`, CM7 `Core/Src/stm32h7xx_hal_msp.c`, plus language settings and `.mxproject`.
- Its `*deleting` lines mean **new items in the generated destination compared with the older backup**, including the expected new FDCAN HAL files and CubeMX `Backup/*.bak` directories; the command used `rsync -rcni --delete` (dry run), so this output does **not** report actual deletion.
- The inventory did **not** show changes to linker scripts or `freertos.c`, subject to rechecking the actual files and successful target build.

**Current acceptance:** generated FDCAN HAL files present and six application preservation substring checks pass. **Pending:** inspect actual `MX_FDCAN1_Init`, `HAL_FDCAN_MspInit`, active HAL module define, project source registration and both core FreeRTOS output; compile/link **both actual CubeIDE projects**. The `FDCAN` adapter's active branch has **not** yet passed a target build. Do **not** sync the older canonical firmware directory over this newly generated local verifier before inspecting/importing its generated changes. Physical bus operation still NOT RUN.


### CubeMX regeneration line-by-line preservation check (2026-10-14)

The operator ran a **read-only Python comparison** of `GP_Repo_Verification` against `GP_Repo_Verification_Backup_20261014_140623` after CubeMX generated FDCAN1 (the local working tree, not yet imported into the GitHub firmware source tree). Reported results:

- **PASS:** `HAL_FDCAN_MODULE_ENABLED` is set in CM7 HAL config; `MX_FDCAN1_Init()` is called from M7 `main.c`; `HAL_FDCAN_MspInit` exists; official HAL driver `stm32h7xx_hal_fdcan.c` and header exist.
- **PASS:** M7 `intercore_shared_memory_initialize()` and `application_supervisor_start(&config)` remain present; M4 `intercore_startup_validate()` and `application_ipc_service_run()` remain present.
- **PASS:** CM7 `.cproject` still references **Application** and **Board** source folders.
- **Byte-for-byte unchanged:** both cores' `freertos.c` and `FreeRTOSConfig.h` plus each of the four CM7/CM4 Flash/RAM linker scripts.
- **M7 main.c generated additions:** `FDCAN_HandleTypeDef hfdcan1`, `MX_GPIO_Init()`, `MX_FDCAN1_Init()`; Classic CAN / Normal mode, auto retransmission enabled, nominal timing `1/8/34/8`, FIFO0 RX 16 × 8 B, TX FIFO 16 × 8 B, 0 dedicated TX/RX buffers, no standard/extended dedicated filters.
- **M7 MSP generated additions:** `HAL_RCCEx_PeriphCLKConfig` for FDCAN PLL2 (`M=32,N=129,P=2,Q=6,R=2`, PLL2Q selected), peripheral/GPIOA clocks and `PA11 FDCAN1_RX` / `PA12 FDCAN1_TX` (AF9, no pull, Low GPIO speed). This is the selected *provisional* pin arrangement, not a final PCB pin approval.
- M7 `SystemClock_Config` also gains `__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI)`; no other unexpected main/MSP edits appeared in the provided text diff. Generated startup order and runtime clock behavior still need a target/board test.

**HAL data-length interpretation check:** ST's [HAL1 FDCAN header (`stm32h7xx-hal-driver` v1.11.4)](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/blob/v1.11.4/Inc/stm32h7xx_hal_fdcan.h) defines `FDCAN_DLC_BYTES_0=0` through `FDCAN_DLC_BYTES_8=8`. Therefore the portable backend's current `received.dlc=(uint8_t)header.DataLength` agrees with the HAL1 Classic CAN interpretation. The *generated, installed* CubeH7 V1.13.0 header must still be the authoritative build-time version.

**Next pending acceptance gate:** use the **newly generated** project without overwriting it from the older repository snapshot; **Clean → Build** CM7 and CM4 in STM32CubeIDE, capture the first genuine warning/error, and inspect HAL FDCAN and app symbols in the resulting CM7 ELF. This source-level regeneration check is *not* an active HAL compile/link pass, physical CAN pass, or completed robot board composition.


### Cortex-M7 HAL FDCAN linker resolution and successful build (2026-10-14)

The first post-generation **CM7 CubeIDE build failed** at link time: `undefined reference to HAL_FDCAN_Init` and consequential ARM/Thumb relocation errors. Diagnosis established that the official `Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c` existed on disk and was listed in the *on-disk* CM7 `.project`, but **was absent from the IDE's Project Explorer / Linked Resources**, had no `Debug` makefile entries, and produced no HAL FDCAN object. The user re-registered the existing ST source file as a **linked file in the CM7 Drivers/STM32H7xx_HAL_Driver folder** through STM32CubeIDE (rather than copying or rewriting the HAL source). This fixed the build.

**User screenshot of the subsequent CM7 Build Console** shows:

```text
Finished building target: GP_STM32H745_CM7.elf
text    data    bss     dec     hex
159792  112     91536   251440  3d630
14:47:15 Build Finished. 0 errors, 0 warnings.
```

This constitutes **an actual Cortex-M7 compile-and-link PASS with FDCAN included**, not a board-level execution test. Pending before Stage 3C acceptance: build CM4, capture `arm-none-eabi-nm` proof of HAL FDCAN and app/IPC symbols from both new ELFs, inspect and commit local generated HAL source/header and updated CM7 Eclipse link metadata into the repository without overwriting the verifier. The weak M7 board configuration provider still fails closed, and physical CAN operation is unverified.


### Cortex-M4 companion firmware build PASS (2026-10-14)

The operator's STM32CubeIDE screenshot confirms a successful **CM4 Debug firmware ELF link** after the M7 HAL FDCAN link fix:

```text
Finished building target: GP_STM32H745_CM4.elf
text    data    bss     dec     hex
24644   100     21148   45892   b344
14:51:02 Build Finished. 0 errors, 0 warnings.
```

Combined observed build results in the Windows verifier on this date:

| Core | text (B) | data (B) | bss (B) | Build |
| --- | ---: | ---: | ---: | --- |
| CM7 | 159792 | 112 | 91536 | PASS: 0 errors, 0 warnings |
| CM4 | 24644 | 100 | 21148 | PASS: 0 errors, 0 warnings |

**Stage 3C-2 acceptance, scoped precisely:** Both updated Cortex-M7/M4 firmware projects compile/link; the CM7 project now includes the official ST HAL FDCAN linked source. Still pending: ARM ELF symbol inspection; import the generated HAL files, updated CM7/CM4 project metadata and initialization into source control; clean reproduction from source; physical CAN bus, motors, STM32 board power/boot, and timing verification. The weak M7 board provider remains fail-closed, so **neither successful build implies usable robot drive output**.
