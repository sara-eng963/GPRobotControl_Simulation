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
