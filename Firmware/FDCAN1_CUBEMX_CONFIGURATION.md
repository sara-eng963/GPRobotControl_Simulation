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
| Auto Retransmission | **Disable** | Decide on enablement after reviewing CANopen error recovery and latency; ordinarily retransmission is beneficial, but analyze repeated errors and control deadlines |
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
- [ ] Complete Auto Retransmission / message RAM / FIFO / filters / GPIO / NVIC configuration
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

**Now confirmed by subsequent operator screenshot (2026-10-14):** `Rx Fifo0 Elmts Nbr = 16`, meeting the prepared HAL adapter's `RxFifo0ElmtsNbr >= 16` requirement. No FIFO timing, occupancy, overflow, or delivery guarantee follows merely from allocation. Message RAM use and interaction with a second FDCAN instance, if selected later, must be checked against the STM32H745 reference manual. Automatic retransmission and physical I/O are still pending.

**Status:** Both 16-element TX FIFO and 16-element Rx FIFO0 configurations have GUI screenshot evidence. They remain uncommitted to the canonical `.ioc` and unverified in active HAL firmware. No changed CubeMX `.ioc` has been fetched into the repository and no active HAL build has been claimed.

### Receive FIFO0 confirmation (2026-10-14)

A second screenshot explicitly shows `Rx Fifo0 Elmts Nbr = 16`, `Rx Fifo0 Elmt Size = 8 bytes data field`, `Rx Fifo1 Elmts Nbr = 0`, `Std Filters Nbr = 0`, `Ext Filters Nbr = 0`, and `Message RAM Offset = 0`. The standard-filter count of zero is an intermediate setting; the current board adapter uses HAL's global acceptance for standard frames and rejection for extended/remote frames. This permissive standard-frame policy must be revisited after actual node IDs/COB-IDs and bus traffic are confirmed; do not present it as a production-optimized acceptance filter. No CubeMX generation or physical bus test has happened yet.
