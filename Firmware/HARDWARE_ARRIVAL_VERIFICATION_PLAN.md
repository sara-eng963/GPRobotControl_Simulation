---
title: "STM32H745 Robot Controller — Hardware Arrival & Verification Plan"
date: 2026-10-14
status: "PRE-HARDWARE PLAN — no physical tests executed"
branch: "sara/can-replacement"
tags: [graduation-project, stm32h745, hardware-bringup, canopen, safety, verification]
---

# Hardware Arrival & Verification Plan

> **Purpose:** When the actual STM32H745 board, CAN transceiver, six drives, external NOR flash, sensors and robot hardware arrive, work through this plan **in order**. For each stage, capture measurements, compare them to a stated requirement, decide what firmware/configuration must change, and retain evidence. This is an engineering test plan, **not a claim that any hardware has already passed**.
>
> **Critical rule:** **No uncontrolled motor energization or robot motion.** All initial tests use disconnected actuator power or a properly secured bench setup. A software E-stop flag, CAN command, camera/person detector or FreeRTOS task is **not** a substitute for independent safety-related stop hardware. Electrical commissioning, brake handling and powered motion need competent supervision and a documented risk assessment.

## 0. Current project baseline — distinguish code from hardware

| Subsystem | Implemented / verified in software so far | NOT YET VERIFIED / still to implement |
| --- | --- | --- |
| STM32H745 | Separate CM7 and CM4 CubeIDE images, ST-generated boot infrastructure, HAL + FreeRTOS | Flashing/booting the actual chip; real clocks, temperature, power and resets |
| Inter-core IPC | Shared SRAM4 FreeRTOS Message Buffers, HSEM/EXTI notification source checks, M4 IPC task linked | Actual M7↔M4 traffic, contention, reset recovery, MPU/cache behavior under load |
| CM7 application | Real Supervisor and CANopen code linked; **weak board provider intentionally returns false** | Real safety I/O, complete board composition, actual drive commissioning/motion authorization |
| FDCAN1 | HAL1 Classic CAN adapter source; inactive/host-mock gates passed | Active HAL FDCAN driver generated/linked, transceiver, correct pins, live bus frames, FIFO health |
| FDCAN1 GUI | M7, Classic/Normal, PLL2Q 43 MHz; nominal prescaler 1, SJW 8, Seg1 34, Seg2 8; 1 Mbit/s readout; TX/RX FIFO0 16 × 8 bytes; auto retransmit enabled; PA11 RX/PA12 TX provisional; NVIC interrupts off | **Verify the final saved .ioc and generated files**. GUI screenshots alone are not source-control configuration |
| CANopen/AVATAR | Existing NMT, SDO, heartbeat, RPDO4, TPDO4, SYNC, CiA-402 decoding in repository | Firmware/EDS/object dictionary/version of purchased drives, actual node IDs, EMCY behavior and repeatable 500 Hz |
| Motion path | M7 trajectory consumer + portable validation/execution code; 2 ms sample contract | Dedicated monotonic timer, real scheduling jitter, physical encoder and following-error evidence |
| M4 external memory | Host / Renode NOR simulations, CRC, streaming and simulated power-failure tests | CubeMX HAL QSPI/NOR board driver and pins, exact flash part, on-device transactions and endurance |
| Shared validation storage | Portable bridge/host tests | Final SRAM4 allocation, producer/worker integration and physically durable flash acknowledgments |
| HMI | Portable command/status contract | Actual M4 HMI physical transport and electrical interface |
| Hand guiding | Proposed external handle with a **6-axis force/torque sensor on the handle itself** | Sensor procurement, mounting/calibration, RS-485 electrical/protocol integration, handling, safe operation |
| Safety | Fail-closed software state machine and no-operation default board hook | Real independent E-stop, protective stop, drive inhibit/STO, brakes, wiring and systematic validation |

For configuration decisions made in CubeMX on 2026-10-14, use **[FDCAN1_CUBEMX_CONFIGURATION.md](FDCAN1_CUBEMX_CONFIGURATION.md)**. For progress and unresolved source integration, use **[STAGE3_APPLICATION_INTEGRATION.md](STAGE3_APPLICATION_INTEGRATION.md)**. These files are **design evidence, not physical acceptance certificates**.

## 1. Before arrival: assemble an evidence pack

- [ ] Record **exact MCU order code and package**, actual PCB revision, schematic, layout, pin map, bill of materials, oscillator frequencies, voltage rails and regulator/current limits.
- [ ] Obtain MCU datasheet, **RM0399**, device errata **ES0445**, STM32CubeH7 package version, transceiver datasheet and board-specific power/clock notes.
- [ ] Obtain **exact drive make/model, firmware, EDS/object dictionary, node ID assignment and connector pinout** for all six motors; do not assume a simulation AVATAR object map applies to a substituted motor.
- [ ] Obtain gearbox ratios, encoder locations and resolution, joint direction conventions, mechanical zero procedure, torque/current/velocity/soft-limit values and brake behavior.
- [ ] Obtain flash manufacturer/part/revision datasheet, supply voltage, memory size, erase/page timing and endurance; confirm whether the physical part truly is W25Q512JV or another model.
- [ ] Obtain F/T sensor's final part/revision, force and torque ranges, 6-axis cross-coupling/noise, update rate, RS-485 protocol, pinout, 24 V electrical requirements and STEP model. The sensor is on the **external guiding handle, not the welding flange**.
- [ ] Finalize system wiring for CAN_H/L, termination, CAN transceiver TXD/RXD/STB/EN, shield/grounding, E-stop circuit, brakes, motor enable, watchdog, FT interface, RS-485 bias/termination and flash signals.
- [ ] Secure suitable equipment: current-limited bench supply, DMM, oscilloscope, CAN analyzer/USB-CAN interface with timestamps and decoding, logic analyzer, debugger/ST-LINK, ESD protection, appropriate load/fixture and emergency power disconnect.
- [ ] Assign owner, reviewer, date, board serial number and revision to every test. Save raw traces in a controlled folder with firmware commit SHA.

**Configuration freeze:** Back up the CubeMX verification workspace before code regeneration. Capture Git diff of .ioc, clock tree, HAL config, pins, generated init/IRQ, linker scripts and user sections. No guessed alternate functions or hand-edited HAL drivers. Store firmware-build hashes and target-specific build logs.

## 2. Gate map — do not skip ahead

| Gate | Condition / test level | Go / no-go before proceeding |
| --- | --- | --- |
| **H0** | Documentary, schematic and firmware baseline | Actual device/board identified; pin, voltage and safety architecture known |
| **H1** | Unpowered visual/electrical inspection | No short/incorrect polarity; protection and stop chain understood |
| **H2** | Power-up and dual-core boot, **drives not enabled** | Stable supply, correct image placement, independent schedulers/clock diagnostics |
| **H3** | SRAM4 IPC and RTOS stress | Nonblocking cross-core exchange, fresh telemetry, safe reset/restart handling |
| **H4** | FDCAN physical link with passive bench node | Correct 1 Mbit/s operation; trace RX/TX, bus errors, FIFO depth/backpressure |
| **H5** | Six-node CANopen commissioning, motors **inhibited** | Identity/EDS, NMT, SDO/PDO/SYNC/heartbeat and CiA-402 states verified |
| **H6** | 500 Hz / 2 ms communication and CPU deadline benchmark | Empirical 2 ms deadline and bus traffic evidence; no unbounded draining |
| **H7** | M4 flash, validation, trajectory streaming and power recovery | CRC/commit/restart/cancel/refill/underrun cases evidenced on actual memory |
| **H8** | Physical safety controls and sensor/HMI integration | Independent safety hardware validated; no software bypass |
| **H9** | Limited, supervised single-axis then multi-axis motion | Joint direction/limits/brakes/feedback and stopping proven |
| **H10** | Integrated operation, repeatability, environmental and fault runs | Signed test report; unresolved hazards blocked from motion |

**Gate failures:** Record **FAIL/BLOCKED**, stop the test, prevent motion as appropriate, correct the specific board/config/application issue, rerun the failed gate **and** relevant regressions. **Never turn an untested assumption into PASS.**

## 3. H0–H2: electrical and CPU bring-up

### H0 — Pinout and hardware-design confirmation

- [ ] Compare selected STM32CubeMX package **STM32H745IITx / LQFP176** with the **actual** component; verify each pin against the final PCB, not a development board.
- [ ] Check provisional **PA11=FDCAN1_RX and PA12=FDCAN1_TX**. They can overlap with USB OTG FS functions; select final AF pins only after peripheral/PCB conflict review.
- [ ] Confirm PLL2Q **43 MHz** and 1 Mbit/s setup are in the **saved .ioc** (not just GUI), and reconcile startup CPU clocks, voltage scale, regulator mode, flash wait states and errata.
- [ ] Confirm required FDCAN message RAM allocation and startup for HAL-generated driver; if FDCAN2 is added, verify both controllers' non-overlapping RAM offsets.
- [ ] Before generating QSPI, timer, RS-485, GPIO or CAN, check ownership: **FDCAN + motion + Supervisor on CM7; flash/HMI/trajectory producer on CM4; SRAM4 IPC shared**.

### H1 — Unpowered board

- [ ] Inspect assembly, soldering, orientation, connectors, keyed supplies, ground continuity, transceiver polarity, voltage compatibility, fuses and accessible test points.
- [ ] Check no unintended short exists between rails/ground or at motor driver power terminals. Confirm expected CAN termination topology using the schematic and meter; **do not assume a measured resistance proves bus quality**.
- [ ] Confirm the E-stop/stop circuits are hard-wired through the correct safety architecture and motor power/drive inhibit remains inactive during MCU startup, reset and programming.
- [ ] Do not attach actuator load for first electrical tests; follow lab electrical-safety procedures, including isolation for oscilloscope measurements.

### H2 — First powered MCU test

1. Apply **current-limited** power with external motor drive enable disconnected; record inrush, steady current and rail voltages.
2. Flash **both CM7 and CM4** images using STM32CubeIDE/ST-LINK; verify flash addresses, option bytes, boot settings and debugger connectivity. Avoid modifying option bytes without the board owner.
3. Verify exact generated clock tree at runtime (system and FDCAN kernel), reset cause, voltage-scale readiness and any clock/security fault indicators.
4. Verify CM7 initializes MPU/shared objects **before HSEM-0 wake**; CM4 wakes, validates them, and both kernel schedulers start.
5. Power-cycle repeatedly and record boot time and intermittent failures. Exercise controlled debugger/manual resets **without motor enable**.
6. Confirm a missing CM4 image, missing peripheral or failed initialization leaves motor permission false and reports a diagnostic rather than continuing as healthy.

**Metrics:** 3.3 V/other rail min-max, current (idle/boot), reset reason, both firmware SHAs, boot time distribution, flash usage, uptime, error counts. **Acceptance:** within actual PCB/MCU datasheet limits, no unexplained reset/hardfault, and a verified fail-closed boot state. Numeric power/boot limits are **TBD from board design**, not invented here.

## 4. H3: dual-core SRAM4 IPC, RTOS and reset resilience

Relevant source: **Intercore/ipc_mailbox_freertos.c**, **intercore_notifications**, **CM4 application_ipc_service.c**, **StateMachine/supervisor_task.c**, SRAM4 linker slots and MPU.

- [ ] Read back/link-check physical SRAM4 assignments: IPC **0x38000000 / 4 KiB**, trajectory **0x38001000 / 16 KiB**, reserved **0x38005000 / 12 KiB**; check the M7 non-cacheable window, 32-bit atomics, memory barriers and linker section ownership.
- [ ] Verify real HSEM wake/notification and EXTI delivery between kernels with sequence-stamped traffic, not only mocked interrupt functions.
- [ ] Stress simultaneous one-writer/one-reader status and command traffic. Check corruption, duplicates, ordering, queue-full returns, bounded retry, missed/coalesced EXTI and periodic poll fallback.
- [ ] Disconnect/stop/restart the CM4 service: M7 must not accept replayed HMI sequences; M4 must reject commands after **100 ms status freshness timeout** currently implemented. Confirm this against real RTOS clocks, including tick wrap.
- [ ] Reset either core independently with drives inhibited: require a defined **new boot epoch / resynchronization policy** and no stale shared-object acceptance. If the existing ABI lacks the required reset handshake, **change the IPC protocol before deploying**; do not blindly reinterpret pre-reset queues.
- [ ] Measure round-trip and one-way delays with source and destination timestamps or GPIO toggles and a common timing reference. Report median, p99, max and lost/dropped messages; include long-run stress and wraps.
- [ ] Inspect task high-water marks/stack, each FreeRTOS heap, ISR stack, CPU load, priorities, watchdog configuration, HSEM lock behavior, D-cache interactions and stalled peer detection. Static link sizes alone **do not prove stack safety**.

**Update firmware if tests fail:** fix the specific synchronization or memory-order issue; review MPU/MPU cache settings via CubeMX and ST's AMP examples; add reset epoch/handshake, increase appropriately justified task stacks, tune bounded queue drain/period; **do not invent a parallel IPC layer**.

## 5. H4: FDCAN electrical, polling, FIFO and error behavior

**Starting design (not yet active HAL hardware):** CM7 FDCAN1; 11-bit Classic CAN; target 1 Mbit/s; 43 MHz PLL2Q; nominal prescaler **1**, Seg1 **34**, Seg2 **8**, SJW **8**, ~81.4% nominal sample point; **16 TX FIFO slots and 16 RX FIFO0 slots of 8 bytes**, TX FIFO, auto retransmission on, Rx FIFO1 off, NVIC IRQs currently disabled. PA11/PA12 provisional.

1. **Transceiver and passive bring-up:** verify transceiver supply/reference voltages, TXD/RXD/STB/EN states, actual pins/AF, CAN_H/CAN_L differential idle/dominant levels, common-mode range, cable topology and 120-ohm termination at **both physical ends** only as applicable. Use the transceiver datasheet. Do not connect to energized drives while the physical electrical interface is unverified.
2. Use a **separate known-good CAN analyzer/bench node** capable of acknowledging frames. A single controller in Normal mode may see ACK-related retransmissions if there is no other active CAN node.
3. Validate 1 Mbit/s nominal bit timing by an external trace; verify CAN IDs, DLC, byte ordering, transmit order, inter-frame spacing, SYNC (COB-ID **0x080**, DLC **0**) and frame-count behavior. **Tx FIFO enqueue success is not proof of physical transmission**.
4. Instrument HAL TX free-level, pending depth/age, RX FIFO0 fill/peak, consumed frames per supervisor cycle, overflow/message-lost flags, transmit/receive error counters, warning/passive/bus-off transitions, arbitration/retransmissions and application send return values.
5. Test with controlled high-rate external traffic, escalating to realistic six-drive frames. At each load, measure **worst-case duration of CANopen master polling** and highest RX FIFO occupancy, not just averages.
6. **Known code review point:** existing **CANComm/CANopen/canopen_master.c → canopen_master_poll()** has a receive-until-empty loop with **no explicit maximum frame budget**. Under sustained RX it can consume an unbounded share of the supervisor period. Before motion use, benchmark and, if warranted, introduce a **bounded receive budget or dedicated RX handling** that preserves ordering and reports backlog. Confirm this is compatible with the single-owner CANopen architecture.
7. Current **fdcan_classic_backend.c** returns WOULD_BLOCK if TX FIFO is full and checks protocol BusOff; it **does not implement a complete hardware fault monitor** or measured TX completion/deadline semantics. Review whether supervisory fault response also needs FDCAN error/overflow notifications or an interrupt-driven path.
8. Test lost arbitration, error frames, missing ACK, disconnected/failed partner, bus-off and recovery **on a controlled bench using approved test fixtures or simulator-based fault injection**. Do not short live CAN wires or intentionally damage transceivers. Automatic retransmission can delay or make queued commands stale; specify what to cancel/drop after a missed motion deadline.
9. Decide **polling versus interrupt/DMA** using measurements: if polling demonstrably meets the arrival/deadline budget and fault-detection latency, keep it; otherwise configure **CubeMX-generated NVIC**, HAL notifications and RTOS-safe ISR handoff. Interrupt priority must satisfy FreeRTOS ISR API restrictions; never call blocking operations in ISR context.
10. Change filters only after verifying required CANopen COB-IDs/EDS: initial global standard-ID acceptance may permit unrelated traffic to consume FIFO. Measure unwanted frames before adding hardware acceptance filters; keep required EMCY, heartbeat, SDO, PDO and SYNC traffic.

**Evidence:** raw timestamped CAN trace, oscillograms, clock/timing values, idle/load bus utilization, dropped RX, backlog high-water, error counters, bus-off log and recovery timing. **Accept only with actual agreed deadline and error budgets; zero unexplained drops/overflows during the specified nominal test.**

### Analytical 1 Mbit/s feasibility warning

The current AVATAR encoding is **RPDO4 = 4 bytes per motor**, **TPDO4 = 6 bytes per motor**, plus **one zero-data SYNC**. **If** every motor produces one TPDO4 per 2 ms cycle, this means **6 RPDO + 6 TPDO + 1 SYNC = 13 frames/cycle**. Ignoring bit stuffing and assuming 11-bit identifiers plus standard CAN intermission, the baseline wire budget is approximately:

- 6 × 79 bits (4-byte frames) + 6 × 95 bits (6-byte frames) + 1 × 47 bits (zero-data SYNC) = **1,091 bits/cycle**.
- At 500 cycles/s = **545,500 bit/s (~54.6% of a 1 Mbit/s bus)** *before* stuff bits, heartbeats, SDO, NMT, EMCY, error retransmissions or other nodes.
- This is an **illustrative lower-bound model**, not a measured busload, validated device mapping or guarantee of a 2 ms deadline.

**Decisions driven by measurement:** maintain one CAN bus at 500 Hz if the worst-case budget and freshness pass; otherwise evaluate reduced TPDO/command rate, optimized PDO mapping, event-driven feedback, a **two-FDCAN-bus (3+3)** architecture if physical FDCAN2 and driver composition support it, or a different target control architecture. **Never silently relax position/feedback guarantees or assume FIFO buffering fixes overload.**

## 6. H5: real six-drive CANopen and CiA-402 commissioning

Use exact purchased manufacturer firmware, EDS, CANopen manual and CiA 301 / CiA 402 semantics; confirm AVATAR-specific objects rather than automatically treating GDZ/ATAR/SigGear alternatives as equivalent.

- [ ] Independently check six node IDs (intended 1…6 in the current coordinator), baud rate, termination, cable lengths, power-up order and identical/compatible object dictionaries.
- [ ] With drives **inhibited**: read manufacturer/product/revision and serial/identity via **0x1018**; retain SDO abort codes, wrong/missing node IDs and firmware compatibility evidence.
- [ ] Verify NMT reset communication → boot-up → pre-operational → operational, heartbeat producer **0x1017**, heartbeat consumer **0x1016**, controller heartbeat, watchdog behavior and timeout. Confirm any manufacturer-specific heartbeat expectations.
- [ ] Verify SDO timeout and aborted/partial transactions. Test response matching by node/index/subindex; do not permit another task to transact on the single-owner master.
- [ ] Confirm PDO4 mapping, payload byte order, SYNC behavior, rate (TPDO every cycle or otherwise), statusword, position scaling and default COB-ID from **the exact installed drives**, not only virtual AVATAR frames.
- [ ] Confirm CiA-402 state-machine transitions (Switch On Disabled, Ready to Switch On, Switched On, Operation Enabled, Quick Stop Active, Fault, Fault Reset), operation mode/interpolation mode, manufacturer alarm and EMCY behavior.
- [ ] Ensure startup fails **closed** on wrong identity, heartbeat stale, E-stop, drive alarm, missing feedback, unexpected mode or incomplete six-axis node count.
- [ ] Confirm encoder reading is **motor-side versus joint-side**, sign, gear ratio and offset per joint. The current AVATAR source uses an assumed **32,768 units per motor revolution**; do not carry this into a substituted part without documentation and independent measurement.
- [ ] Ensure all drive controlword/power/brake sequencing is physically safe and agreed with manufacturer instructions **before** permitting Operation Enabled.

**Possible code changes:** update **avatar_m_registers**, **avatar_m_pdo**, **canopen_master**, CiA-402 adapter, **avatar_m_position** scales/zeros, commissioning timeouts, acceptance filters, and safety fault mapping. Keep one canonical coordinator; decide explicitly whether to reuse/replace protocol pieces with a mature CANopenNode implementation after a standards/effort review.

## 7. H6: deterministic real-time control benchmark

**Requirement candidate:** a **2,000 µs nominal motion-sample period / 500 Hz**. This is **not yet a hardware-proven rate**; a 1 ms FreeRTOS tick and a 2 ms sample field are insufficient evidence.

- [ ] Configure/verify a **monotonic microsecond timebase** using a CubeMX-generated timer/DWT time source as appropriate, with rollover handled; do not re-purpose TIM6/TIM7 HAL tick functions silently.
- [ ] Trace scheduled release, actual wake, start/end of Supervisor step, RX polling, six RPDO enqueue, physical SYNC transmission, TPDO reception/age and trajectory sample consumption. Use a logic analyzer/oscilloscope for edge timing where possible; avoid printing inside real-time loops.
- [ ] Measure period error, deadline misses, max response time, execution-time min/mean/p99/max, wake jitter, interrupt-disabled duration, preemption, FreeRTOS tick effects and CPU utilization. Use long runs at idle and worst credible concurrent M4 flash/HMI/IPC load.
- [ ] Distinguish **command queued, command transmitted on CAN, command applied at SYNC, feedback received**. Bus arbitration and transceiver latency are not the same as RTOS scheduling delay.
- [ ] Check TPDO age, missed feedback cycles, following error, queue saturation, stale reference rejection, controlled stop on underflow and supervisor queue overrun. Test sustained 500 Hz with 1, 3 and 6 simulated/real nodes (drives inhibited when measuring CAN scheduling).
- [ ] Review whether **1 kHz supervisor state processing** and the **500 Hz target cycle** require distinct scheduling paths; ensure lower-priority IK, validation, flash, logs and UI cannot block the motion-critical path.
- [ ] Check that the 16-frame CAN FIFOs plus software backpressure are sufficient under worst-case bursts without permitting an old 2 ms command to execute late.

**Acceptance is not “the average is 2 ms.”** Define the maximum tolerated jitter, end-to-end age, stale feedback threshold, queue backlog and missed-cycle response with supervisors/drive specs. Target **zero unexplained deadline misses during the defined nominal stress run**; measure maxima and report failures. Any unmet budget forces redesign/timing optimization **before motion authorization**.

## 8. H7: M4 external NOR flash, validation and trajectory producer

The current **simulation/Renode W25Q512JV code is not a production HAL board port**. The real implementation must use matching STM32CubeMX QSPI/OSPI peripheral configuration and the purchased flash datasheet; source-level flash semantics remain application-owned.

- [ ] Verify actual QSPI/OSPI pins, memory supply, reset/hold/wp, flash command set, quad-enable, JEDEC ID, erase/program/read status, address mode, page/sector sizes, timeouts and wear.
- [ ] Test read-after-write, erase-before-write, CRC, boundary/alignment, page crossing, end-of-device access, failed/time-out operation and unexpected power loss using a safely designed test fixture **without energizing actuators**.
- [ ] Check **two-slot atomic commit/recovery semantics** with a variety of power-interruption points, corrupt data/header, incomplete commit and reboot; select only a completely validated program.
- [ ] Implement/verify M4 worker ownership, **GpValidationShared** ABI/slot/linker/MPU allocation, cancellation acknowledgment and persistent artifact identity. **Do not assume the reserved 12 KiB SRAM4 slot fits the validation queue** until sizes and linker map are checked.
- [ ] Verify 49,312-sample-style long streams and other realistic workloads. Test M4 **256-sample prefetch batch** feeding a **512-sample shared trajectory ring**; log minimum fill, refill latency, underruns, cancellation, epoch reuse, no old-trajectory replay and CRC checks.
- [ ] Simultaneously stress M7 CAN/IPC during M4 flash erase/program/reader operations, including reset and flash busy time.
- [ ] Power-cycle and reboot between interrupted writes; make sure FLASH_COMMITTED status is not signaled before durable verify. Distinguish CRC-proven artifact from stale cache or uncommitted write.

**Firmware adaptation:** add an ST HAL-based flash transport and production storage service on **CM4**, finalize validation shared address/size, connect producer and worker callbacks, tune buffering only **after measured worst-case flash latency**. Keep simulated file/host/Renode code out of target images.

## 9. H8: physical safety chain, HMI and force/torque handle

### Safety: independent of the MCU

- [ ] Complete a documented machine/workcell risk assessment. Determine protective measures, performance level/safety integrity needs and stop categories from applicable machine safety standards and certified component specifications.
- [ ] Wire a safety-rated **E-stop / protective-stop chain and drive torque-removal path (STO or approved equivalent if supported)** independent of the M7/M4 application. Confirm brakes and gravity-loaded axes remain safe with torque removed.
- [ ] With actuator power safely inhibited, test E-stop asserted/released, wire disconnection, switch bounce, undervoltage, transceiver failure, M7 hang, M4 hang and lost bus. **No command may re-enable drives merely because a software flag changes.**
- [ ] Confirm fault latching, operator acknowledgment/restart interlock and no unexpected restart after power is restored. Require actual safety-hardware validation by qualified team members before powered robot motion.
- [ ] Camera-based human detection, vision zones and software current-collision detection are **supplementary functions**, not safety-rated protective devices unless the whole solution is designed/assessed/certified for that role.
- [ ] Confirm software inputs **SUP_IO_ESTOP**, **SUP_IO_PROTECTIVE_STOP**, HOME/RESET and status LEDs map to correct electrical polarity; a wire-break must not be interpreted as “safe.”
- [ ] Monitor safety signals from the genuine board hardware in **SupervisorIoServices**. The weak **application_supervisor_board_configure** currently returns false and **must remain so** until CANopen master, real input snapshots and all required motion services are wired and validated.

### HMI on CM4

- [ ] Select the actual transport/protocol; define pinout, physical isolation, CRC/framing, parser limits, debounce, access control, status freshness and watchdog behavior.
- [ ] Check an HMI command only enters via **CM4 local queue → SRAM4 M4→M7 IPC → M7 Supervisor**. No CM4 task may call a CM7-local queue or independently authorize motion.
- [ ] Test overflow, invalid command, stale M7 status (currently **100 ms**), reconnection, replay, repeated button press, reset and acknowledgment semantics. A delivered IPC sequence is not proof of action acceptance.

### 6-axis force/torque sensor on the **external hand-guiding handle**

- [ ] Confirm the exact F/T sensor and controller electronics, allowable **handle grip-to-sensor** force/moment lever arm, payload/grip loads, axis sign convention and sensor mounting directions.
- [ ] Load-path requirement: the sensor measures wrench transferred through the **handle's own sensor interface**; **do not** calculate its moments from a presumed wrist-to-handle offset when that offset does not cross the sensor. Include operator hand force position, handle geometry, mount stiffness and cables.
- [ ] Verify 24 V sensor supply, RS-485 transceiver electrical levels, grounding/termination, actual Modbus RTU or proprietary framing, baud rate, CRC, error detection, six-axis timestamped update rate and startup calibration.
- [ ] Bench-calibrate zero, bias drift, static known loads and known moment arms; quantify resolution, RMS noise, cross-axis sensitivity, temperature drift and saturation against manufacturer datasheet **and intended human forces**.
- [ ] Check end-to-end sensor age, missing samples, serial bursts, cable noise in the welding environment and fail-closed behavior on invalid/stale/saturated wrench. Log maximum latency and operator-perceived lag.
- [ ] Do not tune or approve powered admittance/hand-guiding behavior until a controlled, low-energy test setup exists. This document covers interface fitness and safety/measurement gates, **not** the admittance control equation.

## 10. H9–H10: motion, robot accuracy and factory environment

### H9 — limited motor and joint acceptance **only after H0–H8 pass**

1. Obtain authorization from the responsible engineer; secure the test stand, keep the robot workspace clear, establish travel limits and prove independent stop chain.
2. Start with drive power/communications commissioning **without joint motion**, then one unloaded or mechanically secured actuator at manufacturer-recommended low energy/speed.
3. Verify positive direction, mechanical zero, gear ratio, encoder unwrap/absolute-position recovery, target scaling, homing limits, brake release/engage and commanded-versus-measured following error. **Wrong sign must block movement immediately**.
4. Verify quick stop, protective stop, actual E-stop torque removal, power loss and restored power restart in a safe, supervised test. Never trigger hazardous fault tests on a freely moving robot.
5. Integrate additional axes gradually with fixture protection and collision-aware limits; verify all six motors track intended commands and watchdog faults cause the documented safe response.
6. Validate line/arc trajectories, path preview, singularity/limit rejection, interpolation and 500 Hz freshness under slow, supervised motion. A path that is mathematically valid still needs physical speed/force/stop checks.
7. Measure **repeatability** separately from **absolute accuracy**; use a calibrated external measurement method and documented pose/loading/temperature protocol. Compare against the graduation-project's defined requirements rather than a commercial reference robot datasheet.

### H10 — integration and worst-case robustness

- [ ] Repeat timing and CAN measurements with **all six nodes**, M4 external flash activity, HMI traffic and F/T sensor streaming concurrently.
- [ ] Test prolonged operation, MCU/drives/transceiver/flash heat, supply dips, EMI/noise/dust and (only in a controlled approved environment) welding equipment interference. Evaluate enclosure, shielding, filtering and wiring changes based on traces.
- [ ] Reboot, transient disconnect and reconnect, stale feedback, corrupted trajectory, flash busy, invalid safety input, loss of either core and watchdog events. Expected response: motion inhibited/controlled safe stop as defined by safety system; no automatic unsafe restart.
- [ ] Repeat reproducibility tests on the final hardware revision and final firmware **Release** and **Debug** builds if their timing/optimization differs.
- [ ] Retest critical areas after every firmware/clock/pin/timer/CAN filter or mechanical/safety wiring change. Archive failing traces as well as passes.

## 11. Metrics, evidence and acceptance — fill these before running

Do **not** invent numeric limits. For each item, the team must derive a specification from the manufacturer, machine risk assessment, controller performance requirement or approved benchmark. Suggested instrumentation and evidence:

| Metric | Instrument / log | Required acceptance definition |
| --- | --- | --- |
| MCU rail and boot stability | DMM/scope, reset reason | Min/max voltage/current, allowed reset rate, boot state |
| CM7/CM4 scheduling & stack | RTOS trace, stack high-water | Stack margin, CPU headroom, no overflow/watchdog reset |
| IPC delay and loss | GPIO timing, epoch/sequence counters | Max permissible delivery age and accepted reset policy |
| CAN bit rate and signal integrity | Oscilloscope + CAN analyzer | 1 Mbit/s measured, adequate electrical margins/no unexplained bit errors |
| CAN load & transmission completion | Timestamped CAN trace | Worst-case utilization, 2 ms sequencing, late-command policy |
| FDCAN TX/RX FIFO backlog | HAL diagnostic counters | No unexplained loss/overflow, bounded occupancy and wait |
| CAN retry/error behavior | Error counters, bus-off logs | Safe reaction time, no stale motion after deadline |
| Heartbeat/SDO/PDO/EMCY | Analyzer + controller diagnostics | Per-node validity, timeout, fault response and object match |
| 500 Hz schedule | Timer pulses, logic analyzer | Worst-case jitter/WCET/deadline, not only average rate |
| Trajectory refills and flash | Per-batch timestamps + CRC | No underrun for defined workload, durable commit/restart |
| Stop and drive disable | Safety hardware test records | Approved stop category/time/distance and no unexpected restart |
| F/T handle measurement | Known loads, timestamps, temperature | Force/torque errors/noise, measurement age and safe invalid-input response |
| Robot position repeatability | External calibrated metrology | Repeatability at specified poses/load/temperature, recorded uncertainty |
| Environmental robustness | Long trace, EMI/thermal tests | Approved enclosure/temperature and fault-rate criteria |

**Test dataset naming suggestion:** record **date, board revision/serial, drive firmware, transceiver and flash part, source commit SHA, compiled ELF SHA256, .ioc SHA256, test configuration, instrument model/calibration, start/end time, raw CSV/logic-analyzer trace and decision owner**. The HMI/oscilloscope/firmware timestamps must share a known reference or synchronization method; otherwise do not compare them as absolute timing.

## 12. Code/configuration change map — only if justified by evidence

| Hardware finding | Change location / preferred procedure | Mandatory regression |
| --- | --- | --- |
| Invalid PLL2Q/1 Mbps/sample point | CubeMX **Clock Configuration / FDCAN1** + regenerate .ioc/HAL | Rebuild both; remeasure actual CAN rate, CPU and timing |
| GPIO or CAN transceiver pin differs | CubeMX pinout + physical schematic and board adapter | Verify AF conflict, scope TXD/RXD/CAN_H/CAN_L |
| FIFO saturation or missed frames | **CM7/Board/Src/fdcan_classic_backend.c**, CANopen task schedule; CubeMX FIFO/filters | Full 1/3/6-node load test, bus-off/retry/overflow |
| Unbounded CAN receive poll | **CANComm/CANopen/canopen_master.c** | Bounded-loop functional regressions, missed-frame and WCET tests |
| Need interrupts | CubeMX NVIC/HAL notifications + verified FreeRTOS-safe ISR/task handoff | ISR priority legality, latency, stress, no starvation |
| Wrong drive object/firmware mapping | **ServoDrive/AvatarM/**, **CANComm/CANopen/**, or device-specific replacement port | CiA 301/402 device tests, BOOT, PDO/SDO identity checks |
| Encoder units/gear ratio/direction wrong | **ServoDrive/AvatarM/avatar_m_position.c** and per-axis installation data | Stationary/read-only verify, then one-axis motion check |
| 500 Hz misses/variable latency | CubeMX timebase and **StateMachine/supervisor_task.c**, motion orchestration | Measured GPIO/WCET/jitter + full task/flash contention |
| IPC stalls or stale status | **Intercore/**, **CM4/Core/Src/application_ipc_service.c**, MPU/linker startup | Two-core message sequence, reset, overflow and cache stress |
| Flash program/erase/CRC issues | CubeMX QSPI/OSPI + **CM4 production storage adapter**, never directly transplant Renode register code | Power cut, abort, rollback, stream/refill/load |
| F/T noise/mismatch | RS-485 board transport, sensor calibration, handle mechanical design | Bench load/error/age tests and safe missing-data behavior |
| Safety-input polarity/stop failure | PCB/safety circuit + **SupervisorIoServices** board port, state service callbacks | Independent safety acceptance **before motion** |
| External actuator power/temperature issues | Hardware power/thermal design first, then fault logging | Thermal and repeated stress tests under approved conditions |

**Coding rules:** Use STM32CubeMX for generated pins, clocks, FDCAN, QSPI, timer and NVIC; official STM32CubeH7 HAL/LL and CMSIS/FreeRTOS for infrastructure; reuse existing CANopen/CiA-402 logic only after device compliance checks. Keep code changes minimal, testable, and reviewable. Do not add fake successful safety callbacks or a simulated CAN backend to production images.

## 13. Reproducible pre-hardware regression gate

From WSL in the repository, this **source-only** set is available now (each command must report its own result). These checks are NOT replacements for physical test gates:

~~~bash
cd ~/freertos-pc-test
git switch sara/can-replacement
git pull --ff-only origin sara/can-replacement

bash Firmware/Tests/check_shared_layout.sh
bash Firmware/Tests/check_sram4_linkers.sh
bash Firmware/Tests/check_production_shared_objects.sh
bash Firmware/Tests/check_intercore_startup_compile.sh
bash Firmware/Tests/check_intercore_notifications.sh
bash Firmware/Tests/check_m4_application_ipc.sh
bash Firmware/Tests/check_m7_application_integration.sh
bash Firmware/Tests/check_m7_application_staging.sh
bash Firmware/Tests/check_fdcan_hal_adapter_mock.sh
~~~

The **check_fdcan_board_disabled.sh** test applies **only while FDCAN is intentionally ungenerated**; once FDCAN is enabled in the saved .ioc it should be replaced by a real HAL-enabled build gate, **not made to pass by disabling the hardware again**. Continue independent STM32CubeIDE Clean → Build of both cores and verify ELF sections/symbols. For timing and CAN host regressions, use the existing **scripts/run_timing_benchmark.sh** and **scripts/run_full_can_silkit_simulation.sh** where applicable; inspect their flags and use the correct required SIL Kit environment. Host simulation does not validate actual transceiver, FDCAN or board scheduling.

## 14. Reusable field-test report template

Copy the following block into one issue/file **per test and per hardware revision**, attach its traces, and have a reviewer sign off:

~~~text
Test ID:
Gate:
Date / location / operator / reviewer:
Firmware branch + commit SHA:
CM7 ELF SHA256 / CM4 ELF SHA256:
CubeMX .ioc SHA256 / HAL package:
Board revision / MCU order code / board serial:
CAN transceiver / nominal bit rate / termination:
Drive models/EDS/firmware/node IDs:
Flash part/revision / FT part/revision (if applicable):
Fixture and power/safety configuration:
Instrument model + calibration / data logging clock:
Preconditions (especially drives inhibited?):
Procedure / test duration / repetitions:
Measured data (min / median / p99 / MAX / counters):
Required acceptance limit and source:
Actual result: PASS / FAIL / BLOCKED / NOT RUN
Failure symptom + reproduction steps:
Trace/log/scope/CSV location:
Hardware changes:
Firmware/config changes (Git commit):
Regression results:
Residual risks / next permitted gate:
Approved by / date:
~~~

## 15. Final sign-off checklist

- [ ] Exact hardware, wiring, schematic, datasheets and installation firmware are frozen/reviewed.
- [ ] Both real cores boot reliably; voltage/clock/MPU/SRAM4/IPC pass under load.
- [ ] Active CubeMX FDCAN HAL built and measured; transceiver, filters and error handling validated.
- [ ] Real six-node CANopen object mapping, direction/units and CiA-402 status verified.
- [ ] 500 Hz worst-case timing, CAN utilization and stale-data response measured on physical configuration.
- [ ] Flash commit/rollback, M4 producer, cancellation and low-water/refill tests physically verified.
- [ ] Independent hardware stop/STO/brakes and restart prevention validated and approved.
- [ ] F/T handle, HMI, GPIO signals and sensor age/error responses validated.
- [ ] Supervised limited motion and six-axis path/limit/fault behavior accepted.
- [ ] Environmental, long-duration, restart and post-change regression suite passed.
- [ ] All failed tests resolved or formally blocked from deployment; no open unmitigated hazardous condition.
- [ ] Sign-off packet includes actual test traces, board/firmware hashes, acceptance sources and reviewer.

## 16. Authoritative engineering references

**ST platform / hardware:**
- [STM32H745/755 documentation hub: RM0399, DS12923/DS12919, ES0445](https://www.st.com/en/microcontrollers-microprocessors/stm32h745-755/documentation.html)
- [RM0399 reference manual — FDCAN, RCC/MPU/HSEM and memory architecture](https://www.st.com/resource/en/reference_manual/dm00176879-stm32h745755-and-stm32h747757-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [AN5617 — introduction to inter-processor communication on STM32H745/755 and STM32H747/757](https://www.st.com/resource/en/application_note/an5617-introduction-to-interprocessor-communications-for-stm32h745755-and-stm32h747757-mcus-stmicroelectronics.pdf)
- [Official STM32CubeH7 firmware/examples](https://github.com/STMicroelectronics/STM32CubeH7)
- [STM32 HAL1 FDCAN API reference source](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/blob/v1.11.4/Inc/stm32h7xx_hal_fdcan.h) — representative HAL1 revision; **the installed STM32Cube FW_H7 V1.13.0 version** remains the build authority.
- [FreeRTOS documented dual-core STM32H745 example](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/02-Supported-demos)

**CAN/CANopen:**
- [CAN in Automation: CiA 301/CANopen communication services](https://www.can-cia.org/can-knowledge/canopen)
- [CAN in Automation: CiA 402 drive profile](https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control)
- [CiA heartbeat / boot-up / error-control concepts](https://www.can-cia.org/can-knowledge/error-control-protocols)
- [CiA SYNC mechanism](https://www.can-cia.org/can-knowledge/special-function-protocols)
- [CANopenNode project (mature open implementation to benchmark against)](https://github.com/CANopenNode/CANopenNode)

**Machine safety:** Use the *applicable current editions* of **ISO 10218 (industrial robot safety)**, **ISO 13849 (safety-related control systems)**, **IEC 60204-1 (electrical equipment of machines)** and **IEC 61800-5-2 (functional safety of drives)** as determined by the machine safety review. These are **not** compliance certificates for the graduation project. Welding/industrial electrical safety and the exact stop architecture must be reviewed with responsible professionals.

---

**Status on 2026-10-14:** Pre-hardware verification plan prepared. **All H0–H10 physical gates remain NOT RUN.** No autonomous or hand-guided robot movement, safety certification or 500 Hz physical control has been demonstrated. 
