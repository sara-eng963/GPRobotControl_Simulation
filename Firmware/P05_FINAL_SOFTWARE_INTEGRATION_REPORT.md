# P05 — Final Laptop-Only Integration Candidate and Verification Evidence

**Disposition: INCOMPLETE / UNVERIFIED.** Host transport tests have run. The whole integrated demonstration, ARM source graph and the QSPI-backed end-to-end run have **not** been executed. No hardware acceptance is claimed.

## Exact provenance

- Canonical repository: `sara-eng963/GPRobotControl_Simulation`.
- Authorized branch: `sara/can-replacement`.
- Exact committed baseline: `ba5ab122868bd50a4626c568574be106dfce98fe` (checked through connected GitHub, source read from that ref).
- The user's WSL checkout `~/freertos-pc-test` is not mounted here; GitHub DNS is unavailable to local shell. Source files were inspected by connected GitHub at the exact baseline. No user Git files were changed. No commits, pushes, merges, resets, deletes or branches.

## Reuse and source ownership

| Component | Production or host owner | P05 change |
|---|---|---|
| STM32 USART2 PD4(DE), PD5(TX), PD6(RX), DMA1 stream0 | CM7 HAL main/MSP/interrupts | Original CubeMX files untouched; new `ft_uart2_dma_bridge.[ch]` implements opt-in circular DMA half handling, task copying and cache maintenance. **Not started by an uncommissioned board** |
| CM7 `.project` | CubeIDE-linked project | Adds only portable `ft_serial_transport.c` to CM7 graph. CM4 HMI and IPC project unchanged |
| `HandGuiding/ft_serial_transport.[ch]` | Portable task/ISR handoff | C11 SPSC ring, 512 entries, whole-batch overflow rejection, error latch, clock/sequence/freshness validation, bounded task work increased from 128 to 256 bytes/call |
| `HandGuiding/Tests/p05_mock_ft_protocol.h` | Host-only | 34-byte test protocol: A5 5A + u32 LE sequence + six signed i32 (synthetic milli-units) + IEEE reflected CRC32 LE. **NOT A SENSOR VENDOR SPECIFICATION.** |
| Existing wrench frame, Tustin admittance, Jacobian mapper, FK | ControlCore/HandGuiding | Reused without editing the algorithms |
| Existing CANopen master, NMT/SDO, RPDO4/TPDO4/SYNC, CiA402 | CM7 production; host library | Reused; `Simulation/P05/p05_virtual_system.c` supplies a **host-only six-node backend** that batches six RPDO4 and releases them on SYNC, then emits TPDO4/heartbeat. Host gate is not firmware authorization |
| `state_path_validation`, teaching types, intercore validation bridge, committed-artifact verification, trajectory ring | CM7/CM4 code reused in host | Host harness records two FK-derived teach points, runs real bounded validator/CM4 queue and verifies a readback epoch. Host backing array is a **simulation fixture, not W25Q512JV NOR** |
| `Simulation/Renode/Storage/w25q512jv_flash.c`, `qspi_nor_validated_storage.c`, `stm32h7_w25q_bus.c` | Existing QSPI / Renode | No modifications, no duplicate emulator; **not connected to the executable P05 host artifact fixture** |
| `Firmware/STM32H745/CM4/Core/Src/application_validation_service.c` | CM4 physical flash provider | Unchanged, remains fail-closed with missing board providers |
| `supervisor_task.c` P01–P04 safety/timing and CM4 HMI IPC | CM7 | Unchanged. Host gate cannot energize actual motors. **Production Supervisor not invoked by this host chain** |
| FDCAN2 / HMI transport, Ethernet, USB, SDMMC, torch, power-good | Outside P05 | Unchanged and excluded |

### USART2 quantitative receive budget

At a *configured*, unverified 1,000,000 baud 8N1: 10 bits per octet => 100,000 bytes/s, 100 bytes/1ms, 200 bytes/2ms. A budget of 128 bytes in an independently scheduled 1ms receive task provides a nominal 28% service margin; 256 bytes per 2ms also provides 28%. The initial candidate's 128 bytes per 2ms has a 36% deficit and cannot sustain a full-speed continuous byte stream.

The 512-byte DMA circular buffer has two 256-byte halves (each 2.56ms at this rate), and the 512-entry SPSC ring holds 5.12ms of traffic in the worst-case byte-rate model. DMA half/full callbacks publish a counter and a CM7-clock timestamp; the background task handles **at most one 256-byte half per invocation**, in 4x64-byte copies, and drains at most 128 parser bytes per service. A backlog exceeding one completed half is discarded and latched as a UART error. DMA buffers are 32-byte aligned with line-isolated invalidation. Board memory placement must ensure DMA1-addressable **non-DTCM** SRAM, and must be verified in an ARM ELF before deployment. No complete throughput or ISR WCET has been measured on STM32. A stalled task can lose data, correctly becoming a fail-closed transport fault.

`ft_uart2_dma_start` requires an explicit commissioned caller, a monotonic timestamp callback and existing UART2/DMA HAL initialization. Because the vendor frame specification is unknown, P05 does not start a physical sensor decoder and does not authorize physical guidance. A real low-priority CM7 task call site, restart policy and same-clock timestamp binding are still pending firmware integration; those are **not** silently reported PASS.

### Host demonstration execution

```bash
bash scripts/run_p05_virtual_end_to_end.sh
# or specify a NEW external directory:
bash scripts/run_p05_virtual_end_to_end.sh /tmp/p05_evidence_01
```

This builds the `p05_virtual_system` target from the actual repository's `HandGuiding`, `ControlCore`, CANopen, path-validation and intercore libraries. The following chain is implemented as source, but **has not been executed against the complete repository in this authoring session**:

1. Generate 34-byte synthetic F/T packets, accept via `ft_serial_isr_receive`, decode with test-only mock and require fresh sequence/time.
2. Execute actual `hand_guiding_pipeline_arm/step`, wrench processing, existing Tustin admittance and Cartesian-to-joint mapping; validate finite six-axis references.
3. Convert joint radians with the existing AVATAR encoder scales; send through existing `canopen_master_send_target_cycle`. The virtual backend enforces six RPDO4 frames followed by one SYNC and emits six TPDO4 samples and heartbeat frames; `canopen_master_poll` checks actual decoded feedback.
4. Use the measured virtual joint positions for two actual taught points and FK-derived Cartesian poses; calculate the repository's draft CRC and run `state_path_validation_enter/step` using the existing control/IK validation implementation.
5. Submit the validator output via existing `validation_m7_storage` queue to `validation_m4_step`. A host-only artifact array stands in for absent physical storage; it has no flash semantics and is not represented as a W25Q commit.
6. Request the exact identity via `playback_m7_request` and use existing full-sample CRC verification and ring/epoch streaming; `trajectory_m7_take` returns the exact committed sample; send its targets via the same CANopen master to virtual nodes, record readback comparison.
7. Negative cases: change one stored sample to require CRC-rejected playback; suppress TPDO4 and revoke simulation permission; fail a CAN TX then reject recovered traffic without a fresh authorization; verify F/T staleness.

Evidence outputs into the selected external directory: `joint_trace.csv`, `cartesian_trace.csv`, `trajectory_compare.csv`, `faults.csv`, `integration.json`, `summary.json`, configure/build/runtime logs. `summary.json` deliberately sets `full_p05_acceptance=false` even when the host subsystem returns PASS, because QSPI and production Supervisor integration remain unexecuted. CSV is MATLAB-compatible; MATLAB was not run.

### Executed verification (this session only)

| Check | Status | Evidence |
|---|---|---|
| New `test_ft_serial_transport.c` (9 groups) built with native GCC against known `FtSensorSample` fixture | **PASS** | Corruption, deletion/truncation, sequencing, stale sample, link timeout, ISR overflow, UART fault, 32-bit wrap, 204 bytes/2ms drain |
| Exact baseline `HandGuiding/CMakeLists.txt` blob | **PASS** | SHA `9b0e06ce8ecb477901835a5bbe79e0067d39584f` matches GitHub source |
| Compiled source mutations against actual P05 serial source with a minimal FtSensorSample fixture | **PASS — 3/3 rejected** | Replay, stale-acceptance and overflow bypasses compiled then failed behavioral assertions |
| Serial-to-guidance test in complete real source tree | **NOT RUN** | Requires actual WSL checkout |
| New connected virtual end-to-end executable | **NOT RUN** | Source and runner supplied, no actual checkout to compile against here |
| QSPI Renode JEDEC/write/erase/CRC/power-loss/streaming | **NOT RUN** this session | Existing separate scripts remain unmodified |
| CANopen / IPC / trajectory / P01–P04 regressions and negative mutations | **NOT RUN** this session | Original tests retained |
| Four ARM builds and both firmware integration checks | **NOT RUN** | `arm-none-eabi-gcc` unavailable |
| Production USART2 task scheduling and DMA cache placement | **NOT RUN** | ARM build/hardware needed |
| `git diff --check` on user WSL checkout | **NOT RUN** | Patch-level whitespace inspection available only here |
| Hardware timing, real sensor scaling, physical motor stop | **NOT TESTABLE** | No hardware and unknown vendor protocol |

**Separate user-reported P04 evidence** (56 CTests, 11 scenario groups, 12 compiled mutations, 4 ARM builds) is historical information, not new P05 test execution.

### Remaining acceptance blockers

1. Host executable still uses a test-only artifact backing array, not actual `QspiNorValidatedStorage` wired through Renode. A QSPI-backed *same-artifact* write/readback/stream must be demonstrated; isolated existing NOR regressions do not close this requirement.
2. Real production `supervisor_task_entry` authorization, CM4 liveness, pre-dispatch timing faults, and `state_path_execution` are not invoked by the host-only demonstration. Existing tests must be run separately and integrated P05 coverage added; do not count the host SimGate as P01–P04 validation.
3. STM32 DMA reception still lacks a scheduled, live CM7 background service and a vendor-specification-backed physical protocol decoder. The new HAL bridge is opt-in and uncommissioned.
4. Full-repo compilation/CTest, Renode, ARM build and external independent WSL verification have not been executed. The new host harness may require corrections against actual source integration and its outcome is unknown.
5. No commissioning of real sensors, board safety verifier, motors or physical timing has been performed.

**Final disposition: INCOMPLETE / UNVERIFIED.** The updated candidate is one consolidated reviewable patch and a runnable host integration attempt, not a completed P05 final acceptance.
