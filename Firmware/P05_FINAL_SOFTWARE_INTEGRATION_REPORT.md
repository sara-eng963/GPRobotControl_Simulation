# P05 Final Software Integration — independently executed in WSL

Disposition: **P05 ACCEPTED (software simulation only)**. Execution date:
2026-10-29, Africa/Cairo. No physical commissioning is claimed.

## Git provenance and preservation

- Original P05/P04 baseline: `ba5ab122868bd50a4626c568574be106dfce98fe`.
- Actual local starting HEAD and independently fetched remote starting HEAD:
  `345d5602358024e1ce05522f99aebab1db29b3b6`, on
  `sara/can-replacement`. Contrary to the supplied environment description,
  the earlier P05 candidate and scoped CMake warning fix were already committed;
  no tracked local modifications were present. Only the correction ZIP was untracked.
- The original installer refused the HEAD guard without editing. All five
  guarded candidate SHA-256 values matched exactly; all four production files
  patched by the package were byte-identical to `ba5ab12`; incremental
  `git apply --check --whitespace=error` passed. An external audited installer
  copy changed only BASE to the verified equivalent committed candidate.
  Its source-hash, branch, collision and patch guards were retained and executed.
  The original package was kept intact; the archival patch was never applied.
- A complete pre-edit source backup, empty initial tracked/index diffs,
  initial status/heads, correction installer logs and starting-history Git bundle
  are retained outside the repository.
- The final implementation descends from the remote partial P05 commit.
  Historical tracked review archives remain untouched; the new review ZIP,
  patches, builds and raw evidence are excluded from the new commit.
  No force push, reset, clean, unrelated branch or PR is part of this workflow.

## Executed integration boundary

`Simulation/P05/p05_production_supervisor.c` includes the **actual production
CM7 supervisor_task.c** through the existing P04 production-source fixture.
It does not implement a substitute Supervisor. CMake links the existing
production CANopen coordinator, real JointDrivePort and production state units
into the same executable as the existing six-drive virtual CAN backend.

The host seam supplies injected board/safety/CM4 observations and deterministic
clock releases. It authorizes a distinct host commissioning session using the
real release/press restart policy. It executes the real task entry, initial
timing assessment, pre-dispatch evidence/feedback checks, permission update,
dispatch-adjacent timing recheck and production PathExecution dispatch.
The P04-style scheduling seam exits at the second synthetic timer wait and
re-enters the task with the same context/master on the next release.
It rebinds the same host task identity at entry. This is source execution,
not a running physical FreeRTOS scheduler or proof of dual-core timing.

Every replay sample comes from the existing CM4 playback worker and trajectory
consumer, bound to the reloaded QSPI backend. Production PathExecution consumes
that sample and issues six ordered RPDO4 frames followed by SYNC through the
same production coordinator. The virtual drives update only at SYNC.
Exactly **1,312** validated, committed, reloaded and CRC-checked samples were
replayed. The simulator checks all six target values against flash-submitted
values and feedback; the gate independently checks trace length/order and
metadata sample counts.

The isolated host teaching demonstration retains its synthetic F/T decoder and
host-only guidance authority. Neither this host harness nor its authorization
providers are included in either ARM image. Production hand-guiding remains
observation-only; production Supervisor safety source and P01–P04 protections
were not changed.

## QSPI evidence and limits

The demonstration runs the existing `w25q_nor_model.c`,
`w25q512jv_flash.c` and `qspi_nor_validated_storage.c`, with the real
validation/storage/playback services. The recorder captures validator submissions
only for comparison; it does not supply successful playback samples.

The run commits the artifact through the actual W25Q bus command path, power
cycles the model, creates a new storage runtime, discovers the committed slot,
checks exact identity/metadata, reads every sample through the QSPI driver,
recomputes sample CRC and replays the same stream. Corruption of one committed
payload byte then prevents a new playback epoch.

- Sample count: **1,312**.
- Artifact CRC: **2767309324**.
- Sample-data CRC: **2369567794**.
- `same_artifact`, `same_samples`, `crc_verified` and
  `read_from_simulated_flash`: executable assertions passed.

This P05 path uses the **software NOR/QSPI model**. Separately, all five existing
Renode firmware/MMIO QSPI regressions ran successfully, with their runner
self-test. The P05 executable itself was not executed inside Renode, and there
was no physical flash access or endurance measurement.

## Same-process Supervisor fault verification

Each case first sends a verified positive-control six-RPDO/SYNC transaction in
a distinct explicitly commissioned host session. After injection, it requires
zero additional RPDO target frames and zero SYNCs, a production fault and an
inhibited/unauthorized real coordinator. Restored feedback and board/peer
observations plus RESET/RESUME attempts must not restart that session.
A direct coordinator target-send attempt must also fail.

| Injected case | Result |
|---|---|
| Revoked motor authorization | PASS; production E0B1 containment |
| Missing drive-feedback observation in actual coordinator cache | PASS |
| TPDO loss with healthy cadence and node heartbeats | PASS; production 20 ms cyclic-feedback watchdog |
| CM4 liveness expiration | PASS; production E018 containment |
| Deadline crossed during input acquisition | PASS; dispatch-adjacent E004 veto |
| Queued latched fault | PASS |
| CAN transmission failure | PASS |
| Recovery/automatic restart attempt after each of the seven cases | 7/7 denied |

The executable emits `production_supervisor_trace.json` only after these
assertions. It records real fault codes, path errors, permission/inhibit state
and denied frame counts. The strengthened final gate requires the same-process
and task-entry evidence plus every negative scenario. Separate production-source
unit-test success cannot satisfy this gate.

Two additional mutations recompile actual production Supervisor source into the
live CANopen process. Removing the dispatch-adjacent timing veto or CM4-expiry
veto compiles successfully but fails the executed P05 behavioral assertions.
Neither compile errors nor abnormal infrastructure failures count as kills.

## USART2 / RS-485 and DMA ownership

PD4 DE / PD5 TX / PD6 RX and the existing UART/DMA configuration remain intact.
The optional CM7 background service is called from the actual application
startup and schedules bounded work every 1 ms at low priority. Missing
`board_supply_ft_serial_config` or an uncommissioned decoder prevents task/DMA
startup. No vendor sensor protocol was invented.

The bridge latches UART errors/overrun and requires explicit restart. It copies
a completed 256-byte DMA half into a CPU-owned snapshot, then rechecks DMA
completion/error counters **before** publishing bytes to the serial decoder.
An injected next-half completion during cache acquisition verifies that raced
bytes are rejected before decoding. Existing error, starvation, stale-data and
explicit restart assertions also pass.

The director package originally placed the buffer at D2 start, overlapping
CM4's RAM alias. Both CM7 linker variants now reserve exactly 512 bytes at
**0x30047E00–0x30047FFF** in a dedicated DMA region. Both CM4 linker variants
exclude the corresponding **0x10047E00–0x10047FFF** alias from general RAM.
Debug and Release ELF checks verify the 32-byte-aligned 512-byte NOLOAD
section, linked service/callback symbols, every CM4 allocated section and CM4
stack ceiling. CM4 stack top is **0x10047E00**, outside the DMA buffer.
The four Debug/Release builds use the flash-boot linker scripts; the RAM-debug
variants have matching source reservations, with no separate RAM-boot execution claim.

## Build/runtime repairs

1. Guard baseline mismatch: independently verified source equivalence as above;
   changed only the external audited installer's HEAD guard.
2. Director QSPI evidence writer had an unterminated C string: corrected the
   string literal without changing storage assertions.
3. Production-source runner regex excluded digits and rejected the actually
   passing `cm4_lost_and_recovered` line: accepted digits in scenario identifiers.
4. Initial live harness include lookup selected desktop Supervisor definitions:
   selected the production TU explicitly and used production include paths.
5. Linker retained unused RTOS task/queue-creation entry points from the fixture:
   used the same function/data-section and linker garbage-collection approach
   as the existing P04 source compiler; no replacement Supervisor was added.
6. Production coordinator correctly denied initial host sends without runtime
   ownership: bound the isolated host owner and explicitly handed ownership to
   the actual production task. Its permission/owner guards remain enforced.
7. An initial fault injector referenced a nonexistent AvatarMDrive EMCY field:
   replaced it with the explicit missing-observation cache stimulus described
   above and removed an unused old host-gate helper.
8. Fixed the DMA copy/ownership race and the cross-core D2 alias collision
   described above.
9. Acceptance formerly required HEAD to equal the original baseline, making
   committed P05 unverifiable: it still requires the authorized branch and now
   verifies that the original baseline is an ancestor.

The previously committed legacy-only CMake
`state_path_validation.c` pedantic-warning exception is preserved. New host
code retains strict warnings; the production fixture's legacy enum warning is
scoped to its source.

## Independent verification results

| Verification | Independently observed result |
|---|---|
| Complete corrected acceptance runner | **19/19 PASS**, no skipped/NOT RUN steps |
| Full root CMake build and CTest | **60/60 PASS** |
| Live P05 QSPI + production Supervisor chain and strengthened evidence gate | **PASS** |
| Actual UART DMA source callbacks, faults, restart and copy-boundary preemption | **PASS** |
| P05 serial source mutations | **3/3 compiled, behaviorally rejected** |
| P05 production Supervisor in-chain mutations | **2/2 compiled, behaviorally rejected** |
| P04 source mutations | **12/12 compiled, behaviorally rejected** |
| Additional P01–P04/CANopen/IPC/timer/configuration scripts | **14/14 PASS** |
| Renode QSPI MMIO regressions | **5/5 PASS**, plus runner self-test |
| CM7 Debug / Release ARM builds | **PASS / PASS**, 103 translation units each |
| CM4 Debug / Release ARM builds | **PASS / PASS**, 48 translation units each |
| Debug / Release firmware integration | **PASS / PASS** |
| Debug / Release DMA link and cross-core alias checks | **PASS / PASS** |
| Missing-component acceptance negative tests | **PASS** |
| git diff --check | **PASS** |

The earlier complete run also passed all 18 checks then present. After final
include-graph/alias fixes and adding the in-chain mutations to the runner, the
complete final 19-step runner was rerun successfully. Results above describe
executed binaries and assertions, not merely command exit codes.


Evidence, starting backup and machine-readable summaries are retained outside
the checkout at:
`/home/sara/p05-verification/p05-20261029-KFxjLb`.
The temporary execution audit remains at
`/tmp/p05-audit-20261029T002800Z`.
Raw logs/traces/build products are intentionally not committed.

Reproduce on the authorized branch, with the repository's WSL/Linux
dependencies (including the sibling FreeRTOS-Kernel, ARM GCC and Renode):

```bash
python3 Firmware/Tests/run_p05_acceptance.py --output /tmp/p05-new-evidence
```

Use a new external output directory. The runner records every command and wall
time and returns failure if any required result is not PASS. These host wall
times are not STM32 WCET measurements.

## Remaining limitations and phone handoff

No physical STM32 deadline/jitter measurements, motor stopping, CAN electrical
qualification, flash endurance, sensor accuracy or vendor F/T commissioning
were performed. Board-specific physical providers remain absent and fail
closed. Simulated authorization and injected liveness are not commissioning
evidence. Hardware timing, DMA/MPU/bus operation and physical safety require
separate controlled validation.

For a fresh Codex Cloud task, select this GitHub repository and
`sara/can-replacement`, fetch the latest branch, verify its HEAD, and read this
report first. Do not reapply either review patch. Start with a requested next
software task while preserving production motion denial and the separate
physical-commissioning boundary. The cloud environment needs the same build
dependencies; local `/tmp` or external WSL evidence is not automatically
available there.
