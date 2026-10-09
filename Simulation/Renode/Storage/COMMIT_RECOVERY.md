# A/B trajectory commit integrity and recovery

Batch 2 retains format version 2, the existing slot layout, CRCs, generation
selection, and external storage API. It changes when a slot may be published
and how an update discovers its target. All flash/storage access still requires
one serialized owner. No flash operation belongs in trajectory execution.

## Update and publication

1. `begin` scans both slot headers and verifies each candidate's payload CRC.
   It does this for every update, including after reboot without an explicit
   `load_committed`, and after an ambiguous earlier commit acknowledgement.
2. A transport failure in either scan blocks all erases. An invalid format or
   mismatched CRC is distinct from an I/O error. Reload can recover a separately
   valid slot, but an update requires a complete, successful scan of both slots.
3. The update selects the slot opposite the newest validated trajectory. With
   no valid slot it starts at A. It erases and reads back the complete 4 KiB
   header sector, requiring every byte to be FF before any payload is written.
   This prevents reuse of an old commit marker after an ignored/incomplete erase.
4. Payload pages are programmed using the existing batching, then CRC checked.
5. Header fields and CRC are programmed with the marker still erased. Readback
   compares the complete header byte for byte, including marker and padding.
6. The commit marker is programmed separately. A second complete header readback
   verifies it, and a final payload CRC catches corruption introduced after the
   first payload check. Only then are the new slot, metadata and verified flag
   published in the runtime object.

The previous active slot is never erased/programmed by the replacement. Reads
are blocked while `writing` is true. Failure does not publish the new slot;
callers must abort/reload before playback or another update.

The marker is the persistent commit point. A loss of power or a readback error
after its complete programming can leave a fully valid new trajectory despite
`commit` returning failure. Reload validates the header, marker, artifact CRC
and payload CRC before selecting that trajectory. An abort does not erase it;
the next `begin` rediscovers both slots before selecting an update target.

## Deterministic host evidence

`qspi_storage_power_loss_test` uses an independent device model with literal
opcodes, NOR AND programming, page wrapping, WREN/WEL, delayed operation
completion, and preserved bytes across reset of MCU/flash volatile state.
Model addresses cover a 32 KiB A/B test region; this is not new full-device or
physical timing evidence. Every recovered sample checks all six signed joints.

The sweep interrupts each addressed transaction before, during and after it.
Mutating operations also use first/middle/last-byte prefix persistence. Four
fixtures cover sole/both valid trajectories with either A or B active.

| Stage | Injected cuts |
|---|---:|
| Slot discovery | 546 |
| Header erase | 20 |
| Erase readback | 192 |
| Payload erase | 40 |
| Payload program | 580 |
| Initial payload verification | 348 |
| Header program | 40 |
| Header verification | 24 |
| Marker program | 20 |
| Marker verification | 24 |
| Final payload verification | 348 |

These 2182 cuts plus four resets immediately after successful commit produce
**2186 power-cut/reboot cases**. Another **478 integrity cases** cover ignored
erases/writes, acknowledged truncated programs, silent corruption, verification
read errors, and late header/payload corruption at marker programming. Additional
tests cover discovery retry without reboot, lost acknowledgement followed by
abort/update, consecutive updates, corruption fallback, and both payloads bad.

Negative controls reject the Batch 1 backend (wrong slot erased without discovery)
and a backend that reads headers but ignores byte mismatches (invalid live commit).

Run the existing host script, which now includes both storage suites and uses
fresh temporary runtime/output directories:

```bash
./scripts/run_host_flash_tests.sh
ctest --test-dir <isolated-build> -R 'w25q512jv_verified|qspi_storage_power_loss' --output-on-failure
```

## Datasheet basis and limits

The supplied W25Q512JV June 25, 2019 Rev. B specifies page boundaries and partial
programs (§8.2.23, printed p.50), four-byte programming (§8.2.24, p.51), protected
erase behavior (§8.2.27, p.54), and four-byte erase (§8.2.28, p.55). Reset can
interrupt an operation and corrupt its target (§8.2.51, p.77). The model uses
typical program/erase latency from p.84 to exercise WIP; it does not simulate
the supply waveform, physical error distribution or electrical timing.

Scans and CRC passes are proportional to trajectory length: each update now
reads both candidates before erase and checks the new payload twice. Measure
validation latency/stack use on the board before enabling a physical backend.
This code does not protect against arbitrary corruption after the last check,
CRC collisions, concurrent owners, or a change of configured layout. Generation
rollover, format portability, physical power interruption, MCU-only reset with
flash still busy, and hardware validation remain later work.

Two Batch 1 review issues remain open: the Supervisor's potentially blocking
worker-quiescence wait, and unverified actuator stopping on storage fault. The
simulator disables wire feed and enters FAULT, but its safe-output callback does
not issue drive hold/stop and the Supervisor continues heartbeat service. Batch 2
introduces no drive safety policy and changes no CANopen or prefetch behavior.
