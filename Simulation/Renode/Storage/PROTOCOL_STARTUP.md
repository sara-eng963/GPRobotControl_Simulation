# Batch 3: W25Q512JV startup, protocol and degraded recovery

This batch retains dedicated four-byte read/program/erase commands (13h/12h/21h),
the version-2 A/B format, existing CANopen execution and SRAM prefetch architecture.

## Confirmed fixes and startup contract

The public read/program/erase APIs reject null, zeroed, unidentified objects and
missing bus callbacks before dereferencing a timer or using transport. A rejected
reinitialization clears `identified`, including invalid bus arguments. C callers
must still provide valid object addresses/function pointers and valid payload
buffers; this does not validate dangling pointers or arbitrary uninitialized RAM.

Before calling initialization, the board must establish stable VCC at or above
VCC(min), safe /CS, configured SPI pins/clocks and a working monotonic timer.
Initialization waits a conservative 5 ms from entry: this covers tVSL (20 us)
and tPUW (up to 5 ms), then polls WIP for up to 500 ms before reading JEDEC.
Identity is published only after readiness and exact EF 40 20 identification.
An MCU-only reset therefore waits for an ongoing sector erase instead of issuing
an identification instruction that the busy flash ignores.

The 500 ms bound covers the supported 4 KiB erase (400 ms maximum) and page
program (3.5 ms maximum). Other erase instructions are not part of this design.
Frozen clocks have a secondary two-million-iteration guard; backward clocks and
ready responses received after the deadline fail. The iteration guard establishes
termination, not an elapsed-time guarantee for a failed timer. Bus callbacks must
also be bounded; the flash driver cannot interrupt a blocked callback.

Initialization never automatically resets the device, writes status registers,
or unlocks protected regions. Reset during internal program/erase can corrupt
data, so tests simulate explicit external device resets rather than adding an
automatic driver recovery command. Safe retry requires restoring transport/timer
health and reinitializing, with persistent slot validation before playback.

## Program/erase semantics and protection

Success now requires WIP and WEL cleared plus readback of the requested bytes.
Program verifies at most 256 bytes; erase verifies all 4096 bytes in 256-byte
reads. The fixed buffers use stack storage, not dynamic embedded allocation.
Attempted 0->1 programming without erase fails if the resulting bytes differ.
Ignored/protected writes cannot supply a newly committed header or marker.

Verification proves the resulting contents, not that the chip actually performed
an operation. An ignored operation whose desired bytes already match can be
indistinguishable if WEL also clears. This is safe for the existing A/B commit
protocol: the complete header/marker and payload still must match before a new
slot is published. A failed operation may already have changed bytes; callers
must retain the existing abort/reload discipline.

Readback adds validation traffic and latency. All these operations remain outside
the time-critical trajectory callback. This batch does not establish throughput
or scheduling deadlines and does not implement a QSPI timing model.

## Degraded reload policy: retained, now observable

`qspi_nor_validated_storage_load_committed()` still returns true when one slot's
header, marker, artifact CRC and payload CRC are fully valid, even if scanning the
other slot fails due to I/O. Changing that acceptance policy requires separate
review. The unread slot may contain a newer generation; degraded success must
not be described as proof that the newest trajectory was recovered.

The runtime-only `scan_io_error_mask` reports the last scan: bit 0 is A, bit 1
is B. A nonzero mask with successful load is degraded recovery; no valid candidate
with a nonzero mask is recovery failure. A successful later scan clears it. It
does not change persistent bytes, format version or the loader's boolean result.
Callers should surface the mask in diagnostics and decide any stricter execution
policy explicitly. No HMI or motion-policy change is made here.

`begin()` continues to require a complete scan of both slots and returns false
before any erase if either has I/O failure, including after a successful degraded
load. Tests cover A/B header and payload read failures, diagnostic masks, preserved
sample values, blocked updates and recovery on a later healthy scan.

## Independent model reuse and evidence

The Batch 2 model was extracted into `Simulation/Tests/w25q_nor_model.{c,h}`.
Its literal opcodes, NOR AND programming, page wrap and delayed completion are
shared by storage power-loss and new protocol tests. The storage suite keeps its
32 KiB fixture; protocol tests provide a 64 MiB host image for address boundaries.
Existing legacy host tests remain intact. No second new simulator was introduced.

Protocol tests cover power-up inhibit, WREN/WEL/auto-clear, busy rejection,
stuck-WIP, stopped/backward clocks, timeout and late completion, failed callbacks,
wrong JEDEC, rejected reinitialization, MCU reset during program/erase, explicit
device reset and its 30 us recovery, protected regions, partial/full pages, NOR
0->1 attempts, raw page wrapping, crossing 16 MiB and address 03FFFFFFh. Negative
controls reject JEDEC-before-ready and a driver using the wrong read opcode.

Driver readback expands the retained fault sweep to 3146 power-cut/reboot cases
and 638 integrity-fault cases. The Batch 2 counts in COMMIT_RECOVERY.md describe
the earlier checkpoint set. Protection tests additionally check that no unlock
command is issued and incomplete/protected writes cannot publish a new slot.

The host model rejects three-byte design-incompatible commands intentionally;
the physical W25Q512JV does support them in its address modes. Busy rejection
returns false to flag a misuse of the mock bus; physical SPI has no command-error
acknowledgement. Abstract protected ranges model denied operations, not the full
BP/TB/CMP/WPS register map, electrical /WP effects or individual lock reset rules.
Raw page wrapping is tested for up to 256 bytes; oversized program-buffer behavior
is outside this model. Partial persistence patterns are representative, not every
possible physical brownout pattern. Device reset is a deterministic test event,
not proof of supply/reset-pin behavior.

Installed Renode's GenericSpiFlash was previously probed to support four-byte
transfers and EF 40 20, but permits 0->1 writes, lacks page wrap, completes writes
immediately and does not implement 66h/99h reset. These host protocol cases must
not be presented as Renode or STM32 hardware results. Freestanding firmware build
checks establish compatibility only. Physical voltage/timing/FIFO validation,
per-read integrity after verification, and dual-core ownership remain open.

The potentially blocking Supervisor quiescence wait and the absence of verified
actuator stopping after storage fault remain documented in PREFETCH_README.md.
No drive safety policy or flash-backed CANopen integration is added in this batch.

## Supplied datasheet references

W25Q512JV, June 25, 2019 Rev. B, printed pages (PDF viewer page = printed + 1):
§6.1.4 p.12 dedicated addressing; §§7.1.1–2 p.14 busy/WEL; §8.1.1 p.24 identity;
§8.2.11 p.38 read; §§8.2.23–24 pp.50–51 program/page wrap/protection;
§§8.2.27–28 pp.54–55 erase; §8.2.51 p.77 reset; §9.3 p.79 power-up;
§§9.6–7 pp.83–84 clock limits and operation/reset timing. The 13h read clock limit
remains 50 MHz and must be established by board configuration and measurement.
