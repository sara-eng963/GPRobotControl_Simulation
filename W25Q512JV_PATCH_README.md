# W25Q512JV storage hardening / CAN-AVATAR integration patch

This archive overlays **only** `Simulation/Renode/Storage/` plus a new host test and scripts; it does **not** merge the old EtherCAT branch. Extract over the current `test/external-memory-can-avatar` checkout.

## Use

```bash
unzip -o w25q512jv_fixes.zip -d ~/freertos-pc-test
cd ~/freertos-pc-test
./scripts/run_host_flash_tests.sh
python3 scripts/upgrade_renode_tests.py
```

The host test is a stateful flash-device model. It simulates WEL/WIP, program/erase latencies, page boundaries, NOR one-way bit programming, 64 MiB addressing and corruption. It is **not physical chip verification**.

## Code changes

- `w25q512jv_flash.{c,h}`: dedicated 4-byte commands 0x13/0x12/0x21, JEDEC EF 40 20 check, WIP/WEL polling with a real monotonic clock, page/erase timeouts, CRC32.
- `qspi_nor_validated_storage.{c,h}`: two-slot power-fail-tolerant updates, page batching, metadata/artifact CRC, full-data readback CRC before commit/reload, ability to refill multiple samples, full 64 MiB bounds.
- `stm32h7_w25q_bus.{c,h}`: bare-metal QUADSPI register adapter for Renode and future MCU integration. Uses standard single-wire SPI command/data, not Quad I/O yet. QUADSPI GPIO and pin AF setup are board-dependent and are **not** implemented here. Real MCU builds must provide `uint64_t w25q_board_monotonic_us(void)`; this function must be a stable timebase. Renode builds use synthetic time explicitly (not suitable for timing claims).
- `upgrade_renode_tests.py`: migrates old Renode tests from fabricated CRCs to actual CRC calculations and adds new driver sources. Adjusts the generic model's JEDEC ID, but **does not make that model a Winbond-accurate simulation**.

## Limits and validation gates

1. Verify host test passes without errors.
2. Rebuild 7.4/7.5 Renode tests with `make clean && make`, then execute the respective `.resc` scripts. Four-byte opcodes may **not** be implemented in your Renode `SPI.GenericSpiFlash` model. If unsupported, do not treat simulator test failures as physical chip failures. An expanded SPI flash model is required for those scenarios.
3. Existing 7.1/7.2/7.3 bare-metal QSPI tests still use 24-bit opcodes; do **not** use them to claim full 64-MiB compatibility.
4. Flash data can be read with 32-byte chunks through this conservative adapter; FIFO behavior and WIP status timings need physical STM32 validation. It does not establish 2-ms worst-case execution timing.
5. No hardware has been accessed from this environment. Never actuate a real robot using an unvalidated flash trajectory. Keep CAN drives disabled during first hardware tests.
6. Two-slot storage consumes approximately twice the logical trajectory size; plan capacity accordingly. This is simple slot alternation, not general wear leveling. Preserve board-specific low-level pin, clocks, DMA, cache/MPU and concurrency responsibilities.
7. `StorageHeader` was intentionally upgraded to format version 2. Version-1 files/flash artifacts are not supported by the new loader. Begin with blank/test flash, and migrate old trajectory data separately if necessary.

## Sequence for first silicon test

- Check 3.3V supply and soldering, MCU/QSPI pin mux, /CS pull-up, and pin-safe reset.
- Read JEDEC ID (`EF 40 20` for specified -IQ variant); abort on mismatch.
- Erase an isolated scratch sector, verify erased `0xFF`, write multiple pages with 4-byte commands, read back and compare every byte.
- Exercise addresses on both sides of `0x01000000`, including `0x03FFFFFF` boundary; ensure no wraparound or aliasing.
- Measure worst-case page program, sector erase, 6 KiB sequential read, current draw, and task CPU/interrupt load on the actual MCU.
- Prove fail-safe behavior for timeout, CRC mismatch, power interruption, and buffer underrun. Only then test on real drives in a safe setup.
