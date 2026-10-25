# W25Q512JV STM32H745 QUADSPI FIFO fix (Test 7.7)

The archive contains a **drop-in replacement** for `Simulation/Renode/Storage/stm32h7_w25q_bus.{c,h}`, and a standalone host register-mock test. It does **not** touch CAN, FreeRTOS, trajectory CRC/headers, old STM32 application code, or your Renode .resc files. Do not merge to the robot's control path until physical verification.

## Changes (STM32 peripheral adapter)

- TX byte writes wait for QUADSPI `SR.FTF` before writing `QUADSPI_DR`. `CR.FTHRES=0` configures a 1-byte FIFO threshold. Unlike the old unbounded 256-byte burst, this cannot deliberately overflow the FIFO when the hardware reports it full.
- RX reads wait for `SR.FLEVEL != 0`, consuming data before waiting for `TCF`/`BUSY` to clear. Reading the DR only **after** TCF risks an already full RX FIFO.
- Each transaction has a 10,000 microsecond deadline, a loop-count guard (for stuck hardware clocks), checks `TEF` and `TOF`, and requests an abort on a transfer failure.
- Reads are limited to 256-byte hardware bursts with 32-bit W25Q commands and address increments (including beyond 16 MiB). Program pages remain <=256 bytes and cannot cross physical page boundaries.
- Uses a conservative QUADSPI clock prescaler divisor 8 by default (`-DW25Q_QSPI_PRESCALER_DIV=<integer>`, 1..256). This divisor must be validated against the board's actual QUADSPI kernel clock and W25Q512JV voltage/clock rating.
- Retains a separate `W25Q_RENODE_VIRTUAL_TIME` compatibility path for **generic Renode functional tests only**. It intentionally does not use hardware FIFO flags or measure real latency. The prior 7.4/7.5 results are therefore not evidence of silicon FIFO behavior.

## Install (WSL repo root)

```bash
cd ~/freertos-pc-test
cp Simulation/Renode/Storage/stm32h7_w25q_bus.c Simulation/Renode/Storage/stm32h7_w25q_bus.c.bak
cp Simulation/Renode/Storage/stm32h7_w25q_bus.h Simulation/Renode/Storage/stm32h7_w25q_bus.h.bak
unzip -o w25q_stm32_fifo_fix.zip -d .
./scripts/run_w25q_fifo_mock_tests.sh
```

## Host mock expectations

- 256-byte page write deliberately fills the simulated 32-byte FIFO, but `FTF` backpressure prevents overflow.
- 600-byte multi-burst read correctly drains the FIFO and increments 32-bit addresses over 16 MiB.
- 4-byte address erase and invalid-request protection.
- Injected QUADSPI `TEF` and stalled FIFO cause explicit failures; simulated abort can recover.

The test uses a host mock peripheral and wall-clock-independent, synthetic microsecond increments. It does **not** validate hardware latency, electrical waveform, SPI clock polarity, physical FIFO microarchitecture, concurrency, or 500 Hz scheduling.

## Rebuild and rerun previously passing Renode tests

```bash
make -C Simulation/Renode/Firmware/qspi_storage_backend_test clean all && \
make -C Simulation/Renode/Firmware/qspi_streaming_test clean all
~/renode_portable/renode --console --disable-gui
```

At the Renode monitor:

```
include @Simulation/Renode/run_qspi_storage_backend_test.resc
include @Simulation/Renode/run_qspi_streaming_test.resc
```

Both scripts should report `status 0x50415353` and the same counters as before. If they regress, capture full output; do **not** claim the STM32 hardware FIFO logic failed based on the generic model.

## Hardware gates not yet implemented

- Configure/verify QUADSPI kernel source frequency and GPIO alternate-function/memory pins for your STM32H745 board; the adapter only enables the peripheral RCC clock.
- Implement and validate `uint64_t w25q_board_monotonic_us(void)` from a real microsecond-capable timer; check clock-domain coherence and timer wraparound.
- Confirm QUADSPI prescaler and CSHT against the actual board and W25Q datasheet. This driver currently uses **one-line** command/address/data, not 4-line Quad SPI.
- Verify transaction CRC/readback with an actual W25Q512JV above the 16MiB boundary, and measure the peripheral FIFO behavior and worst-case burst latency under interrupt load.
- Only one task at a time may use QSPI; serialize background reads and all offline program/erase operations. The bus is synchronous, NOT a background DMA reader, and is not safe to call from the 2ms control loop.
- The actual 500Hz refill task, CAN arbitration, cache/MPU, and brownout recovery remain separate integration and system tests.
