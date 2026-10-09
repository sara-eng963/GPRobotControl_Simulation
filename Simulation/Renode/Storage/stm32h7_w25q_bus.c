/* STM32H745 QUADSPI indirect-transfer adapter for W25Q512JV.
 * Hardware path: FIFO serviced through FTF (TX) and FLEVEL (RX).
 * Simulator path: retained for Renode functional compatibility only; its
 * synthetic clock, FIFO semantics and SPI timings are NOT hardware evidence.
 * The caller must serialize ALL QSPI operations and keep writes / erase away
 * from the time-critical robot control task.
 */
#include "stm32h7_w25q_bus.h"
#include <stdint.h>
#include <stddef.h>

#define QSPI_BASE UINT32_C(0x52005000)
#define QSPI_CR  (QSPI_BASE + 0x00U)
#define QSPI_DCR (QSPI_BASE + 0x04U)
#define QSPI_SR  (QSPI_BASE + 0x08U)
#define QSPI_FCR (QSPI_BASE + 0x0CU)
#define QSPI_DLR (QSPI_BASE + 0x10U)
#define QSPI_CCR (QSPI_BASE + 0x14U)
#define QSPI_AR  (QSPI_BASE + 0x18U)
#define QSPI_DR  (QSPI_BASE + 0x20U)
#define RCC_AHB3ENR UINT32_C(0x580244D4)
#define RCC_QSPIEN (1UL << 14)

/* STM32H7 QUADSPI register fields (RM0433 QUADSPI). */
#define CR_EN           (1UL << 0)
#define CR_ABORT        (1UL << 1)
#define CR_PRESCALER(n) ((uint32_t)((n) - 1U) << 24)
#define DCR_CSHT        (3UL << 8)   /* chip-select high: 4 kernel cycles */
#define DCR_FSIZE_64M   (25UL << 16) /* 2^(FSIZE+1) = 64MiB */
#define IMODE_1LINE     (1UL << 8)
#define ADMODE_1LINE    (1UL << 10)
#define ADSIZE_32BIT    (3UL << 12)
#define DMODE_1LINE     (1UL << 24)
#define INDIRECT_READ   (1UL << 26)
#define SR_TEF          (1UL << 0)
#define SR_TCF          (1UL << 1)
#define SR_FTF          (1UL << 2)
#define SR_TOF          (1UL << 4)
#define SR_BUSY         (1UL << 5)
#define SR_FLEVEL(s)    (((s) >> 8) & 0x3FUL)
#define SR_ERRORS       (SR_TEF | SR_TOF)
#define FCR_CLEAR_ALL   (SR_TEF | SR_TCF | SR_TOF)

/* Hardware never handles more than 256 bytes per indirect transfer. A burst
 * is bounded, allowing timeouts to be meaningful and keeping payload buffers
 * compatible with program pages. Renode still uses 32-byte read chunks. */
#define HARDWARE_BURST_BYTES 256U
#define RENODE_BURST_BYTES     32U

/* Timeout here is for a single QUADSPI *bus transaction*, not the flash's
 * internal erase/program WIP. The flash driver polls WIP separately. */
#ifndef W25Q_QSPI_TRANSACTION_TIMEOUT_US
#define W25Q_QSPI_TRANSACTION_TIMEOUT_US UINT64_C(10000)
#endif
#ifndef W25Q_QSPI_PRESCALER_DIV
#define W25Q_QSPI_PRESCALER_DIV 8U
#endif
#if W25Q_QSPI_PRESCALER_DIV < 1U || W25Q_QSPI_PRESCALER_DIV > 256U
#error "W25Q_QSPI_PRESCALER_DIV must be in 1..256"
#endif

/* This hook substitutes MMIO with a scripted peripheral for HOST TEST ONLY.
 * Do not set W25Q_TEST_MMIO in production firmware. */
#ifdef W25Q_TEST_MMIO
extern uint32_t w25_test_read32(uintptr_t address);
extern void w25_test_write32(uintptr_t address, uint32_t value);
extern uint8_t w25_test_read8(uintptr_t address);
extern void w25_test_write8(uintptr_t address, uint8_t value);
#define READ32(a)     w25_test_read32((a))
#define WRITE32(a,v)  w25_test_write32((a),(v))
#define READ8(a)      w25_test_read8((a))
#define WRITE8(a,v)   w25_test_write8((a),(v))
#else
#define READ32(a)     (*(volatile uint32_t *)(uintptr_t)(a))
#define WRITE32(a,v)  (*(volatile uint32_t *)(uintptr_t)(a) = (v))
/* QUADSPI_DR byte accesses must target the least-significant byte. */
#define READ8(a)      (*(volatile uint8_t *)(uintptr_t)(a))
#define WRITE8(a,v)   (*(volatile uint8_t *)(uintptr_t)(a) = (v))
#endif

#ifdef W25Q_RENODE_VIRTUAL_TIME
/* Synthetic clock is deliberately NOT a performance measurement. */
static uint64_t synthetic_clock;
static uint64_t chip_time(void *ctx) {
    (void)ctx;
    return ++synthetic_clock * UINT64_C(10);
}
#else
/* Board must supply e.g. a free-running hardware timer, monotonic in microseconds. */
extern uint64_t w25q_board_monotonic_us(void);
static uint64_t chip_time(void *ctx) {
    (void)ctx;
    return w25q_board_monotonic_us();
}
#endif

/* Secondary loop guard prevents a frozen/unconfigured timer causing a hang.
 * The guard is not a substitute for a verified hardware clock. */
#define QSPI_LOOP_GUARD UINT32_C(2000000)

typedef struct {
    uint64_t begin_us;
    uint32_t spins_left;
} TransferDeadline;

static TransferDeadline deadline_start(void) {
    TransferDeadline d;
    d.begin_us = chip_time(NULL);
    d.spins_left = QSPI_LOOP_GUARD;
    return d;
}

static bool deadline_valid(TransferDeadline *d) {
    const uint64_t now = chip_time(NULL);
    if (d->spins_left == 0U || now < d->begin_us ||
        now - d->begin_us >= W25Q_QSPI_TRANSACTION_TIMEOUT_US) return false;
    --d->spins_left;
    return true;
}

static bool wait_not_busy(TransferDeadline *d) {
    for (;;) {
        const uint32_t sr = READ32(QSPI_SR);
        if ((sr & SR_ERRORS) != 0U) return false;
        if ((sr & SR_BUSY) == 0U) return true;
        if (!deadline_valid(d)) return false;
    }
}

static bool wait_complete(TransferDeadline *d) {
    for (;;) {
        const uint32_t sr = READ32(QSPI_SR);
        if ((sr & SR_ERRORS) != 0U) return false;
        if ((sr & SR_TCF) != 0U) {
            /* Hardware RX drain occurs before checking TCF. Renode's generic
             * model uses the old completion-before-read behavior, and it is
             * not a hardware-accurate model of BUSY/FLEVEL. */
#ifndef W25Q_RENODE_VIRTUAL_TIME
            if (!wait_not_busy(d)) return false;
#endif
            WRITE32(QSPI_FCR, FCR_CLEAR_ALL);
            return true;
        }
        if (!deadline_valid(d)) return false;
    }
}

static void abort_on_error(void) {
    /* Request abort; never wait without a bound on an unhealthy peripheral. */
    WRITE32(QSPI_CR, READ32(QSPI_CR) | CR_ABORT);
    TransferDeadline d = deadline_start();
    (void)wait_not_busy(&d);
    WRITE32(QSPI_FCR, FCR_CLEAR_ALL);
}

static bool valid_request(uint8_t op, uint32_t addr,
                          const uint8_t *tx, size_t txlen,
                          const uint8_t *rx, size_t rxlen) {
    const bool addressed = op == W25_CMD_READ4 || op == W25_CMD_PROG4 ||
                           op == W25_CMD_ERASE4K4;
    if ((txlen != 0U && tx == NULL) || (rxlen != 0U && rx == NULL) ||
        (txlen != 0U && rxlen != 0U)) return false;
    if (addressed && (uint64_t)addr + (txlen ? txlen : (rxlen ? rxlen : 1U)) >
                     W25Q512JV_SIZE_BYTES) return false;
    if (op == W25_CMD_READ4) return txlen == 0U && rxlen != 0U;
    if (op == W25_CMD_PROG4) return txlen != 0U && rxlen == 0U &&
            txlen <= W25Q512JV_PAGE_BYTES &&
            (addr % W25Q512JV_PAGE_BYTES) + txlen <= W25Q512JV_PAGE_BYTES;
    if (op == W25_CMD_ERASE4K4) return txlen == 0U && rxlen == 0U &&
                                     addr % W25Q512JV_SECTOR_BYTES == 0U;
    if (op == W25_CMD_JEDEC) return txlen == 0U && rxlen == 3U;
    if (op == W25_CMD_RDSR1) return txlen == 0U && rxlen == 1U;
    if (op == W25_CMD_WREN) return txlen == 0U && rxlen == 0U;
    return false;
}

static bool one(void *ctx, uint8_t op, uint32_t addr,
                const uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen) {
    (void)ctx;
    const bool has_address = op == W25_CMD_READ4 || op == W25_CMD_PROG4 ||
                             op == W25_CMD_ERASE4K4;
    const bool data_read = rxlen != 0U;
    const bool data_write = txlen != 0U;
    if (!valid_request(op, addr, tx, txlen, rx, rxlen)) return false;
    TransferDeadline d = deadline_start();
    if (!wait_not_busy(&d)) return false;

    WRITE32(QSPI_FCR, FCR_CLEAR_ALL);
    if (data_read || data_write)
        WRITE32(QSPI_DLR, (uint32_t)(data_read ? rxlen : txlen) - 1U);
    uint32_t ccr = op | IMODE_1LINE;
    if (has_address) ccr |= ADMODE_1LINE | ADSIZE_32BIT;
    if (data_read || data_write) ccr |= DMODE_1LINE;
    if (data_read) ccr |= INDIRECT_READ;
    WRITE32(QSPI_CCR, ccr);
    if (has_address) WRITE32(QSPI_AR, addr);

#ifdef W25Q_RENODE_VIRTUAL_TIME
    /* Renode GenericSpiFlash currently passes the 7.4/7.5 functional tests
     * with direct accesses and 32-byte read fragments. Do not use this code
     * path to validate silicon FIFO behavior. */
    if (data_write) {
        for (size_t i = 0; i < txlen; ++i) WRITE8(QSPI_DR, tx[i]);
    }
    if (!wait_complete(&d)) { abort_on_error(); return false; }
    if (data_read) {
        for (size_t i = 0; i < rxlen; ++i) rx[i] = READ8(QSPI_DR);
    }
#else
    if (data_write) {
        for (size_t i = 0; i < txlen; ++i) {
            /* RM0433/ST HAL: FTF indicates room available in TX FIFO when
             * FTHRES is programmed as one byte (CR.FTHRES = 0). */
            for (;;) {
                const uint32_t sr = READ32(QSPI_SR);
                if ((sr & SR_ERRORS) != 0U ||
                    (sr & SR_TCF) != 0U || !deadline_valid(&d)) {
                    abort_on_error();
                    return false;
                }
                if ((sr & SR_FTF) != 0U) break;
            }
            WRITE8(QSPI_DR, tx[i]);
        }
    }
    if (data_read) {
        for (size_t i = 0; i < rxlen; ++i) {
            for (;;) {
                const uint32_t sr = READ32(QSPI_SR);
                if ((sr & SR_ERRORS) != 0U || !deadline_valid(&d)) {
                    abort_on_error();
                    return false;
                }
                if (SR_FLEVEL(sr) != 0U) break;
                if ((sr & SR_TCF) != 0U) { /* no bytes available after completion */
                    abort_on_error();
                    return false;
                }
            }
            rx[i] = READ8(QSPI_DR);
        }
    }
    if (!wait_complete(&d)) { abort_on_error(); return false; }
#endif
    return true;
}

static bool bus_command(void *ctx, uint8_t op, uint32_t addr,
                        const uint8_t *tx, size_t txlen,
                        uint8_t *rx, size_t rxlen) {
    if (!valid_request(op, addr, tx, txlen, rx, rxlen)) return false;
    /* Reads can span 64 MiB logically but are broken into bounded commands. */
    if (op == W25_CMD_READ4) {
#ifdef W25Q_RENODE_VIRTUAL_TIME
        const size_t max_burst = RENODE_BURST_BYTES;
#else
        const size_t max_burst = HARDWARE_BURST_BYTES;
#endif
        while (rxlen > 0U) {
            const size_t n = rxlen < max_burst ? rxlen : max_burst;
            if (!one(ctx, op, addr, NULL, 0U, rx, n)) return false;
            rxlen -= n;
            rx += n;
            addr += (uint32_t)n;
        }
        return true;
    }
    return one(ctx, op, addr, tx, txlen, rx, rxlen);
}

bool stm32h7_w25q_bus_make(W25Q512JVBus *bus) {
    if (bus == NULL) return false;
    WRITE32(RCC_AHB3ENR, READ32(RCC_AHB3ENR) | RCC_QSPIEN);
    WRITE32(QSPI_CR, 0U);
    WRITE32(QSPI_DCR, DCR_FSIZE_64M | DCR_CSHT);
    WRITE32(QSPI_FCR, FCR_CLEAR_ALL);
#ifdef W25Q_RENODE_VIRTUAL_TIME
    WRITE32(QSPI_CR, CR_EN);
#else
    /* Use a slow starting clock. Board must confirm the QUADSPI kernel clock,
     * CS setup/hold timing, GPIO alternate functions and memory package pins. */
    WRITE32(QSPI_CR, CR_PRESCALER(W25Q_QSPI_PRESCALER_DIV) | CR_EN);
#endif
    bus->command = bus_command;
    bus->monotonic_us = chip_time;
    bus->idle = NULL;
    bus->ctx = NULL;
    return true;
}
