#include "w25q512jv_flash.h"

#define W25_PAGE_TIMEOUT_US UINT64_C(10000)   /* Max datasheet 3.5 ms */
#define W25_ERASE_TIMEOUT_US UINT64_C(500000) /* Max datasheet 400 ms */
#define W25_POLL_GUARD UINT32_C(2000000) /* Also prevent stalled clocks */
#define W25_STARTUP_WAIT_US UINT64_C(5000) /* tVSL=20us, tPUW max=5ms */

static bool valid_bus(const W25Q512JV *f)
{
    return f != NULL && f->bus.command != NULL && f->bus.monotonic_us != NULL;
}

static bool valid_flash(const W25Q512JV *f)
{
    return valid_bus(f) && f->identified;
}

static bool command(W25Q512JV *f, uint8_t cmd, uint32_t a,
                    const uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen)
{
    /* Internal transport is also used before initialization publishes identity. */
    return valid_bus(f) &&
           f->bus.command(f->bus.ctx, cmd, a, tx, txlen, rx, rxlen);
}

static bool startup_wait(W25Q512JV *f)
{
    if (!valid_bus(f)) return false;
    const uint64_t start=f->bus.monotonic_us(f->bus.ctx);
    uint64_t previous=start;
    for (uint32_t spins=0; spins<W25_POLL_GUARD; ++spins) {
        const uint64_t now=f->bus.monotonic_us(f->bus.ctx);
        if (now < previous) return false;
        if (now-start >= W25_STARTUP_WAIT_US) return true;
        previous=now;
        if (f->bus.idle != NULL) f->bus.idle(f->bus.ctx);
    }
    return false;
}

static bool poll_ready(W25Q512JV *f, uint64_t limit_us)
{
    if (!valid_bus(f)) return false;
    const uint64_t start = f->bus.monotonic_us(f->bus.ctx);
    uint64_t previous=start;
    for (uint32_t spins=0; spins<W25_POLL_GUARD; ++spins) {
        uint8_t status = 0xFFU;
        if (!command(f, W25_CMD_RDSR1, 0, NULL, 0, &status, 1)) return false;
        const uint64_t now = f->bus.monotonic_us(f->bus.ctx);
        /* A late ready response is a timeout too. Check time before success. */
        if (now < previous || now-start > limit_us) return false;
        if ((status & W25_STATUS_WIP) == 0U) return true;
        if (now-start == limit_us) return false;
        previous=now;
        if (f->bus.idle != NULL) f->bus.idle(f->bus.ctx);
    }
    return false;
}

static bool write_completed(W25Q512JV *f, uint64_t timeout_us)
{
    uint8_t status=0xFFU;
    return poll_ready(f,timeout_us) &&
        command(f,W25_CMD_RDSR1,0,NULL,0,&status,1) &&
        (status & (W25_STATUS_WIP | W25_STATUS_WEL)) == 0U;
}

static bool verify_contents(W25Q512JV *f, uint32_t at,
                            const uint8_t *expected, size_t count)
{
    uint8_t bytes[W25Q512JV_PAGE_BYTES];
    while (count > 0U) {
        const size_t n=count < sizeof(bytes) ? count : sizeof(bytes);
        if (!command(f,W25_CMD_READ4,at,NULL,0,bytes,n)) return false;
        for (size_t i=0; i<n; ++i)
            if (bytes[i] != (expected != NULL ? expected[i] : 0xFFU)) return false;
        at+=(uint32_t)n; count-=n;
        if (expected != NULL) expected+=n;
    }
    return true;
}

static bool enable_write(W25Q512JV *f)
{
    uint8_t status = 0;
    return poll_ready(f, W25_ERASE_TIMEOUT_US) &&
           command(f, W25_CMD_WREN, 0, NULL, 0, NULL, 0) &&
           command(f, W25_CMD_RDSR1, 0, NULL, 0, &status, 1) &&
           (status & W25_STATUS_WEL) != 0U;
}

bool w25q512jv_init(W25Q512JV *f, const W25Q512JVBus *bus)
{
    if (f == NULL) return false;
    f->identified=false; /* A rejected re-init must invalidate an old identity. */
    if (bus == NULL || bus->command == NULL || bus->monotonic_us == NULL) return false;
    f->bus = *bus;
    /* Caller establishes stable VCC first. Wait conservatively for power-up
     * access/write inhibit, then allow an MCU-only reset's ongoing operation
     * to finish. Never reset/unlock the chip automatically while WIP is set. */
    if (!startup_wait(f) || !poll_ready(f,W25_ERASE_TIMEOUT_US)) return false;
    uint8_t id[3] = {0};
    if (!command(f, W25_CMD_JEDEC, 0, NULL, 0, id, 3) ||
        id[0] != 0xEFU || id[1] != 0x40U || id[2] != 0x20U) {
        f->identified = false;
        return false;
    }
    f->identified=true;
    return true;
}

bool w25q512jv_read(W25Q512JV *f, uint32_t address, void *data, size_t count)
{
    if (!valid_flash(f) || data == NULL || count == 0U || count > W25Q512JV_SIZE_BYTES ||
        (uint64_t)address + count > W25Q512JV_SIZE_BYTES) return false;
    return poll_ready(f, W25_ERASE_TIMEOUT_US) &&
           command(f, W25_CMD_READ4, address, NULL, 0, (uint8_t *)data, count);
}

bool w25q512jv_erase_sector(W25Q512JV *f, uint32_t address)
{
    if (!valid_flash(f) || (address % W25Q512JV_SECTOR_BYTES) != 0U ||
        address >= W25Q512JV_SIZE_BYTES) return false;
    return enable_write(f) && command(f, W25_CMD_ERASE4K4, address, NULL, 0, NULL, 0) &&
           write_completed(f, W25_ERASE_TIMEOUT_US) &&
           verify_contents(f,address,NULL,W25Q512JV_SECTOR_BYTES);
}

bool w25q512jv_program_page(W25Q512JV *f, uint32_t address, const void *data, size_t count)
{
    if (!valid_flash(f) || data == NULL || count == 0U || count > W25Q512JV_PAGE_BYTES ||
        (uint64_t)address + count > W25Q512JV_SIZE_BYTES ||
        (address % W25Q512JV_PAGE_BYTES) + count > W25Q512JV_PAGE_BYTES) return false;
    return enable_write(f) &&
           command(f, W25_CMD_PROG4, address, (const uint8_t *)data, count, NULL, 0) &&
           write_completed(f, W25_PAGE_TIMEOUT_US) &&
           verify_contents(f,address,(const uint8_t *)data,count);
}

uint32_t w25q512jv_crc32(uint32_t crc, const void *data, size_t count)
{
    const uint8_t *b = (const uint8_t *)data;
    for (size_t i = 0; i < count; ++i) {
        crc ^= b[i];
        for (int k = 0; k < 8; ++k) {
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
        }
    }
    return crc;
}
