#include "w25q512jv_flash.h"

#define W25_PAGE_TIMEOUT_US UINT64_C(10000)   /* Max datasheet 3.5 ms */
#define W25_ERASE_TIMEOUT_US UINT64_C(500000) /* Max datasheet 400 ms */
#define W25_POLL_GUARD UINT32_C(2000000) /* Also prevent stalled clocks */

static bool command(W25Q512JV *f, uint8_t cmd, uint32_t a,
                    const uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen)
{
    return f != NULL && f->identified && f->bus.command != NULL &&
           f->bus.command(f->bus.ctx, cmd, a, tx, txlen, rx, rxlen);
}

static bool poll_ready(W25Q512JV *f, uint64_t limit_us)
{
    const uint64_t start = f->bus.monotonic_us(f->bus.ctx);
    uint32_t spins = 0U;
    for (;;) {
        uint8_t status = 0xFFU;
        if (!command(f, W25_CMD_RDSR1, 0, NULL, 0, &status, 1)) return false;
        if ((status & W25_STATUS_WIP) == 0U) return true;
        const uint64_t now = f->bus.monotonic_us(f->bus.ctx);
        if (now < start || now - start > limit_us || ++spins > W25_POLL_GUARD) return false;
        if (f->bus.idle != NULL) f->bus.idle(f->bus.ctx);
    }
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
    if (f == NULL || bus == NULL || bus->command == NULL || bus->monotonic_us == NULL) return false;
    f->bus = *bus;
    f->identified = true; /* Bootstraps JEDEC transfer only. */
    uint8_t id[3] = {0};
    if (!command(f, W25_CMD_JEDEC, 0, NULL, 0, id, 3) ||
        id[0] != 0xEFU || id[1] != 0x40U || id[2] != 0x20U) {
        f->identified = false;
        return false;
    }
    if (!poll_ready(f, W25_ERASE_TIMEOUT_US)) {
        f->identified = false;
        return false;
    }
    return true;
}

bool w25q512jv_read(W25Q512JV *f, uint32_t address, void *data, size_t count)
{
    if (data == NULL || count == 0U || count > W25Q512JV_SIZE_BYTES ||
        (uint64_t)address + count > W25Q512JV_SIZE_BYTES) return false;
    return poll_ready(f, W25_ERASE_TIMEOUT_US) &&
           command(f, W25_CMD_READ4, address, NULL, 0, (uint8_t *)data, count);
}

bool w25q512jv_erase_sector(W25Q512JV *f, uint32_t address)
{
    if ((address % W25Q512JV_SECTOR_BYTES) != 0U ||
        address >= W25Q512JV_SIZE_BYTES) return false;
    return enable_write(f) && command(f, W25_CMD_ERASE4K4, address, NULL, 0, NULL, 0) &&
           poll_ready(f, W25_ERASE_TIMEOUT_US);
}

bool w25q512jv_program_page(W25Q512JV *f, uint32_t address, const void *data, size_t count)
{
    if (data == NULL || count == 0U || count > W25Q512JV_PAGE_BYTES ||
        (uint64_t)address + count > W25Q512JV_SIZE_BYTES ||
        (address % W25Q512JV_PAGE_BYTES) + count > W25Q512JV_PAGE_BYTES) return false;
    return enable_write(f) &&
           command(f, W25_CMD_PROG4, address, (const uint8_t *)data, count, NULL, 0) &&
           poll_ready(f, W25_PAGE_TIMEOUT_US);
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
