#ifndef W25Q512JV_FLASH_H
#define W25Q512JV_FLASH_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define W25Q512JV_SIZE_BYTES UINT32_C(67108864)
#define W25Q512JV_SECTOR_BYTES 4096U
#define W25Q512JV_PAGE_BYTES 256U
#define W25Q512JV_JEDEC_ID UINT32_C(0xEF4020)
/* All addresses use *dedicated* 4-byte opcodes, independent of ADS/ADP. */
#define W25_CMD_JEDEC 0x9FU
#define W25_CMD_RDSR1 0x05U
#define W25_CMD_WREN 0x06U
#define W25_CMD_READ4 0x13U
#define W25_CMD_PROG4 0x12U
#define W25_CMD_ERASE4K4 0x21U
#define W25_STATUS_WIP 0x01U
#define W25_STATUS_WEL 0x02U

typedef struct {
    /* All command transfers are synchronous at the bus level; may fail. */
    bool (*command)(void *ctx, uint8_t opcode, uint32_t address,
                    const uint8_t *tx, size_t tx_size,
                    uint8_t *rx, size_t rx_size);
    uint64_t (*monotonic_us)(void *ctx); /* Must be monotonic on physical hardware */
    void (*idle)(void *ctx);             /* Optional: yield while WIP */
    void *ctx;
} W25Q512JVBus;

typedef struct {
    W25Q512JVBus bus;
    bool identified;
} W25Q512JV;

bool w25q512jv_init(W25Q512JV *flash, const W25Q512JVBus *bus);
bool w25q512jv_read(W25Q512JV *flash, uint32_t address, void *data, size_t count);
bool w25q512jv_erase_sector(W25Q512JV *flash, uint32_t address);
bool w25q512jv_program_page(W25Q512JV *flash, uint32_t address,
                            const void *data, size_t count);
uint32_t w25q512jv_crc32(uint32_t crc, const void *data, size_t count);
#endif
