#ifndef QSPI_NOR_VALIDATED_STORAGE_H
#define QSPI_NOR_VALIDATED_STORAGE_H
#include "../../../StateMachine/States/state_path_validation.h"
#include "w25q512jv_flash.h"
#include <stdbool.h>
#include <stdint.h>
#define QSPI_NOR_STORAGE_SECTOR_BYTES W25Q512JV_SECTOR_BYTES
#define QSPI_NOR_STORAGE_PAGE_BYTES W25Q512JV_PAGE_BYTES

typedef struct {
    uint32_t base_address;
    uint32_t capacity_samples;
    uint32_t sample_count;
    uint32_t next_sector_to_erase;
    uint32_t slot_bytes;
    uint32_t next_generation;
    uint32_t write_address;
    int8_t active_slot;
    int8_t write_slot;
    bool writing;
    bool committed;
    bool readback_verified;
    uint32_t crc_state;
    uint16_t page_used;
    uint8_t page[QSPI_NOR_STORAGE_PAGE_BYTES];
    ValidatedTrajectory metadata;
} QspiNorValidatedStorage;

/* Supply an initialized, JEDEC-verified transport before init.
 * Testing may inject a mock. The board's STM32 adapter is selected by
 * W25Q_STM32H7_BACKEND for the Renode/bare-metal targets. */
bool qspi_nor_validated_storage_set_bus(const W25Q512JVBus *bus);
bool qspi_nor_validated_storage_init(QspiNorValidatedStorage *s,
                                     uint32_t base_address, uint32_t capacity_samples);
void qspi_nor_validated_storage_bind(QspiNorValidatedStorage *s,
                                     PathValidationStorage *out);
bool qspi_nor_validated_storage_begin(void *context);
bool qspi_nor_validated_storage_write_sample(uint32_t index,
                    const PvExecutionSample *sample, void *context);
bool qspi_nor_validated_storage_commit(const ValidatedTrajectory *m, void *context);
void qspi_nor_validated_storage_abort(void *context);
bool qspi_nor_validated_storage_load_committed(QspiNorValidatedStorage *s);
bool qspi_nor_validated_storage_read_sample(uint32_t index,
                    PvExecutionSample *sample, void *context);
/* Fast continuous sequential sample transfer; never accepted pre-commit. */
bool qspi_nor_validated_storage_read_samples(uint32_t index,
                    PvExecutionSample *samples, uint32_t count, void *context);
#endif
