#ifndef QSPI_NOR_VALIDATED_STORAGE_H
#define QSPI_NOR_VALIDATED_STORAGE_H

#include "../../../StateMachine/States/state_path_validation.h"

#include <stdbool.h>
#include <stdint.h>

#define QSPI_NOR_STORAGE_SECTOR_BYTES 4096U
#define QSPI_NOR_STORAGE_PAGE_BYTES   256U

typedef struct
{
    uint32_t base_address;
    uint32_t capacity_samples;
    uint32_t sample_count;

    uint32_t next_sector_to_erase;

    bool writing;
    bool committed;

    ValidatedTrajectory metadata;

} QspiNorValidatedStorage;

/*
 * Renode/Test implementation of the robot's persistent validated-trajectory
 * backend. It implements the same PathValidationStorage callback contract used
 * by FileValidatedStorage, but the bytes are sent through STM32H7 QUADSPI to
 * the simulated SPI-NOR device.
 *
 * Current limitation:
 *   3-byte flash addressing is used, so this implementation is intentionally
 *   constrained below the 16 MiB address boundary.
 */
bool qspi_nor_validated_storage_init(
    QspiNorValidatedStorage *storage,
    uint32_t base_address,
    uint32_t capacity_samples
);

void qspi_nor_validated_storage_bind(
    QspiNorValidatedStorage *storage,
    PathValidationStorage *interface_out
);

/* PathValidationStorage-compatible callbacks. */
bool qspi_nor_validated_storage_begin(void *context);

bool qspi_nor_validated_storage_write_sample(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
);

bool qspi_nor_validated_storage_commit(
    const ValidatedTrajectory *metadata,
    void *context
);

void qspi_nor_validated_storage_abort(void *context);

/*
 * Reload a previously committed header/metadata after recreating the runtime
 * storage object. This models controller restart/persistent storage discovery.
 */
bool qspi_nor_validated_storage_load_committed(
    QspiNorValidatedStorage *storage
);

bool qspi_nor_validated_storage_read_sample(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
);

#endif /* QSPI_NOR_VALIDATED_STORAGE_H */
