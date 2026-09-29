#ifndef FILE_VALIDATED_STORAGE_H
#define FILE_VALIDATED_STORAGE_H

#include "../../StateMachine/States/state_path_validation.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define FILE_STORAGE_PATH_MAX 256U

typedef struct
{
    FILE *file;

    char path[FILE_STORAGE_PATH_MAX];

    uint32_t capacity_samples;
    uint32_t sample_count;

    bool writing;
    bool committed;

    ValidatedTrajectory metadata;

} FileValidatedStorage;


/*
 * Initialize the simulated external-memory device.
 */
bool file_validated_storage_init(
    FileValidatedStorage *storage,
    const char *path,
    uint32_t capacity_samples
);


/*
 * PathValidationStorage-compatible callbacks.
 */
bool file_validated_storage_begin(
    void *context
);

bool file_validated_storage_write_sample(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
);

bool file_validated_storage_commit(
    const ValidatedTrajectory *metadata,
    void *context
);

void file_validated_storage_abort(
    void *context
);


/*
 * Read one validated trajectory sample back from
 * the simulated external memory.
 */
bool file_validated_storage_read_sample(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
);


/*
 * Close the backing file when the simulator exits.
 */
void file_validated_storage_close(
    FileValidatedStorage *storage
);

#endif