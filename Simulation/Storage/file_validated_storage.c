#include "file_validated_storage.h"

#include <string.h>


bool file_validated_storage_init(
    FileValidatedStorage *storage,
    const char *path,
    uint32_t capacity_samples
)
{
    if (
        storage == NULL ||
        path == NULL ||
        capacity_samples == 0U
    )
    {
        return false;
    }

    memset(
        storage,
        0,
        sizeof(*storage)
    );

    if (strlen(path) >= FILE_STORAGE_PATH_MAX)
    {
        return false;
    }

    strcpy(
        storage->path,
        path
    );

    storage->capacity_samples =
        capacity_samples;

    return true;
}


bool file_validated_storage_begin(
    void *context
)
{
    FileValidatedStorage *storage =
        (FileValidatedStorage *)context;

    if (storage == NULL)
    {
        return false;
    }

    /*
     * Close an old file first if one is still open.
     */
    if (storage->file != NULL)
    {
        fclose(storage->file);
        storage->file = NULL;
    }

    /*
     * w+b:
     * - create file if it does not exist
     * - erase/truncate old contents
     * - allow both reading and writing
     */
    storage->file =
        fopen(
            storage->path,
            "w+b"
        );

    if (storage->file == NULL)
    {
        return false;
    }

    storage->sample_count =
        0U;

    storage->writing =
        true;

    storage->committed =
        false;

    memset(
        &storage->metadata,
        0,
        sizeof(storage->metadata)
    );

    return true;
}


bool file_validated_storage_write_sample(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
)
{
    FileValidatedStorage *storage =
        (FileValidatedStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        storage->file == NULL ||
        !storage->writing ||
        sample_index >= storage->capacity_samples
    )
    {
        return false;
    }

    const long offset =
        (long)(
            sample_index *
            sizeof(PvExecutionSample)
        );

    if (
        fseek(
            storage->file,
            offset,
            SEEK_SET
        ) != 0
    )
    {
        return false;
    }

    if (
        fwrite(
            sample,
            sizeof(PvExecutionSample),
            1U,
            storage->file
        ) != 1U
    )
    {
        return false;
    }

    if (
        sample_index + 1U >
        storage->sample_count
    )
    {
        storage->sample_count =
            sample_index + 1U;
    }

    return true;
}


bool file_validated_storage_commit(
    const ValidatedTrajectory *metadata,
    void *context
)
{
    FileValidatedStorage *storage =
        (FileValidatedStorage *)context;

    if (
        storage == NULL ||
        metadata == NULL ||
        storage->file == NULL ||
        !storage->writing ||
        metadata->sample_count > storage->sample_count
    )
    {
        return false;
    }

    /*
     * Force buffered writes to the backing file.
     */
    if (fflush(storage->file) != 0)
    {
        return false;
    }

    storage->metadata =
        *metadata;

    storage->writing =
        false;

    storage->committed =
        true;

    return true;
}


void file_validated_storage_abort(
    void *context
)
{
    FileValidatedStorage *storage =
        (FileValidatedStorage *)context;

    if (storage == NULL)
    {
        return;
    }

    storage->writing =
        false;

    storage->committed =
        false;

    storage->sample_count =
        0U;

    memset(
        &storage->metadata,
        0,
        sizeof(storage->metadata)
    );

    if (storage->file != NULL)
    {
        fclose(storage->file);
        storage->file =
            NULL;
    }
}


bool file_validated_storage_read_sample(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
)
{
    FileValidatedStorage *storage =
        (FileValidatedStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        storage->file == NULL ||
        !storage->committed ||
        sample_index >= storage->metadata.sample_count ||
        sample_index >= storage->sample_count
    )
    {
        return false;
    }

    const long offset =
        (long)(
            sample_index *
            sizeof(PvExecutionSample)
        );

    if (
        fseek(
            storage->file,
            offset,
            SEEK_SET
        ) != 0
    )
    {
        return false;
    }

    if (
        fread(
            sample,
            sizeof(PvExecutionSample),
            1U,
            storage->file
        ) != 1U
    )
    {
        return false;
    }

    return true;
}


void file_validated_storage_close(
    FileValidatedStorage *storage
)
{
    if (storage == NULL)
    {
        return;
    }

    if (storage->file != NULL)
    {
        fclose(storage->file);
        storage->file =
            NULL;
    }

    storage->writing =
        false;
}