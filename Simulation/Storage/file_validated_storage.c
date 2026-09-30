#include "file_validated_storage.h"

#include <inttypes.h>
#include <string.h>


#define TRAJECTORY_MEMORY_LOG_PATH "trajectory_memory_log.csv"
#define SIMULATED_FLASH_SECTOR_BYTES UINT64_C(4096)


static uint64_t round_up_u64(
    uint64_t value,
    uint64_t alignment
)
{
    return
        ((value + alignment - 1U) / alignment) * alignment;
}


static const char *segment_type_name(
    uint8_t segment_type
)
{
    switch ((TeachingSegmentType)segment_type)
    {
        case TEACH_SEGMENT_LINE:
            return "LINE";

        case TEACH_SEGMENT_ARC:
            return "ARC";

        case TEACH_SEGMENT_CIRCLE:
            return "CIRCLE";

        default:
            return "UNKNOWN";
    }
}


/*
 * Simulator-only Test 7.6 instrumentation.
 *
 * The payload file itself models one currently committed trajectory and is
 * intentionally truncated when a new validation begins. This side-car CSV is
 * therefore only an experiment history: it records the memory footprint of
 * each successful validation so several user-created HMI trajectories can be
 * analysed together afterwards.
 *
 * This does NOT change PathValidationStorage semantics and is not part of the
 * production QSPI format.
 */
static void append_trajectory_memory_log(
    const FileValidatedStorage *storage,
    const ValidatedTrajectory *metadata
)
{
    if (
        storage == NULL ||
        metadata == NULL ||
        strcmp(storage->path, "external_flash.bin") != 0
    )
    {
        return;
    }

    FILE *log =
        fopen(
            TRAJECTORY_MEMORY_LOG_PATH,
            "a+"
        );

    if (log == NULL)
    {
        fprintf(
            stderr,
            "SIM STORAGE: warning - could not append %s\n",
            TRAJECTORY_MEMORY_LOG_PATH
        );

        return;
    }

    if (
        fseek(log, 0L, SEEK_END) != 0
    )
    {
        fclose(log);
        return;
    }

    const long current_size =
        ftell(log);

    if (current_size == 0L)
    {
        fprintf(
            log,
            "program_id,source_revision,segment_count,segments,sample_count,"
            "sample_period_us,duration_s,path_length_m,raw_bytes,"
            "flash_allocated_bytes\n"
        );
    }

    const uint64_t raw_bytes =
        (uint64_t)metadata->sample_count *
        (uint64_t)sizeof(PvExecutionSample);

    const uint64_t rounded_sample_bytes =
        round_up_u64(
            raw_bytes,
            SIMULATED_FLASH_SECTOR_BYTES
        );

    const uint64_t flash_allocated_bytes =
        SIMULATED_FLASH_SECTOR_BYTES +
        rounded_sample_bytes;

    fprintf(
        log,
        "%" PRIu32 ",%" PRIu32 ",%u,\"",
        metadata->program_id,
        metadata->source_revision,
        (unsigned)metadata->segment_count
    );

    for (
        uint16_t segment = 0U;
        segment < metadata->segment_count;
        ++segment
    )
    {
        if (segment > 0U)
        {
            fputc(';', log);
        }

        fputs(
            segment_type_name(
                metadata->segments[segment].segment_type
            ),
            log
        );
    }

    fprintf(
        log,
        "\",%" PRIu32 ",%" PRIu32 ",%.9f,%.9f,%" PRIu64 ",%" PRIu64 "\n",
        metadata->sample_count,
        metadata->sample_period_us,
        (double)metadata->duration_s,
        (double)metadata->path_length_m,
        raw_bytes,
        flash_allocated_bytes
    );

    fflush(log);
    fclose(log);

    printf(
        "SIM STORAGE LOG: trajectory committed -> samples=%" PRIu32
        " duration=%.3f s flash=%.3f MiB history=%s\n",
        metadata->sample_count,
        (double)metadata->duration_s,
        (double)flash_allocated_bytes / (1024.0 * 1024.0),
        TRAJECTORY_MEMORY_LOG_PATH
    );
}


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

    append_trajectory_memory_log(
        storage,
        metadata
    );

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
