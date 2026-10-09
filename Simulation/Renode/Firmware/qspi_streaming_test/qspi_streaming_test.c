#include <stdint.h>
#include "../../Storage/w25_artifact_crc.h"

#include "../../Storage/qspi_nor_validated_storage.h"
#include "../../../Storage/validated_stream_buffer.h"

#define TEST_STORAGE_BASE       0x00020000UL
#define TEST_CAPACITY_SAMPLES   4096U
#define TEST_SAMPLE_COUNT       1536U

#define STREAM_BUFFER_SAMPLES   512U
#define REFILL_CHUNK_SAMPLES    256U

#define TEST_RESULT_ADDRESS     0x24000000UL

#define TEST_MAGIC              0x50563735UL /* "PV75" */
#define TEST_RUNNING            0x52554E4EUL /* "RUNN" */
#define TEST_PASS               0x50415353UL /* "PASS" */
#define TEST_FAIL_INIT          0x494E4954UL /* "INIT" */
#define TEST_FAIL_BEGIN         0x4245474EUL /* "BEGN" */
#define TEST_FAIL_WRITE         0x57524954UL /* "WRIT" */
#define TEST_FAIL_COMMIT        0x434F4D54UL /* "COMT" */
#define TEST_FAIL_RELOAD        0x524C4F44UL /* "RLOD" */
#define TEST_FAIL_BUFFER        0x42554646UL /* "BUFF" */
#define TEST_FAIL_REFILL        0x52454649UL /* "REFI" */
#define TEST_FAIL_POP           0x504F5021UL /* "POP!" */
#define TEST_FAIL_COMPARE       0x434D5021UL /* "CMP!" */

typedef struct
{
    uint32_t magic;
    uint32_t status;

    uint32_t stored_samples;
    uint32_t reloaded_samples;

    uint32_t buffer_capacity;
    uint32_t refill_chunk;
    uint32_t refill_count;

    uint32_t pushed;
    uint32_t popped;
    uint32_t underruns;
    uint32_t high_water;
    uint32_t remaining;

    uint32_t mismatch_sample;
    uint32_t mismatch_joint;

    int32_t final_j1;
    int32_t final_j6;

} Test75Result;

static volatile Test75Result *const result =
    (volatile Test75Result *)TEST_RESULT_ADDRESS;

static QspiNorValidatedStorage writer_storage;
static QspiNorValidatedStorage reader_storage;
static PathValidationStorage storage_interface;
static ValidatedTrajectory metadata;

static PvExecutionSample buffer_memory[STREAM_BUFFER_SAMPLES];
static ValidatedStreamBuffer stream_buffer;

static uint32_t next_flash_sample = 0U;

static void stop_forever(void)
{
    for (;;)
    {
        __asm volatile ("wfi");
    }
}

static PvExecutionSample make_sample(uint32_t sample_index)
{
    PvExecutionSample sample;

    for (uint32_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
    {
        const int32_t base =
            (int32_t)(
                500000U +
                sample_index * 100U +
                joint * 7U
            );

        sample.target_position_units[joint] =
            (joint % 2U) == 0U
                ? base
                : -base;
    }

    return sample;
}

static int sample_matches(
    const PvExecutionSample *actual,
    uint32_t sample_index
)
{
    const PvExecutionSample expected =
        make_sample(sample_index);

    for (uint32_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
    {
        if (
            actual->target_position_units[joint] !=
            expected.target_position_units[joint]
        )
        {
            result->mismatch_sample = sample_index;
            result->mismatch_joint = joint;
            return 0;
        }
    }

    return 1;
}

static int refill_stream_buffer(void)
{
    if (next_flash_sample >= TEST_SAMPLE_COUNT)
    {
        return 1;
    }

    uint32_t target =
        TEST_SAMPLE_COUNT - next_flash_sample;

    if (target > REFILL_CHUNK_SAMPLES)
    {
        target = REFILL_CHUNK_SAMPLES;
    }

    if (
        validated_stream_buffer_free(&stream_buffer) <
        target
    )
    {
        return 0;
    }

    for (uint32_t i = 0U; i < target; ++i)
    {
        PvExecutionSample sample;

        if (
            !qspi_nor_validated_storage_read_sample(
                next_flash_sample,
                &sample,
                &reader_storage
            )
        )
        {
            return 0;
        }

        if (
            !validated_stream_buffer_push(
                &stream_buffer,
                &sample
            )
        )
        {
            return 0;
        }

        next_flash_sample++;
    }

    result->refill_count++;
    return 1;
}

int main(void)
{
    result->magic = TEST_MAGIC;
    result->status = TEST_RUNNING;
    result->stored_samples = 0U;
    result->reloaded_samples = 0U;
    result->buffer_capacity = STREAM_BUFFER_SAMPLES;
    result->refill_chunk = REFILL_CHUNK_SAMPLES;
    result->refill_count = 0U;
    result->pushed = 0U;
    result->popped = 0U;
    result->underruns = 0U;
    result->high_water = 0U;
    result->remaining = 0U;
    result->mismatch_sample = 0xFFFFFFFFUL;
    result->mismatch_joint = 0xFFFFFFFFUL;
    result->final_j1 = 0;
    result->final_j6 = 0;

    if (
        !qspi_nor_validated_storage_init(
            &writer_storage,
            TEST_STORAGE_BASE,
            TEST_CAPACITY_SAMPLES
        )
    )
    {
        result->status = TEST_FAIL_INIT;
        stop_forever();
    }

    qspi_nor_validated_storage_bind(
        &writer_storage,
        &storage_interface
    );

    if (!storage_interface.begin(storage_interface.context))
    {
        result->status = TEST_FAIL_BEGIN;
        stop_forever();
    }

    uint32_t sample_crc=0xFFFFFFFFUL;

    for (
        uint32_t sample_index = 0U;
        sample_index < TEST_SAMPLE_COUNT;
        ++sample_index
    )
    {
        const PvExecutionSample sample =
            make_sample(sample_index);
        sample_crc = w25q512jv_crc32(sample_crc,&sample,sizeof(sample));

        if (
            !storage_interface.write_sample(
                sample_index,
                &sample,
                storage_interface.context
            )
        )
        {
            result->status = TEST_FAIL_WRITE;
            stop_forever();
        }

        result->stored_samples =
            sample_index + 1U;
    }

    metadata.program_id = 75U;
    metadata.source_revision = 10U;
    metadata.source_crc = 0x07507507UL;

    metadata.sample_data_crc = ~sample_crc;
    metadata.sample_count = TEST_SAMPLE_COUNT;
    metadata.segment_count = 1U;
    metadata.sample_period_us =
        PATH_VALIDATION_SAMPLE_PERIOD_US;
    metadata.duration_s =
        (real_t)TEST_SAMPLE_COUNT *
        PATH_VALIDATION_SAMPLE_PERIOD_S;
    metadata.path_length_m = 1.0;

    metadata.segments[0].first_sample = 0U;
    metadata.segments[0].sample_count =
        TEST_SAMPLE_COUNT;
    metadata.segments[0].segment_type =
        (uint8_t)TEACH_SEGMENT_LINE;
    metadata.segments[0].reserved = 0U;
    metadata.artifact_crc = w25_artifact_crc(&metadata);

    if (
        !storage_interface.commit(
            &metadata,
            storage_interface.context
        )
    )
    {
        result->status = TEST_FAIL_COMMIT;
        stop_forever();
    }

    /*
     * Recreate the storage object exactly as a later Preview/Weld consumer
     * would after validation has committed the persistent artifact.
     */
    if (
        !qspi_nor_validated_storage_init(
            &reader_storage,
            TEST_STORAGE_BASE,
            TEST_CAPACITY_SAMPLES
        ) ||
        !qspi_nor_validated_storage_load_committed(
            &reader_storage
        )
    )
    {
        result->status = TEST_FAIL_RELOAD;
        stop_forever();
    }

    result->reloaded_samples =
        reader_storage.sample_count;

    if (
        !validated_stream_buffer_init(
            &stream_buffer,
            buffer_memory,
            STREAM_BUFFER_SAMPLES
        )
    )
    {
        result->status = TEST_FAIL_BUFFER;
        stop_forever();
    }

    /*
     * Prefill the 512-sample RAM buffer using two 256-sample QSPI refills.
     */
    if (
        !refill_stream_buffer() ||
        !refill_stream_buffer()
    )
    {
        result->status = TEST_FAIL_REFILL;
        stop_forever();
    }

    for (
        uint32_t execution_index = 0U;
        execution_index < TEST_SAMPLE_COUNT;
        ++execution_index
    )
    {
        PvExecutionSample sample;

        if (
            !validated_stream_buffer_pop(
                &stream_buffer,
                &sample
            )
        )
        {
            result->status = TEST_FAIL_POP;
            stop_forever();
        }

        if (
            !sample_matches(
                &sample,
                execution_index
            )
        )
        {
            result->status = TEST_FAIL_COMPARE;
            stop_forever();
        }

        /*
         * This pop represents the deterministic execution consumer. In the
         * real robot one sample is consumed per 1 ms CSP cycle.
         *
         * Refill only when at least one full 256-sample chunk is free.
         */
        if (
            next_flash_sample < TEST_SAMPLE_COUNT &&
            validated_stream_buffer_free(&stream_buffer) >=
                REFILL_CHUNK_SAMPLES
        )
        {
            if (!refill_stream_buffer())
            {
                result->status = TEST_FAIL_REFILL;
                stop_forever();
            }
        }

        if (
            execution_index ==
            TEST_SAMPLE_COUNT - 1U
        )
        {
            result->final_j1 =
                sample.target_position_units[0];
            result->final_j6 =
                sample.target_position_units[5];
        }
    }

    result->pushed =
        (uint32_t)stream_buffer.total_pushed;
    result->popped =
        (uint32_t)stream_buffer.total_popped;
    result->underruns =
        (uint32_t)stream_buffer.underrun_count;
    result->high_water =
        (uint32_t)stream_buffer.high_water_mark;
    result->remaining =
        (uint32_t)validated_stream_buffer_count(
            &stream_buffer
        );

    if (
        result->pushed != TEST_SAMPLE_COUNT ||
        result->popped != TEST_SAMPLE_COUNT ||
        result->underruns != 0U ||
        result->high_water != STREAM_BUFFER_SAMPLES ||
        result->remaining != 0U ||
        result->refill_count !=
            (TEST_SAMPLE_COUNT / REFILL_CHUNK_SAMPLES)
    )
    {
        result->status = TEST_FAIL_BUFFER;
        stop_forever();
    }

    result->status = TEST_PASS;
    stop_forever();
}
