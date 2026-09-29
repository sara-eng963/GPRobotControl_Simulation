#include <stdint.h>

#include "../../Storage/qspi_nor_validated_storage.h"

#define TEST_STORAGE_BASE       0x00010000UL
#define TEST_CAPACITY_SAMPLES   1024U
#define TEST_SAMPLE_COUNT       12U
#define TEST_RESULT_ADDRESS     0x24000000UL

#define TEST_MAGIC              0x50563734UL /* "PV74" */
#define TEST_RUNNING            0x52554E4EUL /* "RUNN" */
#define TEST_PASS               0x50415353UL /* "PASS" */
#define TEST_FAIL_INIT          0x494E4954UL /* "INIT" */
#define TEST_FAIL_BIND          0x42494E44UL /* "BIND" */
#define TEST_FAIL_BEGIN         0x4245474EUL /* "BEGN" */
#define TEST_FAIL_WRITE         0x57524954UL /* "WRIT" */
#define TEST_FAIL_COMMIT        0x434F4D54UL /* "COMT" */
#define TEST_FAIL_RELOAD        0x524C4F44UL /* "RLOD" */
#define TEST_FAIL_METADATA      0x4D455441UL /* "META" */
#define TEST_FAIL_READ          0x52454144UL /* "READ" */
#define TEST_FAIL_COMPARE       0x434D5021UL /* "CMP!" */

typedef struct
{
    uint32_t magic;
    uint32_t status;

    uint32_t callbacks_bound;
    uint32_t written_samples;
    uint32_t committed_before_reload;
    uint32_t reloaded_committed;
    uint32_t metadata_ok;

    uint32_t reloaded_sample_count;
    uint32_t sample_size;

    uint32_t mismatch_sample;
    uint32_t mismatch_joint;

    int32_t last_sample_j1;
    int32_t last_sample_j6;

} Test74Result;

static volatile Test74Result *const result =
    (volatile Test74Result *)TEST_RESULT_ADDRESS;

static QspiNorValidatedStorage writer_storage;
static QspiNorValidatedStorage reloaded_storage;
static PathValidationStorage storage_interface;
static ValidatedTrajectory metadata;

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
        const int32_t magnitude =
            (int32_t)(
                10000U +
                sample_index * 100U +
                joint * 10U
            );

        sample.target_position_units[joint] =
            (joint % 2U) == 0U
                ? magnitude
                : -magnitude;
    }

    return sample;
}

static int samples_equal(
    const PvExecutionSample *actual,
    const PvExecutionSample *expected,
    uint32_t sample_index
)
{
    for (uint32_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
    {
        if (
            actual->target_position_units[joint] !=
            expected->target_position_units[joint]
        )
        {
            result->mismatch_sample = sample_index;
            result->mismatch_joint = joint;
            return 0;
        }
    }

    return 1;
}

static int metadata_matches(void)
{
    return
        reloaded_storage.metadata.program_id == 74U &&
        reloaded_storage.metadata.source_revision == 9U &&
        reloaded_storage.metadata.source_crc == 0x12345678UL &&
        reloaded_storage.metadata.artifact_crc == 0x87654321UL &&
        reloaded_storage.metadata.sample_data_crc == 0xA5A55A5AUL &&
        reloaded_storage.metadata.sample_count == TEST_SAMPLE_COUNT &&
        reloaded_storage.metadata.segment_count == 1U &&
        reloaded_storage.metadata.sample_period_us ==
            PATH_VALIDATION_SAMPLE_PERIOD_US;
}

int main(void)
{
    result->magic = TEST_MAGIC;
    result->status = TEST_RUNNING;
    result->callbacks_bound = 0U;
    result->written_samples = 0U;
    result->committed_before_reload = 0U;
    result->reloaded_committed = 0U;
    result->metadata_ok = 0U;
    result->reloaded_sample_count = 0U;
    result->sample_size = (uint32_t)sizeof(PvExecutionSample);
    result->mismatch_sample = 0xFFFFFFFFUL;
    result->mismatch_joint = 0xFFFFFFFFUL;
    result->last_sample_j1 = 0;
    result->last_sample_j6 = 0;

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

    if (
        storage_interface.begin == 0 ||
        storage_interface.write_sample == 0 ||
        storage_interface.commit == 0 ||
        storage_interface.abort == 0 ||
        storage_interface.capacity_samples != TEST_CAPACITY_SAMPLES ||
        storage_interface.context != &writer_storage
    )
    {
        result->status = TEST_FAIL_BIND;
        stop_forever();
    }

    result->callbacks_bound = 1U;

    if (!storage_interface.begin(storage_interface.context))
    {
        result->status = TEST_FAIL_BEGIN;
        stop_forever();
    }

    for (
        uint32_t sample_index = 0U;
        sample_index < TEST_SAMPLE_COUNT;
        ++sample_index
    )
    {
        const PvExecutionSample sample =
            make_sample(sample_index);

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

        result->written_samples =
            sample_index + 1U;
    }

    metadata.program_id = 74U;
    metadata.source_revision = 9U;
    metadata.source_crc = 0x12345678UL;
    metadata.artifact_crc = 0x87654321UL;
    metadata.sample_data_crc = 0xA5A55A5AUL;
    metadata.sample_count = TEST_SAMPLE_COUNT;
    metadata.segment_count = 1U;
    metadata.sample_period_us =
        PATH_VALIDATION_SAMPLE_PERIOD_US;
    metadata.duration_s =
        (real_t)TEST_SAMPLE_COUNT *
        PATH_VALIDATION_SAMPLE_PERIOD_S;
    metadata.path_length_m = 0.25;

    metadata.segments[0].first_sample = 0U;
    metadata.segments[0].sample_count =
        TEST_SAMPLE_COUNT;
    metadata.segments[0].segment_type =
        (uint8_t)TEACH_SEGMENT_LINE;
    metadata.segments[0].reserved = 0U;

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

    result->committed_before_reload =
        writer_storage.committed ? 1U : 0U;

    /*
     * Recreate the runtime object and discover the committed artifact from
     * flash. This proves commit data is not only being kept in RAM.
     */
    if (
        !qspi_nor_validated_storage_init(
            &reloaded_storage,
            TEST_STORAGE_BASE,
            TEST_CAPACITY_SAMPLES
        )
    )
    {
        result->status = TEST_FAIL_INIT;
        stop_forever();
    }

    if (
        !qspi_nor_validated_storage_load_committed(
            &reloaded_storage
        )
    )
    {
        result->status = TEST_FAIL_RELOAD;
        stop_forever();
    }

    result->reloaded_committed =
        reloaded_storage.committed ? 1U : 0U;

    result->reloaded_sample_count =
        reloaded_storage.sample_count;

    result->metadata_ok =
        metadata_matches() ? 1U : 0U;

    if (result->metadata_ok == 0U)
    {
        result->status = TEST_FAIL_METADATA;
        stop_forever();
    }

    for (
        uint32_t sample_index = 0U;
        sample_index < TEST_SAMPLE_COUNT;
        ++sample_index
    )
    {
        PvExecutionSample actual;
        const PvExecutionSample expected =
            make_sample(sample_index);

        if (
            !qspi_nor_validated_storage_read_sample(
                sample_index,
                &actual,
                &reloaded_storage
            )
        )
        {
            result->status = TEST_FAIL_READ;
            stop_forever();
        }

        if (
            !samples_equal(
                &actual,
                &expected,
                sample_index
            )
        )
        {
            result->status = TEST_FAIL_COMPARE;
            stop_forever();
        }

        if (
            sample_index ==
            TEST_SAMPLE_COUNT - 1U
        )
        {
            result->last_sample_j1 =
                actual.target_position_units[0];

            result->last_sample_j6 =
                actual.target_position_units[5];
        }
    }

    result->status = TEST_PASS;
    stop_forever();
}
