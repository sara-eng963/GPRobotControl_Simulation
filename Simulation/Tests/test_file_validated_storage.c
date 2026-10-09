#include "../Storage/file_validated_storage.h"

#include <stdio.h>
#include <string.h>

#define TEST_SAMPLE_COUNT 600000U
#define TEST_STORAGE_PATH "external_flash.bin"

static void fill_sample(
    PvExecutionSample *sample,
    uint32_t index
)
{
    for (uint32_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
    {
        sample->target_position_units[joint] =
            (int32_t)(index * 10U + joint);
    }
}


static bool verify_sample(
    const PvExecutionSample *sample,
    uint32_t index
)
{
    for (uint32_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
    {
        const int32_t expected =
            (int32_t)(index * 10U + joint);

        if (sample->target_position_units[joint] != expected)
        {
            return false;
        }
    }

    return true;
}


int main(void)
{
    FileValidatedStorage storage;

    if (!file_validated_storage_init(
            &storage,
            TEST_STORAGE_PATH,
            TEST_SAMPLE_COUNT))
    {
        printf("[FAIL] init\n");
        return 1;
    }

    if (!file_validated_storage_begin(&storage))
    {
        printf("[FAIL] begin\n");
        return 1;
    }

    printf("[PASS] Storage begin\n");

    PvExecutionSample sample;

    for (uint32_t i = 0U; i < TEST_SAMPLE_COUNT; ++i)
    {
        fill_sample(
            &sample,
            i
        );

        if (!file_validated_storage_write_sample(
                i,
                &sample,
                &storage))
        {
            printf("[FAIL] write sample %u\n", i);
            return 1;
        }
    }

    printf(
        "[PASS] %u samples written\n",
        TEST_SAMPLE_COUNT
    );

    ValidatedTrajectory metadata;

    memset(
        &metadata,
        0,
        sizeof(metadata)
    );

    metadata.sample_count =
        TEST_SAMPLE_COUNT;

    metadata.sample_period_us =
        PATH_VALIDATION_SAMPLE_PERIOD_US;

    if (!file_validated_storage_commit(
            &metadata,
            &storage))
    {
        printf("[FAIL] commit\n");
        return 1;
    }

    printf("[PASS] Storage commit\n");

    const uint32_t test_indices[] =
    {
        0U,
        1U,
        100U,
        100000U,
        599999U
    };

    const size_t test_count =
        sizeof(test_indices) /
        sizeof(test_indices[0]);

    for (size_t i = 0U; i < test_count; ++i)
    {
        const uint32_t index =
            test_indices[i];

        memset(
            &sample,
            0,
            sizeof(sample)
        );

        if (!file_validated_storage_read_sample(
                index,
                &sample,
                &storage))
        {
            printf(
                "[FAIL] read sample %u\n",
                index
            );

            return 1;
        }

        if (!verify_sample(
                &sample,
                index))
        {
            printf(
                "[FAIL] verify sample %u\n",
                index
            );

            return 1;
        }

        printf(
            "[PASS] Sample %u verified\n",
            index
        );
    }

    file_validated_storage_close(
        &storage
    );

    printf("[PASS] Test 6A\n");

    return 0;
}