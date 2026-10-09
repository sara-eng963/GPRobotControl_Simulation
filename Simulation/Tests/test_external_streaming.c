#include "../Storage/file_validated_storage.h"
#include "../Storage/validated_stream_buffer.h"

#include <stdio.h>
#include <string.h>


#define TEST_SAMPLE_COUNT     600000U
#define STREAM_BUFFER_SAMPLES 512U
#define REFILL_CHUNK_SAMPLES  256U

#define TEST_STORAGE_PATH \
    "external_stream_test.bin"


static void fill_sample(
    PvExecutionSample *sample,
    uint32_t index
)
{
    for (
        uint32_t joint = 0U;
        joint < PATH_VALIDATION_DOF;
        ++joint
    )
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
    for (
        uint32_t joint = 0U;
        joint < PATH_VALIDATION_DOF;
        ++joint
    )
    {
        const int32_t expected =
            (int32_t)(index * 10U + joint);

        if (
            sample->target_position_units[joint] !=
            expected
        )
        {
            return false;
        }
    }

    return true;
}


int main(void)
{
    FileValidatedStorage external_storage;

    /*
     * ------------------------------------------------------------
     * CREATE EXTERNAL TRAJECTORY
     * ------------------------------------------------------------
     */

    if (
        !file_validated_storage_init(
            &external_storage,
            TEST_STORAGE_PATH,
            TEST_SAMPLE_COUNT
        )
    )
    {
        printf("[FAIL] External storage init\n");
        return 1;
    }

    if (
        !file_validated_storage_begin(
            &external_storage
        )
    )
    {
        printf("[FAIL] External storage begin\n");
        return 1;
    }

    PvExecutionSample sample;

    for (
        uint32_t i = 0U;
        i < TEST_SAMPLE_COUNT;
        ++i
    )
    {
        fill_sample(
            &sample,
            i
        );

        if (
            !file_validated_storage_write_sample(
                i,
                &sample,
                &external_storage
            )
        )
        {
            printf(
                "[FAIL] External write at %u\n",
                i
            );

            return 1;
        }
    }

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

    if (
        !file_validated_storage_commit(
            &metadata,
            &external_storage
        )
    )
    {
        printf("[FAIL] External commit\n");
        return 1;
    }

    printf(
        "[PASS] External trajectory created: %u samples\n",
        TEST_SAMPLE_COUNT
    );


    /*
     * ------------------------------------------------------------
     * CREATE SMALL INTERNAL-RAM BUFFER
     * ------------------------------------------------------------
     */

    PvExecutionSample ram_storage[
        STREAM_BUFFER_SAMPLES
    ];

    ValidatedStreamBuffer buffer;

    if (
        !validated_stream_buffer_init(
            &buffer,
            ram_storage,
            STREAM_BUFFER_SAMPLES
        )
    )
    {
        printf("[FAIL] Stream buffer init\n");
        return 1;
    }

    printf(
        "[PASS] RAM stream buffer created: %u samples\n",
        STREAM_BUFFER_SAMPLES
    );


    /*
     * ------------------------------------------------------------
     * STREAM EXTERNAL STORAGE -> RAM BUFFER -> CONSUMER
     * ------------------------------------------------------------
     */

    uint32_t next_external_sample =
        0U;

    uint32_t next_expected_sample =
        0U;

    uint32_t refill_count =
        0U;


    while (
        next_expected_sample <
        TEST_SAMPLE_COUNT
    )
    {
        /*
         * Refill when at least one refill chunk fits.
         */
        if (
            validated_stream_buffer_free(
                &buffer
            ) >= REFILL_CHUNK_SAMPLES
            &&
            next_external_sample <
                TEST_SAMPLE_COUNT
        )
        {
            uint32_t samples_to_load =
                REFILL_CHUNK_SAMPLES;

            const uint32_t samples_remaining =
                TEST_SAMPLE_COUNT -
                next_external_sample;

            if (
                samples_to_load >
                samples_remaining
            )
            {
                samples_to_load =
                    samples_remaining;
            }

            for (
                uint32_t i = 0U;
                i < samples_to_load;
                ++i
            )
            {
                PvExecutionSample loaded;

                if (
                    !file_validated_storage_read_sample(
                        next_external_sample,
                        &loaded,
                        &external_storage
                    )
                )
                {
                    printf(
                        "[FAIL] External read at %u\n",
                        next_external_sample
                    );

                    return 1;
                }

                if (
                    !validated_stream_buffer_push(
                        &buffer,
                        &loaded
                    )
                )
                {
                    printf("[FAIL] Buffer push\n");
                    return 1;
                }

                next_external_sample++;
            }

            refill_count++;
        }


        /*
         * One logical 1 ms execution cycle.
         */
        PvExecutionSample executing;

        if (
            !validated_stream_buffer_pop(
                &buffer,
                &executing
            )
        )
        {
            printf(
                "[FAIL] BUFFER UNDERRUN at sample %u\n",
                next_expected_sample
            );

            return 1;
        }

        if (
            !verify_sample(
                &executing,
                next_expected_sample
            )
        )
        {
            printf(
                "[FAIL] Incorrect sample order at %u\n",
                next_expected_sample
            );

            return 1;
        }

        next_expected_sample++;
    }


    /*
     * ------------------------------------------------------------
     * RESULTS
     * ------------------------------------------------------------
     */

    printf(
        "[PASS] All %u samples streamed in correct order\n",
        TEST_SAMPLE_COUNT
    );

    printf("\nStreaming statistics:\n");

    printf(
        "  External samples = %u\n",
        TEST_SAMPLE_COUNT
    );

    printf(
        "  RAM capacity     = %zu samples\n",
        buffer.capacity
    );

    printf(
        "  RAM bytes        = %zu B\n",
        buffer.capacity *
        sizeof(PvExecutionSample)
    );

    printf(
        "  Refill chunk     = %u samples\n",
        REFILL_CHUNK_SAMPLES
    );

    printf(
        "  Refills          = %u\n",
        refill_count
    );

    printf(
        "  High water mark  = %zu samples\n",
        buffer.high_water_mark
    );

    printf(
        "  Total pushed     = %zu\n",
        buffer.total_pushed
    );

    printf(
        "  Total popped     = %zu\n",
        buffer.total_popped
    );

    printf(
        "  Underruns        = %zu\n",
        buffer.underrun_count
    );

    file_validated_storage_close(
        &external_storage
    );

    printf("\n[PASS] Test 6B.2\n");

    return 0;
}