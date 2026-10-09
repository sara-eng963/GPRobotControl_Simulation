#include "../Storage/validated_stream_buffer.h"

#include <stdio.h>


#define TEST_BUFFER_CAPACITY 8U


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
            (int32_t)(index * 100U + joint);
    }
}


static bool verify_sample(
    const PvExecutionSample *sample,
    uint32_t expected_index
)
{
    for (
        uint32_t joint = 0U;
        joint < PATH_VALIDATION_DOF;
        ++joint
    )
    {
        const int32_t expected =
            (int32_t)(
                expected_index * 100U +
                joint
            );

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
    PvExecutionSample storage[
        TEST_BUFFER_CAPACITY
    ];

    ValidatedStreamBuffer buffer;

    if (
        !validated_stream_buffer_init(
            &buffer,
            storage,
            TEST_BUFFER_CAPACITY
        )
    )
    {
        printf("[FAIL] Buffer init\n");
        return 1;
    }

    printf("[PASS] Buffer init\n");


    /* ------------------------------------------------------------
     * TEST 1 — Fill the complete buffer
     * ------------------------------------------------------------ */

    for (
        uint32_t i = 0U;
        i < TEST_BUFFER_CAPACITY;
        ++i
    )
    {
        PvExecutionSample sample;

        fill_sample(
            &sample,
            i
        );

        if (
            !validated_stream_buffer_push(
                &buffer,
                &sample
            )
        )
        {
            printf(
                "[FAIL] Push sample %u\n",
                i
            );

            return 1;
        }
    }

    if (
        validated_stream_buffer_count(
            &buffer
        ) != TEST_BUFFER_CAPACITY
    )
    {
        printf("[FAIL] Full buffer count\n");
        return 1;
    }

    if (
        !validated_stream_buffer_is_full(
            &buffer
        )
    )
    {
        printf("[FAIL] Full flag\n");
        return 1;
    }

    printf("[PASS] Buffer filled to capacity\n");


    /* ------------------------------------------------------------
     * TEST 2 — Pop first four samples
     * ------------------------------------------------------------ */

    for (
        uint32_t i = 0U;
        i < 4U;
        ++i
    )
    {
        PvExecutionSample sample;

        if (
            !validated_stream_buffer_pop(
                &buffer,
                &sample
            )
        )
        {
            printf(
                "[FAIL] Pop sample %u\n",
                i
            );

            return 1;
        }

        if (
            !verify_sample(
                &sample,
                i
            )
        )
        {
            printf(
                "[FAIL] Verify popped sample %u\n",
                i
            );

            return 1;
        }
    }

    printf("[PASS] First four samples popped correctly\n");


    /* ------------------------------------------------------------
     * TEST 3 — Push four more samples
     *
     * This forces the ring buffer to wrap around.
     * ------------------------------------------------------------ */

    for (
        uint32_t i = 8U;
        i < 12U;
        ++i
    )
    {
        PvExecutionSample sample;

        fill_sample(
            &sample,
            i
        );

        if (
            !validated_stream_buffer_push(
                &buffer,
                &sample
            )
        )
        {
            printf(
                "[FAIL] Wrapped push sample %u\n",
                i
            );

            return 1;
        }
    }

    if (
        validated_stream_buffer_count(
            &buffer
        ) != TEST_BUFFER_CAPACITY
    )
    {
        printf("[FAIL] Wrapped buffer count\n");
        return 1;
    }

    printf("[PASS] Ring-buffer wraparound push\n");


    /* ------------------------------------------------------------
     * TEST 4 — Verify remaining FIFO order
     *
     * Expected:
     *
     * 4 5 6 7 8 9 10 11
     * ------------------------------------------------------------ */

    for (
        uint32_t expected = 4U;
        expected < 12U;
        ++expected
    )
    {
        PvExecutionSample sample;

        if (
            !validated_stream_buffer_pop(
                &buffer,
                &sample
            )
        )
        {
            printf(
                "[FAIL] Final pop %u\n",
                expected
            );

            return 1;
        }

        if (
            !verify_sample(
                &sample,
                expected
            )
        )
        {
            printf(
                "[FAIL] FIFO order at %u\n",
                expected
            );

            return 1;
        }
    }

    printf("[PASS] FIFO ordering after wraparound\n");


    /* ------------------------------------------------------------
     * TEST 5 — Empty buffer / underrun detection
     * ------------------------------------------------------------ */

    if (
        !validated_stream_buffer_is_empty(
            &buffer
        )
    )
    {
        printf("[FAIL] Empty flag\n");
        return 1;
    }

    PvExecutionSample dummy;

    if (
        validated_stream_buffer_pop(
            &buffer,
            &dummy
        )
    )
    {
        printf("[FAIL] Empty pop should fail\n");
        return 1;
    }

    if (buffer.underrun_count != 1U)
    {
        printf("[FAIL] Underrun counter\n");
        return 1;
    }

    printf("[PASS] Underrun detected correctly\n");


    /* ------------------------------------------------------------
     * Statistics
     * ------------------------------------------------------------ */

    printf("\nBuffer statistics:\n");
    printf(
        "  Capacity        = %zu samples\n",
        buffer.capacity
    );

    printf(
        "  High water mark = %zu samples\n",
        buffer.high_water_mark
    );

    printf(
        "  Total pushed    = %zu\n",
        buffer.total_pushed
    );

    printf(
        "  Total popped    = %zu\n",
        buffer.total_popped
    );

    printf(
        "  Underruns       = %zu\n",
        buffer.underrun_count
    );

    printf("\n[PASS] Test 6B.1\n");

    return 0;
}