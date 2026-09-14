#include "trajectory_buffer.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_CAPACITY 4

static void require_true(
    int condition,
    const char *message
)
{
    if (!condition)
    {
        printf("TEST FAILED: %s\n", message);
        exit(1);
    }
}

static JointVector make_sample(
    real_t base
)
{
    JointVector sample;

    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        sample.q[joint] =
            base +
            (real_t)joint;
    }

    return sample;
}

static void require_sample(
    const JointVector *sample,
    real_t expectedBase
)
{
    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        real_t expected =
            expectedBase +
            (real_t)joint;

        require_true(
            fabs(
                sample->q[joint] -
                expected
            ) < 1e-12,
            "sample contents are incorrect"
        );
    }
}

int main(void)
{
    JointVector storage[TEST_CAPACITY];

    TrajectoryBuffer buffer;

    require_true(
        trajectory_buffer_init(
            &buffer,
            storage,
            TEST_CAPACITY
        ),
        "init failed"
    );

    require_true(
        trajectory_buffer_is_empty(&buffer),
        "new buffer should be empty"
    );

    JointVector a = make_sample(10.0);
    JointVector b = make_sample(20.0);
    JointVector c = make_sample(30.0);
    JointVector d = make_sample(40.0);
    JointVector e = make_sample(50.0);
    JointVector out;

    require_true(
        trajectory_buffer_push(&buffer, &a),
        "push A failed"
    );

    require_true(
        trajectory_buffer_push(&buffer, &b),
        "push B failed"
    );

    require_true(
        trajectory_buffer_push(&buffer, &c),
        "push C failed"
    );

    require_true(
        trajectory_buffer_push(&buffer, &d),
        "push D failed"
    );

    require_true(
        trajectory_buffer_is_full(&buffer),
        "buffer should be full"
    );

    require_true(
        !trajectory_buffer_push(&buffer, &e),
        "push into full buffer should fail"
    );

    require_true(
        trajectory_buffer_pop(&buffer, &out),
        "pop A failed"
    );
    require_sample(&out, 10.0);

    require_true(
        trajectory_buffer_pop(&buffer, &out),
        "pop B failed"
    );
    require_sample(&out, 20.0);

    /*
     * Write index now wraps around. These two pushes verify that samples can
     * be written into slots that were already consumed.
     */
    require_true(
        trajectory_buffer_push(&buffer, &e),
        "wrap-around push E failed"
    );

    JointVector f = make_sample(60.0);

    require_true(
        trajectory_buffer_push(&buffer, &f),
        "wrap-around push F failed"
    );

    require_true(
        trajectory_buffer_pop(&buffer, &out),
        "pop C failed"
    );
    require_sample(&out, 30.0);

    require_true(
        trajectory_buffer_pop(&buffer, &out),
        "pop D failed"
    );
    require_sample(&out, 40.0);

    require_true(
        trajectory_buffer_pop(&buffer, &out),
        "pop E failed"
    );
    require_sample(&out, 50.0);

    require_true(
        trajectory_buffer_pop(&buffer, &out),
        "pop F failed"
    );
    require_sample(&out, 60.0);

    require_true(
        trajectory_buffer_is_empty(&buffer),
        "buffer should be empty after final pop"
    );

    require_true(
        !trajectory_buffer_pop(&buffer, &out),
        "pop from empty buffer should fail"
    );

    require_true(
        buffer.highWaterMark == TEST_CAPACITY,
        "high-water mark is incorrect"
    );

    require_true(
        buffer.totalPushed == 6,
        "totalPushed is incorrect"
    );

    require_true(
        buffer.totalPopped == 6,
        "totalPopped is incorrect"
    );

    printf("\nTRAJECTORY BUFFER TEST PASSED\n");
    printf("Capacity:        %zu samples\n", buffer.capacity);
    printf("High-water mark:%zu samples\n", buffer.highWaterMark);
    printf("Total pushed:    %zu\n", buffer.totalPushed);
    printf("Total popped:    %zu\n\n", buffer.totalPopped);

    return 0;
}
