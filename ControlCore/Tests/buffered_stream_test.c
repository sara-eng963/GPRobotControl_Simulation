#include "single_segment_line.h"
#include "single_segment_line_stream.h"
#include "trajectory_buffer.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * Intentionally tiny execution buffer.
 *
 * 64 samples at dt = 1 ms = only 64 ms of queued motion.
 *
 * The test trajectory is > 10 seconds long, so success proves that total
 * trajectory duration is no longer tied to execution-buffer capacity.
 */
#define TEST_BUFFER_SAMPLES 64U
#define GEOMETRY_CAPACITY   512U

static JointVector executionStorage[TEST_BUFFER_SAMPLES];

static Vec3 rawGeometry[GEOMETRY_CAPACITY];
static Vec3 arcGeometry[GEOMETRY_CAPACITY];

static real_t lOriginal[GEOMETRY_CAPACITY];
static real_t lArc[GEOMETRY_CAPACITY];

static ADLSInfo ikScratch;


static void fail(
    const char *message
)
{
    printf("TEST FAILED: %s\n", message);
    exit(1);
}


int main(void)
{
    RobotConfig robot;

    robot_config_init_ur5(
        &robot
    );


    /*
     * Start from the already validated reference Cartesian path, but halve the
     * TCP speed. This keeps the exact same reachable A -> B geometry while
     * deliberately making the trajectory much longer than the old 7000-sample
     * full-array limit.
     */
    SingleLineRequest request =
        single_line_matlab_reference_request();

    request.desiredTCPSpeed =
        0.05;

    request.dt =
        0.001;


    SingleLineStreamWorkspace workspace;

    memset(
        &workspace,
        0,
        sizeof(workspace)
    );

    workspace.geometryCapacity =
        GEOMETRY_CAPACITY;

    workspace.rawGeometry =
        rawGeometry;

    workspace.arcGeometry =
        arcGeometry;

    workspace.lOriginal =
        lOriginal;

    workspace.lArc =
        lArc;

    workspace.ikScratch =
        &ikScratch;


    SingleLineStream stream;

    if (
        !single_line_stream_init(
            &stream,
            &robot,
            &request,
            &workspace
        )
    )
    {
        printf(
            "Stream init status: %s\n",
            single_line_stream_status_string(
                single_line_stream_status(
                    &stream
                )
            )
        );

        fail(
            "single-line stream initialization failed"
        );
    }


    const size_t expectedSamples =
        single_line_stream_sample_count(
            &stream
        );


    if (expectedSamples <= 7000U)
    {
        printf(
            "Expected >7000 samples, got %zu\n",
            expectedSamples
        );

        fail(
            "test trajectory is not long enough"
        );
    }


    TrajectoryBuffer buffer;

    if (
        !trajectory_buffer_init(
            &buffer,
            executionStorage,
            TEST_BUFFER_SAMPLES
        )
    )
    {
        fail(
            "trajectory buffer initialization failed"
        );
    }


    size_t produced =
        0;

    size_t consumed =
        0;

    bool producerFinished =
        false;

    bool haveLastConsumed =
        false;

    JointVector lastConsumed;


    /*
     * Simulate the architecture:
     *
     *      planner -> fills available buffer space
     *      executor -> consumes exactly one q sample
     *
     * The same 64 slots are reused thousands of times.
     */
    while (
        !producerFinished ||
        !trajectory_buffer_is_empty(
            &buffer
        )
    )
    {
        /*
         * Producer side:
         *
         * Fill every currently available slot.
         */
        while (
            !producerFinished &&
            !trajectory_buffer_is_full(
                &buffer
            )
        )
        {
            SingleLineStreamSample generatedSample;

            if (
                single_line_stream_next(
                    &stream,
                    &generatedSample
                )
            )
            {
                if (
                    !trajectory_buffer_push(
                        &buffer,
                        &generatedSample.q
                    )
                )
                {
                    fail(
                        "buffer push failed unexpectedly"
                    );
                }

                produced++;
            }
            else
            {
                if (
                    single_line_stream_is_finished(
                        &stream
                    )
                )
                {
                    producerFinished =
                        true;
                }
                else
                {
                    printf(
                        "Producer stopped with status: %s\n",
                        single_line_stream_status_string(
                            single_line_stream_status(
                                &stream
                            )
                        )
                    );

                    fail(
                        "trajectory generation failed before completion"
                    );
                }
            }
        }


        /*
         * Consumer side:
         *
         * One queued q sample = one simulated 1 ms EtherCAT execution cycle.
         */
        if (
            !trajectory_buffer_is_empty(
                &buffer
            )
        )
        {
            JointVector sample;

            if (
                !trajectory_buffer_pop(
                    &buffer,
                    &sample
                )
            )
            {
                fail(
                    "buffer pop failed unexpectedly"
                );
            }

            lastConsumed =
                sample;

            haveLastConsumed =
                true;

            consumed++;
        }
    }


    if (!haveLastConsumed)
    {
        fail(
            "no trajectory sample was consumed"
        );
    }


    if (
        produced !=
        expectedSamples
    )
    {
        printf(
            "Produced: %zu\n"
            "Expected: %zu\n",
            produced,
            expectedSamples
        );

        fail(
            "producer count mismatch"
        );
    }


    if (
        consumed !=
        expectedSamples
    )
    {
        printf(
            "Consumed: %zu\n"
            "Expected: %zu\n",
            consumed,
            expectedSamples
        );

        fail(
            "consumer count mismatch"
        );
    }


    if (
        !trajectory_buffer_is_empty(
            &buffer
        )
    )
    {
        fail(
            "buffer should be empty after complete execution"
        );
    }


    /*
     * The final q only exists to make sure the compiler/test really consumed
     * the trajectory all the way to the end.
     */
    real_t finalJointMagnitude =
        0.0;

    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        finalJointMagnitude +=
            fabs(
                lastConsumed.q[joint]
            );
    }


    if (
        !isfinite(
            finalJointMagnitude
        )
    )
    {
        fail(
            "final joint sample is invalid"
        );
    }


    printf(
        "\nBUFFERED STREAMING TRAJECTORY TEST PASSED\n"
    );

    printf(
        "Trajectory samples:     %zu\n",
        expectedSamples
    );

    printf(
        "Trajectory duration:    %.6f s\n",
        stream.profile.info.T
    );

    printf(
        "Execution buffer:       %u samples\n",
        (unsigned)TEST_BUFFER_SAMPLES
    );

    printf(
        "Buffer time at 1 ms:    %.3f s\n",
        (double)TEST_BUFFER_SAMPLES *
        0.001
    );

    printf(
        "Produced:               %zu\n",
        produced
    );

    printf(
        "Consumed:               %zu\n",
        consumed
    );

    printf(
        "Buffer high-water mark: %zu / %zu\n",
        buffer.highWaterMark,
        buffer.capacity
    );

    printf(
        "Final buffer count:     %zu\n\n",
        trajectory_buffer_count(
            &buffer
        )
    );


    return 0;
}
