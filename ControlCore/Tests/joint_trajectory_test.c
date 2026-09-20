#include "joint_trajectory.h"
#include "trajectory_buffer.h"
#include "robot_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


/* ============================================================================
 * TEST CONFIGURATION
 * ============================================================================
 */

#define TEST_BUFFER_SAMPLES 64U

#define DEG2RAD(x) \
    ((x) * ROBOT_PI / 180.0)


static JointVector executionStorage[
    TEST_BUFFER_SAMPLES
];


/* ============================================================================
 * TEST HELPERS
 * ============================================================================
 */

static void fail(
    const char *message
)
{
    printf(
        "\nTEST FAILED: %s\n",
        message
    );

    exit(1);
}


static real_t max_abs_joint_error(
    const JointVector *a,
    const JointVector *b
)
{
    real_t maxError =
        0.0;


    for (
        int joint = 0;
        joint < ROBOT_DOF;
        joint++
    )
    {
        real_t error =
            fabs(
                a->q[joint]
                -
                b->q[joint]
            );


        if (error > maxError)
        {
            maxError =
                error;
        }
    }


    return maxError;
}


/* ============================================================================
 * MAIN TEST
 * ============================================================================
 */

int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " JOINT-SPACE TRAJECTORY C TEST\n"
        "============================================================\n"
    );


    /* ========================================================================
     * 1. ROBOT CONFIGURATION
     * ========================================================================
     */

    RobotConfig robot;


    robot_config_init_ur5(
        &robot
    );


    /* ========================================================================
     * 2. SAME TEST CONFIGURATIONS USED IN MATLAB
     * ========================================================================
     */

    JointVector qStart =
    {
        .q =
        {
            DEG2RAD( 30.0),
            DEG2RAD(-45.0),
            DEG2RAD( 60.0),
            DEG2RAD( 20.0),
            DEG2RAD(-30.0),
            DEG2RAD( 45.0)
        }
    };


    JointVector qGoal =
    {
        .q =
        {
            DEG2RAD(  0.0),
            DEG2RAD(-90.0),
            DEG2RAD( 90.0),
            DEG2RAD(  0.0),
            DEG2RAD(  0.0),
            DEG2RAD(  0.0)
        }
    };


    const real_t T =
        5.0;


    const real_t dt =
        0.001;


    /* ========================================================================
     * 3. INITIALIZE JOINT TRAJECTORY
     * ========================================================================
     */

    JointTrajectory trajectory;


    if (
        !joint_trajectory_init(
            &trajectory,
            &qStart,
            &qGoal,
            T,
            dt
        )
    )
    {
        fail(
            "joint_trajectory_init() failed"
        );
    }


    size_t expectedSamples =
        joint_trajectory_sample_count(
            &trajectory
        );


    printf(
        "Profile:        %s\n",
        trajectory.info.type
    );

    printf(
        "Duration:       %.3f s\n",
        trajectory.info.duration
    );

    printf(
        "Samples:        %zu\n",
        expectedSamples
    );

    printf(
        "Sampling time:  %.4f s\n",
        trajectory.info.dt
    );


    /* ========================================================================
     * 4. SAMPLE COUNT
     * ========================================================================
     *
     * MATLAB:
     *
     *      T  = 5.0
     *      dt = 0.001
     *
     * gives:
     *
     *      5001 samples
     *
     * including:
     *
     *      t = 0
     *      ...
     *      t = 5
     */

    if (
        expectedSamples !=
        5001U
    )
    {
        printf(
            "Expected 5001 samples, got %zu\n",
            expectedSamples
        );

        fail(
            "sample count does not match MATLAB"
        );
    }


    /* ========================================================================
     * 5. ENDPOINT VALIDATION
     * ========================================================================
     */

    JointTrajectorySample firstSample;

    JointTrajectorySample finalSample;


    if (
        !joint_trajectory_evaluate_index(
            &trajectory,
            0,
            &firstSample
        )
    )
    {
        fail(
            "could not evaluate first sample"
        );
    }


    if (
        !joint_trajectory_evaluate_index(
            &trajectory,
            expectedSamples - 1U,
            &finalSample
        )
    )
    {
        fail(
            "could not evaluate final sample"
        );
    }


    real_t startError =
        max_abs_joint_error(
            &firstSample.q,
            &qStart
        );


    real_t goalError =
        max_abs_joint_error(
            &finalSample.q,
            &qGoal
        );


    printf(
        "\nEndpoint validation\n"
        "-------------------\n"
    );


    printf(
        "Start error: %.3e rad\n",
        startError
    );


    printf(
        "Goal error:  %.3e rad\n",
        goalError
    );


    const real_t endpointTolerance =
        1e-12;


    if (
        startError >
        endpointTolerance
    )
    {
        fail(
            "start configuration mismatch"
        );
    }


    if (
        goalError >
        endpointTolerance
    )
    {
        fail(
            "goal configuration mismatch"
        );
    }


    /* ========================================================================
     * 6. MIDPOINT CHECK
     * ========================================================================
     *
     * Quintic scaling:
     *
     *      s(T/2) = 0.5
     *
     * Therefore every joint should be exactly halfway between
     * qStart and qGoal.
     */

    JointTrajectorySample middleSample;


    if (
        !joint_trajectory_evaluate(
            &trajectory,
            T / 2.0,
            &middleSample
        )
    )
    {
        fail(
            "midpoint evaluation failed"
        );
    }


    if (
        fabs(
            middleSample.s -
            0.5
        ) >
        1e-12
    )
    {
        fail(
            "quintic midpoint s != 0.5"
        );
    }


    printf(
        "Midpoint s:   %.6f\n",
        middleSample.s
    );


    /* ========================================================================
     * 7. INITIALIZE EXISTING EXECUTION BUFFER
     * ========================================================================
     */

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


    /* ========================================================================
     * 8. STREAM TRAJECTORY THROUGH BUFFER
     * ========================================================================
     *
     * Architecture being simulated:
     *
     *      JointTrajectory
     *             ↓
     *      producer generates q[k]
     *             ↓
     *      TrajectoryBuffer
     *             ↓
     *      consumer removes one q[k]
     *             ↓
     *      future EtherCAT CSP cycle
     */

    size_t produced =
        0U;


    size_t consumed =
        0U;


    bool producerFinished =
        false;


    bool haveLastConsumed =
        false;


    JointVector lastConsumed =
        {0};


    bool positionViolation[
        ROBOT_DOF
    ] =
        {false};


    bool velocityViolation[
        ROBOT_DOF
    ] =
        {false};


    real_t measuredPeakVelocity[
        ROBOT_DOF
    ] =
        {0.0};


    while (
        !producerFinished ||
        !trajectory_buffer_is_empty(
            &buffer
        )
    )
    {
        /* ================================================================
         * PRODUCER
         * ================================================================
         *
         * Fill every currently available buffer slot.
         */

        while (
            !producerFinished &&
            !trajectory_buffer_is_full(
                &buffer
            )
        )
        {
            JointTrajectorySample generatedSample;


            if (
                joint_trajectory_next(
                    &trajectory,
                    &generatedSample
                )
            )
            {
                /* --------------------------------------------------------
                 * Validate every generated joint sample.
                 * --------------------------------------------------------
                 */

                for (
                    int joint = 0;
                    joint < ROBOT_DOF;
                    joint++
                )
                {
                    real_t q =
                        generatedSample.q.q[joint];


                    real_t qDot =
                        generatedSample.qDot.q[joint];


                    if (
                        q <
                        robot.limits.qMin[joint]
                        ||
                        q >
                        robot.limits.qMax[joint]
                    )
                    {
                        positionViolation[joint] =
                            true;
                    }


                    if (
                        fabs(qDot) >
                        robot.limits.qdMax[joint]
                    )
                    {
                        velocityViolation[joint] =
                            true;
                    }


                    if (
                        fabs(qDot) >
                        measuredPeakVelocity[joint]
                    )
                    {
                        measuredPeakVelocity[joint] =
                            fabs(qDot);
                    }
                }


                /*
                 * Only q is needed by the execution buffer.
                 *
                 * One JointVector =
                 * one future EtherCAT CSP cycle.
                 */

                if (
                    !trajectory_buffer_push(
                        &buffer,
                        &generatedSample.q
                    )
                )
                {
                    fail(
                        "trajectory buffer push failed"
                    );
                }


                produced++;
            }

            else
            {
                if (
                    joint_trajectory_is_finished(
                        &trajectory
                    )
                )
                {
                    producerFinished =
                        true;
                }

                else
                {
                    printf(
                        "Trajectory status: %s\n",
                        joint_trajectory_status_string(
                            joint_trajectory_status(
                                &trajectory
                            )
                        )
                    );


                    fail(
                        "trajectory generation stopped unexpectedly"
                    );
                }
            }
        }


        /* ================================================================
         * CONSUMER
         * ================================================================
         *
         * Simulate one EtherCAT cycle consuming one joint command.
         */

        if (
            !trajectory_buffer_is_empty(
                &buffer
            )
        )
        {
            JointVector command;


            if (
                !trajectory_buffer_pop(
                    &buffer,
                    &command
                )
            )
            {
                fail(
                    "trajectory buffer pop failed"
                );
            }


            lastConsumed =
                command;


            haveLastConsumed =
                true;


            consumed++;
        }
    }


    /* ========================================================================
     * 9. STREAMING VALIDATION
     * ========================================================================
     */

    if (!haveLastConsumed)
    {
        fail(
            "no samples were consumed"
        );
    }


    if (
        produced !=
        expectedSamples
    )
    {
        fail(
            "produced sample count mismatch"
        );
    }


    if (
        consumed !=
        expectedSamples
    )
    {
        fail(
            "consumed sample count mismatch"
        );
    }


    if (
        !trajectory_buffer_is_empty(
            &buffer
        )
    )
    {
        fail(
            "buffer not empty after execution"
        );
    }


    real_t finalConsumedError =
        max_abs_joint_error(
            &lastConsumed,
            &qGoal
        );


    if (
        finalConsumedError >
        endpointTolerance
    )
    {
        fail(
            "final buffered command does not equal qGoal"
        );
    }


    /* ========================================================================
     * 10. JOINT VALIDATION SUMMARY
     * ========================================================================
     */

    printf(
        "\nJoint validation\n"
        "----------------\n"
    );


    bool anyPositionViolation =
        false;


    bool anyVelocityViolation =
        false;


    for (
        int joint = 0;
        joint < ROBOT_DOF;
        joint++
    )
    {
        printf(
            "J%d | start = %7.2f deg | "
            "goal = %7.2f deg | "
            "peak velocity = %.4f rad/s | "
            "position violation = %d | "
            "velocity violation = %d\n",
            joint + 1,
            qStart.q[joint] *
                180.0 / ROBOT_PI,
            qGoal.q[joint] *
                180.0 / ROBOT_PI,
            measuredPeakVelocity[joint],
            positionViolation[joint],
            velocityViolation[joint]
        );


        if (positionViolation[joint])
        {
            anyPositionViolation =
                true;
        }


        if (velocityViolation[joint])
        {
            anyVelocityViolation =
                true;
        }
    }


    if (anyPositionViolation)
    {
        fail(
            "joint position limit violation detected"
        );
    }


    if (anyVelocityViolation)
    {
        fail(
            "joint velocity limit violation detected"
        );
    }


    /* ========================================================================
     * 11. FINAL RESULT
     * ========================================================================
     */

    printf(
        "\n"
        "============================================================\n"
        " JOINT-SPACE TRAJECTORY TEST PASSED\n"
        "============================================================\n"
    );


    printf(
        "Trajectory samples:      %zu\n",
        expectedSamples
    );


    printf(
        "Trajectory duration:     %.3f s\n",
        trajectory.info.duration
    );


    printf(
        "Execution buffer:        %u samples\n",
        (unsigned)TEST_BUFFER_SAMPLES
    );


    printf(
        "Buffer time at 1 ms:     %.3f s\n",
        (double)TEST_BUFFER_SAMPLES *
        dt
    );


    printf(
        "Produced:                %zu\n",
        produced
    );


    printf(
        "Consumed:                %zu\n",
        consumed
    );


    printf(
        "Buffer high-water mark:  %zu / %zu\n",
        buffer.highWaterMark,
        buffer.capacity
    );


    printf(
        "Final goal error:        %.3e rad\n",
        finalConsumedError
    );


    printf(
        "Joint position limits:   PASS\n"
        "Joint velocity limits:   PASS\n"
    );


    printf(
        "============================================================\n\n"
    );


    return 0;
}