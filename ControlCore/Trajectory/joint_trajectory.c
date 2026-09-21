#include "joint_trajectory.h"

#include <math.h>
#include <string.h>


static void set_error(
    JointTrajectory *trajectory,
    JointTrajectoryStatus status
)
{
    if (trajectory == NULL)
    {
        return;
    }


    trajectory->status =
        status;
}


bool joint_trajectory_init(
    JointTrajectory *trajectory,
    const JointVector *qStart,
    const JointVector *qGoal,
    real_t T,
    real_t dt
)
{
    if (
        trajectory == NULL ||
        qStart == NULL ||
        qGoal == NULL ||
        T <= 0.0 ||
        dt <= 0.0
    )
    {
        return false;
    }


    memset(
        trajectory,
        0,
        sizeof(*trajectory)
    );


    trajectory->status =
        JOINT_TRAJECTORY_INVALID_ARGUMENT;


    /* ========================================================================
     * COPY START / GOAL CONFIGURATIONS
     * ========================================================================
     */

    trajectory->qStart =
        *qStart;

    trajectory->qGoal =
        *qGoal;


    /* ========================================================================
     * JOINT DISPLACEMENT
     * ========================================================================
     *
     * MATLAB:
     *
     *      deltaQ =
     *          qGoal - qStart;
     */

    for (
        int joint = 0;
        joint < ROBOT_DOF;
        joint++
    )
    {
        trajectory->deltaQ.q[joint] =
            qGoal->q[joint]
            -
            qStart->q[joint];
    }


    /* ========================================================================
     * QUINTIC TIME SCALING
     * ========================================================================
     */

    if (
        !quintic_time_scaling_init(
            T,
            dt,
            &trajectory->profile
        )
    )
    {
        set_error(
            trajectory,
            JOINT_TRAJECTORY_PROFILE_FAILED
        );

        return false;
    }


    /* ========================================================================
     * INFORMATION
     * ========================================================================
     */

    trajectory->info.type =
        "Joint-Space Quintic";

    trajectory->info.duration =
        T;

    trajectory->info.dt =
        dt;

    trajectory->info.samples =
        quintic_time_scaling_sample_count(
            &trajectory->profile
        );

    trajectory->info.qStart =
        trajectory->qStart;

    trajectory->info.qGoal =
        trajectory->qGoal;

    trajectory->info.deltaQ =
        trajectory->deltaQ;


    /*
     * MATLAB:
     *
     * peakJointVelocity =
     *      max(abs(qDot), [], 1)
     *
     * Since:
     *
     *      qDot = deltaQ * sDot
     *
     * the maximum is:
     *
     *      |deltaQ| * max(|sDot|)
     */

    for (
        int joint = 0;
        joint < ROBOT_DOF;
        joint++
    )
    {
        trajectory->info.peakJointVelocity.q[joint] =
            fabs(
                trajectory->deltaQ.q[joint]
            )
            *
            trajectory->profile.info.peakPathVelocity;


        trajectory->info.peakJointAcceleration.q[joint] =
            fabs(
                trajectory->deltaQ.q[joint]
            )
            *
            trajectory->profile.info.peakPathAcceleration;
    }


    trajectory->nextSampleIndex =
        0U;

    trajectory->initialized =
        true;

    trajectory->finished =
        false;

    trajectory->status =
        JOINT_TRAJECTORY_OK;


    return true;
}


bool joint_trajectory_evaluate(
    const JointTrajectory *trajectory,
    real_t time,
    JointTrajectorySample *sample
)
{
    if (
        trajectory == NULL ||
        sample == NULL ||
        !trajectory->initialized
    )
    {
        return false;
    }


    QuinticSample quinticSample;


    if (
        !quintic_time_scaling_evaluate(
            &trajectory->profile,
            time,
            &quinticSample
        )
    )
    {
        return false;
    }


    memset(
        sample,
        0,
        sizeof(*sample)
    );


    sample->t =
        quinticSample.t;

    sample->s =
        quinticSample.s;

    sample->sDot =
        quinticSample.sDot;

    sample->sDDot =
        quinticSample.sDDot;

    sample->sDDDot =
        quinticSample.sDDDot;


    /* ========================================================================
     * JOINT-SPACE TRAJECTORY
     * ========================================================================
     *
     * MATLAB:
     *
     *      q =
     *          qStart.'
     *          +
     *          s.' * deltaQ.';
     *
     *      qDot =
     *          sDot.' * deltaQ.';
     *
     *      qDDot =
     *          sDDot.' * deltaQ.';
     *
     *      qDDDot =
     *          sDDDot.' * deltaQ.';
     */

    for (
        int joint = 0;
        joint < ROBOT_DOF;
        joint++
    )
    {
        real_t delta =
            trajectory->deltaQ.q[joint];


        sample->q.q[joint] =
            trajectory->qStart.q[joint]
            +
            delta *
            quinticSample.s;


        sample->qDot.q[joint] =
            delta *
            quinticSample.sDot;


        sample->qDDot.q[joint] =
            delta *
            quinticSample.sDDot;


        sample->qDDDot.q[joint] =
            delta *
            quinticSample.sDDDot;
    }


    /* ========================================================================
     * FORCE EXACT MATLAB ENDPOINTS
     * ========================================================================
     *
     * MATLAB:
     *
     *      q(1,:)   = qStart.';
     *      q(end,:) = qGoal.';
     *
     *      qDot(1,:)   = 0;
     *      qDot(end,:) = 0;
     *
     *      qDDot(1,:)   = 0;
     *      qDDot(end,:) = 0;
     */

    if (quinticSample.t <= 0.0)
    {
        sample->q =
            trajectory->qStart;


        for (
            int joint = 0;
            joint < ROBOT_DOF;
            joint++
        )
        {
            sample->qDot.q[joint] =
                0.0;

            sample->qDDot.q[joint] =
                0.0;
        }
    }


    if (
        quinticSample.t >=
        trajectory->profile.info.T
    )
    {
        sample->q =
            trajectory->qGoal;


        for (
            int joint = 0;
            joint < ROBOT_DOF;
            joint++
        )
        {
            sample->qDot.q[joint] =
                0.0;

            sample->qDDot.q[joint] =
                0.0;
        }
    }


    return true;
}


bool joint_trajectory_evaluate_index(
    const JointTrajectory *trajectory,
    size_t sampleIndex,
    JointTrajectorySample *sample
)
{
    if (
        trajectory == NULL ||
        sample == NULL ||
        !trajectory->initialized
    )
    {
        return false;
    }


    real_t time;


    if (
        !quintic_time_scaling_sample_time(
            &trajectory->profile,
            sampleIndex,
            &time
        )
    )
    {
        return false;
    }


    if (
        !joint_trajectory_evaluate(
            trajectory,
            time,
            sample
        )
    )
    {
        return false;
    }


    sample->index =
        sampleIndex;


    return true;
}


bool joint_trajectory_next(
    JointTrajectory *trajectory,
    JointTrajectorySample *sample
)
{
    if (
        trajectory == NULL ||
        sample == NULL ||
        !trajectory->initialized
    )
    {
        if (trajectory != NULL)
        {
            set_error(
                trajectory,
                JOINT_TRAJECTORY_INVALID_ARGUMENT
            );
        }

        return false;
    }


    size_t totalSamples =
        joint_trajectory_sample_count(
            trajectory
        );


    if (
        trajectory->nextSampleIndex >=
        totalSamples
    )
    {
        trajectory->finished =
            true;

        trajectory->status =
            JOINT_TRAJECTORY_FINISHED;

        return false;
    }


    if (
        !joint_trajectory_evaluate_index(
            trajectory,
            trajectory->nextSampleIndex,
            sample
        )
    )
    {
        set_error(
            trajectory,
            JOINT_TRAJECTORY_PROFILE_FAILED
        );

        return false;
    }


    trajectory->nextSampleIndex++;


    if (
        trajectory->nextSampleIndex >=
        totalSamples
    )
    {
        trajectory->finished =
            true;
    }


    trajectory->status =
        JOINT_TRAJECTORY_OK;


    return true;
}


size_t joint_trajectory_sample_count(
    const JointTrajectory *trajectory
)
{
    if (
        trajectory == NULL ||
        !trajectory->initialized
    )
    {
        return 0;
    }


    return
        quintic_time_scaling_sample_count(
            &trajectory->profile
        );
}


size_t joint_trajectory_samples_generated(
    const JointTrajectory *trajectory
)
{
    if (
        trajectory == NULL ||
        !trajectory->initialized
    )
    {
        return 0;
    }


    return
        trajectory->nextSampleIndex;
}


size_t joint_trajectory_samples_remaining(
    const JointTrajectory *trajectory
)
{
    if (
        trajectory == NULL ||
        !trajectory->initialized
    )
    {
        return 0;
    }


    size_t total =
        joint_trajectory_sample_count(
            trajectory
        );


    if (
        trajectory->nextSampleIndex >=
        total
    )
    {
        return 0;
    }


    return
        total
        -
        trajectory->nextSampleIndex;
}


bool joint_trajectory_is_finished(
    const JointTrajectory *trajectory
)
{
    return
        trajectory != NULL &&
        trajectory->initialized &&
        trajectory->finished;
}


JointTrajectoryStatus joint_trajectory_status(
    const JointTrajectory *trajectory
)
{
    if (trajectory == NULL)
    {
        return
            JOINT_TRAJECTORY_INVALID_ARGUMENT;
    }


    return
        trajectory->status;
}


const char *joint_trajectory_status_string(
    JointTrajectoryStatus status
)
{
    switch (status)
    {
        case JOINT_TRAJECTORY_OK:
            return "OK";


        case JOINT_TRAJECTORY_FINISHED:
            return "FINISHED";


        case JOINT_TRAJECTORY_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";


        case JOINT_TRAJECTORY_PROFILE_FAILED:
            return "PROFILE_FAILED";


        default:
            return "UNKNOWN";
    }
}