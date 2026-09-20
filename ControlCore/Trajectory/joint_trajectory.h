#ifndef JOINT_TRAJECTORY_H
#define JOINT_TRAJECTORY_H


#include "quintic_time_scaling.h"

#include "../Math/control_types.h"

#include <stdbool.h>
#include <stddef.h>


/* ============================================================================
 * JOINT TRAJECTORY STATUS
 * ============================================================================
 */

typedef enum
{
    JOINT_TRAJECTORY_OK = 0,
    JOINT_TRAJECTORY_FINISHED,

    JOINT_TRAJECTORY_INVALID_ARGUMENT,
    JOINT_TRAJECTORY_PROFILE_FAILED

} JointTrajectoryStatus;


/* ============================================================================
 * JOINT TRAJECTORY INFORMATION
 * ============================================================================
 *
 * C equivalent of MATLAB generateJointTrajectory() info output.
 * ============================================================================
 */

typedef struct
{
    const char *type;

    real_t duration;
    real_t dt;

    size_t samples;

    JointVector qStart;
    JointVector qGoal;
    JointVector deltaQ;

    JointVector peakJointVelocity;
    JointVector peakJointAcceleration;

} JointTrajectoryInfo;


/* ============================================================================
 * ONE STREAMING JOINT SAMPLE
 * ============================================================================
 */

typedef struct
{
    size_t index;

    real_t t;

    real_t s;
    real_t sDot;
    real_t sDDot;
    real_t sDDDot;

    JointVector q;
    JointVector qDot;
    JointVector qDDot;
    JointVector qDDDot;

} JointTrajectorySample;


/* ============================================================================
 * JOINT TRAJECTORY
 * ============================================================================
 */

typedef struct
{
    bool initialized;
    bool finished;

    JointTrajectoryStatus status;

    QuinticProfile profile;

    JointVector qStart;
    JointVector qGoal;
    JointVector deltaQ;

    size_t nextSampleIndex;

    JointTrajectoryInfo info;

} JointTrajectory;


/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

/*
 * Initialize one synchronized joint-space point-to-point trajectory.
 *
 * All joints share the same quintic progress variable s(t), therefore:
 *
 *      q(t) =
 *          qStart +
 *          deltaQ * s(t)
 */
bool joint_trajectory_init(
    JointTrajectory *trajectory,
    const JointVector *qStart,
    const JointVector *qGoal,
    real_t T,
    real_t dt
);


/*
 * Evaluate the trajectory at any timestamp.
 */
bool joint_trajectory_evaluate(
    const JointTrajectory *trajectory,
    real_t time,
    JointTrajectorySample *sample
);


/*
 * Evaluate a specific discrete trajectory sample.
 */
bool joint_trajectory_evaluate_index(
    const JointTrajectory *trajectory,
    size_t sampleIndex,
    JointTrajectorySample *sample
);


/*
 * Streaming interface.
 *
 * Generates exactly ONE next trajectory sample.
 *
 * This is the function a planner/buffer producer will normally use.
 */
bool joint_trajectory_next(
    JointTrajectory *trajectory,
    JointTrajectorySample *sample
);


size_t joint_trajectory_sample_count(
    const JointTrajectory *trajectory
);


size_t joint_trajectory_samples_generated(
    const JointTrajectory *trajectory
);


size_t joint_trajectory_samples_remaining(
    const JointTrajectory *trajectory
);


bool joint_trajectory_is_finished(
    const JointTrajectory *trajectory
);


JointTrajectoryStatus joint_trajectory_status(
    const JointTrajectory *trajectory
);


const char *joint_trajectory_status_string(
    JointTrajectoryStatus status
);


#endif /* JOINT_TRAJECTORY_H */