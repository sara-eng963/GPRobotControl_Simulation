#ifndef SINGLE_SEGMENT_CIRCULAR_H
#define SINGLE_SEGMENT_CIRCULAR_H


#include "../Config/robot_config.h"
#include "../Math/control_types.h"
#include "../Kinematics/adls_ik.h"
#include "../Trajectory/circular_path.h"
#include "../Trajectory/s_curve_time_scaling.h"

#include <stdbool.h>
#include <stddef.h>


/* ============================================================================
 * SINGLE-SEGMENT CIRCULAR CARTESIAN PIPELINE
 * ============================================================================
 *
 * Shared implementation for:
 *
 *      1. Circular arc through three taught points
 *      2. Full 360-degree circle through three taught points
 *
 * Pipeline:
 *
 *      taught Cartesian points
 *          -> circular geometry
 *          -> arc-length parameterization
 *          -> S-curve time scaling
 *          -> path-induced orientation + leftover twist
 *          -> sequential ADLS IK
 *          -> independent FK validation
 *          -> joint velocity / acceleration
 *          -> joint-limit validation
 * ============================================================================
 */


typedef enum
{
    CIRCULAR_SEGMENT_ARC = 0,
    CIRCULAR_SEGMENT_FULL_CIRCLE = 1

} CircularSegmentType;


typedef struct
{
    CircularSegmentType type;

    /* Initial ADLS seed only. Does not define a Cartesian waypoint. */
    JointVector qSeed;

    /*
     * ARC:
     *      point1 = start
     *      point2 = mid point that the arc MUST pass through
     *      point3 = end
     *
     * FULL CIRCLE:
     *      point1 = start/end
     *      point2, point3 = additional points defining the circle
     */
    Vec3 point1;
    Vec3 point2;
    Vec3 point3;

    /* Used only for a full circle: +1 CCW, -1 CW about fitted axis. */
    int direction;

    /* Absolute orientations in the robot base frame. */
    Quat startOrientation;
    Quat endOrientation;

    size_t numGeometryPointsPerSegment;
    real_t arcLengthSpacing;

    real_t desiredTCPSpeed;
    real_t desiredTCPAccel;
    real_t desiredTCPJerk;

    real_t dt;

    ADLSParameters ikParameters;

} SingleCircularRequest;


/* ============================================================================
 * OUTPUT TRAJECTORY
 * ============================================================================
 */

typedef struct
{
    size_t capacity;
    size_t count;

    real_t *t;
    real_t *s;
    real_t *sDot;
    real_t *sDDot;
    real_t *sDDDot;

    real_t *arcPosition;
    real_t *tcpSpeed;

    Vec3 *pDesired;
    Quat *quatDesired;

    JointVector *qPath;
    JointVector *qDot;
    JointVector *qDDot;

    int *ikIterations;

    real_t *positionError;
    real_t *orientationError;

} SingleCircularTrajectory;


/* ============================================================================
 * WORKSPACE
 * ============================================================================
 */

typedef struct
{
    size_t geometryCapacity;

    Vec3 *rawGeometry;
    Vec3 *arcGeometry;
    real_t *lOriginal;
    real_t *lArc;

    size_t timeCapacity;

    real_t *tempT;
    real_t *tempS;
    real_t *tempSDot;
    real_t *tempSDDot;
    real_t *tempSDDDot;

    ADLSInfo *ikScratch;

} SingleCircularWorkspace;


/* ============================================================================
 * REPORT
 * ============================================================================
 */

typedef struct
{
    bool success;

    CircularSegmentType type;

    Vec3 point1;
    Vec3 point2;
    Vec3 point3;

    CircularPathInfo geometry;

    size_t samples;
    size_t rawGeometryPoints;
    size_t arcGeometryPoints;

    real_t pathLength;
    real_t duration;
    real_t peakTCPSpeed;

    real_t maxPositionError;
    real_t maxOrientationError;
    int maxIKIterations;

    bool positionLimitsPass;
    bool velocityLimitsPass;

    real_t peakJointVelocity[ROBOT_DOF];
    real_t maxJointStep[ROBOT_DOF];

    SCurveInfo profile;

} SingleCircularReport;


/* ============================================================================
 * MATLAB REFERENCE REQUESTS
 * ============================================================================
 */

SingleCircularRequest single_arc_matlab_reference_request(void);

SingleCircularRequest single_full_circle_matlab_reference_request(void);


/* ============================================================================
 * PLANNERS
 * ============================================================================
 */

bool plan_single_segment_arc(
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularWorkspace *workspace,
    SingleCircularTrajectory *trajectory,
    SingleCircularReport *report
);


bool plan_single_segment_full_circle(
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularWorkspace *workspace,
    SingleCircularTrajectory *trajectory,
    SingleCircularReport *report
);


#endif /* SINGLE_SEGMENT_CIRCULAR_H */