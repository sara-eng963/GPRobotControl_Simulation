#ifndef SINGLE_SEGMENT_LINE_H
#define SINGLE_SEGMENT_LINE_H


#include "../Config/robot_config.h"
#include "../Math/control_types.h"
#include "../Kinematics/adls_ik.h"
#include "../Trajectory/s_curve_time_scaling.h"

#include <stdbool.h>
#include <stddef.h>


/* ============================================================================
 * SINGLE-SEGMENT STRAIGHT-LINE CARTESIAN PIPELINE
 * ============================================================================
 *
 * C equivalent of:
 *
 *      SingleSegmentTest_line.m
 *
 *
 * Pipeline:
 *
 *     Absolute Waypoint A + Waypoint B
    |
    v
 *          |
 *          v
 *      Straight geometric path
 *          |
 *          v
 *      Arc-length parameterization
 *          |
 *          v
 *      S-curve time scaling
 *          |
 *          v
 *      Cartesian position + orientation SLERP
 *          |
 *          v
 *      Desired TCP pose T(t)
 *          |
 *          v
 *      Sequential ADLS IK
 *          |
 *          v
 *      q(t)
 *          |
 *          v
 *      Independent FK validation
 *          |
 *          v
 *      Joint velocity / acceleration
 *          |
 *          v
 *      Joint-limit validation
 *
 * ============================================================================
 */


/* ============================================================================
 * PIPELINE REQUEST
 * ============================================================================
 *
 * Defines one absolute Cartesian line motion:
 *
 *      Waypoint A
 *          ->
 *      Waypoint B
 *
 * Both waypoint positions and orientations are expressed absolutely
 * in the robot BASE frame.
 *
 * The starting joint vector is NOT used to define waypoint A.
 *
 * qSeed is used only as the initial seed for the first ADLS IK solution.
 * Every later IK sample is seeded by the previous successful solution.
 *
 * ============================================================================
 */

typedef struct
{
    /* ------------------------------------------------------------------------
     * Initial IK seed [rad]
     * ------------------------------------------------------------------------
     *
     * This does NOT define waypoint A.
     *
     * It only selects/helps converge to the desired IK solution for the first
     * Cartesian trajectory sample.
     */

    JointVector qSeed;


    /* ------------------------------------------------------------------------
     * ABSOLUTE CARTESIAN WAYPOINT A
     * ------------------------------------------------------------------------
     *
     * Expressed in the robot base frame.
     */

    Vec3 startPosition;          /* [m] */

    Quat startOrientation;      /* unit quaternion [w x y z] */


    /* ------------------------------------------------------------------------
     * ABSOLUTE CARTESIAN WAYPOINT B
     * ------------------------------------------------------------------------
     *
     * Expressed in the robot base frame.
     */

    Vec3 endPosition;            /* [m] */

    Quat endOrientation;        /* unit quaternion [w x y z] */


    /* ------------------------------------------------------------------------
     * Geometry settings
     * ------------------------------------------------------------------------
     */

    size_t numGeometryPointsPerSegment;

    real_t arcLengthSpacing;


    /* ------------------------------------------------------------------------
     * TCP motion limits
     * ------------------------------------------------------------------------
     *
     * Physical Cartesian limits:
     *
     *      desiredTCPSpeed   [m/s]
     *      desiredTCPAccel   [m/s^2]
     *      desiredTCPJerk    [m/s^3]
     */

    real_t desiredTCPSpeed;

    real_t desiredTCPAccel;

    real_t desiredTCPJerk;


    /* ------------------------------------------------------------------------
     * Time sampling period [s]
     * ------------------------------------------------------------------------
     */

    real_t dt;


    /* ------------------------------------------------------------------------
     * ADLS parameters
     * ------------------------------------------------------------------------
     */

    ADLSParameters ikParameters;

} SingleLineRequest;

/* ============================================================================
 * COMPLETE GENERATED TRAJECTORY
 * ============================================================================
 *
 * All arrays are supplied by the caller.
 *
 * This avoids dynamic memory allocation inside ControlCore.
 * ============================================================================
 */

typedef struct
{
    /*
     * Maximum number of samples the supplied buffers can contain.
     */
    size_t capacity;


    /*
     * Actual generated number of samples.
     */
    size_t count;


    /* ------------------------------------------------------------------------
     * Time scaling
     * ------------------------------------------------------------------------
     */

    real_t *t;

    real_t *s;

    real_t *sDot;

    real_t *sDDot;

    real_t *sDDDot;


    /* ------------------------------------------------------------------------
     * Physical path progress
     * ------------------------------------------------------------------------
     */

    real_t *arcPosition;

    real_t *tcpSpeed;


    /* ------------------------------------------------------------------------
     * Desired Cartesian trajectory
     * ------------------------------------------------------------------------
     */

    Vec3 *pDesired;

    Quat *quatDesired;


    /* ------------------------------------------------------------------------
     * Joint trajectory from sequential ADLS
     * ------------------------------------------------------------------------
     */

    JointVector *qPath;

    JointVector *qDot;

    JointVector *qDDot;


    /* ------------------------------------------------------------------------
     * IK diagnostics
     * ------------------------------------------------------------------------
     */

    int *ikIterations;


    /* ------------------------------------------------------------------------
     * Independent FK validation
     * ------------------------------------------------------------------------
     */

    real_t *positionError;

    real_t *orientationError;

} SingleLineTrajectory;


/* ============================================================================
 * PIPELINE WORKSPACE
 * ============================================================================
 *
 * Temporary memory used while planning.
 *
 * The caller owns this memory.
 *
 * This is intentionally separate from SingleLineTrajectory because most of
 * these arrays are only needed during trajectory generation.
 * ============================================================================
 */

typedef struct
{
    /* ------------------------------------------------------------------------
     * Geometry workspace
     * ------------------------------------------------------------------------
     */

    size_t geometryCapacity;


    /*
     * MATLAB:
     *
     *      rawGeometryPath
     */
    Vec3 *rawGeometry;


    /*
     * MATLAB:
     *
     *      arcGeometryPath
     */
    Vec3 *arcGeometry;


    /*
     * MATLAB:
     *
     *      lOriginal
     */
    real_t *lOriginal;


    /*
     * MATLAB:
     *
     *      lArc
     */
    real_t *lArc;


    /* ------------------------------------------------------------------------
     * Temporary S-curve workspace
     * ------------------------------------------------------------------------
     */

    size_t timeCapacity;

    real_t *tempT;

    real_t *tempS;

    real_t *tempSDot;

    real_t *tempSDDot;

    real_t *tempSDDDot;


    /* ------------------------------------------------------------------------
     * ADLS scratch memory
     * ------------------------------------------------------------------------
     *
     * ADLSInfo contains diagnostic histories and is intentionally not placed
     * on the function stack.
     *
     * This is especially important later under FreeRTOS / STM32.
     */

    ADLSInfo *ikScratch;

} SingleLineWorkspace;


/* ============================================================================
 * PIPELINE REPORT
 * ============================================================================
 *
 * Equivalent to the diagnostic summaries printed by the MATLAB test.
 * ============================================================================
 */

typedef struct
{
    bool success;


    /* ------------------------------------------------------------------------
     * Waypoints
     * ------------------------------------------------------------------------
     */

    Vec3 waypointA;

    Vec3 waypointB;


    /* ------------------------------------------------------------------------
     * Geometry / time information
     * ------------------------------------------------------------------------
     */

    size_t samples;

    size_t rawGeometryPoints;

    size_t arcGeometryPoints;

    real_t pathLength;

    real_t duration;

    real_t peakTCPSpeed;


    /* ------------------------------------------------------------------------
     * IK / FK validation
     * ------------------------------------------------------------------------
     */

    real_t maxPositionError;

    real_t maxOrientationError;

    int maxIKIterations;


    /* ------------------------------------------------------------------------
     * Joint validation
     * ------------------------------------------------------------------------
     */

    bool positionLimitsPass;

    bool velocityLimitsPass;

    real_t peakJointVelocity[ROBOT_DOF];

    real_t maxJointStep[ROBOT_DOF];


    /* ------------------------------------------------------------------------
     * S-curve information
     * ------------------------------------------------------------------------
     */

    SCurveInfo profile;

} SingleLineReport;


/* ============================================================================
 * MATLAB REFERENCE TEST REQUEST
 * ============================================================================
 *
 * Returns the same test parameters currently used by:
 *
 *      SingleSegmentTest_line.m
 *
 * qStart:
 *
 *      [ 30
 *       -45
 *        60
 *        20
 *       -30
 *        45 ] deg
 *
 * displacement:
 *
 *      [0.50, 0, 0] m
 *
 * orientation:
 *
 *      yaw   = 15 deg
 *      pitch = 15 deg
 *      roll  = 15 deg
 *
 * geometry points:
 *
 *      100
 *
 * arc spacing:
 *
 *      0.005 m
 *
 * TCP limits:
 *
 *      0.10 m/s
 *      0.25 m/s^2
 *      1.00 m/s^3
 *
 * dt:
 *
 *      0.05 s
 *
 * ============================================================================
 */

SingleLineRequest single_line_matlab_reference_request(void);


/* ============================================================================
 * PLAN SINGLE STRAIGHT-LINE SEGMENT
 * ============================================================================
 *
 * Returns:
 *
 *      true
 *          complete trajectory successfully generated and validated
 *
 *      false
 *          invalid input, insufficient buffer capacity, IK failure, or other
 *          planning failure
 *
 * ============================================================================
 */

bool plan_single_segment_line(
    const RobotConfig *robot,
    const SingleLineRequest *request,
    SingleLineWorkspace *workspace,
    SingleLineTrajectory *trajectory,
    SingleLineReport *report
);


#endif /* SINGLE_SEGMENT_LINE_H */