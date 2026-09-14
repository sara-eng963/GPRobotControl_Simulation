#include "single_segment_line.h"


#include "../Math/math3d.h"

#include "../Kinematics/control_fk.h"

#include "../Trajectory/generate_line_waypoints.h"
#include "../Trajectory/arc_length_parameterize.h"
#include "../Trajectory/s_curve_time_scaling.h"


#include <math.h>
#include <stddef.h>
#include <string.h>


/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================
 */


/*
 * Degrees -> radians.
 */
static real_t degrees_to_radians(
    real_t degrees
)
{
    return
        degrees *
        ROBOT_PI /
        180.0;
}


/* ============================================================================
 * RAW 4x4 ARRAY -> Mat4
 * ============================================================================
 */

static Mat4 array_to_mat4(
    const double input[4][4]
)
{
    Mat4 result;


    for (int row = 0;
         row < 4;
         row++)
    {
        for (int column = 0;
             column < 4;
             column++)
        {
            result.m[row][column] =
                input[row][column];
        }
    }


    return result;
}


/* ============================================================================
 * Mat4 -> RAW 4x4 ARRAY
 * ============================================================================
 */

static void mat4_to_array(
    Mat4 input,
    double output[4][4]
)
{
    for (int row = 0;
         row < 4;
         row++)
    {
        for (int column = 0;
             column < 4;
             column++)
        {
            output[row][column] =
                input.m[row][column];
        }
    }
}


/* ============================================================================
 * RAW JOINT ARRAY -> JointVector
 * ============================================================================
 */

static JointVector array_to_joint_vector(
    const double q[ROBOT_DOF]
)
{
    JointVector result;


    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        result.q[joint] =
            q[joint];
    }


    return result;
}


/* ============================================================================
 * MATLAB-LIKE GRADIENT
 * ============================================================================
 *
 * Equivalent to:
 *
 *      gradient(qPath(joint,:), tGlobal)
 *
 * First sample:
 *
 *      forward difference
 *
 * Interior samples:
 *
 *      central difference
 *
 * Last sample:
 *
 *      backward difference
 *
 * ============================================================================
 */

static void compute_joint_gradient(
    const JointVector *values,
    const real_t *time,
    size_t count,
    JointVector *gradient
)
{
    if (
        values == NULL ||
        time == NULL ||
        gradient == NULL ||
        count == 0
    )
    {
        return;
    }


    /*
     * Only one sample:
     *
     * derivative is defined as zero here.
     */
    if (count == 1)
    {
        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            gradient[0].q[joint] =
                0.0;
        }


        return;
    }


    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        /* --------------------------------------------------------------------
         * First sample
         * --------------------------------------------------------------------
         */

        real_t dtFirst =
            time[1] -
            time[0];


        if (fabs(dtFirst) > 1e-15)
        {
            gradient[0].q[joint] =
                (
                    values[1].q[joint] -
                    values[0].q[joint]
                ) /
                dtFirst;
        }
        else
        {
            gradient[0].q[joint] =
                0.0;
        }


        /* --------------------------------------------------------------------
         * Interior samples
         * --------------------------------------------------------------------
         */

        for (size_t sample = 1;
             sample + 1 < count;
             sample++)
        {
            real_t dtCentral =
                time[sample + 1] -
                time[sample - 1];


            if (fabs(dtCentral) > 1e-15)
            {
                gradient[sample].q[joint] =
                    (
                        values[sample + 1].q[joint] -
                        values[sample - 1].q[joint]
                    ) /
                    dtCentral;
            }
            else
            {
                gradient[sample].q[joint] =
                    0.0;
            }
        }


        /* --------------------------------------------------------------------
         * Final sample
         * --------------------------------------------------------------------
         */

        real_t dtLast =
            time[count - 1] -
            time[count - 2];


        if (fabs(dtLast) > 1e-15)
        {
            gradient[count - 1].q[joint] =
                (
                    values[count - 1].q[joint] -
                    values[count - 2].q[joint]
                ) /
                dtLast;
        }
        else
        {
            gradient[count - 1].q[joint] =
                0.0;
        }
    }
}


/* ============================================================================
 * MATLAB REFERENCE REQUEST
 * ============================================================================
 */

SingleLineRequest single_line_matlab_reference_request(void)
{
    SingleLineRequest request;


    memset(
        &request,
        0,
        sizeof(request)
    );


    /* ========================================================================
     * ROBOT CONFIGURATION
     * ========================================================================
     *
     * This function only exists to recreate the original validated MATLAB
     * reference case.
     */

    RobotConfig robot;


    robot_config_init_ur5(
        &robot
    );


    /* ========================================================================
     * ORIGINAL MATLAB START CONFIGURATION
     * ========================================================================
     *
     * qStart = deg2rad([
     *
     *      30
     *     -45
     *      60
     *      20
     *     -30
     *      45
     *
     * ]);
     *
     * qSeed now serves ONLY as the initial IK seed.
     * ========================================================================
     */

    const real_t qStartDegrees[ROBOT_DOF] =
    {
         30.0,
        -45.0,
         60.0,
         20.0,
        -30.0,
         45.0
    };


    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        request.qSeed.q[joint] =
            degrees_to_radians(
                qStartDegrees[joint]
            );
    }


    /* ========================================================================
     * RECREATE ORIGINAL ABSOLUTE WAYPOINT A
     * ========================================================================
     *
     * The OLD pipeline calculated:
     *
     *      A = FK(qStart)
     *
     * We calculate that here once so the reference test still represents
     * exactly the same Cartesian trajectory.
     */

    double TStartArray[4][4];


    control_fk(
        &robot,
        request.qSeed.q,
        TStartArray
    );


    Mat4 TStart =
        array_to_mat4(
            TStartArray
        );


    Mat3 R_A =
        mat4_rotation(
            TStart
        );


    request.startPosition =
        mat4_translation(
            TStart
        );


    request.startOrientation =
        rotm_to_quat(
            R_A
        );


    /* ========================================================================
     * RECREATE ORIGINAL ABSOLUTE WAYPOINT B POSITION
     * ========================================================================
     *
     * Original MATLAB:
     *
     *      pB = pA + [0.50; 0; 0];
     */

    Vec3 referenceDisplacement =
        (Vec3)
        {{
            0.50,
            0.00,
            0.00
        }};


    request.endPosition =
        vec3_add(
            request.startPosition,
            referenceDisplacement
        );


    /* ========================================================================
     * RECREATE ORIGINAL ABSOLUTE WAYPOINT B ORIENTATION
     * ========================================================================
     *
     * Original MATLAB:
     *
     *      R_B =
     *          R_A *
     *          Rz(15 deg) *
     *          Ry(15 deg) *
     *          Rx(15 deg);
     */

    Mat3 relativeEndRotation =
        eul_zyx(
            degrees_to_radians(15.0),
            degrees_to_radians(15.0),
            degrees_to_radians(15.0)
        );


    Mat3 R_B =
        mat3_mul(
            R_A,
            relativeEndRotation
        );


    request.endOrientation =
        rotm_to_quat(
            R_B
        );


    /* ========================================================================
     * PATH + TRAJECTORY SETTINGS
     * ========================================================================
     */

    request.numGeometryPointsPerSegment =
        100;


    request.arcLengthSpacing =
        0.005;


    request.desiredTCPSpeed =
        0.10;


    request.desiredTCPAccel =
        0.25;


    request.desiredTCPJerk =
        1.00;


    request.dt =
        0.001;


    /* ========================================================================
     * ADLS DEFAULTS
     * ========================================================================
     */

    adls_default_parameters(
        &request.ikParameters
    );


    return request;
}

/* ============================================================================
 * SINGLE-SEGMENT STRAIGHT-LINE PIPELINE
 * ============================================================================
 */

bool plan_single_segment_line(
    const RobotConfig *robot,
    const SingleLineRequest *request,
    SingleLineWorkspace *workspace,
    SingleLineTrajectory *trajectory,
    SingleLineReport *report
)
{
    /* ========================================================================
     * VALIDATE POINTERS
     * ========================================================================
     */

    if (
        robot == NULL ||
        request == NULL ||
        workspace == NULL ||
        trajectory == NULL ||
        report == NULL
    )
    {
        return false;
    }


    memset(
        report,
        0,
        sizeof(*report)
    );


    /* ========================================================================
     * VALIDATE REQUEST
     * ========================================================================
     */

    if (
        request->numGeometryPointsPerSegment < 2 ||
        request->arcLengthSpacing <= 0.0 ||
        request->desiredTCPSpeed <= 0.0 ||
        request->desiredTCPAccel <= 0.0 ||
        request->desiredTCPJerk <= 0.0 ||
        request->dt <= 0.0
    )
    {
        return false;
    }


    /* ========================================================================
     * VALIDATE WORKSPACE
     * ========================================================================
     */

    if (
        workspace->rawGeometry == NULL ||
        workspace->arcGeometry == NULL ||
        workspace->lOriginal == NULL ||
        workspace->lArc == NULL ||
        workspace->tempT == NULL ||
        workspace->tempS == NULL ||
        workspace->tempSDot == NULL ||
        workspace->tempSDDot == NULL ||
        workspace->tempSDDDot == NULL ||
        workspace->ikScratch == NULL
    )
    {
        return false;
    }


    if (
        request->numGeometryPointsPerSegment >
        workspace->geometryCapacity
    )
    {
        return false;
    }


    /* ========================================================================
     * VALIDATE OUTPUT TRAJECTORY BUFFERS
     * ========================================================================
     */

    if (
        trajectory->t == NULL ||
        trajectory->s == NULL ||
        trajectory->sDot == NULL ||
        trajectory->sDDot == NULL ||
        trajectory->sDDDot == NULL ||
        trajectory->arcPosition == NULL ||
        trajectory->tcpSpeed == NULL ||
        trajectory->pDesired == NULL ||
        trajectory->quatDesired == NULL ||
        trajectory->qPath == NULL ||
        trajectory->qDot == NULL ||
        trajectory->qDDot == NULL ||
        trajectory->ikIterations == NULL ||
        trajectory->positionError == NULL ||
        trajectory->orientationError == NULL
    )
    {
        return false;
    }


    trajectory->count =
        0;


    /* ========================================================================
     * ABSOLUTE CARTESIAN WAYPOINTS
     * ========================================================================
     *
     * Waypoints A and B are supplied directly by the caller.
     *
     * Both positions and orientations are absolute and expressed
     * in the robot base frame.
     *
     * qSeed is used only later as the initial ADLS IK seed.
     * ========================================================================
     */

    Vec3 pA =
        request->startPosition;


    Vec3 pB =
        request->endPosition;


    Quat quatA =
        request->startOrientation;


    Quat quatB =
        request->endOrientation;


    /*
     * Store actual Cartesian waypoints in the report.
     */

    report->waypointA =
        pA;


    report->waypointB =
        pB;

    /* ========================================================================
     * 8A. RAW GEOMETRIC LINE
     * ========================================================================
     *
     * MATLAB:
     *
     * rawSegment =
     *      generateLineWaypoints(
     *          pStartSegment,
     *          pEndSegment,
     *          numGeometryPointsPerSegment
     *      );
     */

    if (
        !generate_line_waypoints(
            pA,
            pB,
            request->numGeometryPointsPerSegment,
            workspace->rawGeometry
        )
    )
    {
        return false;
    }


    /* ========================================================================
     * 8B. ARC-LENGTH PARAMETERIZATION
     * ========================================================================
     *
     * MATLAB:
     *
     * [arcSegment,lOriginal,lArc] =
     *      arcLengthParameterize(
     *          rawSegment,
     *          arcLengthSpacing
     *      );
     */

    ArcLengthInfo arcInfo;


    if (
        !arc_length_parameterize(
            workspace->rawGeometry,
            request->numGeometryPointsPerSegment,
            request->arcLengthSpacing,
            workspace->arcGeometry,
            workspace->lOriginal,
            workspace->lArc,
            workspace->geometryCapacity,
            &arcInfo
        )
    )
    {
        return false;
    }


    real_t segmentLength =
        arcInfo.totalLength;


    /*
     * MATLAB:
     *
     *      if segmentLength <= eps
     *          error(...)
     *      end
     */

    if (segmentLength <= 1e-15)
    {
        return false;
    }


    /* ========================================================================
     * 8C. NORMALIZED LIMITS
     * ========================================================================
     *
     * MATLAB:
     *
     *      sDotMax =
     *          desiredTCPSpeed /
     *          segmentLength;
     *
     *      sDDotMax =
     *          desiredTCPAccel /
     *          segmentLength;
     *
     *      sDDDotMax =
     *          desiredTCPJerk /
     *          segmentLength;
     */

    real_t sDotMax =
        request->desiredTCPSpeed /
        segmentLength;


    real_t sDDotMax =
        request->desiredTCPAccel /
        segmentLength;


    real_t sDDDotMax =
        request->desiredTCPJerk /
        segmentLength;


    /* ========================================================================
     * 8D. S-CURVE TIME SCALING
     * ========================================================================
     */

    size_t numberOfSamples =
        0;


    SCurveInfo profileInfo;


    if (
        !s_curve_time_scaling(
            sDotMax,
            sDDotMax,
            sDDDotMax,
            request->dt,
            workspace->tempT,
            workspace->tempS,
            workspace->tempSDot,
            workspace->tempSDDot,
            workspace->tempSDDDot,
            workspace->timeCapacity,
            &numberOfSamples,
            &profileInfo
        )
    )
    {
        return false;
    }


    if (
        numberOfSamples >
        trajectory->capacity
    )
    {
        return false;
    }


    /* ========================================================================
     * SEQUENTIAL ADLS SEED
     * ========================================================================
     *
     * MATLAB:
     *
     *      qSeed = qStart;
     */

    JointVector qSeed =
    request->qSeed;


    real_t peakTCPSpeed =
        0.0;


    real_t maxPositionError =
        0.0;


    real_t maxOrientationError =
        0.0;


    int maxIKIterations =
        0;


    /* ========================================================================
     * 8D.5 -> 14
     *
     * BUILD TIMED CARTESIAN POSES
     * RUN SEQUENTIAL ADLS
     * RUN INDEPENDENT FK VALIDATION
     * ========================================================================
     */

    for (size_t sample = 0;
         sample < numberOfSamples;
         sample++)
    {
        /* --------------------------------------------------------------------
         * Normalized path coordinate
         * --------------------------------------------------------------------
         */

        real_t s =
            workspace->tempS[sample];


        /* --------------------------------------------------------------------
         * D.5 Orientation SLERP
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      quatSegment(:,k) =
         *          quatSlerp_custom(
         *              quatStartSegment,
         *              quatEndSegment,
         *              sSegment(k)
         *          );
         */

        Quat desiredQuaternion =
            quat_slerp(
                quatA,
                quatB,
                s
            );


        /* --------------------------------------------------------------------
         * E. Physical arc length
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      lTimed =
         *          segmentLength *
         *          sSegment;
         *
         *      tcpSpeedSegment =
         *          segmentLength *
         *          sDotSegment;
         */

        real_t lTimed =
            segmentLength *
            s;


        real_t tcpSpeed =
            segmentLength *
            workspace->tempSDot[sample];


        /* --------------------------------------------------------------------
         * F. Cartesian position p(l(t))
         * --------------------------------------------------------------------
         */

        Vec3 desiredPosition =
            arc_length_interpolate(
                workspace->arcGeometry,
                workspace->lArc,
                arcInfo.count,
                lTimed
            );


        /* --------------------------------------------------------------------
         * G. Cartesian pose
         * --------------------------------------------------------------------
         */

        Mat4 desiredPose =
            mat4_identity();


        mat4_set_rotation(
            &desiredPose,
            quat_to_rotm(
                desiredQuaternion
            )
        );


        mat4_set_translation(
            &desiredPose,
            desiredPosition
        );


        double desiredPoseArray[4][4];


        mat4_to_array(
            desiredPose,
            desiredPoseArray
        );


        /* --------------------------------------------------------------------
         * 13. Sequential ADLS IK
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      [qSolution,info] =
         *          ADLS_IK(
         *              robot,
         *              TPath(:,:,k),
         *              qSeed
         *          );
         */

        double qSolutionArray[ROBOT_DOF];


        bool ikSucceeded =
            adls_ik(
                robot,
                desiredPoseArray,
                qSeed.q,
                &request->ikParameters,
                qSolutionArray,
                workspace->ikScratch
            );


        if (!ikSucceeded)
        {
            report->success =
                false;

            return false;
        }


        JointVector qSolution =
            array_to_joint_vector(
                qSolutionArray
            );


        /* --------------------------------------------------------------------
         * MATLAB:
         *
         *      qPath(:,k) =
         *          qSolution;
         *
         *      qSeed =
         *          qSolution;
         *
         * This is what makes the IK sequential.
         * --------------------------------------------------------------------
         */

        qSeed =
            qSolution;


        /* --------------------------------------------------------------------
         * 14. INDEPENDENT FK VALIDATION
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      TAchieved =
         *          controlFK(
         *              robot,
         *              qPath(:,k)
         *          );
         */

        double achievedPoseArray[4][4];


        control_fk(
            robot,
            qSolution.q,
            achievedPoseArray
        );


        Mat4 achievedPose =
            array_to_mat4(
                achievedPoseArray
            );


        Vec3 achievedPosition =
            mat4_translation(
                achievedPose
            );


        /* --------------------------------------------------------------------
         * Position error
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      positionError(k) =
         *          norm(
         *              desired -
         *              achieved
         *          );
         */

        real_t positionError =
            vec3_norm(
                vec3_sub(
                    desiredPosition,
                    achievedPosition
                )
            );


        /* --------------------------------------------------------------------
         * Orientation error
         * --------------------------------------------------------------------
         */

        real_t orientationError =
            rotation_error(
                mat4_rotation(
                    desiredPose
                ),
                mat4_rotation(
                    achievedPose
                )
            );


        /* --------------------------------------------------------------------
         * STORE COMPLETE TRAJECTORY SAMPLE
         * --------------------------------------------------------------------
         */

        trajectory->t[sample] =
            workspace->tempT[sample];


        trajectory->s[sample] =
            workspace->tempS[sample];


        trajectory->sDot[sample] =
            workspace->tempSDot[sample];


        trajectory->sDDot[sample] =
            workspace->tempSDDot[sample];


        trajectory->sDDDot[sample] =
            workspace->tempSDDDot[sample];


        trajectory->arcPosition[sample] =
            lTimed;


        trajectory->tcpSpeed[sample] =
            tcpSpeed;


        trajectory->pDesired[sample] =
            desiredPosition;


        trajectory->quatDesired[sample] =
            desiredQuaternion;


        trajectory->qPath[sample] =
            qSolution;


        trajectory->ikIterations[sample] =
            workspace->ikScratch->iterations;


        trajectory->positionError[sample] =
            positionError;


        trajectory->orientationError[sample] =
            orientationError;


        /* --------------------------------------------------------------------
         * REPORT MAXIMUM VALUES
         * --------------------------------------------------------------------
         */

        if (tcpSpeed > peakTCPSpeed)
        {
            peakTCPSpeed =
                tcpSpeed;
        }


        if (positionError > maxPositionError)
        {
            maxPositionError =
                positionError;
        }


        if (orientationError > maxOrientationError)
        {
            maxOrientationError =
                orientationError;
        }


        if (
            workspace->ikScratch->iterations >
            maxIKIterations
        )
        {
            maxIKIterations =
                workspace->ikScratch->iterations;
        }
    }


    trajectory->count =
        numberOfSamples;


    /* ========================================================================
     * 15. JOINT VELOCITY
     * ========================================================================
     *
     * MATLAB:
     *
     *      qDot(joint,:) =
     *          gradient(
     *              qPath(joint,:),
     *              tGlobal
     *          );
     */

    compute_joint_gradient(
        trajectory->qPath,
        trajectory->t,
        numberOfSamples,
        trajectory->qDot
    );


    /* ========================================================================
     * 15. JOINT ACCELERATION
     * ========================================================================
     *
     * MATLAB:
     *
     *      qDDot(joint,:) =
     *          gradient(
     *              qDot(joint,:),
     *              tGlobal
     *          );
     */

    compute_joint_gradient(
        trajectory->qDot,
        trajectory->t,
        numberOfSamples,
        trajectory->qDDot
    );


    /* ========================================================================
     * 16. JOINT LIMIT CHECKS
     * ========================================================================
     */

    bool positionLimitsPass =
        true;


    bool velocityLimitsPass =
        true;


    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        real_t peakJointVelocity =
            0.0;


        real_t maxJointStep =
            0.0;


        for (size_t sample = 0;
             sample < numberOfSamples;
             sample++)
        {
            /* ----------------------------------------------------------------
             * Position limits
             * ----------------------------------------------------------------
             */

            if (
                trajectory->qPath[sample].q[joint] <
                    robot->limits.qMin[joint] ||

                trajectory->qPath[sample].q[joint] >
                    robot->limits.qMax[joint]
            )
            {
                positionLimitsPass =
                    false;
            }


            /* ----------------------------------------------------------------
             * Peak velocity
             * ----------------------------------------------------------------
             */

            real_t jointVelocity =
                fabs(
                    trajectory->qDot[sample].q[joint]
                );


            if (
                jointVelocity >
                peakJointVelocity
            )
            {
                peakJointVelocity =
                    jointVelocity;
            }


            /* ----------------------------------------------------------------
             * Largest joint step
             * ----------------------------------------------------------------
             */

            if (sample > 0)
            {
                real_t jointStep =
                    fabs(
                        trajectory
                            ->qPath[sample]
                            .q[joint]
                        -
                        trajectory
                            ->qPath[sample - 1]
                            .q[joint]
                    );


                if (
                    jointStep >
                    maxJointStep
                )
                {
                    maxJointStep =
                        jointStep;
                }
            }
        }


        report->peakJointVelocity[joint] =
            peakJointVelocity;


        report->maxJointStep[joint] =
            maxJointStep;


        /*
         * MATLAB:
         *
         *      velocityViolation =
         *          peakJointVelocity >
         *          qdMax;
         */

        if (
            peakJointVelocity >
            robot->limits.qdMax[joint]
        )
        {
            velocityLimitsPass =
                false;
        }
    }


    /* ========================================================================
     * FINAL REPORT
     * ========================================================================
     */

    report->samples =
        numberOfSamples;


    report->rawGeometryPoints =
        request->numGeometryPointsPerSegment;


    report->arcGeometryPoints =
        arcInfo.count;


    report->pathLength =
        segmentLength;


    report->duration =
        profileInfo.T;


    report->peakTCPSpeed =
        peakTCPSpeed;


    report->maxPositionError =
        maxPositionError;


    report->maxOrientationError =
        maxOrientationError;


    report->maxIKIterations =
        maxIKIterations;


    report->positionLimitsPass =
        positionLimitsPass;


    report->velocityLimitsPass =
        velocityLimitsPass;


    report->profile =
        profileInfo;


    report->success =
        true;


    return true;
}