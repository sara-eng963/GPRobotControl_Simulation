#include "single_segment_circular.h"

#include "../Math/math3d.h"
#include "../Kinematics/control_fk.h"
#include "../Trajectory/arc_length_parameterize.h"
#include "../Trajectory/s_curve_time_scaling.h"

#include <math.h>
#include <stddef.h>
#include <string.h>


/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================
 */

static real_t degrees_to_radians(
    real_t degrees
)
{
    return degrees * ROBOT_PI / 180.0;
}


static Mat4 array_to_mat4(
    const double input[4][4]
)
{
    Mat4 result;

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            result.m[row][column] = input[row][column];
        }
    }

    return result;
}


static void mat4_to_array(
    Mat4 input,
    double output[4][4]
)
{
    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            output[row][column] = input.m[row][column];
        }
    }
}


static JointVector array_to_joint_vector(
    const double q[ROBOT_DOF]
)
{
    JointVector result;

    for (int joint = 0; joint < ROBOT_DOF; joint++)
    {
        result.q[joint] = q[joint];
    }

    return result;
}


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

    if (count == 1)
    {
        for (int joint = 0; joint < ROBOT_DOF; joint++)
        {
            gradient[0].q[joint] = 0.0;
        }

        return;
    }

    for (int joint = 0; joint < ROBOT_DOF; joint++)
    {
        real_t dtFirst = time[1] - time[0];

        gradient[0].q[joint] =
            fabs(dtFirst) > 1e-15
            ? (values[1].q[joint] - values[0].q[joint]) / dtFirst
            : 0.0;

        for (size_t sample = 1; sample + 1 < count; sample++)
        {
            real_t dtCentral = time[sample + 1] - time[sample - 1];

            gradient[sample].q[joint] =
                fabs(dtCentral) > 1e-15
                ? (values[sample + 1].q[joint] - values[sample - 1].q[joint]) /
                  dtCentral
                : 0.0;
        }

        real_t dtLast = time[count - 1] - time[count - 2];

        gradient[count - 1].q[joint] =
            fabs(dtLast) > 1e-15
            ? (values[count - 1].q[joint] - values[count - 2].q[joint]) /
              dtLast
            : 0.0;
    }
}


static bool validate_common_inputs(
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    const SingleCircularWorkspace *workspace,
    const SingleCircularTrajectory *trajectory,
    const SingleCircularReport *report
)
{
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

    if (
        request->numGeometryPointsPerSegment < 3 ||
        request->arcLengthSpacing <= 0.0 ||
        request->desiredTCPSpeed <= 0.0 ||
        request->desiredTCPAccel <= 0.0 ||
        request->desiredTCPJerk <= 0.0 ||
        request->dt <= 0.0
    )
    {
        return false;
    }

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
        request->numGeometryPointsPerSegment > workspace->geometryCapacity
    )
    {
        return false;
    }

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

    return true;
}


/* ============================================================================
 * REFERENCE REQUEST COMMON INITIALIZATION
 * ============================================================================
 */

static SingleCircularRequest reference_request_common(void)
{
    SingleCircularRequest request;
    memset(&request, 0, sizeof(request));

    RobotConfig robot;
    robot_config_init_ur5(&robot);

    const real_t qStartDegrees[ROBOT_DOF] =
    {
         30.0,
        -45.0,
         60.0,
         20.0,
        -30.0,
         45.0
    };

    for (int joint = 0; joint < ROBOT_DOF; joint++)
    {
        request.qSeed.q[joint] =
            degrees_to_radians(qStartDegrees[joint]);
    }

    double TStartArray[4][4];
    control_fk(&robot, request.qSeed.q, TStartArray);

    Mat4 TStart = array_to_mat4(TStartArray);
    Mat3 RStart = mat4_rotation(TStart);

    request.point1 = mat4_translation(TStart);
    request.startOrientation = rotm_to_quat(RStart);
    request.endOrientation = request.startOrientation;

    request.arcLengthSpacing = 0.005;
    request.desiredTCPSpeed = 0.10;
    request.desiredTCPAccel = 0.25;
    request.desiredTCPJerk = 1.00;
    request.dt = 0.001;

    adls_default_parameters(&request.ikParameters);

    return request;
}


SingleCircularRequest single_arc_matlab_reference_request(void)
{
    SingleCircularRequest request = reference_request_common();

    request.type = CIRCULAR_SEGMENT_ARC;
    request.direction = 0;

    request.point2 = vec3_add(
        request.point1,
        (Vec3){{0.20, 0.14, 0.00}}
    );

    request.point3 = vec3_add(
        request.point1,
        (Vec3){{0.40, 0.00, 0.00}}
    );

    request.numGeometryPointsPerSegment = 200;

    return request;
}


SingleCircularRequest single_full_circle_matlab_reference_request(void)
{
    SingleCircularRequest request = reference_request_common();

    request.type = CIRCULAR_SEGMENT_FULL_CIRCLE;
    request.direction = +1;

    request.point2 = vec3_add(
        request.point1,
        (Vec3){{0.15, 0.15, 0.00}}
    );

    request.point3 = vec3_add(
        request.point1,
        (Vec3){{0.30, 0.00, 0.00}}
    );

    request.numGeometryPointsPerSegment = 400;

    return request;
}


/* ============================================================================
 * SHARED CIRCULAR PIPELINE
 * ============================================================================
 */

static bool plan_single_segment_circular_internal(
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularWorkspace *workspace,
    SingleCircularTrajectory *trajectory,
    SingleCircularReport *report
)
{
    if (
        !validate_common_inputs(
            robot,
            request,
            workspace,
            trajectory,
            report
        )
    )
    {
        return false;
    }

    memset(report, 0, sizeof(*report));
    trajectory->count = 0;

    report->type = request->type;
    report->point1 = request->point1;
    report->point2 = request->point2;
    report->point3 = request->point3;

    CircularPathInfo circularInfo;

    bool geometrySucceeded = false;

    if (request->type == CIRCULAR_SEGMENT_ARC)
    {
        geometrySucceeded =
            generate_arc_waypoints(
                request->point1,
                request->point2,
                request->point3,
                request->numGeometryPointsPerSegment,
                workspace->rawGeometry,
                &circularInfo
            );
    }
    else if (request->type == CIRCULAR_SEGMENT_FULL_CIRCLE)
    {
        geometrySucceeded =
            generate_full_circle_waypoints(
                request->point1,
                request->point2,
                request->point3,
                request->direction,
                request->numGeometryPointsPerSegment,
                workspace->rawGeometry,
                &circularInfo
            );
    }

    if (!geometrySucceeded)
    {
        return false;
    }

    ArcLengthInfo arcLengthInfo;

    if (
        !arc_length_parameterize(
            workspace->rawGeometry,
            request->numGeometryPointsPerSegment,
            request->arcLengthSpacing,
            workspace->arcGeometry,
            workspace->lOriginal,
            workspace->lArc,
            workspace->geometryCapacity,
            &arcLengthInfo
        )
    )
    {
        return false;
    }

    real_t segmentLength = arcLengthInfo.totalLength;

    if (segmentLength <= 1e-15)
    {
        return false;
    }

    real_t sDotMax = request->desiredTCPSpeed / segmentLength;
    real_t sDDotMax = request->desiredTCPAccel / segmentLength;
    real_t sDDDotMax = request->desiredTCPJerk / segmentLength;

    size_t numberOfSamples = 0;
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

    if (numberOfSamples > trajectory->capacity)
    {
        return false;
    }

    JointVector qSeed = request->qSeed;

    real_t peakTCPSpeed = 0.0;
    real_t maxPositionError = 0.0;
    real_t maxOrientationError = 0.0;
    int maxIKIterations = 0;

    for (size_t sample = 0; sample < numberOfSamples; sample++)
    {
        real_t s = workspace->tempS[sample];

        Quat desiredQuaternion =
            circular_path_orientation(
                request->startOrientation,
                request->endOrientation,
                circularInfo.axis,
                circularInfo.thetaTotal,
                s
            );

        real_t lTimed = segmentLength * s;
        real_t tcpSpeed = segmentLength * workspace->tempSDot[sample];

        Vec3 desiredPosition =
            arc_length_interpolate(
                workspace->arcGeometry,
                workspace->lArc,
                arcLengthInfo.count,
                lTimed
            );

        Mat4 desiredPose = mat4_identity();
        mat4_set_rotation(&desiredPose, quat_to_rotm(desiredQuaternion));
        mat4_set_translation(&desiredPose, desiredPosition);

        double desiredPoseArray[4][4];
        mat4_to_array(desiredPose, desiredPoseArray);

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
            report->success = false;
            return false;
        }

        JointVector qSolution = array_to_joint_vector(qSolutionArray);
        qSeed = qSolution;

        double achievedPoseArray[4][4];
        control_fk(robot, qSolution.q, achievedPoseArray);

        Mat4 achievedPose = array_to_mat4(achievedPoseArray);
        Vec3 achievedPosition = mat4_translation(achievedPose);

        real_t positionError =
            vec3_norm(
                vec3_sub(
                    desiredPosition,
                    achievedPosition
                )
            );

        real_t orientationError =
            rotation_error(
                mat4_rotation(desiredPose),
                mat4_rotation(achievedPose)
            );

        trajectory->t[sample] = workspace->tempT[sample];
        trajectory->s[sample] = workspace->tempS[sample];
        trajectory->sDot[sample] = workspace->tempSDot[sample];
        trajectory->sDDot[sample] = workspace->tempSDDot[sample];
        trajectory->sDDDot[sample] = workspace->tempSDDDot[sample];

        trajectory->arcPosition[sample] = lTimed;
        trajectory->tcpSpeed[sample] = tcpSpeed;

        trajectory->pDesired[sample] = desiredPosition;
        trajectory->quatDesired[sample] = desiredQuaternion;

        trajectory->qPath[sample] = qSolution;
        trajectory->ikIterations[sample] = workspace->ikScratch->iterations;

        trajectory->positionError[sample] = positionError;
        trajectory->orientationError[sample] = orientationError;

        if (tcpSpeed > peakTCPSpeed)
        {
            peakTCPSpeed = tcpSpeed;
        }

        if (positionError > maxPositionError)
        {
            maxPositionError = positionError;
        }

        if (orientationError > maxOrientationError)
        {
            maxOrientationError = orientationError;
        }

        if (workspace->ikScratch->iterations > maxIKIterations)
        {
            maxIKIterations = workspace->ikScratch->iterations;
        }
    }

    trajectory->count = numberOfSamples;

    compute_joint_gradient(
        trajectory->qPath,
        trajectory->t,
        numberOfSamples,
        trajectory->qDot
    );

    compute_joint_gradient(
        trajectory->qDot,
        trajectory->t,
        numberOfSamples,
        trajectory->qDDot
    );

    bool positionLimitsPass = true;
    bool velocityLimitsPass = true;

    for (int joint = 0; joint < ROBOT_DOF; joint++)
    {
        real_t peakJointVelocity = 0.0;
        real_t maxJointStep = 0.0;

        for (size_t sample = 0; sample < numberOfSamples; sample++)
        {
            if (
                trajectory->qPath[sample].q[joint] < robot->limits.qMin[joint] ||
                trajectory->qPath[sample].q[joint] > robot->limits.qMax[joint]
            )
            {
                positionLimitsPass = false;
            }

            real_t jointVelocity =
                fabs(trajectory->qDot[sample].q[joint]);

            if (jointVelocity > peakJointVelocity)
            {
                peakJointVelocity = jointVelocity;
            }

            if (sample > 0)
            {
                real_t jointStep =
                    fabs(
                        trajectory->qPath[sample].q[joint] -
                        trajectory->qPath[sample - 1].q[joint]
                    );

                if (jointStep > maxJointStep)
                {
                    maxJointStep = jointStep;
                }
            }
        }

        report->peakJointVelocity[joint] = peakJointVelocity;
        report->maxJointStep[joint] = maxJointStep;

        if (peakJointVelocity > robot->limits.qdMax[joint])
        {
            velocityLimitsPass = false;
        }
    }

    report->geometry = circularInfo;

    report->samples = numberOfSamples;
    report->rawGeometryPoints = request->numGeometryPointsPerSegment;
    report->arcGeometryPoints = arcLengthInfo.count;

    report->pathLength = segmentLength;
    report->duration = profileInfo.T;
    report->peakTCPSpeed = peakTCPSpeed;

    report->maxPositionError = maxPositionError;
    report->maxOrientationError = maxOrientationError;
    report->maxIKIterations = maxIKIterations;

    report->positionLimitsPass = positionLimitsPass;
    report->velocityLimitsPass = velocityLimitsPass;

    report->profile = profileInfo;
    report->success = true;

    return true;
}


/* ============================================================================
 * PUBLIC ARC WRAPPER
 * ============================================================================
 */

bool plan_single_segment_arc(
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularWorkspace *workspace,
    SingleCircularTrajectory *trajectory,
    SingleCircularReport *report
)
{
    if (
        request == NULL ||
        request->type != CIRCULAR_SEGMENT_ARC
    )
    {
        return false;
    }

    return
        plan_single_segment_circular_internal(
            robot,
            request,
            workspace,
            trajectory,
            report
        );
}


/* ============================================================================
 * PUBLIC FULL-CIRCLE WRAPPER
 * ============================================================================
 */

bool plan_single_segment_full_circle(
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularWorkspace *workspace,
    SingleCircularTrajectory *trajectory,
    SingleCircularReport *report
)
{
    if (
        request == NULL ||
        request->type != CIRCULAR_SEGMENT_FULL_CIRCLE ||
        (request->direction != 1 && request->direction != -1)
    )
    {
        return false;
    }

    return
        plan_single_segment_circular_internal(
            robot,
            request,
            workspace,
            trajectory,
            report
        );
}