#include "single_segment_circular_stream.h"

#include "../Math/math3d.h"
#include "../Kinematics/control_fk.h"
#include "../Kinematics/adls_ik.h"

#include <math.h>
#include <stdio.h>
#include <string.h>


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


static void set_error(
    SingleCircularStream *stream,
    SingleCircularStreamStatus status
)
{
    if (stream == NULL)
    {
        return;
    }

    stream->status = status;
}


bool single_circular_stream_init(
    SingleCircularStream *stream,
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularStreamWorkspace *workspace
)
{
    if (
        stream == NULL ||
        robot == NULL ||
        request == NULL ||
        workspace == NULL
    )
    {
        return false;
    }

    memset(stream, 0, sizeof(*stream));

    stream->status = SINGLE_CIRCULAR_STREAM_INVALID_ARGUMENT;

    bool valid_type =
        request->type == CIRCULAR_SEGMENT_ARC ||
        request->type == CIRCULAR_SEGMENT_FULL_CIRCLE;

    if (
        !valid_type ||
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
        request->type == CIRCULAR_SEGMENT_FULL_CIRCLE &&
        request->direction != 1 &&
        request->direction != -1
    )
    {
        return false;
    }

    if (
        workspace->geometryCapacity == 0 ||
        workspace->rawGeometry == NULL ||
        workspace->arcGeometry == NULL ||
        workspace->lOriginal == NULL ||
        workspace->lArc == NULL ||
        workspace->ikScratch == NULL ||
        request->numGeometryPointsPerSegment > workspace->geometryCapacity
    )
    {
        return false;
    }

    stream->robot = robot;
    stream->request = *request;
    stream->workspace = workspace;

    bool geometry_succeeded = false;

    if (request->type == CIRCULAR_SEGMENT_ARC)
    {
        geometry_succeeded =
            generate_arc_waypoints(
                request->point1,
                request->point2,
                request->point3,
                request->numGeometryPointsPerSegment,
                workspace->rawGeometry,
                &stream->circularInfo
            );
    }
    else
    {
        geometry_succeeded =
            generate_full_circle_waypoints(
                request->point1,
                request->point2,
                request->point3,
                request->direction,
                request->numGeometryPointsPerSegment,
                workspace->rawGeometry,
                &stream->circularInfo
            );
    }

    if (!geometry_succeeded)
    {
        set_error(stream, SINGLE_CIRCULAR_STREAM_GEOMETRY_FAILED);
        return false;
    }

    if (
        !arc_length_parameterize(
            workspace->rawGeometry,
            request->numGeometryPointsPerSegment,
            request->arcLengthSpacing,
            workspace->arcGeometry,
            workspace->lOriginal,
            workspace->lArc,
            workspace->geometryCapacity,
            &stream->arcInfo
        )
    )
    {
        set_error(stream, SINGLE_CIRCULAR_STREAM_GEOMETRY_FAILED);
        return false;
    }

    stream->segmentLength = stream->arcInfo.totalLength;

    if (stream->segmentLength <= 1e-15)
    {
        set_error(stream, SINGLE_CIRCULAR_STREAM_GEOMETRY_FAILED);
        return false;
    }

    real_t s_dot_max =
        request->desiredTCPSpeed /
        stream->segmentLength;

    real_t s_ddot_max =
        request->desiredTCPAccel /
        stream->segmentLength;

    real_t s_dddot_max =
        request->desiredTCPJerk /
        stream->segmentLength;

    if (
        !s_curve_profile_init(
            s_dot_max,
            s_ddot_max,
            s_dddot_max,
            request->dt,
            &stream->profile
        )
    )
    {
        set_error(stream, SINGLE_CIRCULAR_STREAM_PROFILE_FAILED);
        return false;
    }

    stream->qSeed = request->qSeed;
    stream->nextSampleIndex = 0;
    stream->initialized = true;
    stream->finished = false;
    stream->status = SINGLE_CIRCULAR_STREAM_OK;

    return true;
}


bool single_circular_stream_next(
    SingleCircularStream *stream,
    SingleCircularStreamSample *sample
)
{
    if (
        stream == NULL ||
        sample == NULL ||
        !stream->initialized ||
        stream->workspace == NULL ||
        stream->robot == NULL
    )
    {
        if (stream != NULL)
        {
            set_error(stream, SINGLE_CIRCULAR_STREAM_INVALID_ARGUMENT);
        }

        return false;
    }

    size_t total_samples =
        s_curve_profile_sample_count(&stream->profile);

    if (stream->nextSampleIndex >= total_samples)
    {
        stream->finished = true;
        stream->status = SINGLE_CIRCULAR_STREAM_FINISHED;
        return false;
    }

    SCurveSample profile_sample;

    if (
        !s_curve_profile_evaluate_index(
            &stream->profile,
            stream->nextSampleIndex,
            &profile_sample
        )
    )
    {
        set_error(stream, SINGLE_CIRCULAR_STREAM_PROFILE_FAILED);
        return false;
    }

    Quat desired_quaternion =
        circular_path_orientation(
            stream->request.startOrientation,
            stream->request.endOrientation,
            stream->circularInfo.axis,
            stream->circularInfo.thetaTotal,
            profile_sample.s
        );

    real_t l_timed =
        stream->segmentLength *
        profile_sample.s;

    real_t tcp_speed =
        stream->segmentLength *
        profile_sample.sDot;

    Vec3 desired_position =
        arc_length_interpolate(
            stream->workspace->arcGeometry,
            stream->workspace->lArc,
            stream->arcInfo.count,
            l_timed
        );

    Mat4 desired_pose = mat4_identity();

    mat4_set_rotation(
        &desired_pose,
        quat_to_rotm(desired_quaternion)
    );

    mat4_set_translation(
        &desired_pose,
        desired_position
    );

    double desired_pose_array[4][4];
    mat4_to_array(desired_pose, desired_pose_array);

    double q_solution_array[ROBOT_DOF];

    bool ik_succeeded =
        adls_ik(
            stream->robot,
            desired_pose_array,
            stream->qSeed.q,
            &stream->request.ikParameters,
            q_solution_array,
            stream->workspace->ikScratch
        );

    if (!ik_succeeded)
    {
        printf(
            "\nCIRCULAR IK FAILED\n"
            "type:   %s\n"
            "sample: %zu / %zu\n"
            "t:      %.6f s\n"
            "s:      %.6f\n"
            "p:      [%.6f %.6f %.6f] m\n",
            stream->request.type == CIRCULAR_SEGMENT_ARC
                ? "ARC"
                : "FULL CIRCLE",
            stream->nextSampleIndex,
            total_samples,
            profile_sample.t,
            profile_sample.s,
            desired_position.v[0],
            desired_position.v[1],
            desired_position.v[2]
        );

        set_error(stream, SINGLE_CIRCULAR_STREAM_IK_FAILED);
        return false;
    }

    JointVector q_solution =
        array_to_joint_vector(q_solution_array);

    stream->qSeed = q_solution;

    double achieved_pose_array[4][4];

    control_fk(
        stream->robot,
        q_solution.q,
        achieved_pose_array
    );

    Mat4 achieved_pose =
        array_to_mat4(achieved_pose_array);

    Vec3 achieved_position =
        mat4_translation(achieved_pose);

    real_t position_error =
        vec3_norm(
            vec3_sub(
                desired_position,
                achieved_position
            )
        );

    real_t orientation_error =
        rotation_error(
            mat4_rotation(desired_pose),
            mat4_rotation(achieved_pose)
        );

    memset(sample, 0, sizeof(*sample));

    sample->index = stream->nextSampleIndex;
    sample->t = profile_sample.t;
    sample->s = profile_sample.s;
    sample->sDot = profile_sample.sDot;
    sample->sDDot = profile_sample.sDDot;
    sample->sDDDot = profile_sample.sDDDot;
    sample->arcPosition = l_timed;
    sample->tcpSpeed = tcp_speed;
    sample->pDesired = desired_position;
    sample->quatDesired = desired_quaternion;
    sample->q = q_solution;
    sample->ikIterations = stream->workspace->ikScratch->iterations;
    sample->positionError = position_error;
    sample->orientationError = orientation_error;

    if (tcp_speed > stream->peakTCPSpeed)
    {
        stream->peakTCPSpeed = tcp_speed;
    }

    if (position_error > stream->maxPositionError)
    {
        stream->maxPositionError = position_error;
    }

    if (orientation_error > stream->maxOrientationError)
    {
        stream->maxOrientationError = orientation_error;
    }

    if (
        stream->workspace->ikScratch->iterations >
        stream->maxIKIterations
    )
    {
        stream->maxIKIterations =
            stream->workspace->ikScratch->iterations;
    }

    stream->nextSampleIndex++;

    if (stream->nextSampleIndex >= total_samples)
    {
        stream->finished = true;
    }

    stream->status = SINGLE_CIRCULAR_STREAM_OK;

    return true;
}


size_t single_circular_stream_sample_count(
    const SingleCircularStream *stream
)
{
    if (stream == NULL || !stream->initialized)
    {
        return 0;
    }

    return s_curve_profile_sample_count(&stream->profile);
}


size_t single_circular_stream_samples_generated(
    const SingleCircularStream *stream
)
{
    if (stream == NULL || !stream->initialized)
    {
        return 0;
    }

    return stream->nextSampleIndex;
}


size_t single_circular_stream_samples_remaining(
    const SingleCircularStream *stream
)
{
    if (stream == NULL || !stream->initialized)
    {
        return 0;
    }

    size_t total = single_circular_stream_sample_count(stream);

    if (stream->nextSampleIndex >= total)
    {
        return 0;
    }

    return total - stream->nextSampleIndex;
}


bool single_circular_stream_is_finished(
    const SingleCircularStream *stream
)
{
    return
        stream != NULL &&
        stream->initialized &&
        stream->finished;
}


SingleCircularStreamStatus single_circular_stream_status(
    const SingleCircularStream *stream
)
{
    if (stream == NULL)
    {
        return SINGLE_CIRCULAR_STREAM_INVALID_ARGUMENT;
    }

    return stream->status;
}


const char *single_circular_stream_status_string(
    SingleCircularStreamStatus status
)
{
    switch (status)
    {
        case SINGLE_CIRCULAR_STREAM_OK:
            return "OK";

        case SINGLE_CIRCULAR_STREAM_FINISHED:
            return "FINISHED";

        case SINGLE_CIRCULAR_STREAM_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";

        case SINGLE_CIRCULAR_STREAM_GEOMETRY_FAILED:
            return "GEOMETRY_FAILED";

        case SINGLE_CIRCULAR_STREAM_PROFILE_FAILED:
            return "PROFILE_FAILED";

        case SINGLE_CIRCULAR_STREAM_IK_FAILED:
            return "IK_FAILED";

        default:
            return "UNKNOWN";
    }
}
