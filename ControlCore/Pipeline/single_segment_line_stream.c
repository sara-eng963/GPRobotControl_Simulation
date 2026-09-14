#include "single_segment_line_stream.h"

#include "../Math/math3d.h"
#include "../Kinematics/control_fk.h"
#include "../Kinematics/adls_ik.h"
#include "../Trajectory/generate_line_waypoints.h"

#include <math.h>
#include <string.h>


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


static void set_error(
    SingleLineStream *stream,
    SingleLineStreamStatus status
)
{
    if (stream == NULL)
    {
        return;
    }

    stream->status =
        status;
}


bool single_line_stream_init(
    SingleLineStream *stream,
    const RobotConfig *robot,
    const SingleLineRequest *request,
    SingleLineStreamWorkspace *workspace
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

    memset(
        stream,
        0,
        sizeof(*stream)
    );

    stream->status =
        SINGLE_LINE_STREAM_INVALID_ARGUMENT;

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

    if (
        workspace->geometryCapacity == 0 ||
        workspace->rawGeometry == NULL ||
        workspace->arcGeometry == NULL ||
        workspace->lOriginal == NULL ||
        workspace->lArc == NULL ||
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


    stream->robot =
        robot;

    stream->request =
        *request;

    stream->workspace =
        workspace;


    /* ========================================================================
     * FIXED-SIZE GEOMETRY SETUP
     * ========================================================================
     */

    if (
        !generate_line_waypoints(
            request->startPosition,
            request->endPosition,
            request->numGeometryPointsPerSegment,
            workspace->rawGeometry
        )
    )
    {
        set_error(
            stream,
            SINGLE_LINE_STREAM_GEOMETRY_FAILED
        );

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
        set_error(
            stream,
            SINGLE_LINE_STREAM_GEOMETRY_FAILED
        );

        return false;
    }


    stream->segmentLength =
        stream->arcInfo.totalLength;

    if (
        stream->segmentLength <=
        1e-15
    )
    {
        set_error(
            stream,
            SINGLE_LINE_STREAM_GEOMETRY_FAILED
        );

        return false;
    }


    /* ========================================================================
     * ANALYTICAL S-CURVE SETUP
     * ========================================================================
     */

    real_t sDotMax =
        request->desiredTCPSpeed /
        stream->segmentLength;

    real_t sDDotMax =
        request->desiredTCPAccel /
        stream->segmentLength;

    real_t sDDDotMax =
        request->desiredTCPJerk /
        stream->segmentLength;


    if (
        !s_curve_profile_init(
            sDotMax,
            sDDotMax,
            sDDDotMax,
            request->dt,
            &stream->profile
        )
    )
    {
        set_error(
            stream,
            SINGLE_LINE_STREAM_PROFILE_FAILED
        );

        return false;
    }


    /* ========================================================================
     * SEQUENTIAL IK STATE
     * ========================================================================
     */

    stream->qSeed =
        request->qSeed;

    stream->nextSampleIndex =
        0;

    stream->initialized =
        true;

    stream->finished =
        false;

    stream->status =
        SINGLE_LINE_STREAM_OK;

    return true;
}


bool single_line_stream_next(
    SingleLineStream *stream,
    SingleLineStreamSample *sample
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
            set_error(
                stream,
                SINGLE_LINE_STREAM_INVALID_ARGUMENT
            );
        }

        return false;
    }


    size_t totalSamples =
        s_curve_profile_sample_count(
            &stream->profile
        );


    if (
        stream->nextSampleIndex >=
        totalSamples
    )
    {
        stream->finished =
            true;

        stream->status =
            SINGLE_LINE_STREAM_FINISHED;

        return false;
    }


    SCurveSample profileSample;

    if (
        !s_curve_profile_evaluate_index(
            &stream->profile,
            stream->nextSampleIndex,
            &profileSample
        )
    )
    {
        set_error(
            stream,
            SINGLE_LINE_STREAM_PROFILE_FAILED
        );

        return false;
    }


    /* ========================================================================
     * CARTESIAN SAMPLE
     * ========================================================================
     */

    Quat desiredQuaternion =
        quat_slerp(
            stream->request.startOrientation,
            stream->request.endOrientation,
            profileSample.s
        );


    real_t lTimed =
        stream->segmentLength *
        profileSample.s;


    real_t tcpSpeed =
        stream->segmentLength *
        profileSample.sDot;


    Vec3 desiredPosition =
        arc_length_interpolate(
            stream->workspace->arcGeometry,
            stream->workspace->lArc,
            stream->arcInfo.count,
            lTimed
        );


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


    /* ========================================================================
     * ONE SEQUENTIAL ADLS SOLVE
     * ========================================================================
     */

    double qSolutionArray[ROBOT_DOF];


    bool ikSucceeded =
        adls_ik(
            stream->robot,
            desiredPoseArray,
            stream->qSeed.q,
            &stream->request.ikParameters,
            qSolutionArray,
            stream->workspace->ikScratch
        );


    if (!ikSucceeded)
    {
        set_error(
            stream,
            SINGLE_LINE_STREAM_IK_FAILED
        );

        return false;
    }


    JointVector qSolution =
        array_to_joint_vector(
            qSolutionArray
        );


    /*
     * Preserve sequential IK continuity:
     * this solution becomes the seed for the next 1 ms sample.
     */
    stream->qSeed =
        qSolution;


    /* ========================================================================
     * INDEPENDENT FK VALIDATION
     * ========================================================================
     */

    double achievedPoseArray[4][4];


    control_fk(
        stream->robot,
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


    real_t positionError =
        vec3_norm(
            vec3_sub(
                desiredPosition,
                achievedPosition
            )
        );


    real_t orientationError =
        rotation_error(
            mat4_rotation(
                desiredPose
            ),
            mat4_rotation(
                achievedPose
            )
        );


    /* ========================================================================
     * OUTPUT ONE SAMPLE
     * ========================================================================
     */

    memset(
        sample,
        0,
        sizeof(*sample)
    );


    sample->index =
        stream->nextSampleIndex;

    sample->t =
        profileSample.t;

    sample->s =
        profileSample.s;

    sample->sDot =
        profileSample.sDot;

    sample->sDDot =
        profileSample.sDDot;

    sample->sDDDot =
        profileSample.sDDDot;

    sample->arcPosition =
        lTimed;

    sample->tcpSpeed =
        tcpSpeed;

    sample->pDesired =
        desiredPosition;

    sample->quatDesired =
        desiredQuaternion;

    sample->q =
        qSolution;

    sample->ikIterations =
        stream->workspace->ikScratch->iterations;

    sample->positionError =
        positionError;

    sample->orientationError =
        orientationError;


    /* ========================================================================
     * INCREMENTAL DIAGNOSTICS
     * ========================================================================
     */

    if (
        tcpSpeed >
        stream->peakTCPSpeed
    )
    {
        stream->peakTCPSpeed =
            tcpSpeed;
    }

    if (
        positionError >
        stream->maxPositionError
    )
    {
        stream->maxPositionError =
            positionError;
    }

    if (
        orientationError >
        stream->maxOrientationError
    )
    {
        stream->maxOrientationError =
            orientationError;
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


    if (
        stream->nextSampleIndex >=
        totalSamples
    )
    {
        stream->finished =
            true;
    }


    stream->status =
        SINGLE_LINE_STREAM_OK;

    return true;
}


size_t single_line_stream_sample_count(
    const SingleLineStream *stream
)
{
    if (
        stream == NULL ||
        !stream->initialized
    )
    {
        return 0;
    }

    return
        s_curve_profile_sample_count(
            &stream->profile
        );
}


size_t single_line_stream_samples_generated(
    const SingleLineStream *stream
)
{
    if (
        stream == NULL ||
        !stream->initialized
    )
    {
        return 0;
    }

    return
        stream->nextSampleIndex;
}


size_t single_line_stream_samples_remaining(
    const SingleLineStream *stream
)
{
    if (
        stream == NULL ||
        !stream->initialized
    )
    {
        return 0;
    }

    size_t total =
        single_line_stream_sample_count(
            stream
        );

    if (
        stream->nextSampleIndex >=
        total
    )
    {
        return 0;
    }

    return
        total -
        stream->nextSampleIndex;
}


bool single_line_stream_is_finished(
    const SingleLineStream *stream
)
{
    return
        stream != NULL &&
        stream->initialized &&
        stream->finished;
}


SingleLineStreamStatus single_line_stream_status(
    const SingleLineStream *stream
)
{
    if (stream == NULL)
    {
        return
            SINGLE_LINE_STREAM_INVALID_ARGUMENT;
    }

    return
        stream->status;
}


const char *single_line_stream_status_string(
    SingleLineStreamStatus status
)
{
    switch (status)
    {
        case SINGLE_LINE_STREAM_OK:
            return "OK";

        case SINGLE_LINE_STREAM_FINISHED:
            return "FINISHED";

        case SINGLE_LINE_STREAM_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";

        case SINGLE_LINE_STREAM_GEOMETRY_FAILED:
            return "GEOMETRY_FAILED";

        case SINGLE_LINE_STREAM_PROFILE_FAILED:
            return "PROFILE_FAILED";

        case SINGLE_LINE_STREAM_IK_FAILED:
            return "IK_FAILED";

        default:
            return "UNKNOWN";
    }
}
