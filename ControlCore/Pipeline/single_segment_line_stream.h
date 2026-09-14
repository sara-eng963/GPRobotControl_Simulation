#ifndef SINGLE_SEGMENT_LINE_STREAM_H
#define SINGLE_SEGMENT_LINE_STREAM_H

#include "single_segment_line.h"

#include "../Trajectory/arc_length_parameterize.h"
#include "../Trajectory/s_curve_profile.h"

#include <stdbool.h>
#include <stddef.h>

/*
 * Incremental version of the validated single Cartesian line pipeline.
 *
 * It preserves the same mathematical sequence:
 *
 *   line geometry
 *      -> arc-length parameterization
 *      -> S-curve s(t)
 *      -> Cartesian position + SLERP
 *      -> sequential ADLS IK
 *      -> q[k]
 *
 * Difference from plan_single_segment_line():
 *
 *   plan_single_segment_line()
 *       generates and stores the whole trajectory.
 *
 *   single_line_stream_next()
 *       generates exactly ONE next sample.
 *
 * Therefore total motion duration is no longer tied to a trajectory-sized
 * qPath[] allocation.
 */

typedef enum
{
    SINGLE_LINE_STREAM_OK = 0,
    SINGLE_LINE_STREAM_FINISHED,
    SINGLE_LINE_STREAM_INVALID_ARGUMENT,
    SINGLE_LINE_STREAM_GEOMETRY_FAILED,
    SINGLE_LINE_STREAM_PROFILE_FAILED,
    SINGLE_LINE_STREAM_IK_FAILED

} SingleLineStreamStatus;


typedef struct
{
    size_t geometryCapacity;

    Vec3 *rawGeometry;
    Vec3 *arcGeometry;

    real_t *lOriginal;
    real_t *lArc;

    ADLSInfo *ikScratch;

} SingleLineStreamWorkspace;


typedef struct
{
    size_t index;

    real_t t;

    real_t s;
    real_t sDot;
    real_t sDDot;
    real_t sDDDot;

    real_t arcPosition;
    real_t tcpSpeed;

    Vec3 pDesired;
    Quat quatDesired;

    JointVector q;

    int ikIterations;

    real_t positionError;
    real_t orientationError;

} SingleLineStreamSample;


typedef struct
{
    bool initialized;
    bool finished;

    SingleLineStreamStatus status;

    const RobotConfig *robot;

    SingleLineRequest request;

    SingleLineStreamWorkspace *workspace;

    ArcLengthInfo arcInfo;

    real_t segmentLength;

    SCurveProfile profile;

    size_t nextSampleIndex;

    JointVector qSeed;

    /* Incremental diagnostics. */
    real_t peakTCPSpeed;
    real_t maxPositionError;
    real_t maxOrientationError;
    int maxIKIterations;

} SingleLineStream;


/*
 * Initialize one absolute A -> B motion.
 *
 * This performs only fixed-size planning work:
 * - line geometry
 * - arc-length parameterization
 * - analytical seven-stage S-curve setup
 *
 * It does NOT generate all time samples or all IK solutions.
 */
bool single_line_stream_init(
    SingleLineStream *stream,
    const RobotConfig *robot,
    const SingleLineRequest *request,
    SingleLineStreamWorkspace *workspace
);


/*
 * Generate exactly one next 1 ms trajectory sample.
 *
 * Returns true when a sample was produced.
 * Returns false when finished or when an error occurred.
 * Inspect single_line_stream_status() to distinguish the two cases.
 */
bool single_line_stream_next(
    SingleLineStream *stream,
    SingleLineStreamSample *sample
);


size_t single_line_stream_sample_count(
    const SingleLineStream *stream
);


size_t single_line_stream_samples_generated(
    const SingleLineStream *stream
);


size_t single_line_stream_samples_remaining(
    const SingleLineStream *stream
);


bool single_line_stream_is_finished(
    const SingleLineStream *stream
);


SingleLineStreamStatus single_line_stream_status(
    const SingleLineStream *stream
);


const char *single_line_stream_status_string(
    SingleLineStreamStatus status
);


#endif /* SINGLE_SEGMENT_LINE_STREAM_H */
