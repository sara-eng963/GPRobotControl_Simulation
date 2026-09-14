#ifndef SINGLE_SEGMENT_CIRCULAR_STREAM_H
#define SINGLE_SEGMENT_CIRCULAR_STREAM_H

#include "single_segment_circular.h"

#include "../Trajectory/arc_length_parameterize.h"
#include "../Trajectory/s_curve_profile.h"

#include <stdbool.h>
#include <stddef.h>

/*
 * Incremental / streaming version of the circular Cartesian pipeline.
 *
 * It supports both:
 *
 *   - three-point circular arc: A -> via B -> C
 *   - full circle through A, B and C
 *
 * Mathematical sequence:
 *
 *   circular geometry
 *      -> arc-length parameterization
 *      -> analytical S-curve s(t)
 *      -> circular position + path-induced orientation
 *      -> sequential ADLS IK
 *      -> q[k]
 *
 * One call to single_circular_stream_next() produces exactly one trajectory
 * sample, so the live controller can feed the same fixed-size FIFO used by the
 * straight-line stream without allocating a trajectory-sized qPath array.
 */

typedef enum
{
    SINGLE_CIRCULAR_STREAM_OK = 0,
    SINGLE_CIRCULAR_STREAM_FINISHED,
    SINGLE_CIRCULAR_STREAM_INVALID_ARGUMENT,
    SINGLE_CIRCULAR_STREAM_GEOMETRY_FAILED,
    SINGLE_CIRCULAR_STREAM_PROFILE_FAILED,
    SINGLE_CIRCULAR_STREAM_IK_FAILED

} SingleCircularStreamStatus;


typedef struct
{
    size_t geometryCapacity;

    Vec3 *rawGeometry;
    Vec3 *arcGeometry;

    real_t *lOriginal;
    real_t *lArc;

    ADLSInfo *ikScratch;

} SingleCircularStreamWorkspace;


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

} SingleCircularStreamSample;


typedef struct
{
    bool initialized;
    bool finished;

    SingleCircularStreamStatus status;

    const RobotConfig *robot;
    SingleCircularRequest request;
    SingleCircularStreamWorkspace *workspace;

    CircularPathInfo circularInfo;
    ArcLengthInfo arcInfo;

    real_t segmentLength;
    SCurveProfile profile;

    size_t nextSampleIndex;
    JointVector qSeed;

    real_t peakTCPSpeed;
    real_t maxPositionError;
    real_t maxOrientationError;
    int maxIKIterations;

} SingleCircularStream;


bool single_circular_stream_init(
    SingleCircularStream *stream,
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularStreamWorkspace *workspace
);


bool single_circular_stream_next(
    SingleCircularStream *stream,
    SingleCircularStreamSample *sample
);


size_t single_circular_stream_sample_count(
    const SingleCircularStream *stream
);


size_t single_circular_stream_samples_generated(
    const SingleCircularStream *stream
);


size_t single_circular_stream_samples_remaining(
    const SingleCircularStream *stream
);


bool single_circular_stream_is_finished(
    const SingleCircularStream *stream
);


SingleCircularStreamStatus single_circular_stream_status(
    const SingleCircularStream *stream
);


const char *single_circular_stream_status_string(
    SingleCircularStreamStatus status
);


#endif /* SINGLE_SEGMENT_CIRCULAR_STREAM_H */
