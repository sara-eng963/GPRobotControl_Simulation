#ifndef ARC_LENGTH_PARAMETERIZE_H
#define ARC_LENGTH_PARAMETERIZE_H


#include "../Math/control_types.h"

#include <stdbool.h>
#include <stddef.h>


/* ============================================================================
 * ARC-LENGTH RESULT INFORMATION
 * ============================================================================
 */

typedef struct
{
    /*
     * Number of resampled arc-length points.
     */
    size_t count;


    /*
     * Total physical path length [m].
     *
     * MATLAB:
     *
     *      totalLength =
     *          sOriginal(end);
     */
    real_t totalLength;

} ArcLengthInfo;


/* ============================================================================
 * ARC-LENGTH PARAMETERIZATION
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      arcLengthParameterize.m
 *
 *
 * MATLAB:
 *
 * [resampledPath,sOriginal,sNew] =
 *      arcLengthParameterize(
 *          pathPoints,
 *          spacing
 *      );
 *
 *
 * Inputs:
 *
 *      pathPoints
 *          Original Cartesian path.
 *
 *      pointCount
 *          Number of original Cartesian samples.
 *
 *      spacing
 *          Desired physical spacing [m].
 *
 *
 * Outputs:
 *
 *      resampledPath
 *          Arc-length-spaced Cartesian path.
 *
 *      sOriginal
 *          Cumulative distance at each original path point.
 *
 *      sNew
 *          Physical arc-length values of the resampled path.
 *
 *      info
 *          Number of resampled points and total length.
 *
 * ============================================================================
 */

bool arc_length_parameterize(
    const Vec3 *pathPoints,
    size_t pointCount,
    real_t spacing,
    Vec3 *resampledPath,
    real_t *sOriginal,
    real_t *sNew,
    size_t outputCapacity,
    ArcLengthInfo *info
);


/* ============================================================================
 * INTERPOLATE POSITION AT A PHYSICAL ARC LENGTH
 * ============================================================================
 *
 * Equivalent to the pipeline's MATLAB:
 *
 *      interp1(
 *          lArc,
 *          arcSegment(axisIndex,:),
 *          lTimed,
 *          'linear'
 *      )
 *
 * This helper evaluates all three Cartesian coordinates together.
 * ============================================================================
 */

Vec3 arc_length_interpolate(
    const Vec3 *path,
    const real_t *arcLength,
    size_t count,
    real_t queryArcLength
);


#endif /* ARC_LENGTH_PARAMETERIZE_H */