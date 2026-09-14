#include "arc_length_parameterize.h"

#include <math.h>
#include <stddef.h>


/* ============================================================================
 * INTERNAL DISTANCE BETWEEN TWO CARTESIAN POINTS
 * ============================================================================
 *
 * Equivalent to:
 *
 *      vecnorm(segmentVectors,2,1)
 */

static real_t point_distance(
    Vec3 a,
    Vec3 b
)
{
    real_t dx =
        b.v[0] - a.v[0];

    real_t dy =
        b.v[1] - a.v[1];

    real_t dz =
        b.v[2] - a.v[2];


    return sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    );
}


/* ============================================================================
 * INTERPOLATE ONE CARTESIAN POINT
 * ============================================================================
 */

Vec3 arc_length_interpolate(
    const Vec3 *path,
    const real_t *arcLength,
    size_t count,
    real_t queryArcLength
)
{
    Vec3 result =
    {{
        0.0,
        0.0,
        0.0
    }};


    if (
        path == NULL ||
        arcLength == NULL ||
        count == 0
    )
    {
        return result;
    }


    /*
     * Clamp to first point.
     */
    if (queryArcLength <= arcLength[0])
    {
        return path[0];
    }


    /*
     * Clamp to final point.
     */
    if (
        queryArcLength >=
        arcLength[count - 1]
    )
    {
        return path[count - 1];
    }


    /* ========================================================================
     * Equivalent to MATLAB linear interp1().
     * ========================================================================
     */

    size_t upperIndex =
        1;


    while (
        upperIndex < count &&
        arcLength[upperIndex] <
            queryArcLength
    )
    {
        upperIndex++;
    }


    size_t lowerIndex =
        upperIndex - 1;


    real_t l0 =
        arcLength[lowerIndex];

    real_t l1 =
        arcLength[upperIndex];


    real_t denominator =
        l1 - l0;


    /*
     * Protect against duplicate arc-length values.
     */
    if (fabs(denominator) < 1e-15)
    {
        return path[lowerIndex];
    }


    real_t u =
        (
            queryArcLength -
            l0
        )
        /
        denominator;


    for (int axis = 0;
         axis < 3;
         axis++)
    {
        result.v[axis] =
            path[lowerIndex].v[axis]
            +
            u *
            (
                path[upperIndex].v[axis]
                -
                path[lowerIndex].v[axis]
            );
    }


    return result;
}


/* ============================================================================
 * ARC-LENGTH PARAMETERIZATION
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
)
{
    /* ========================================================================
     * VALIDATE INPUTS
     * ========================================================================
     *
     * MATLAB checks:
     *
     *      size(pathPoints,1) == 3
     *
     * That dimension is guaranteed by Vec3 in C.
     */

    if (
        pathPoints == NULL ||
        resampledPath == NULL ||
        sOriginal == NULL ||
        sNew == NULL ||
        info == NULL ||
        pointCount == 0 ||
        spacing <= 0.0
    )
    {
        return false;
    }


    /*
     * sOriginal contains one entry per original path point.
     */
    if (pointCount > outputCapacity)
    {
        return false;
    }


    /* ========================================================================
     * CALCULATE CUMULATIVE ARC LENGTH
     * ========================================================================
     *
     * MATLAB:
     *
     *      segmentVectors =
     *          diff(pathPoints,1,2);
     *
     *      segmentLengths =
     *          vecnorm(
     *              segmentVectors,
     *              2,
     *              1
     *          );
     *
     *      sOriginal =
     *          [0,cumsum(segmentLengths)];
     */

    sOriginal[0] =
        0.0;


    for (size_t i = 1;
         i < pointCount;
         i++)
    {
        real_t segmentLength =
            point_distance(
                pathPoints[i - 1],
                pathPoints[i]
            );


        sOriginal[i] =
            sOriginal[i - 1]
            +
            segmentLength;
    }


    real_t totalLength =
        sOriginal[pointCount - 1];


    /* ========================================================================
     * GENERATE:
     *
     *      sNew = 0:spacing:totalLength
     * ========================================================================
     */

    size_t newCount =
        0;


    real_t currentArcLength =
        0.0;


    /*
     * MATLAB colon:
     *
     *      0:spacing:totalLength
     */
    while (
        currentArcLength <=
        totalLength
    )
    {
        if (newCount >= outputCapacity)
        {
            return false;
        }


        sNew[newCount] =
            currentArcLength;


        newCount++;


        currentArcLength =
            (real_t)newCount *
            spacing;
    }


    /*
     * Normally 0 is always generated.
     *
     * Keep defensive handling for numerical edge cases.
     */
    if (newCount == 0)
    {
        if (outputCapacity == 0)
        {
            return false;
        }


        sNew[0] =
            0.0;

        newCount =
            1;
    }


    /* ========================================================================
     * MATLAB:
     *
     *      if sNew(end) < totalLength
     *
     *          sNew =
     *              [sNew,totalLength];
     *
     *      end
     * ========================================================================
     */

    if (
        sNew[newCount - 1] <
        totalLength
    )
    {
        if (newCount >= outputCapacity)
        {
            return false;
        }


        sNew[newCount] =
            totalLength;


        newCount++;
    }


    /* ========================================================================
     * INTERPOLATE CARTESIAN COORDINATES
     * ========================================================================
     *
     * MATLAB:
     *
     *      for k = 1:3
     *
     *          resampledPath(k,:) =
     *              interp1(
     *                  sOriginal,
     *                  pathPoints(k,:),
     *                  sNew,
     *                  'linear'
     *              );
     *
     *      end
     * ========================================================================
     */

    for (size_t i = 0;
         i < newCount;
         i++)
    {
        resampledPath[i] =
            arc_length_interpolate(
                pathPoints,
                sOriginal,
                pointCount,
                sNew[i]
            );
    }


    /* ========================================================================
     * OUTPUT INFORMATION
     * ========================================================================
     */

    info->count =
        newCount;


    info->totalLength =
        totalLength;


    return true;
}