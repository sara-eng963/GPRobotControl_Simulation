#ifndef CIRCULAR_PATH_H
#define CIRCULAR_PATH_H


#include "../Math/control_types.h"

#include <stdbool.h>
#include <stddef.h>


/* ============================================================================
 * CIRCULAR CARTESIAN PATH GEOMETRY
 * ============================================================================
 *
 * Shared geometry used by both:
 *
 *      - three-point circular arcs
 *      - full circles
 *
 * The three supplied Cartesian points define the circle plane, center,
 * radius, and reference normal.
 *
 * VALIDATION NOTE:
 *
 * The circular geometry routines in this module were independently
 * constructed from the interface and expected behavior of the available
 * MATLAB trajectory pipelines. The original MATLAB circle-geometry helper
 * implementations were not present in the reference repository at the time
 * this C implementation was written.
 *
 * The implementation is therefore functionally tested and mathematically
 * consistent, but direct source-to-source validation against the original
 * MATLAB geometry helpers is still pending until those files are available.
 * ============================================================================
 */

typedef struct
{
    Vec3 center;
    Vec3 axis;          /* unit plane normal */

    real_t radius;
    real_t thetaTotal;  /* signed sweep [rad] */

} CircularPathInfo;


/* ============================================================================
 * THREE-POINT ARC
 * ============================================================================
 *
 * Generates the circular arc that starts at start, passes through mid,
 * and ends at end.
 *
 * The returned axis direction is chosen from:
 *
 *      cross(mid - start, end - start)
 *
 * so thetaTotal is positive for the ordered start -> mid -> end arc.
 * ============================================================================
 */

bool generate_arc_waypoints(
    Vec3 start,
    Vec3 mid,
    Vec3 end,
    size_t numWaypoints,
    Vec3 *waypoints,
    CircularPathInfo *info
);


/* ============================================================================
 * FULL CIRCLE
 * ============================================================================
 *
 * p1 is the start/end point. p2 and p3 define the same circle geometry.
 *
 * direction:
 *
 *      +1 -> positive rotation about the fitted axis (CCW by right-hand rule)
 *      -1 -> negative rotation about the fitted axis (CW)
 * ============================================================================
 */

bool generate_full_circle_waypoints(
    Vec3 p1,
    Vec3 p2,
    Vec3 p3,
    int direction,
    size_t numWaypoints,
    Vec3 *waypoints,
    CircularPathInfo *info
);


/* ============================================================================
 * ORIENTATION ALONG A CIRCULAR PATH
 * ============================================================================
 *
 * C equivalent of orientationFromArc() used by the MATLAB reference scripts.
 *
 * The tool orientation receives the natural path-induced rotation about the
 * circle axis, plus a gradually applied leftover twist so that:
 *
 *      s = 0 -> startOrientation
 *      s = 1 -> endOrientation
 * ============================================================================
 */

Quat circular_path_orientation(
    Quat startOrientation,
    Quat endOrientation,
    Vec3 axis,
    real_t thetaTotal,
    real_t s
);


#endif /* CIRCULAR_PATH_H */