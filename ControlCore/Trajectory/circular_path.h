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
 * radius, in-plane basis, and reference normal.
 *
 * MATLAB REFERENCE
 * ----------------
 * This implementation now follows the MATLAB helper files uploaded by the
 * trajectory team:
 *
 *      Control/Trajectory/generateCircleWaypoints.m
 *      Control/Trajectory/generateFullCircleWaypoints.m
 *
 * In particular, the C implementation mirrors the MATLAB sequence:
 *
 *      plane normal from cross(P2-P1, P3-P1)
 *          ->
 *      circumcenter from the 3x3 linear system A*C = rhs
 *          ->
 *      e1/e2 in-plane orthonormal basis
 *          ->
 *      signed arc-sweep selection through the middle point
 *          ->
 *      uniform angular sampling
 *
 * The earlier independently reconstructed geometry has therefore been replaced
 * by a source-matched port of the uploaded MATLAB implementation.
 * ============================================================================
 */

typedef struct
{
    Vec3 center;
    Vec3 axis;          /* MATLAB b: unit plane normal */

    real_t radius;
    real_t thetaTotal;  /* signed sweep [rad] */

    Vec3 e1;            /* unit vector center -> start */
    Vec3 e2;            /* cross(axis, e1) */

} CircularPathInfo;


/* ============================================================================
 * THREE-POINT ARC
 * ============================================================================
 *
 * Direct C equivalent of generateCircleWaypoints.m.
 *
 * Generates the circular arc that starts at start, passes through mid,
 * and ends at end. The signed sweep is selected using the same CCW/CW
 * candidate test as the MATLAB helper.
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
 * Direct C equivalent of generateFullCircleWaypoints.m.
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