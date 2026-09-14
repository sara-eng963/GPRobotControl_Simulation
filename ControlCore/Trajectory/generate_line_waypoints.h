#ifndef GENERATE_LINE_WAYPOINTS_H
#define GENERATE_LINE_WAYPOINTS_H


#include "../Math/control_types.h"

#include <stdbool.h>
#include <stddef.h>


/* ============================================================================
 * STRAIGHT-LINE CARTESIAN WAYPOINT GENERATION
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      generateLineWaypoints.m
 *
 *
 * MATLAB:
 *
 *      waypoints =
 *          generateLineWaypoints(
 *              P_start,
 *              P_end,
 *              numWaypoints
 *          );
 *
 *
 * Inputs:
 *
 *      start
 *          Starting Cartesian point [m].
 *
 *      end
 *          Ending Cartesian point [m].
 *
 *      numWaypoints
 *          Number of generated points INCLUDING start and end.
 *
 *
 * Output:
 *
 *      waypoints
 *          Caller-provided array with numWaypoints elements.
 *
 *
 * MATLAB interpolation:
 *
 *      s = (i - 1) / (numWaypoints - 1)
 *
 *      P =
 *          P_start +
 *          s * (P_end - P_start)
 *
 * ============================================================================
 */

bool generate_line_waypoints(
    Vec3 start,
    Vec3 end,
    size_t numWaypoints,
    Vec3 *waypoints
);


#endif /* GENERATE_LINE_WAYPOINTS_H */