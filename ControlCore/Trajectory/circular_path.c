#include "circular_path.h"

#include "../Math/math3d.h"

#include <math.h>
#include <stddef.h>


#define CIRCULAR_EPS 1e-12


/* ============================================================================
 * INTERNAL VECTOR HELPERS
 * ============================================================================
 */

static real_t vec3_dot_local(
    Vec3 a,
    Vec3 b
)
{
    return
        a.v[0] * b.v[0] +
        a.v[1] * b.v[1] +
        a.v[2] * b.v[2];
}


static bool vec3_unit_local(
    Vec3 input,
    Vec3 *output
)
{
    if (output == NULL)
    {
        return false;
    }


    real_t norm =
        vec3_norm(input);


    if (norm <= CIRCULAR_EPS)
    {
        return false;
    }


    *output =
        vec3_scale(
            input,
            1.0 / norm
        );


    return true;
}


/* ============================================================================
 * FIT A CIRCLE THROUGH THREE 3D POINTS
 * ============================================================================
 */

static bool fit_circle_three_points(
    Vec3 p1,
    Vec3 p2,
    Vec3 p3,
    CircularPathInfo *info
)
{
    if (info == NULL)
    {
        return false;
    }


    Vec3 u =
        vec3_sub(
            p2,
            p1
        );


    Vec3 v =
        vec3_sub(
            p3,
            p1
        );


    Vec3 normal =
        vec3_cross(
            u,
            v
        );


    real_t normalSquared =
        vec3_dot_local(
            normal,
            normal
        );


    if (normalSquared <= CIRCULAR_EPS * CIRCULAR_EPS)
    {
        return false;
    }


    real_t uSquared =
        vec3_dot_local(
            u,
            u
        );


    real_t vSquared =
        vec3_dot_local(
            v,
            v
        );


    /*
     * 3D circumcenter formula:
     *
     * c = p1 +
     *     ( |u|^2 (v x n) + |v|^2 (n x u) ) /
     *     ( 2 |n|^2 )
     */

    Vec3 term1 =
        vec3_scale(
            vec3_cross(
                v,
                normal
            ),
            uSquared
        );


    Vec3 term2 =
        vec3_scale(
            vec3_cross(
                normal,
                u
            ),
            vSquared
        );


    Vec3 centerOffset =
        vec3_scale(
            vec3_add(
                term1,
                term2
            ),
            1.0 /
            (2.0 * normalSquared)
        );


    info->center =
        vec3_add(
            p1,
            centerOffset
        );


    if (
        !vec3_unit_local(
            normal,
            &info->axis
        )
    )
    {
        return false;
    }


    info->radius =
        vec3_norm(
            vec3_sub(
                p1,
                info->center
            )
        );


    if (info->radius <= CIRCULAR_EPS)
    {
        return false;
    }


    info->thetaTotal =
        0.0;


    return true;
}


/* ============================================================================
 * POSITIVE ANGLE AROUND AN AXIS
 * ============================================================================
 *
 * Returns an angle in [0, 2*pi).
 * ============================================================================
 */

static real_t positive_angle_about_axis(
    Vec3 from,
    Vec3 to,
    Vec3 axis
)
{
    real_t cosine =
        clamp_real(
            vec3_dot_local(
                from,
                to
            ),
            -1.0,
             1.0
        );


    real_t sine =
        vec3_dot_local(
            axis,
            vec3_cross(
                from,
                to
            )
        );


    real_t angle =
        atan2(
            sine,
            cosine
        );


    if (angle < 0.0)
    {
        angle +=
            2.0 * ROBOT_PI;
    }


    return angle;
}


/* ============================================================================
 * SAMPLE A CIRCLE FROM START RADIAL VECTOR
 * ============================================================================
 */

static Vec3 sample_circle(
    const CircularPathInfo *info,
    Vec3 radialStart,
    real_t theta
)
{
    Vec3 tangent =
        vec3_cross(
            info->axis,
            radialStart
        );


    Vec3 radial =
        vec3_add(
            vec3_scale(
                radialStart,
                cos(theta)
            ),
            vec3_scale(
                tangent,
                sin(theta)
            )
        );


    return
        vec3_add(
            info->center,
            vec3_scale(
                radial,
                info->radius
            )
        );
}


/* ============================================================================
 * THREE-POINT ARC
 * ============================================================================
 */

bool generate_arc_waypoints(
    Vec3 start,
    Vec3 mid,
    Vec3 end,
    size_t numWaypoints,
    Vec3 *waypoints,
    CircularPathInfo *info
)
{
    if (
        waypoints == NULL ||
        info == NULL ||
        numWaypoints < 2
    )
    {
        return false;
    }


    if (
        !fit_circle_three_points(
            start,
            mid,
            end,
            info
        )
    )
    {
        return false;
    }


    Vec3 radialStart;
    Vec3 radialMid;
    Vec3 radialEnd;


    if (
        !vec3_unit_local(
            vec3_sub(
                start,
                info->center
            ),
            &radialStart
        ) ||
        !vec3_unit_local(
            vec3_sub(
                mid,
                info->center
            ),
            &radialMid
        ) ||
        !vec3_unit_local(
            vec3_sub(
                end,
                info->center
            ),
            &radialEnd
        )
    )
    {
        return false;
    }


    real_t thetaMid =
        positive_angle_about_axis(
            radialStart,
            radialMid,
            info->axis
        );


    real_t thetaEnd =
        positive_angle_about_axis(
            radialStart,
            radialEnd,
            info->axis
        );


    /*
     * Because the fitted normal is cross(mid-start, end-start), the positive
     * sweep should encounter mid before end. Reject degenerate/numerically
     * inconsistent input instead of silently choosing the wrong arc.
     */
    if (
        thetaMid <= CIRCULAR_EPS ||
        thetaEnd <= thetaMid + CIRCULAR_EPS ||
        thetaEnd >= 2.0 * ROBOT_PI - CIRCULAR_EPS
    )
    {
        return false;
    }


    info->thetaTotal =
        thetaEnd;


    for (size_t i = 0;
         i < numWaypoints;
         i++)
    {
        real_t s =
            (real_t)i /
            (real_t)(numWaypoints - 1);


        real_t theta =
            s *
            info->thetaTotal;


        waypoints[i] =
            sample_circle(
                info,
                radialStart,
                theta
            );
    }


    /*
     * Force exact taught endpoints so floating-point trig cannot perturb them.
     */
    waypoints[0] =
        start;

    waypoints[numWaypoints - 1] =
        end;


    return true;
}


/* ============================================================================
 * FULL CIRCLE
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
)
{
    if (
        waypoints == NULL ||
        info == NULL ||
        numWaypoints < 3 ||
        (direction != 1 && direction != -1)
    )
    {
        return false;
    }


    if (
        !fit_circle_three_points(
            p1,
            p2,
            p3,
            info
        )
    )
    {
        return false;
    }


    Vec3 radialStart;


    if (
        !vec3_unit_local(
            vec3_sub(
                p1,
                info->center
            ),
            &radialStart
        )
    )
    {
        return false;
    }


    info->thetaTotal =
        (real_t)direction *
        2.0 * ROBOT_PI;


    for (size_t i = 0;
         i < numWaypoints;
         i++)
    {
        real_t s =
            (real_t)i /
            (real_t)(numWaypoints - 1);


        real_t theta =
            s *
            info->thetaTotal;


        waypoints[i] =
            sample_circle(
                info,
                radialStart,
                theta
            );
    }


    /* Closed-loop endpoint must be exactly identical to the start point. */
    waypoints[0] =
        p1;

    waypoints[numWaypoints - 1] =
        p1;


    return true;
}


/* ============================================================================
 * QUATERNION HELPERS FOR ARC ORIENTATION
 * ============================================================================
 */

static Quat quat_multiply_local(
    Quat a,
    Quat b
)
{
    Quat q =
    {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };


    return q;
}


static Quat quat_conjugate_local(
    Quat q
)
{
    Quat result =
    {
         q.w,
        -q.x,
        -q.y,
        -q.z
    };


    return result;
}


static Quat axis_angle_quaternion(
    Vec3 axis,
    real_t angle
)
{
    Vec3 unitAxis;


    if (
        !vec3_unit_local(
            axis,
            &unitAxis
        )
    )
    {
        Quat identity =
        {
            1.0,
            0.0,
            0.0,
            0.0
        };

        return identity;
    }


    real_t halfAngle =
        0.5 * angle;


    real_t sine =
        sin(halfAngle);


    Quat q =
    {
        cos(halfAngle),
        sine * unitAxis.v[0],
        sine * unitAxis.v[1],
        sine * unitAxis.v[2]
    };


    return quat_normalize(q);
}


/* ============================================================================
 * ORIENTATION ALONG A CIRCULAR PATH
 * ============================================================================
 */

Quat circular_path_orientation(
    Quat startOrientation,
    Quat endOrientation,
    Vec3 axis,
    real_t thetaTotal,
    real_t s
)
{
    startOrientation =
        quat_normalize(
            startOrientation
        );


    endOrientation =
        quat_normalize(
            endOrientation
        );


    s =
        clamp_real(
            s,
            0.0,
            1.0
        );


    Quat pathAtS =
        axis_angle_quaternion(
            axis,
            s * thetaTotal
        );


    Quat predictedAtS =
        quat_multiply_local(
            pathAtS,
            startOrientation
        );


    Quat pathAtEnd =
        axis_angle_quaternion(
            axis,
            thetaTotal
        );


    Quat predictedAtEnd =
        quat_multiply_local(
            pathAtEnd,
            startOrientation
        );


    Quat leftover =
        quat_multiply_local(
            endOrientation,
            quat_conjugate_local(
                predictedAtEnd
            )
        );


    leftover =
        quat_normalize(
            leftover
        );


    /* Match the shortest-path convention used by the MATLAB reference. */
    if (leftover.w < 0.0)
    {
        leftover.w = -leftover.w;
        leftover.x = -leftover.x;
        leftover.y = -leftover.y;
        leftover.z = -leftover.z;
    }


    Quat identity =
    {
        1.0,
        0.0,
        0.0,
        0.0
    };


    Quat leftoverAtS =
        quat_slerp(
            identity,
            leftover,
            s
        );


    return
        quat_normalize(
            quat_multiply_local(
                leftoverAtS,
                predictedAtS
            )
        );
}