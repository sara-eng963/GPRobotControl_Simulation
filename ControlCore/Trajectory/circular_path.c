#include "circular_path.h"

#include "../Math/math3d.h"

#include <math.h>
#include <stddef.h>


#define CIRCULAR_COLLINEAR_EPS 1e-12
#define CIRCULAR_RADIUS_EPS    1e-9
#define CIRCULAR_ANGLE_EPS     1e-12
#define CIRCULAR_SOLVE_EPS     1e-15


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


/* ============================================================================
 * MATLAB-LIKE 3x3 LINEAR SOLVE
 * ============================================================================
 *
 * The MATLAB helpers compute the circumcenter with:
 *
 *      C = A \\ rhs
 *
 * This small Gaussian-elimination solver reproduces that same mathematical
 * step for the fixed 3x3 system used by the circle fit.
 * ============================================================================
 */

static bool solve_3x3(
    real_t A[3][3],
    real_t rhs[3],
    Vec3 *solution
)
{
    if (solution == NULL)
    {
        return false;
    }


    real_t augmented[3][4];


    for (int row = 0;
         row < 3;
         row++)
    {
        for (int column = 0;
             column < 3;
             column++)
        {
            augmented[row][column] =
                A[row][column];
        }

        augmented[row][3] =
            rhs[row];
    }


    for (int pivot = 0;
         pivot < 3;
         pivot++)
    {
        int best_row =
            pivot;

        real_t best_value =
            fabs(
                augmented[pivot][pivot]
            );


        for (int row = pivot + 1;
             row < 3;
             row++)
        {
            real_t candidate =
                fabs(
                    augmented[row][pivot]
                );

            if (candidate > best_value)
            {
                best_value =
                    candidate;

                best_row =
                    row;
            }
        }


        if (best_value <= CIRCULAR_SOLVE_EPS)
        {
            return false;
        }


        if (best_row != pivot)
        {
            for (int column = pivot;
                 column < 4;
                 column++)
            {
                real_t temporary =
                    augmented[pivot][column];

                augmented[pivot][column] =
                    augmented[best_row][column];

                augmented[best_row][column] =
                    temporary;
            }
        }


        real_t pivot_value =
            augmented[pivot][pivot];


        for (int row = pivot + 1;
             row < 3;
             row++)
        {
            real_t factor =
                augmented[row][pivot] /
                pivot_value;


            for (int column = pivot;
                 column < 4;
                 column++)
            {
                augmented[row][column] -=
                    factor *
                    augmented[pivot][column];
            }
        }
    }


    real_t x[3] =
    {
        0.0,
        0.0,
        0.0
    };


    for (int row = 2;
         row >= 0;
         row--)
    {
        real_t value =
            augmented[row][3];


        for (int column = row + 1;
             column < 3;
             column++)
        {
            value -=
                augmented[row][column] *
                x[column];
        }


        if (
            fabs(
                augmented[row][row]
            ) <= CIRCULAR_SOLVE_EPS
        )
        {
            return false;
        }


        x[row] =
            value /
            augmented[row][row];
    }


    solution->v[0] = x[0];
    solution->v[1] = x[1];
    solution->v[2] = x[2];


    return true;
}


/* ============================================================================
 * FIT A CIRCLE THROUGH THREE 3D POINTS
 * ============================================================================
 *
 * Direct port of the common geometry setup used by both MATLAB helpers:
 *
 *      generateCircleWaypoints.m
 *      generateFullCircleWaypoints.m
 *
 * MATLAB sequence:
 *
 *      v1 = P2 - P1
 *      v2 = P3 - P1
 *      b  = unit(cross(v1,v2))
 *
 *      A = [v1'; v2'; b']
 *      rhs = [
 *          0.5*(P2'*P2 - P1'*P1)
 *          0.5*(P3'*P3 - P1'*P1)
 *          b'*P1
 *      ]
 *
 *      C = A \\ rhs
 *      e1 = unit(P1-C)
 *      e2 = cross(b,e1)
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


    Vec3 v1 =
        vec3_sub(
            p2,
            p1
        );


    Vec3 v2 =
        vec3_sub(
            p3,
            p1
        );


    Vec3 normal_raw =
        vec3_cross(
            v1,
            v2
        );


    real_t normal_norm =
        vec3_norm(
            normal_raw
        );


    if (normal_norm < CIRCULAR_COLLINEAR_EPS)
    {
        return false;
    }


    info->axis =
        vec3_scale(
            normal_raw,
            1.0 / normal_norm
        );


    real_t A[3][3] =
    {
        {
            v1.v[0],
            v1.v[1],
            v1.v[2]
        },
        {
            v2.v[0],
            v2.v[1],
            v2.v[2]
        },
        {
            info->axis.v[0],
            info->axis.v[1],
            info->axis.v[2]
        }
    };


    real_t rhs[3] =
    {
        0.5 *
        (
            vec3_dot_local(p2, p2) -
            vec3_dot_local(p1, p1)
        ),

        0.5 *
        (
            vec3_dot_local(p3, p3) -
            vec3_dot_local(p1, p1)
        ),

        vec3_dot_local(
            info->axis,
            p1
        )
    };


    if (
        !solve_3x3(
            A,
            rhs,
            &info->center
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


    if (info->radius < CIRCULAR_RADIUS_EPS)
    {
        return false;
    }


    info->e1 =
        vec3_scale(
            vec3_sub(
                p1,
                info->center
            ),
            1.0 /
            info->radius
        );


    info->e2 =
        vec3_cross(
            info->axis,
            info->e1
        );


    info->thetaTotal =
        0.0;


    return true;
}


/* ============================================================================
 * MATLAB mod(angle, 2*pi)
 * ============================================================================
 *
 * C fmod() keeps the sign of the dividend, while MATLAB mod() with a positive
 * divisor returns a value in [0, 2*pi). This helper preserves MATLAB behavior.
 * ============================================================================
 */

static real_t mod_two_pi(
    real_t angle
)
{
    real_t period =
        2.0 * ROBOT_PI;


    real_t result =
        fmod(
            angle,
            period
        );


    if (result < 0.0)
    {
        result +=
            period;
    }


    return result;
}


/* ============================================================================
 * SAMPLE A CIRCLE IN THE MATLAB e1/e2 BASIS
 * ============================================================================
 */

static Vec3 sample_circle(
    const CircularPathInfo *info,
    real_t theta
)
{
    Vec3 radial =
        vec3_add(
            vec3_scale(
                info->e1,
                cos(theta)
            ),
            vec3_scale(
                info->e2,
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
 *
 * Direct C port of generateCircleWaypoints.m.
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


    Vec3 mid_from_center =
        vec3_sub(
            mid,
            info->center
        );


    Vec3 end_from_center =
        vec3_sub(
            end,
            info->center
        );


    real_t thetaMid =
        atan2(
            vec3_dot_local(
                mid_from_center,
                info->e2
            ),
            vec3_dot_local(
                mid_from_center,
                info->e1
            )
        );


    real_t thetaEnd =
        atan2(
            vec3_dot_local(
                end_from_center,
                info->e2
            ),
            vec3_dot_local(
                end_from_center,
                info->e1
            )
        );


    /* ------------------------------------------------------------------------
     * Pick the sweep direction that actually passes through the middle point.
     * This mirrors the MATLAB helper exactly.
     * ------------------------------------------------------------------------
     */

    real_t sweepCCW =
        mod_two_pi(
            thetaEnd
        );


    if (sweepCCW < CIRCULAR_ANGLE_EPS)
    {
        sweepCCW =
            2.0 * ROBOT_PI;
    }


    real_t thetaMidCCW =
        mod_two_pi(
            thetaMid
        );


    bool ccwWorks =
        thetaMidCCW <=
            sweepCCW +
            CIRCULAR_ANGLE_EPS &&
        thetaMidCCW >
            CIRCULAR_ANGLE_EPS;


    real_t sweepCW =
        -mod_two_pi(
            -thetaEnd
        );


    if (sweepCW > -CIRCULAR_ANGLE_EPS)
    {
        sweepCW =
            -2.0 * ROBOT_PI;
    }


    real_t thetaMidCW =
        -mod_two_pi(
            -thetaMid
        );


    bool cwWorks =
        thetaMidCW >=
            sweepCW -
            CIRCULAR_ANGLE_EPS &&
        thetaMidCW <
            -CIRCULAR_ANGLE_EPS;


    if (
        ccwWorks &&
        !cwWorks
    )
    {
        info->thetaTotal =
            sweepCCW;
    }
    else if (
        cwWorks &&
        !ccwWorks
    )
    {
        info->thetaTotal =
            sweepCW;
    }
    else
    {
        return false;
    }


    /* ------------------------------------------------------------------------
     * Uniform angular sampling, matching MATLAB:
     *
     *      s = (i-1)/(N-1)
     *      theta = thetaTotal*s
     * ------------------------------------------------------------------------
     */

    for (size_t i = 0;
         i < numWaypoints;
         i++)
    {
        real_t s =
            (real_t)i /
            (real_t)(numWaypoints - 1);


        real_t theta =
            info->thetaTotal *
            s;


        waypoints[i] =
            sample_circle(
                info,
                theta
            );
    }


    /*
     * Do not overwrite the first/last points here. The MATLAB arc helper also
     * leaves the sampled values exactly as produced by the circle equation.
     */

    return true;
}


/* ============================================================================
 * FULL CIRCLE
 * ============================================================================
 *
 * Direct C port of generateFullCircleWaypoints.m.
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
            info->thetaTotal *
            s;


        waypoints[i] =
            sample_circle(
                info,
                theta
            );
    }


    /*
     * Match MATLAB exactly: only the closing point is forced to P1 in order to
     * eliminate floating-point drift and make the loop truly closed.
     */
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


static bool vec3_unit_orientation(
    Vec3 input,
    Vec3 *output
)
{
    if (output == NULL)
    {
        return false;
    }


    real_t norm =
        vec3_norm(
            input
        );


    if (norm <= CIRCULAR_COLLINEAR_EPS)
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


static Quat axis_angle_quaternion(
    Vec3 axis,
    real_t angle
)
{
    Vec3 unitAxis;


    if (
        !vec3_unit_orientation(
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
