#include "math3d.h"

#include <math.h>


/* ============================================================================
 * SCALAR HELPER
 * ============================================================================
 */

real_t clamp_real(
    real_t x,
    real_t lower,
    real_t upper
)
{
    if (x < lower)
    {
        return lower;
    }

    if (x > upper)
    {
        return upper;
    }

    return x;
}


/* ============================================================================
 * VECTOR OPERATIONS
 * ============================================================================
 */

real_t vec3_norm(
    Vec3 a
)
{
    return sqrt(
        a.v[0] * a.v[0] +
        a.v[1] * a.v[1] +
        a.v[2] * a.v[2]
    );
}


Vec3 vec3_add(
    Vec3 a,
    Vec3 b
)
{
    Vec3 result =
    {{
        a.v[0] + b.v[0],
        a.v[1] + b.v[1],
        a.v[2] + b.v[2]
    }};

    return result;
}


Vec3 vec3_sub(
    Vec3 a,
    Vec3 b
)
{
    Vec3 result =
    {{
        a.v[0] - b.v[0],
        a.v[1] - b.v[1],
        a.v[2] - b.v[2]
    }};

    return result;
}


Vec3 vec3_scale(
    Vec3 a,
    real_t scale
)
{
    Vec3 result =
    {{
        a.v[0] * scale,
        a.v[1] * scale,
        a.v[2] * scale
    }};

    return result;
}


Vec3 vec3_cross(
    Vec3 a,
    Vec3 b
)
{
    Vec3 result =
    {{
        a.v[1] * b.v[2] -
        a.v[2] * b.v[1],

        a.v[2] * b.v[0] -
        a.v[0] * b.v[2],

        a.v[0] * b.v[1] -
        a.v[1] * b.v[0]
    }};

    return result;
}


/* ============================================================================
 * 3x3 MATRIX OPERATIONS
 * ============================================================================
 */

Mat3 mat3_identity(void)
{
    Mat3 result = {0};

    result.m[0][0] = 1.0;
    result.m[1][1] = 1.0;
    result.m[2][2] = 1.0;

    return result;
}


Mat3 mat3_mul(
    Mat3 A,
    Mat3 B
)
{
    Mat3 C = {0};


    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            for (int k = 0; k < 3; k++)
            {
                C.m[row][column] +=
                    A.m[row][k] *
                    B.m[k][column];
            }
        }
    }


    return C;
}


Mat3 mat3_transpose(
    Mat3 A
)
{
    Mat3 result;


    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            result.m[row][column] =
                A.m[column][row];
        }
    }


    return result;
}


Vec3 mat3_mul_vec(
    Mat3 A,
    Vec3 v
)
{
    Vec3 result =
    {{
        0.0,
        0.0,
        0.0
    }};


    for (int row = 0; row < 3; row++)
    {
        for (int k = 0; k < 3; k++)
        {
            result.v[row] +=
                A.m[row][k] *
                v.v[k];
        }
    }


    return result;
}


/* ============================================================================
 * 4x4 MATRIX OPERATIONS
 * ============================================================================
 */

Mat4 mat4_identity(void)
{
    Mat4 result = {0};


    for (int i = 0; i < 4; i++)
    {
        result.m[i][i] =
            1.0;
    }


    return result;
}


Mat4 mat4_mul(
    Mat4 A,
    Mat4 B
)
{
    Mat4 C = {0};


    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            for (int k = 0; k < 4; k++)
            {
                C.m[row][column] +=
                    A.m[row][k] *
                    B.m[k][column];
            }
        }
    }


    return C;
}


Mat3 mat4_rotation(
    Mat4 T
)
{
    Mat3 R;


    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            R.m[row][column] =
                T.m[row][column];
        }
    }


    return R;
}


Vec3 mat4_translation(
    Mat4 T
)
{
    Vec3 p =
    {{
        T.m[0][3],
        T.m[1][3],
        T.m[2][3]
    }};


    return p;
}


void mat4_set_rotation(
    Mat4 *T,
    Mat3 R
)
{
    if (T == NULL)
    {
        return;
    }


    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            T->m[row][column] =
                R.m[row][column];
        }
    }
}


void mat4_set_translation(
    Mat4 *T,
    Vec3 p
)
{
    if (T == NULL)
    {
        return;
    }


    T->m[0][3] = p.v[0];
    T->m[1][3] = p.v[1];
    T->m[2][3] = p.v[2];
}


/* ============================================================================
 * ELEMENTAL ROTATIONS
 * ============================================================================
 */

static Mat3 rotx_local(
    real_t angle
)
{
    real_t c =
        cos(angle);

    real_t s =
        sin(angle);


    Mat3 R =
    {{
        {1.0, 0.0, 0.0},

        {0.0,   c,  -s},

        {0.0,   s,   c}
    }};


    return R;
}


static Mat3 roty_local(
    real_t angle
)
{
    real_t c =
        cos(angle);

    real_t s =
        sin(angle);


    Mat3 R =
    {{
        { c,   0.0,   s},

        {0.0,  1.0, 0.0},

        {-s,   0.0,   c}
    }};


    return R;
}


static Mat3 rotz_local(
    real_t angle
)
{
    real_t c =
        cos(angle);

    real_t s =
        sin(angle);


    Mat3 R =
    {{
        {c,  -s, 0.0},

        {s,   c, 0.0},

        {0.0, 0.0, 1.0}
    }};


    return R;
}


/* ============================================================================
 * ZYX EULER ROTATION
 * ============================================================================
 *
 * Equivalent to the MATLAB pipeline helper:
 *
 *      R =
 *          Rz(rz) *
 *          Ry(ry) *
 *          Rx(rx)
 */

Mat3 eul_zyx(
    real_t rz,
    real_t ry,
    real_t rx
)
{
    Mat3 Rz =
        rotz_local(rz);

    Mat3 Ry =
        roty_local(ry);

    Mat3 Rx =
        rotx_local(rx);


    return mat3_mul(
        mat3_mul(Rz, Ry),
        Rx
    );
}


/* ============================================================================
 * QUATERNION NORMALIZATION
 * ============================================================================
 */

Quat quat_normalize(
    Quat q
)
{
    real_t norm =
        sqrt(
            q.w * q.w +
            q.x * q.x +
            q.y * q.y +
            q.z * q.z
        );


    /*
     * Equivalent to MATLAB protection against a zero quaternion.
     */
    if (norm < 1e-15)
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


    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;


    return q;
}


/* ============================================================================
 * ROTATION MATRIX -> QUATERNION
 * ============================================================================
 */

Quat rotm_to_quat(
    Mat3 R
)
{
    real_t trace =
        R.m[0][0] +
        R.m[1][1] +
        R.m[2][2];


    Quat q;

    real_t S;


    if (trace > 0.0)
    {
        S =
            sqrt(trace + 1.0) *
            2.0;


        q.w =
            0.25 * S;

        q.x =
            (
                R.m[2][1] -
                R.m[1][2]
            ) / S;

        q.y =
            (
                R.m[0][2] -
                R.m[2][0]
            ) / S;

        q.z =
            (
                R.m[1][0] -
                R.m[0][1]
            ) / S;
    }
    else if (
        R.m[0][0] > R.m[1][1] &&
        R.m[0][0] > R.m[2][2]
    )
    {
        S =
            sqrt(
                1.0 +
                R.m[0][0] -
                R.m[1][1] -
                R.m[2][2]
            ) * 2.0;


        q.w =
            (
                R.m[2][1] -
                R.m[1][2]
            ) / S;

        q.x =
            0.25 * S;

        q.y =
            (
                R.m[0][1] +
                R.m[1][0]
            ) / S;

        q.z =
            (
                R.m[0][2] +
                R.m[2][0]
            ) / S;
    }
    else if (
        R.m[1][1] >
        R.m[2][2]
    )
    {
        S =
            sqrt(
                1.0 +
                R.m[1][1] -
                R.m[0][0] -
                R.m[2][2]
            ) * 2.0;


        q.w =
            (
                R.m[0][2] -
                R.m[2][0]
            ) / S;

        q.x =
            (
                R.m[0][1] +
                R.m[1][0]
            ) / S;

        q.y =
            0.25 * S;

        q.z =
            (
                R.m[1][2] +
                R.m[2][1]
            ) / S;
    }
    else
    {
        S =
            sqrt(
                1.0 +
                R.m[2][2] -
                R.m[0][0] -
                R.m[1][1]
            ) * 2.0;


        q.w =
            (
                R.m[1][0] -
                R.m[0][1]
            ) / S;

        q.x =
            (
                R.m[0][2] +
                R.m[2][0]
            ) / S;

        q.y =
            (
                R.m[1][2] +
                R.m[2][1]
            ) / S;

        q.z =
            0.25 * S;
    }


    return quat_normalize(q);
}


/* ============================================================================
 * QUATERNION -> ROTATION MATRIX
 * ============================================================================
 */

Mat3 quat_to_rotm(
    Quat q
)
{
    q =
        quat_normalize(q);


    real_t w = q.w;
    real_t x = q.x;
    real_t y = q.y;
    real_t z = q.z;


    Mat3 R =
    {{
        {
            1.0 - 2.0 * (y*y + z*z),
            2.0 * (x*y - z*w),
            2.0 * (x*z + y*w)
        },

        {
            2.0 * (x*y + z*w),
            1.0 - 2.0 * (x*x + z*z),
            2.0 * (y*z - x*w)
        },

        {
            2.0 * (x*z - y*w),
            2.0 * (y*z + x*w),
            1.0 - 2.0 * (x*x + y*y)
        }
    }};


    return R;
}


/* ============================================================================
 * QUATERNION SLERP
 * ============================================================================
 *
 * Shortest-path spherical linear interpolation.
 *
 * Equivalent to quatSlerp_custom() from the MATLAB trajectory pipeline.
 */

Quat quat_slerp(
    Quat q0,
    Quat q1,
    real_t s
)
{
    q0 =
        quat_normalize(q0);

    q1 =
        quat_normalize(q1);


    real_t dot =
        q0.w * q1.w +
        q0.x * q1.x +
        q0.y * q1.y +
        q0.z * q1.z;


    /*
     * Use shortest quaternion path.
     */
    if (dot < 0.0)
    {
        q1.w = -q1.w;
        q1.x = -q1.x;
        q1.y = -q1.y;
        q1.z = -q1.z;

        dot = -dot;
    }


    dot =
        clamp_real(
            dot,
            -1.0,
             1.0
        );


    /*
     * Nearly parallel quaternions:
     *
     * use normalized linear interpolation.
     */
    if (dot > 0.9995)
    {
        Quat q =
        {
            q0.w + s * (q1.w - q0.w),
            q0.x + s * (q1.x - q0.x),
            q0.y + s * (q1.y - q0.y),
            q0.z + s * (q1.z - q0.z)
        };


        return quat_normalize(q);
    }


    real_t theta0 =
        acos(dot);

    real_t theta =
        theta0 * s;


    Quat q2 =
    {
        q1.w - q0.w * dot,
        q1.x - q0.x * dot,
        q1.y - q0.y * dot,
        q1.z - q0.z * dot
    };


    q2 =
        quat_normalize(q2);


    Quat result =
    {
        q0.w * cos(theta) +
        q2.w * sin(theta),

        q0.x * cos(theta) +
        q2.x * sin(theta),

        q0.y * cos(theta) +
        q2.y * sin(theta),

        q0.z * cos(theta) +
        q2.z * sin(theta)
    };


    return quat_normalize(result);
}


/* ============================================================================
 * SO(3) ROTATION LOGARITHM
 * ============================================================================
 */

Vec3 rotation_log_vector(
    Mat3 R
)
{
    real_t trace =
        R.m[0][0] +
        R.m[1][1] +
        R.m[2][2];


    real_t cosTheta =
        (trace - 1.0) /
        2.0;


    cosTheta =
        clamp_real(
            cosTheta,
            -1.0,
             1.0
        );


    real_t theta =
        acos(cosTheta);


    /* ------------------------------------------------------------------------
     * Near zero rotation
     * ------------------------------------------------------------------------
     */

    if (theta < 1e-8)
    {
        return (Vec3)
        {{
            0.0,
            0.0,
            0.0
        }};
    }


    /* ------------------------------------------------------------------------
     * Near 180 degrees
     * ------------------------------------------------------------------------
     */

    if (
        fabs(ROBOT_PI - theta) <
        1e-6
    )
    {
        Vec3 axis;


        if (
            1.0 + R.m[2][2] >
            1e-8
        )
        {
            real_t denominator =
                sqrt(
                    2.0 *
                    (1.0 + R.m[2][2])
                );


            axis = (Vec3)
            {{
                R.m[0][2] / denominator,
                R.m[1][2] / denominator,
                (1.0 + R.m[2][2]) / denominator
            }};
        }
        else if (
            1.0 + R.m[1][1] >
            1e-8
        )
        {
            real_t denominator =
                sqrt(
                    2.0 *
                    (1.0 + R.m[1][1])
                );


            axis = (Vec3)
            {{
                R.m[0][1] / denominator,
                (1.0 + R.m[1][1]) / denominator,
                R.m[2][1] / denominator
            }};
        }
        else
        {
            real_t denominator =
                sqrt(
                    2.0 *
                    (1.0 + R.m[0][0])
                );


            axis = (Vec3)
            {{
                (1.0 + R.m[0][0]) / denominator,
                R.m[1][0] / denominator,
                R.m[2][0] / denominator
            }};
        }


        real_t axisNorm =
            vec3_norm(axis);


        if (axisNorm > 1e-15)
        {
            axis =
                vec3_scale(
                    axis,
                    1.0 / axisNorm
                );
        }


        return vec3_scale(
            axis,
            theta
        );
    }


    /* ------------------------------------------------------------------------
     * General rotation
     * ------------------------------------------------------------------------
     */

    real_t factor =
        theta /
        (
            2.0 *
            sin(theta)
        );


    Vec3 rotationVector =
    {{
        factor *
        (
            R.m[2][1] -
            R.m[1][2]
        ),

        factor *
        (
            R.m[0][2] -
            R.m[2][0]
        ),

        factor *
        (
            R.m[1][0] -
            R.m[0][1]
        )
    }};


    return rotationVector;
}


/* ============================================================================
 * ORIENTATION ERROR MAGNITUDE
 * ============================================================================
 *
 * Equivalent to:
 *
 *      RRelative =
 *          RTarget * RActual';
 *
 *      angle =
 *          acos(
 *              (trace(RRelative)-1)/2
 *          );
 */

real_t rotation_error(
    Mat3 target,
    Mat3 actual
)
{
    Mat3 relative =
        mat3_mul(
            target,
            mat3_transpose(actual)
        );


    real_t c =
        (
            relative.m[0][0] +
            relative.m[1][1] +
            relative.m[2][2] -
            1.0
        ) / 2.0;


    c =
        clamp_real(
            c,
            -1.0,
             1.0
        );


    return acos(c);
}