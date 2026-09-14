#ifndef MATH3D_H
#define MATH3D_H


#include "control_types.h"


/* ============================================================================
 * SCALAR HELPERS
 * ============================================================================
 */

real_t clamp_real(
    real_t x,
    real_t lower,
    real_t upper
);


/* ============================================================================
 * 3D VECTOR OPERATIONS
 * ============================================================================
 */

real_t vec3_norm(
    Vec3 a
);

Vec3 vec3_add(
    Vec3 a,
    Vec3 b
);

Vec3 vec3_sub(
    Vec3 a,
    Vec3 b
);

Vec3 vec3_scale(
    Vec3 a,
    real_t scale
);

Vec3 vec3_cross(
    Vec3 a,
    Vec3 b
);


/* ============================================================================
 * 3x3 MATRIX OPERATIONS
 * ============================================================================
 */

Mat3 mat3_identity(void);

Mat3 mat3_mul(
    Mat3 A,
    Mat3 B
);

Mat3 mat3_transpose(
    Mat3 A
);

Vec3 mat3_mul_vec(
    Mat3 A,
    Vec3 v
);


/* ============================================================================
 * 4x4 HOMOGENEOUS MATRIX OPERATIONS
 * ============================================================================
 */

Mat4 mat4_identity(void);

Mat4 mat4_mul(
    Mat4 A,
    Mat4 B
);

Mat3 mat4_rotation(
    Mat4 T
);

Vec3 mat4_translation(
    Mat4 T
);

void mat4_set_rotation(
    Mat4 *T,
    Mat3 R
);

void mat4_set_translation(
    Mat4 *T,
    Vec3 p
);


/* ============================================================================
 * ROTATION OPERATIONS
 * ============================================================================
 *
 * ZYX Euler convention:
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
);


/* ============================================================================
 * QUATERNION OPERATIONS
 * ============================================================================
 */

Quat quat_normalize(
    Quat q
);

Quat rotm_to_quat(
    Mat3 R
);

Mat3 quat_to_rotm(
    Quat q
);

Quat quat_slerp(
    Quat q0,
    Quat q1,
    real_t s
);


/* ============================================================================
 * SO(3) ROTATION ERROR
 * ============================================================================
 */

Vec3 rotation_log_vector(
    Mat3 R
);

real_t rotation_error(
    Mat3 target,
    Mat3 actual
);


#endif /* MATH3D_H */