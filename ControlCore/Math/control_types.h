#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H


#include "../Config/robot_config.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


/* ============================================================================
 * COMMON CONTROL TYPES
 * ============================================================================
 *
 * Shared lightweight mathematical/control types used throughout ControlCore.
 *
 * All floating-point control calculations currently use double precision.
 * ============================================================================
 */

typedef double real_t;


/* ============================================================================
 * 3D VECTOR
 * ============================================================================
 */

typedef struct
{
    real_t v[3];

} Vec3;


/* ============================================================================
 * 3x3 MATRIX
 * ============================================================================
 */

typedef struct
{
    real_t m[3][3];

} Mat3;


/* ============================================================================
 * 4x4 HOMOGENEOUS TRANSFORMATION MATRIX
 * ============================================================================
 */

typedef struct
{
    real_t m[4][4];

} Mat4;


/* ============================================================================
 * QUATERNION
 * ============================================================================
 *
 * Convention:
 *
 *      q = [w, x, y, z]
 *
 * where:
 *
 *      w = scalar component
 *      x,y,z = vector components
 * ============================================================================
 */

typedef struct
{
    real_t w;
    real_t x;
    real_t y;
    real_t z;

} Quat;


/* ============================================================================
 * ROBOT JOINT VECTOR
 * ============================================================================
 *
 * Joint order:
 *
 *      q[0] = J1
 *      q[1] = J2
 *      q[2] = J3
 *      q[3] = J4
 *      q[4] = J5
 *      q[5] = J6
 *
 * Units depend on usage:
 *
 *      position     -> rad
 *      velocity     -> rad/s
 *      acceleration -> rad/s^2
 * ============================================================================
 */

typedef struct
{
    real_t q[ROBOT_DOF];

} JointVector;


/* ============================================================================
 * CARTESIAN POSE
 * ============================================================================
 *
 * Pose represented as:
 *
 *      position    p
 *      orientation R
 * ============================================================================
 */

typedef struct
{
    Vec3 p;
    Mat3 R;

} Pose;


#endif /* CONTROL_TYPES_H */