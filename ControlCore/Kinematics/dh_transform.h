#ifndef DH_TRANSFORM_H
#define DH_TRANSFORM_H


/* ============================================================================
 * STANDARD DH TRANSFORMATION
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      dhTransform.m
 *
 *
 * Computes one Standard Denavit-Hartenberg homogeneous transformation:
 *
 *      A =
 *
 *      [ cos(theta)   -sin(theta)cos(alpha)   sin(theta)sin(alpha)   a cos(theta) ]
 *      [ sin(theta)    cos(theta)cos(alpha)  -cos(theta)sin(alpha)   a sin(theta) ]
 *      [     0              sin(alpha)              cos(alpha)            d        ]
 *      [     0                  0                       0                  1        ]
 *
 *
 * Inputs:
 *
 *      a
 *          DH link length [m]
 *
 *      d
 *          DH link offset [m]
 *
 *      alpha
 *          DH link twist [rad]
 *
 *      theta
 *          Joint angle [rad]
 *
 *
 * Output:
 *
 *      A
 *          4x4 homogeneous transformation matrix.
 *
 * ============================================================================
 */

void dh_transform(
    double a,
    double d,
    double alpha,
    double theta,
    double A[4][4]
);


#endif /* DH_TRANSFORM_H */