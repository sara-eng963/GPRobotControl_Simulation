#ifndef MATRIX6_H
#define MATRIX6_H


#include "control_types.h"


/* ============================================================================
 * 6x6 LINEAR SYSTEM SOLVER
 * ============================================================================
 *
 * Solves:
 *
 *      A x = b
 *
 * Used by ADLS for:
 *
 *      (J'J + lambda^2 I) dq = J'e
 *
 * This corresponds to MATLAB:
 *
 *      x = A \ b;
 *
 * Singularity analysis does NOT belong in this module.
 * Smallest-singular-value calculation is owned by:
 *
 *      ControlCore/Analysis/singularity.c
 *
 * ============================================================================
 */


/*
 * Solve:
 *
 *      A x = b
 *
 * using Gaussian elimination with partial pivoting.
 *
 * Returns:
 *
 *      true
 *          system solved successfully.
 *
 *      false
 *          invalid input or numerically singular matrix.
 */
bool matrix6_solve(
    const real_t A[ROBOT_DOF][ROBOT_DOF],
    const real_t b[ROBOT_DOF],
    real_t x[ROBOT_DOF]
);


/* ============================================================================
 * COMPATIBILITY WRAPPER
 * ============================================================================
 *
 * Older ControlCore code may use solve6().
 *
 * Keep this wrapper for compatibility while matrix6_solve() remains the
 * canonical implementation.
 */

int solve6(
    real_t A[ROBOT_DOF][ROBOT_DOF],
    const real_t b[ROBOT_DOF],
    real_t x[ROBOT_DOF]
);


#endif /* MATRIX6_H */