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
 *      dq = A \ b;
 *
 *
 * Returns:
 *
 *      true  -> solve succeeded
 *      false -> matrix was numerically singular
 * ============================================================================
 */

bool matrix6_solve(
    const real_t A[ROBOT_DOF][ROBOT_DOF],
    const real_t b[ROBOT_DOF],
    real_t x[ROBOT_DOF]
);


/* ============================================================================
 * SMALLEST SINGULAR VALUE
 * ============================================================================
 *
 * Returns:
 *
 *      sigma_min(J)
 *
 * Equivalent to:
 *
 *      singularValues = svd(J);
 *      sigmaMin = min(singularValues);
 *
 * The implementation computes eigenvalues of:
 *
 *      J'J
 *
 * using a Jacobi iteration.
 *
 * Since:
 *
 *      eigenvalues(J'J) = sigma_i^2
 *
 * then:
 *
 *      sigma_min =
 *          sqrt(min_eigenvalue(J'J))
 * ============================================================================
 */

real_t matrix6_smallest_singular_value(
    const real_t J[ROBOT_DOF][ROBOT_DOF]
);


/* ============================================================================
 * COMPATIBILITY ALIASES
 * ============================================================================
 *
 * These allow other ControlCore modules to use shorter names if required.
 */

int solve6(
    real_t A[ROBOT_DOF][ROBOT_DOF],
    const real_t b[ROBOT_DOF],
    real_t x[ROBOT_DOF]
);

real_t smallest_singular_value_6x6(
    const real_t J[ROBOT_DOF][ROBOT_DOF]
);


#endif /* MATRIX6_H */