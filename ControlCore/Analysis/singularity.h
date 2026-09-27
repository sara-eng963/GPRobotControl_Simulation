#ifndef CONTROLCORE_ANALYSIS_SINGULARITY_H
#define CONTROLCORE_ANALYSIS_SINGULARITY_H

#include <stdbool.h>

#include "../Math/control_types.h"


/* ============================================================================
 * SINGULARITY ANALYSIS
 * ============================================================================
 *
 * This module owns the reusable Jacobian singularity metric used by:
 *
 *      - inverse kinematics diagnostics / adaptive damping
 *      - path validation
 *      - future motion / HMI diagnostics
 *
 * The singularity metric is the smallest singular value of the 6x6
 * geometric Jacobian:
 *
 *      sigma_min(J)
 *
 * For a square 6-DoF manipulator:
 *
 *      sigma_min -> 0
 *
 * indicates that the Jacobian is approaching rank deficiency and therefore
 * the robot is approaching a kinematic singularity.
 *
 * This module intentionally does NOT calculate the robot Jacobian itself.
 * The caller supplies J. This keeps the module independent of the kinematics
 * layer and allows both Kinematics and StateMachine code to reuse it without
 * creating circular dependencies.
 * ============================================================================
 */


/* ============================================================================
 * RESULT
 * ============================================================================ */

typedef struct
{
    /*
     * Smallest singular value of the supplied Jacobian.
     */
    real_t sigmaMin;

    /*
     * Operational threshold supplied by the caller.
     *
     * This is NOT an IK damping threshold unless the caller chooses to use it
     * that way.
     */
    real_t minimumAllowedSigma;

    /*
     * Positive:
     *      configuration is above the requested singularity margin.
     *
     * Zero:
     *      exactly on the requested threshold.
     *
     * Negative:
     *      below the requested threshold.
     */
    real_t margin;

    /*
     * true when:
     *
     *      sigmaMin >= minimumAllowedSigma
     */
    bool safe;

} SingularityResult;


/* ============================================================================
 * PUBLIC API
 * ============================================================================ */


/*
 * Calculate the smallest singular value of one 6x6 Jacobian.
 *
 * Method:
 *
 *      1. Form A = J' * J
 *      2. Compute the eigenvalues of symmetric A using Jacobi rotations
 *      3. Take the smallest eigenvalue
 *      4. sigma_min = sqrt(lambda_min)
 *
 * This is mathematically equivalent to taking the minimum singular value from
 * an SVD, while avoiding a separate general-purpose SVD dependency.
 *
 * Returns:
 *
 *      true
 *          sigmaMin was calculated successfully.
 *
 *      false
 *          invalid input or non-finite Jacobian data.
 */
bool singularity_sigma_min(
    const real_t J[ROBOT_DOF][ROBOT_DOF],
    real_t *sigmaMin
);


/*
 * Evaluate one Jacobian against an operational singularity threshold.
 *
 * The threshold is supplied by the caller because different consumers may use
 * different meanings:
 *
 *      ADLS:
 *          threshold used to decide when adaptive damping should increase.
 *
 *      Path Validation:
 *          minimum acceptable singularity margin for approving execution.
 *
 * Returns:
 *
 *      true
 *          analysis completed successfully.
 *
 *      false
 *          invalid input / threshold / Jacobian.
 */
bool singularity_analyze(
    const real_t J[ROBOT_DOF][ROBOT_DOF],
    real_t minimumAllowedSigma,
    SingularityResult *result
);


#endif /* CONTROLCORE_ANALYSIS_SINGULARITY_H */
