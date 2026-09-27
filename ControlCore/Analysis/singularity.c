#include "singularity.h"

#include <math.h>
#include <stddef.h>
#include <string.h>


/* ============================================================================
 * INTERNAL VALIDATION
 * ============================================================================ */

static bool jacobian_is_finite(
    const real_t J[ROBOT_DOF][ROBOT_DOF]
)
{
    if (J == NULL)
    {
        return false;
    }

    for (int row = 0;
         row < ROBOT_DOF;
         row++)
    {
        for (int column = 0;
             column < ROBOT_DOF;
             column++)
        {
            if (!isfinite(J[row][column]))
            {
                return false;
            }
        }
    }

    return true;
}


/* ============================================================================
 * SMALLEST SINGULAR VALUE
 * ============================================================================ */

bool singularity_sigma_min(
    const real_t J[ROBOT_DOF][ROBOT_DOF],
    real_t *sigmaMin
)
{
    if (
        J == NULL ||
        sigmaMin == NULL ||
        !jacobian_is_finite(J)
    )
    {
        return false;
    }


    /* ========================================================================
     * 1. FORM A = J' * J
     * ========================================================================
     *
     * A is symmetric positive-semidefinite.
     *
     * Its eigenvalues are:
     *
     *      lambda_i = sigma_i^2
     *
     * where sigma_i are the singular values of J.
     * ========================================================================
     */

    real_t A[ROBOT_DOF][ROBOT_DOF] =
    {
        {0}
    };


    for (int row = 0;
         row < ROBOT_DOF;
         row++)
    {
        for (int column = 0;
             column < ROBOT_DOF;
             column++)
        {
            for (int k = 0;
                 k < ROBOT_DOF;
                 k++)
            {
                A[row][column] +=
                    J[k][row] *
                    J[k][column];
            }
        }
    }


    /* ========================================================================
     * 2. JACOBI EIGENVALUE ITERATION
     * ========================================================================
     *
     * Repeatedly eliminate the largest off-diagonal element of A.
     *
     * After convergence, the diagonal entries approximate the eigenvalues of
     * J'J.
     *
     * These values intentionally preserve the more conservative numerical
     * settings already used by this project's previous ControlCore
     * implementation:
     *
     *      maximum iterations  = 120
     *      convergence epsilon = 1e-14
     * ========================================================================
     */

    const int maxIterations =
        120;

    const real_t convergenceTolerance =
        1e-14;


    for (int iteration = 0;
         iteration < maxIterations;
         iteration++)
    {
        int p = 0;
        int q = 1;

        real_t maximumOffDiagonal =
            fabs(A[p][q]);


        for (int row = 0;
             row < ROBOT_DOF;
             row++)
        {
            for (int column = row + 1;
                 column < ROBOT_DOF;
                 column++)
            {
                const real_t value =
                    fabs(A[row][column]);

                if (value > maximumOffDiagonal)
                {
                    maximumOffDiagonal =
                        value;

                    p =
                        row;

                    q =
                        column;
                }
            }
        }


        /*
         * Matrix is sufficiently diagonal.
         */
        if (
            maximumOffDiagonal <
            convergenceTolerance
        )
        {
            break;
        }


        /* --------------------------------------------------------------------
         * Jacobi rotation
         * --------------------------------------------------------------------
         */

        const real_t app =
            A[p][p];

        const real_t aqq =
            A[q][q];

        const real_t apq =
            A[p][q];


        const real_t phi =
            0.5 *
            atan2(
                2.0 * apq,
                aqq - app
            );


        const real_t c =
            cos(phi);

        const real_t s =
            sin(phi);


        /*
         * Rotate the remaining rows / columns while preserving symmetry.
         */
        for (int k = 0;
             k < ROBOT_DOF;
             k++)
        {
            if (
                k == p ||
                k == q
            )
            {
                continue;
            }


            const real_t akp =
                A[k][p];

            const real_t akq =
                A[k][q];


            A[k][p] =
                c * akp -
                s * akq;

            A[p][k] =
                A[k][p];


            A[k][q] =
                s * akp +
                c * akq;

            A[q][k] =
                A[k][q];
        }


        /*
         * Update the two diagonal entries.
         */
        A[p][p] =
            c * c * app -
            2.0 * s * c * apq +
            s * s * aqq;


        A[q][q] =
            s * s * app +
            2.0 * s * c * apq +
            c * c * aqq;


        A[p][q] =
            0.0;

        A[q][p] =
            0.0;
    }


    /* ========================================================================
     * 3. FIND THE SMALLEST EIGENVALUE OF J'J
     * ========================================================================
     */

    real_t minimumEigenvalue =
        A[0][0];


    for (int i = 1;
         i < ROBOT_DOF;
         i++)
    {
        if (
            A[i][i] <
            minimumEigenvalue
        )
        {
            minimumEigenvalue =
                A[i][i];
        }
    }


    /* ========================================================================
     * 4. NUMERICAL PROTECTION
     * ========================================================================
     *
     * J'J is positive-semidefinite, so its true eigenvalues cannot be
     * negative. Tiny negative values can occur from floating-point roundoff.
     * ========================================================================
     */

    if (minimumEigenvalue < 0.0)
    {
        minimumEigenvalue =
            0.0;
    }


    /* ========================================================================
     * 5. sigma_min = sqrt(lambda_min)
     * ========================================================================
     */

    *sigmaMin =
        sqrt(minimumEigenvalue);


    return
        isfinite(*sigmaMin);
}


/* ============================================================================
 * SINGULARITY THRESHOLD ANALYSIS
 * ============================================================================ */

bool singularity_analyze(
    const real_t J[ROBOT_DOF][ROBOT_DOF],
    real_t minimumAllowedSigma,
    SingularityResult *result
)
{
    if (
        J == NULL ||
        result == NULL ||
        !isfinite(minimumAllowedSigma) ||
        minimumAllowedSigma < 0.0
    )
    {
        return false;
    }


    memset(
        result,
        0,
        sizeof(*result)
    );


    real_t sigmaMin =
        0.0;


    if (
        !singularity_sigma_min(
            J,
            &sigmaMin
        )
    )
    {
        return false;
    }


    result->sigmaMin =
        sigmaMin;

    result->minimumAllowedSigma =
        minimumAllowedSigma;

    result->margin =
        sigmaMin -
        minimumAllowedSigma;

    result->safe =
        sigmaMin >=
        minimumAllowedSigma;


    return true;
}
