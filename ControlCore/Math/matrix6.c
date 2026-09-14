#include "matrix6.h"

#include <math.h>


/* ============================================================================
 * 6x6 LINEAR SYSTEM SOLVER
 * ============================================================================
 *
 * Gaussian elimination with partial pivoting.
 *
 * MATLAB equivalent:
 *
 *      x = A \ b;
 *
 * We do NOT explicitly calculate:
 *
 *      inv(A)
 *
 * which matches the numerical intent of the MATLAB implementation.
 * ============================================================================
 */

bool matrix6_solve(
    const real_t A[ROBOT_DOF][ROBOT_DOF],
    const real_t b[ROBOT_DOF],
    real_t x[ROBOT_DOF]
)
{
    if (
        A == NULL ||
        b == NULL ||
        x == NULL
    )
    {
        return false;
    }


    /*
     * Augmented matrix:
     *
     *      [ A | b ]
     */
    real_t M[ROBOT_DOF][ROBOT_DOF + 1];


    for (int row = 0;
         row < ROBOT_DOF;
         row++)
    {
        for (int column = 0;
             column < ROBOT_DOF;
             column++)
        {
            M[row][column] =
                A[row][column];
        }


        M[row][ROBOT_DOF] =
            b[row];
    }


    /* ========================================================================
     * FORWARD ELIMINATION
     * ========================================================================
     */

    for (int pivotColumn = 0;
         pivotColumn < ROBOT_DOF;
         pivotColumn++)
    {
        /* --------------------------------------------------------------------
         * Find largest pivot in this column
         * --------------------------------------------------------------------
         */

        int pivotRow =
            pivotColumn;


        real_t largestPivot =
            fabs(
                M[pivotColumn][pivotColumn]
            );


        for (int row = pivotColumn + 1;
             row < ROBOT_DOF;
             row++)
        {
            real_t candidate =
                fabs(
                    M[row][pivotColumn]
                );


            if (candidate > largestPivot)
            {
                largestPivot =
                    candidate;

                pivotRow =
                    row;
            }
        }


        /*
         * Numerically singular matrix.
         */
        if (largestPivot < 1e-14)
        {
            return false;
        }


        /* --------------------------------------------------------------------
         * Swap rows if necessary
         * --------------------------------------------------------------------
         */

        if (pivotRow != pivotColumn)
        {
            for (int column = pivotColumn;
                 column <= ROBOT_DOF;
                 column++)
            {
                real_t temporary =
                    M[pivotColumn][column];


                M[pivotColumn][column] =
                    M[pivotRow][column];


                M[pivotRow][column] =
                    temporary;
            }
        }


        /* --------------------------------------------------------------------
         * Eliminate entries below pivot
         * --------------------------------------------------------------------
         */

        for (int row = pivotColumn + 1;
             row < ROBOT_DOF;
             row++)
        {
            real_t factor =
                M[row][pivotColumn] /
                M[pivotColumn][pivotColumn];


            M[row][pivotColumn] =
                0.0;


            for (int column = pivotColumn + 1;
                 column <= ROBOT_DOF;
                 column++)
            {
                M[row][column] -=
                    factor *
                    M[pivotColumn][column];
            }
        }
    }


    /* ========================================================================
     * BACK SUBSTITUTION
     * ========================================================================
     */

    for (int row = ROBOT_DOF - 1;
         row >= 0;
         row--)
    {
        real_t sum =
            M[row][ROBOT_DOF];


        for (int column = row + 1;
             column < ROBOT_DOF;
             column++)
        {
            sum -=
                M[row][column] *
                x[column];
        }


        if (
            fabs(M[row][row]) <
            1e-14
        )
        {
            return false;
        }


        x[row] =
            sum /
            M[row][row];
    }


    return true;
}


/* ============================================================================
 * SMALLEST SINGULAR VALUE OF A 6x6 MATRIX
 * ============================================================================
 *
 * MATLAB:
 *
 *      singularValues = svd(J);
 *      sigmaMin = min(singularValues);
 *
 *
 * C implementation:
 *
 *      A = J'J
 *
 * then use a symmetric Jacobi eigenvalue iteration.
 *
 * The diagonal values after convergence approximate the eigenvalues:
 *
 *      lambda_i = sigma_i^2
 *
 * Therefore:
 *
 *      sigmaMin =
 *          sqrt(min(lambda_i))
 * ============================================================================
 */

real_t matrix6_smallest_singular_value(
    const real_t J[ROBOT_DOF][ROBOT_DOF]
)
{
    if (J == NULL)
    {
        return 0.0;
    }


    /* ========================================================================
     * A = J' * J
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
     * JACOBI EIGENVALUE ITERATION
     * ========================================================================
     */

    const int maxIterations =
        120;


    for (int iteration = 0;
         iteration < maxIterations;
         iteration++)
    {
        /*
         * Find largest off-diagonal element.
         */
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
                real_t value =
                    fabs(
                        A[row][column]
                    );


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
         * Matrix sufficiently diagonal.
         */
        if (
            maximumOffDiagonal <
            1e-14
        )
        {
            break;
        }


        /* --------------------------------------------------------------------
         * Calculate Jacobi rotation angle
         * --------------------------------------------------------------------
         */

        real_t phi =
            0.5 *
            atan2(
                2.0 * A[p][q],
                A[q][q] - A[p][p]
            );


        real_t c =
            cos(phi);

        real_t s =
            sin(phi);


        real_t app =
            A[p][p];

        real_t aqq =
            A[q][q];

        real_t apq =
            A[p][q];


        /* --------------------------------------------------------------------
         * Rotate rows/columns p and q
         * --------------------------------------------------------------------
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


            real_t akp =
                A[k][p];

            real_t akq =
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


        /* --------------------------------------------------------------------
         * Update diagonal entries
         * --------------------------------------------------------------------
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
     * FIND SMALLEST EIGENVALUE
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


    /*
     * J'J is positive semidefinite.
     *
     * Tiny negative values may appear only because of floating-point
     * numerical error.
     */
    if (
        minimumEigenvalue < 0.0 &&
        minimumEigenvalue > -1e-12
    )
    {
        minimumEigenvalue =
            0.0;
    }


    /*
     * Additional numerical protection.
     */
    if (minimumEigenvalue < 0.0)
    {
        minimumEigenvalue =
            0.0;
    }


    return sqrt(
        minimumEigenvalue
    );
}


/* ============================================================================
 * COMPATIBILITY WRAPPERS
 * ============================================================================
 */

int solve6(
    real_t A[ROBOT_DOF][ROBOT_DOF],
    const real_t b[ROBOT_DOF],
    real_t x[ROBOT_DOF]
)
{
    return matrix6_solve(
        A,
        b,
        x
    ) ? 1 : 0;
}


real_t smallest_singular_value_6x6(
    const real_t J[ROBOT_DOF][ROBOT_DOF]
)
{
    return matrix6_smallest_singular_value(
        J
    );
}