#include "matrix6.h"

#include <math.h>
#include <stddef.h>


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
        int pivotRow =
            pivotColumn;


        real_t largestPivot =
            fabs(
                M[pivotColumn][pivotColumn]
            );


        /*
         * Find the largest available pivot in this column.
         */
        for (int row = pivotColumn + 1;
             row < ROBOT_DOF;
             row++)
        {
            const real_t candidate =
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
         * Matrix is numerically singular.
         */
        if (largestPivot < 1e-14)
        {
            return false;
        }


        /*
         * Swap rows when a better pivot was found.
         */
        if (pivotRow != pivotColumn)
        {
            for (int column = pivotColumn;
                 column <= ROBOT_DOF;
                 column++)
            {
                const real_t temporary =
                    M[pivotColumn][column];


                M[pivotColumn][column] =
                    M[pivotRow][column];


                M[pivotRow][column] =
                    temporary;
            }
        }


        /*
         * Eliminate entries below this pivot.
         */
        for (int row = pivotColumn + 1;
             row < ROBOT_DOF;
             row++)
        {
            const real_t factor =
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
 * COMPATIBILITY WRAPPER
 * ============================================================================ */

int solve6(
    real_t A[ROBOT_DOF][ROBOT_DOF],
    const real_t b[ROBOT_DOF],
    real_t x[ROBOT_DOF]
)
{
    return
        matrix6_solve(
            A,
            b,
            x
        )
        ? 1
        : 0;
}