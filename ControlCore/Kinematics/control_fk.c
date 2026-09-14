#include "control_fk.h"

#include "dh_transform.h"

#include <stddef.h>


/* ============================================================================
 * INTERNAL 4x4 MATRIX MULTIPLICATION
 * ============================================================================
 *
 * Computes:
 *
 *      C = A * B
 *
 * Equivalent to MATLAB:
 *
 *      C = A * B;
 *
 * ============================================================================
 */

static void matrix4_multiply(
    const double A[4][4],
    const double B[4][4],
    double C[4][4]
)
{
    double result[4][4];


    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            result[row][column] =
                0.0;


            for (int k = 0; k < 4; k++)
            {
                result[row][column] +=
                    A[row][k] *
                    B[k][column];
            }
        }
    }


    /*
     * Copy through a temporary matrix so the function is safe even if:
     *
     *      C == A
     *
     * or:
     *
     *      C == B
     */
    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            C[row][column] =
                result[row][column];
        }
    }
}


/* ============================================================================
 * CONTROL FORWARD KINEMATICS
 * ============================================================================
 */

void control_fk(
    const RobotConfig *robot,
    const double q[ROBOT_DOF],
    double T_B_TCP[4][4]
)
{
    /*
     * Protect against invalid pointers.
     */
    if (
        robot == NULL ||
        q == NULL ||
        T_B_TCP == NULL
    )
    {
        return;
    }


    /* ========================================================================
     * MATLAB:
     *
     *      a = robot.kinematics.a;
     *      d = robot.kinematics.d;
     *      alpha = robot.kinematics.alpha;
     *      thetaOffset = robot.kinematics.thetaOffset;
     * ========================================================================
     *
     * In C there is no need to copy these arrays.
     *
     * We access them directly through:
     *
     *      robot->kinematics.a
     *      robot->kinematics.d
     *      robot->kinematics.alpha
     *      robot->kinematics.thetaOffset
     */


    /* ========================================================================
     * INITIAL TRANSFORMATION
     * ========================================================================
     *
     * MATLAB:
     *
     *      T = eye(4);
     *
     * This represents:
     *
     *      no rotation
     *      no translation
     *
     * at the Base frame.
     */

    double T[4][4] =
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };


    /* ========================================================================
     * LOOP THROUGH EACH JOINT
     * ========================================================================
     *
     * MATLAB:
     *
     *      for i = 1:robot.dof
     *
     *          theta =
     *              q(i) +
     *              thetaOffset(i);
     *
     *          A =
     *              dhTransform(
     *                  a(i),
     *                  d(i),
     *                  alpha(i),
     *                  theta
     *              );
     *
     *          T = T * A;
     *
     *      end
     */

    for (int i = 0;
         i < robot->identity.dof;
         i++)
    {
        /*
         * Actual revolute joint angle:
         *
         *      theta_i = q_i + thetaOffset_i
         */
        double theta =
            q[i] +
            robot->kinematics.thetaOffset[i];


        /*
         * Transformation from frame i-1 to frame i.
         */
        double A[4][4];


        dh_transform(
            robot->kinematics.a[i],
            robot->kinematics.d[i],
            robot->kinematics.alpha[i],
            theta,
            A
        );


        /*
         * Accumulate transformations:
         *
         *      T = T * A
         */
        matrix4_multiply(
            T,
            A,
            T
        );
    }


    /* ========================================================================
     * APPLY FLANGE -> TCP TRANSFORMATION
     * ========================================================================
     *
     * MATLAB:
     *
     *      T_B_TCP =
     *          T * robot.tool.T_F_TCP;
     *
     * At the moment:
     *
     *      T_F_TCP = identity
     *
     * but keeping this multiplication is important because the welding torch
     * TCP can later be inserted through robot_config without changing FK.
     */

    matrix4_multiply(
        T,
        robot->tool.T_F_TCP,
        T_B_TCP
    );
}