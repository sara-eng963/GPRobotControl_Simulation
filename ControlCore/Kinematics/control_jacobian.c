#include "control_jacobian.h"

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
 * A temporary matrix is used so:
 *
 *      C == A
 *
 * is safe.
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
 * INTERNAL 3D CROSS PRODUCT
 * ============================================================================
 *
 * Computes:
 *
 *      result = a x b
 *
 * Equivalent to MATLAB:
 *
 *      cross(a,b)
 *
 * ============================================================================
 */

static void vector3_cross(
    const double a[3],
    const double b[3],
    double result[3]
)
{
    result[0] =
        a[1] * b[2] -
        a[2] * b[1];

    result[1] =
        a[2] * b[0] -
        a[0] * b[2];

    result[2] =
        a[0] * b[1] -
        a[1] * b[0];
}


/* ============================================================================
 * CONTROL GEOMETRIC JACOBIAN
 * ============================================================================
 */

void control_jacobian(
    const RobotConfig *robot,
    const double q[ROBOT_DOF],
    double J[ROBOT_DOF][ROBOT_DOF]
)
{
    /*
     * Protect against invalid pointers.
     */
    if (
        robot == NULL ||
        q == NULL ||
        J == NULL
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
     *
     * In C we access those fields directly from RobotConfig.
     * ========================================================================
     */


    /* ========================================================================
     * START FROM BASE FRAME
     * ========================================================================
     *
     * MATLAB:
     *
     *      T = eye(4);
     */

    double T[4][4] =
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };


    /* ========================================================================
     * STORAGE FOR JOINT ORIGINS AND AXES
     * ========================================================================
     *
     * MATLAB:
     *
     *      jointOrigins = zeros(3,robot.dof);
     *      jointAxes    = zeros(3,robot.dof);
     *
     * MATLAB stores:
     *
     *      jointOrigins(:,i)
     *
     * C stores:
     *
     *      jointOrigins[i][axis]
     */

    double jointOrigins[ROBOT_DOF][3];

    double jointAxes[ROBOT_DOF][3];


    /* ========================================================================
     * WALK THROUGH THE KINEMATIC CHAIN
     * ========================================================================
     *
     * For each joint:
     *
     *      1. Store origin of frame i-1
     *      2. Store Z-axis of frame i-1
     *      3. Calculate current theta
     *      4. Calculate DH transform
     *      5. Advance T
     *
     * This ordering is important.
     * ========================================================================
     */

    for (int i = 0;
         i < robot->identity.dof;
         i++)
    {
        /* --------------------------------------------------------------------
         * Store origin of frame i-1 in Base coordinates
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      jointOrigins(:,i) =
         *          T(1:3,4);
         */

        jointOrigins[i][0] =
            T[0][3];

        jointOrigins[i][1] =
            T[1][3];

        jointOrigins[i][2] =
            T[2][3];


        /* --------------------------------------------------------------------
         * Store Z-axis of frame i-1 in Base coordinates
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      jointAxes(:,i) =
         *          T(1:3,3);
         */

        jointAxes[i][0] =
            T[0][2];

        jointAxes[i][1] =
            T[1][2];

        jointAxes[i][2] =
            T[2][2];


        /* --------------------------------------------------------------------
         * Calculate actual joint angle
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      theta =
         *          q(i) +
         *          thetaOffset(i);
         */

        double theta =
            q[i] +
            robot->kinematics.thetaOffset[i];


        /* --------------------------------------------------------------------
         * Calculate Standard-DH transform
         * --------------------------------------------------------------------
         */

        double A[4][4];


        dh_transform(
            robot->kinematics.a[i],
            robot->kinematics.d[i],
            robot->kinematics.alpha[i],
            theta,
            A
        );


        /* --------------------------------------------------------------------
         * Move to next frame
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      T = T * A;
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
     */

    double T_B_TCP[4][4];


    matrix4_multiply(
        T,
        robot->tool.T_F_TCP,
        T_B_TCP
    );


    /* ========================================================================
     * EXTRACT TCP POSITION
     * ========================================================================
     *
     * MATLAB:
     *
     *      p_TCP =
     *          T_B_TCP(1:3,4);
     */

    double p_TCP[3];


    p_TCP[0] =
        T_B_TCP[0][3];

    p_TCP[1] =
        T_B_TCP[1][3];

    p_TCP[2] =
        T_B_TCP[2][3];


    /* ========================================================================
     * INITIALIZE JACOBIAN
     * ========================================================================
     *
     * MATLAB:
     *
     *      J = zeros(6,robot.dof);
     */

    for (int row = 0;
         row < ROBOT_DOF;
         row++)
    {
        for (int column = 0;
             column < ROBOT_DOF;
             column++)
        {
            J[row][column] =
                0.0;
        }
    }


    /* ========================================================================
     * BUILD ONE JACOBIAN COLUMN FOR EACH REVOLUTE JOINT
     * ========================================================================
     */

    for (int i = 0;
         i < robot->identity.dof;
         i++)
    {
        /* --------------------------------------------------------------------
         * Joint origin and axis in Base coordinates
         * --------------------------------------------------------------------
         */

        double p_joint[3] =
        {
            jointOrigins[i][0],
            jointOrigins[i][1],
            jointOrigins[i][2]
        };


        double z_joint[3] =
        {
            jointAxes[i][0],
            jointAxes[i][1],
            jointAxes[i][2]
        };


        /* --------------------------------------------------------------------
         * Calculate:
         *
         *      p_TCP - p_joint
         * --------------------------------------------------------------------
         */

        double jointToTCP[3];


        jointToTCP[0] =
            p_TCP[0] -
            p_joint[0];

        jointToTCP[1] =
            p_TCP[1] -
            p_joint[1];

        jointToTCP[2] =
            p_TCP[2] -
            p_joint[2];


        /* --------------------------------------------------------------------
         * Linear velocity contribution
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      Jv =
         *          cross(
         *              z_joint,
         *              p_TCP - p_joint
         *          );
         */

        double Jv[3];


        vector3_cross(
            z_joint,
            jointToTCP,
            Jv
        );


        /* --------------------------------------------------------------------
         * Angular velocity contribution
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      Jw = z_joint;
         */

        double Jw[3];


        Jw[0] =
            z_joint[0];

        Jw[1] =
            z_joint[1];

        Jw[2] =
            z_joint[2];


        /* --------------------------------------------------------------------
         * Store complete Jacobian column
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      J(:,i) = [
         *          Jv
         *          Jw
         *      ];
         *
         * Therefore:
         *
         *      rows 0..2 = Jv
         *      rows 3..5 = Jw
         */

        J[0][i] =
            Jv[0];

        J[1][i] =
            Jv[1];

        J[2][i] =
            Jv[2];

        J[3][i] =
            Jw[0];

        J[4][i] =
            Jw[1];

        J[5][i] =
            Jw[2];
    }
}