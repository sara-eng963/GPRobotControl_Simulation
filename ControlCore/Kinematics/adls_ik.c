#include "adls_ik.h"

#include "control_fk.h"
#include "control_jacobian.h"

#include "../Math/matrix6.h"

#include <math.h>
#include <stddef.h>
#include <string.h>


/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================
 */


/*
 * Clamp one scalar value to:
 *
 *      minimum <= value <= maximum
 */
static double clamp_value(
    double value,
    double minimum,
    double maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


/*
 * Euclidean norm of a 3-element vector.
 *
 * MATLAB equivalent:
 *
 *      norm(v)
 */
static double vector3_norm(
    const double v[3]
)
{
    return sqrt(
        v[0] * v[0] +
        v[1] * v[1] +
        v[2] * v[2]
    );
}


/*
 * Euclidean RMS value of one 6-element joint update.
 *
 * MATLAB:
 *
 *      sqrt(mean(dqApplied.^2))
 */
static double joint_rms(
    const double q[ROBOT_DOF]
)
{
    double sumSquares = 0.0;

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        sumSquares +=
            q[i] * q[i];
    }

    return sqrt(
        sumSquares /
        (double)ROBOT_DOF
    );
}


/*
 * Extract the TCP position:
 *
 *      p = T(1:3,4)
 */
static void extract_translation(
    const double T[4][4],
    double p[3]
)
{
    p[0] = T[0][3];
    p[1] = T[1][3];
    p[2] = T[2][3];
}


/*
 * Extract the rotation matrix:
 *
 *      R = T(1:3,1:3)
 */
static void extract_rotation(
    const double T[4][4],
    double R[3][3]
)
{
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            R[row][column] =
                T[row][column];
        }
    }
}


/*
 * Matrix transpose:
 *
 *      B = A'
 */
static void matrix3_transpose(
    const double A[3][3],
    double B[3][3]
)
{
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            B[row][column] =
                A[column][row];
        }
    }
}


/*
 * 3x3 matrix multiplication:
 *
 *      C = A * B
 */
static void matrix3_multiply(
    const double A[3][3],
    const double B[3][3],
    double C[3][3]
)
{
    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            C[row][column] = 0.0;

            for (int k = 0; k < 3; k++)
            {
                C[row][column] +=
                    A[row][k] *
                    B[k][column];
            }
        }
    }
}


/* ============================================================================
 * LOCAL FUNCTION — SO(3) ROTATION LOGARITHM
 * ============================================================================
 *
 * Direct C equivalent of the local MATLAB function:
 *
 *      rotationLogVector(R)
 *
 * Input:
 *
 *      R
 *          3x3 rotation matrix.
 *
 * Output:
 *
 *      rotationVector
 *          theta * axis [rad].
 *
 * Therefore:
 *
 *      norm(rotationVector) = theta
 *
 * where theta is the shortest rotation angle in [0, pi].
 *
 * Special handling is used:
 *
 *      1. near zero rotation
 *      2. near pi rotation
 *      3. general rotation
 *
 * ============================================================================
 */

static void rotation_log_vector(
    const double R[3][3],
    double rotationVector[3]
)
{
    /* ------------------------------------------------------------------------
     * Calculate rotation angle
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     *      cosTheta = (trace(R) - 1) / 2;
     */

    double trace =
        R[0][0] +
        R[1][1] +
        R[2][2];


    double cosTheta =
        (trace - 1.0) / 2.0;


    /*
     * Floating-point roundoff protection.
     *
     * MATLAB:
     *
     *      cosTheta = max(-1,min(1,cosTheta));
     */
    cosTheta =
        clamp_value(
            cosTheta,
            -1.0,
             1.0
        );


    double theta =
        acos(cosTheta);


    /* ------------------------------------------------------------------------
     * Case 1:
     *
     * Rotation is essentially zero.
     * ------------------------------------------------------------------------
     */

    const double smallAngleTolerance =
        1e-8;


    if (theta < smallAngleTolerance)
    {
        rotationVector[0] = 0.0;
        rotationVector[1] = 0.0;
        rotationVector[2] = 0.0;

        return;
    }


    /* ------------------------------------------------------------------------
     * Case 2:
     *
     * Rotation is close to 180 degrees.
     * ------------------------------------------------------------------------
     */

    const double piTolerance =
        1e-6;


    if (fabs(ROBOT_PI - theta) < piTolerance)
    {
        double axis[3];


        if ((1.0 + R[2][2]) > smallAngleTolerance)
        {
            double scale =
                1.0 /
                sqrt(
                    2.0 *
                    (1.0 + R[2][2])
                );


            axis[0] =
                scale * R[0][2];

            axis[1] =
                scale * R[1][2];

            axis[2] =
                scale * (1.0 + R[2][2]);
        }
        else if ((1.0 + R[1][1]) > smallAngleTolerance)
        {
            double scale =
                1.0 /
                sqrt(
                    2.0 *
                    (1.0 + R[1][1])
                );


            axis[0] =
                scale * R[0][1];

            axis[1] =
                scale * (1.0 + R[1][1]);

            axis[2] =
                scale * R[2][1];
        }
        else
        {
            double scale =
                1.0 /
                sqrt(
                    2.0 *
                    (1.0 + R[0][0])
                );


            axis[0] =
                scale * (1.0 + R[0][0]);

            axis[1] =
                scale * R[1][0];

            axis[2] =
                scale * R[2][0];
        }


        /*
         * MATLAB:
         *
         *      axis = axis / norm(axis);
         */
        double axisNorm =
            vector3_norm(axis);


        if (axisNorm > 0.0)
        {
            axis[0] /= axisNorm;
            axis[1] /= axisNorm;
            axis[2] /= axisNorm;
        }


        /*
         * rotationVector = theta * axis;
         */
        rotationVector[0] =
            theta * axis[0];

        rotationVector[1] =
            theta * axis[1];

        rotationVector[2] =
            theta * axis[2];


        return;
    }


    /* ------------------------------------------------------------------------
     * Case 3:
     *
     * General rotation.
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     *      so3Matrix =
     *          theta/(2*sin(theta)) *
     *          (R - R');
     */

    double scale =
        theta /
        (2.0 * sin(theta));


    double so3Matrix[3][3];


    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            so3Matrix[row][column] =
                scale *
                (
                    R[row][column] -
                    R[column][row]
                );
        }
    }


    /*
     * MATLAB:
     *
     *      rotationVector = [
     *          so3Matrix(3,2)
     *          so3Matrix(1,3)
     *          so3Matrix(2,1)
     *      ];
     */

    rotationVector[0] =
        so3Matrix[2][1];

    rotationVector[1] =
        so3Matrix[0][2];

    rotationVector[2] =
        so3Matrix[1][0];
}


/* ============================================================================
 * DEFAULT ADLS PARAMETERS
 * ============================================================================
 */

void adls_default_parameters(
    ADLSParameters *parameters
)
{
    if (parameters == NULL)
    {
        return;
    }


    parameters->lambdaMax =
        ADLS_DEFAULT_LAMBDA_MAX;

    parameters->sigmaThreshold =
        ADLS_DEFAULT_SIGMA_THRESHOLD;
}


/* ============================================================================
 * ADAPTIVE DAMPED LEAST-SQUARES INVERSE KINEMATICS
 * ============================================================================
 */

bool adls_ik(
    const RobotConfig *robot,
    const double T_B_TCP_target[4][4],
    const double qSeed[ROBOT_DOF],
    const ADLSParameters *parameters,
    double qSolution[ROBOT_DOF],
    ADLSInfo *info
)
{
    /* ------------------------------------------------------------------------
     * Validate C pointers
     * ------------------------------------------------------------------------
     *
     * MATLAB validates matrix/vector sizes.
     *
     * Here those dimensions are fixed by the function declarations, so the
     * equivalent runtime validation is checking that the supplied pointers
     * exist.
     */

    if (
        robot == NULL ||
        T_B_TCP_target == NULL ||
        qSeed == NULL ||
        qSolution == NULL ||
        info == NULL
    )
    {
        return false;
    }


    /* ------------------------------------------------------------------------
     * Optional parameters
     * ------------------------------------------------------------------------
     */

    ADLSParameters localParameters;


    if (parameters == NULL)
    {
        adls_default_parameters(
            &localParameters
        );
    }
    else
    {
        localParameters =
            *parameters;
    }


    /*
     * MATLAB defaults:
     *
     *      lambdaMax     = 0.1
     *      sigmaThreshold = 1e-4
     */

    double lambdaMax =
        localParameters.lambdaMax;

    double sigmaThreshold =
        localParameters.sigmaThreshold;


    /* ------------------------------------------------------------------------
     * Initialize q from qSeed
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     *      q = qSeed(:);
     */

    double q[ROBOT_DOF];


    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        q[joint] =
            qSeed[joint];
    }


    /* ------------------------------------------------------------------------
     * Ensure initial configuration satisfies joint limits
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     *      q = min(max(q,qMin),qMax);
     */

    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        q[joint] =
            clamp_value(
                q[joint],
                robot->limits.qMin[joint],
                robot->limits.qMax[joint]
            );
    }


    /* ------------------------------------------------------------------------
     * Extract desired TCP pose
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     *      pTarget = T_B_TCP_target(1:3,4);
     *      RTarget = T_B_TCP_target(1:3,1:3);
     */

    double pTarget[3];

    double RTarget[3][3];


    extract_translation(
        T_B_TCP_target,
        pTarget
    );


    extract_rotation(
        T_B_TCP_target,
        RTarget
    );


    /* ------------------------------------------------------------------------
     * Initialize output information
     * ------------------------------------------------------------------------
     */

    memset(
        info,
        0,
        sizeof(*info)
    );


    info->converged =
        false;

    info->iterations =
        0;

    info->positionError =
        INFINITY;

    info->orientationError =
        INFINITY;

    info->sigmaMin =
        NAN;

    info->lambda =
        NAN;

    info->rmsJointUpdate =
        NAN;

    info->historyLength =
        0;

    info->qHistoryLength =
        1;


    /* ------------------------------------------------------------------------
     * Initialize histories to NaN
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     *      sigmaMinHistory = nan(maxIterations,1);
     *      lambdaHistory = nan(maxIterations,1);
     *      rmsJointUpdateHistory = nan(maxIterations,1);
     *
     *      qHistory = nan(robot.dof,maxIterations+1);
     */

    for (int iteration = 0;
         iteration < ADLS_MAX_ITERATIONS;
         iteration++)
    {
        info->sigmaMinHistory[iteration] =
            NAN;

        info->lambdaHistory[iteration] =
            NAN;

        info->rmsJointUpdateHistory[iteration] =
            NAN;
    }


    for (int sample = 0;
         sample <= ADLS_MAX_ITERATIONS;
         sample++)
    {
        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            info->qHistory[sample][joint] =
                NAN;
        }
    }


    /*
     * MATLAB:
     *
     *      qHistory(:,1) = q;
     */
    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        info->qHistory[0][joint] =
            q[joint];
    }


    /* ========================================================================
     * ITERATIVE IK LOOP
     * ========================================================================
     */

    for (int iterationIndex = 0;
         iterationIndex < ADLS_MAX_ITERATIONS;
         iterationIndex++)
    {
        /*
         * MATLAB iterations are 1-based.
         */
        int iteration =
            iterationIndex + 1;


        /* --------------------------------------------------------------------
         * 1. Calculate current TCP pose
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      TCurrent = controlFK(robot,q);
         */

        double TCurrent[4][4];


        control_fk(
            robot,
            q,
            TCurrent
        );


        double pCurrent[3];

        double RCurrent[3][3];


        extract_translation(
            TCurrent,
            pCurrent
        );


        extract_rotation(
            TCurrent,
            RCurrent
        );


        /* --------------------------------------------------------------------
         * 2. Position error
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      positionErrorVector =
         *          pTarget - pCurrent;
         */

        double positionErrorVector[3];


        for (int axis = 0;
             axis < 3;
             axis++)
        {
            positionErrorVector[axis] =
                pTarget[axis] -
                pCurrent[axis];
        }


        double positionError =
            vector3_norm(
                positionErrorVector
            );


        /* --------------------------------------------------------------------
         * 3. Orientation error using SO(3) logarithm
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      RError =
         *          RTarget * RCurrent';
         */

        double RCurrentTranspose[3][3];

        double RError[3][3];


        matrix3_transpose(
            RCurrent,
            RCurrentTranspose
        );


        matrix3_multiply(
            RTarget,
            RCurrentTranspose,
            RError
        );


        double orientationErrorVector[3];


        rotation_log_vector(
            RError,
            orientationErrorVector
        );


        double orientationError =
            vector3_norm(
                orientationErrorVector
            );


        /* --------------------------------------------------------------------
         * Store current iteration diagnostics
         * --------------------------------------------------------------------
         */

        info->iterations =
            iteration;

        info->positionError =
            positionError;

        info->orientationError =
            orientationError;


        /* --------------------------------------------------------------------
         * 4. Check Cartesian convergence
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      if positionError <= positionTolerance &&
         *         orientationError <= orientationTolerance
         */

        if (
            positionError <= ADLS_POSITION_TOLERANCE &&
            orientationError <= ADLS_ORIENTATION_TOLERANCE
        )
        {
            info->converged =
                true;


            /*
             * MATLAB:
             *
             * info histories contain only iterations where an actual
             * update was performed.
             *
             * At convergence during MATLAB iteration N:
             *
             *      history length = N - 1
             */
            info->historyLength =
                (size_t)iterationIndex;


            /*
             * MATLAB:
             *
             *      info.qHistory =
             *          qHistory(:,1:iteration);
             */
            info->qHistoryLength =
                (size_t)iteration;


            /*
             * qSolution = q
             */
            for (int joint = 0;
                 joint < ROBOT_DOF;
                 joint++)
            {
                qSolution[joint] =
                    q[joint];
            }


            return true;
        }


        /* --------------------------------------------------------------------
         * 5. Calculate geometric Jacobian
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      J = controlJacobian(robot,q);
         */

        double J[ROBOT_DOF][ROBOT_DOF];


        control_jacobian(
            robot,
            q,
            J
        );


        /* --------------------------------------------------------------------
         * 6. Singular Value Decomposition
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      singularValues = svd(J);
         *
         *      sigmaMin =
         *          min(singularValues);
         *
         * Our matrix module returns the equivalent smallest singular value.
         */

        double sigmaMin =
            matrix6_smallest_singular_value(
                J
            );


        /* --------------------------------------------------------------------
         * 7. Calculate adaptive damping factor
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         * if sigmaMin > sigmaThreshold
         *
         *      lambdaDLS = 0;
         *
         * else
         *
         *      ratio =
         *          sigmaMin / sigmaThreshold;
         *
         *      lambdaDLS =
         *          lambdaMax *
         *          sqrt(max(0,1-ratio^2));
         */

        double lambdaDLS;


        if (sigmaMin > sigmaThreshold)
        {
            lambdaDLS =
                0.0;
        }
        else
        {
            double ratio =
                sigmaMin /
                sigmaThreshold;


            double inside =
                1.0 -
                ratio * ratio;


            if (inside < 0.0)
            {
                inside = 0.0;
            }


            lambdaDLS =
                lambdaMax *
                sqrt(inside);
        }


        /* --------------------------------------------------------------------
         * 8. Construct Cartesian pose-error vector
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      poseError = [
         *          positionErrorVector
         *          orientationErrorVector
         *      ];
         */

        double poseError[ROBOT_DOF];


        poseError[0] =
            positionErrorVector[0];

        poseError[1] =
            positionErrorVector[1];

        poseError[2] =
            positionErrorVector[2];

        poseError[3] =
            orientationErrorVector[0];

        poseError[4] =
            orientationErrorVector[1];

        poseError[5] =
            orientationErrorVector[2];


        /* --------------------------------------------------------------------
         * 9. Adaptive Damped Least-Squares update
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      A =
         *          J' * J +
         *          lambdaDLS^2 * eye(robot.dof);
         *
         *      b =
         *          J' * poseError;
         *
         *      dq =
         *          A \ b;
         */

        double A[ROBOT_DOF][ROBOT_DOF];

        double b[ROBOT_DOF];


        /*
         * Initialize A and b.
         */
        for (int row = 0;
             row < ROBOT_DOF;
             row++)
        {
            b[row] =
                0.0;


            for (int column = 0;
                 column < ROBOT_DOF;
                 column++)
            {
                A[row][column] =
                    0.0;
            }
        }


        /*
         * A = J' * J
         */
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


        /*
         * A = A + lambdaDLS^2 * I
         */
        double lambdaSquared =
            lambdaDLS *
            lambdaDLS;


        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            A[joint][joint] +=
                lambdaSquared;
        }


        /*
         * b = J' * poseError
         */
        for (int row = 0;
             row < ROBOT_DOF;
             row++)
        {
            for (int k = 0;
                 k < ROBOT_DOF;
                 k++)
            {
                b[row] +=
                    J[k][row] *
                    poseError[k];
            }
        }


        /*
         * MATLAB:
         *
         *      dq = A \ b
         *
         * matrix6_solve() performs the equivalent linear solve without
         * explicitly calculating A^-1.
         */

        double dq[ROBOT_DOF];


        bool solveSucceeded =
            matrix6_solve(
                A,
                b,
                dq
            );


        if (!solveSucceeded)
        {
            /*
             * Numerical failure.
             *
             * MATLAB's "\" normally handles this internally and may issue a
             * warning for pathological matrices. In C we explicitly report
             * failure.
             */

            info->sigmaMin =
                sigmaMin;

            info->lambda =
                lambdaDLS;

            info->historyLength =
                (size_t)iterationIndex;

            info->qHistoryLength =
                (size_t)iteration;


            for (int joint = 0;
                 joint < ROBOT_DOF;
                 joint++)
            {
                qSolution[joint] =
                    q[joint];
            }


            return false;
        }


        /* --------------------------------------------------------------------
         * 10. Apply joint-update scaling
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      dqApplied =
         *          stepSize * dq;
         */

        double dqApplied[ROBOT_DOF];


        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            dqApplied[joint] =
                ADLS_STEP_SIZE *
                dq[joint];
        }


        /* --------------------------------------------------------------------
         * 11. Calculate RMS joint update
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      rmsJointUpdate =
         *          sqrt(mean(dqApplied.^2));
         */

        double rmsJointUpdate =
            joint_rms(
                dqApplied
            );


        /* --------------------------------------------------------------------
         * 12. Update joint configuration
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      q = q + dqApplied;
         */

        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            q[joint] +=
                dqApplied[joint];
        }


        /* --------------------------------------------------------------------
         * 13. Enforce joint limits
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      q =
         *          min(max(q,qMin),qMax);
         */

        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            q[joint] =
                clamp_value(
                    q[joint],
                    robot->limits.qMin[joint],
                    robot->limits.qMax[joint]
                );
        }


        /* --------------------------------------------------------------------
         * Record complete joint configuration after update
         * --------------------------------------------------------------------
         *
         * MATLAB:
         *
         *      qHistory(:,iteration + 1) = q;
         */

        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            info->qHistory[iterationIndex + 1][joint] =
                q[joint];
        }


        info->qHistoryLength =
            (size_t)iteration + 1;


        /* --------------------------------------------------------------------
         * 14. Store ADLS diagnostics
         * --------------------------------------------------------------------
         */

        info->sigmaMinHistory[iterationIndex] =
            sigmaMin;

        info->lambdaHistory[iterationIndex] =
            lambdaDLS;

        info->rmsJointUpdateHistory[iterationIndex] =
            rmsJointUpdate;


        info->historyLength =
            (size_t)iteration;


        info->sigmaMin =
            sigmaMin;

        info->lambda =
            lambdaDLS;

        info->rmsJointUpdate =
            rmsJointUpdate;
    }


    /* ========================================================================
     * MAXIMUM ITERATIONS REACHED WITHOUT CONVERGENCE
     * ========================================================================
     *
     * MATLAB:
     *
     *      qSolution = q;
     *
     *      info.converged = false;
     *
     *      info.sigmaMinHistory =
     *          sigmaMinHistory(1:maxIterations);
     *
     *      info.lambdaHistory =
     *          lambdaHistory(1:maxIterations);
     *
     *      info.rmsJointUpdateHistory =
     *          rmsJointUpdateHistory(1:maxIterations);
     *
     *      info.qHistory =
     *          qHistory(:,1:maxIterations+1);
     */

    info->converged =
        false;

    info->historyLength =
        ADLS_MAX_ITERATIONS;

    info->qHistoryLength =
        ADLS_MAX_ITERATIONS + 1;


    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        qSolution[joint] =
            q[joint];
    }


    return false;
}