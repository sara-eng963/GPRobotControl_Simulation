#ifndef ADLS_IK_H
#define ADLS_IK_H

#include <stdbool.h>
#include <stddef.h>

#include "../Config/robot_config.h"


/* ============================================================================
 * ADLS CONFIGURATION
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      ADLS_IK.m
 *
 * MATLAB solver constants:
 *
 *      maxIterations        = 1000
 *      positionTolerance    = 1e-4 m
 *      orientationTolerance = 1e-4 rad
 *      stepSize             = 0.5
 *
 * Optional MATLAB parameters:
 *
 *      lambdaMax
 *      sigmaThreshold
 *
 * ============================================================================
 */

#define ADLS_MAX_ITERATIONS          1000

#define ADLS_POSITION_TOLERANCE      1e-4
#define ADLS_ORIENTATION_TOLERANCE   1e-4
#define ADLS_STEP_SIZE               0.5

#define ADLS_DEFAULT_LAMBDA_MAX      0.1
#define ADLS_DEFAULT_SIGMA_THRESHOLD 1e-4


/* ============================================================================
 * ADLS OPTIONAL PARAMETERS
 * ============================================================================
 *
 * MATLAB:
 *
 *      if nargin < 4 || isempty(lambdaMax)
 *          lambdaMax = 0.1;
 *      end
 *
 *      if nargin < 5 || isempty(sigmaThreshold)
 *          sigmaThreshold = 1e-4;
 *      end
 *
 * C has no optional function arguments, so these two parameters are grouped
 * here.
 */

typedef struct
{
    double lambdaMax;
    double sigmaThreshold;

} ADLSParameters;


/* ============================================================================
 * ADLS DIAGNOSTIC INFORMATION
 * ============================================================================
 *
 * C equivalent of MATLAB's:
 *
 *      info.converged
 *      info.iterations
 *      info.positionError
 *      info.orientationError
 *      info.sigmaMin
 *      info.lambda
 *      info.rmsJointUpdate
 *
 * plus:
 *
 *      info.sigmaMinHistory
 *      info.lambdaHistory
 *      info.rmsJointUpdateHistory
 *      info.qHistory
 *
 * MATLAB stores qHistory as:
 *
 *      6 x (iterations + 1)
 *
 * C stores the same information as:
 *
 *      qHistory[sample][joint]
 *
 * because that layout is more natural in C.
 * ============================================================================
 */

typedef struct
{
    bool converged;

    int iterations;

    double positionError;
    double orientationError;

    double sigmaMin;
    double lambda;
    double rmsJointUpdate;


    /* ------------------------------------------------------------------------
     * Iteration histories
     * ------------------------------------------------------------------------
     */

    double sigmaMinHistory[ADLS_MAX_ITERATIONS];

    double lambdaHistory[ADLS_MAX_ITERATIONS];

    double rmsJointUpdateHistory[ADLS_MAX_ITERATIONS];

    /*
     * Number of valid entries in the three histories above.
     */
    size_t historyLength;


    /* ------------------------------------------------------------------------
     * Joint-configuration history
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     *      qHistory(:,1) = qSeed
     *
     * then one new column after every applied IK update.
     *
     * C:
     *
     *      qHistory[0] = qSeed
     *
     * followed by one row after every applied update.
     */

    double qHistory[ADLS_MAX_ITERATIONS + 1][ROBOT_DOF];

    size_t qHistoryLength;

} ADLSInfo;


/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */


/*
 * Fill an ADLSParameters structure with the same defaults used by MATLAB.
 */
void adls_default_parameters(
    ADLSParameters *parameters
);


/*
 * Adaptive Damped Least-Squares inverse kinematics.
 *
 * Inputs:
 *
 *      robot
 *          Shared robot configuration.
 *
 *      T_B_TCP_target
 *          Desired TCP pose expressed in the Base frame.
 *
 *          4x4 homogeneous transformation.
 *
 *      qSeed
 *          Initial robot joint configuration [rad].
 *
 *      parameters
 *          ADLS optional parameters.
 *
 *          If NULL, MATLAB default values are used:
 *
 *              lambdaMax     = 0.1
 *              sigmaThreshold = 1e-4
 *
 *
 * Outputs:
 *
 *      qSolution
 *          IK solution [rad].
 *
 *      info
 *          Complete solver diagnostic information.
 *
 *
 * Return:
 *
 *      true
 *          IK converged.
 *
 *      false
 *          Maximum iterations were reached or a numerical solve failed.
 */
bool adls_ik(
    const RobotConfig *robot,
    const double T_B_TCP_target[4][4],
    const double qSeed[ROBOT_DOF],
    const ADLSParameters *parameters,
    double qSolution[ROBOT_DOF],
    ADLSInfo *info
);


#endif /* ADLS_IK_H */