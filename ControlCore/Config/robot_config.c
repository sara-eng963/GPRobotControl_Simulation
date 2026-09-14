#include "robot_config.h"

#include <stddef.h>


/* ============================================================================
 * UR5 CONFIGURATION
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      robot = config.UR5()
 *
 * IMPORTANT:
 *
 * This file contains robot DATA only.
 *
 * No FK, IK, Jacobian, dynamics, plotting, or simulation is performed here.
 *
 * ============================================================================
 */


void robot_config_init_ur5(RobotConfig *robot)
{
    /*
     * Protect against an invalid pointer.
     */
    if (robot == NULL)
    {
        return;
    }


    /* ========================================================================
     * 1. ROBOT IDENTITY
     * ========================================================================
     */

    robot->identity.name =
        "UR5";

    robot->identity.manufacturer =
        "Universal Robots";

    robot->identity.role =
        "Reference Robot";

    robot->identity.dof =
        ROBOT_DOF;


    /* ========================================================================
     * 2. PROJECT CONVENTIONS
     * ========================================================================
     */

    robot->conventions.coordinateSystem =
        "right-handed";

    robot->conventions.lengthUnit =
        "m";

    robot->conventions.angleUnit =
        "rad";

    robot->conventions.angularVelocityUnit =
        "rad/s";

    robot->conventions.jointVectorShape =
        "6x1";

    robot->conventions.transformNotation =
        "T_A_B = pose of frame B expressed in frame A";


    /* ========================================================================
     * 3. JOINT DEFINITIONS
     * ========================================================================
     */

    robot->joints.names[0] =
        "J1_Base";

    robot->joints.names[1] =
        "J2_Shoulder";

    robot->joints.names[2] =
        "J3_Elbow";

    robot->joints.names[3] =
        "J4_Wrist1";

    robot->joints.names[4] =
        "J5_Wrist2";

    robot->joints.names[5] =
        "J6_Wrist3";


    /*
     * All UR5 joints are revolute.
     */
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        robot->joints.type[i] =
            "revolute";
    }


    /*
     * MATLAB:
     *
     *      robot.joints.order = (1:robot.dof).'
     */
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        robot->joints.order[i] =
            i + 1;
    }


    /* ========================================================================
     * 4. STANDARD DH KINEMATIC PARAMETERS
     * ========================================================================
     *
     * For each revolute joint:
     *
     *      theta_i = q_i + thetaOffset_i
     *
     * Standard DH:
     *
     *      A_i =
     *          RotZ(theta_i)
     *          TransZ(d_i)
     *          TransX(a_i)
     *          RotX(alpha_i)
     *
     * Units:
     *
     *      a, d     -> metres
     *      alpha    -> radians
     * ========================================================================
     */

    robot->kinematics.convention =
        "Standard DH";


    /*
     * robot.kinematics.a
     */
    robot->kinematics.a[0] =  0.0;
    robot->kinematics.a[1] = -0.425;
    robot->kinematics.a[2] = -0.39225;
    robot->kinematics.a[3] =  0.0;
    robot->kinematics.a[4] =  0.0;
    robot->kinematics.a[5] =  0.0;


    /*
     * robot.kinematics.d
     */
    robot->kinematics.d[0] = 0.089159;
    robot->kinematics.d[1] = 0.0;
    robot->kinematics.d[2] = 0.0;
    robot->kinematics.d[3] = 0.10915;
    robot->kinematics.d[4] = 0.09465;
    robot->kinematics.d[5] = 0.0823;


    /*
     * robot.kinematics.alpha
     */
    robot->kinematics.alpha[0] =  ROBOT_PI / 2.0;
    robot->kinematics.alpha[1] =  0.0;
    robot->kinematics.alpha[2] =  0.0;
    robot->kinematics.alpha[3] =  ROBOT_PI / 2.0;
    robot->kinematics.alpha[4] = -ROBOT_PI / 2.0;
    robot->kinematics.alpha[5] =  0.0;


    /*
     * robot.kinematics.thetaOffset = zeros(robot.dof,1)
     */
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        robot->kinematics.thetaOffset[i] =
            0.0;
    }


    /* ========================================================================
     * 5. JOINT LIMITS
     * ========================================================================
     *
     * Published working range:
     *
     *      +/- 360 deg = +/- 2*pi rad
     *
     * Published maximum joint speed:
     *
     *      180 deg/s = pi rad/s
     * ========================================================================
     */

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        robot->limits.qMin[i] =
            -2.0 * ROBOT_PI;

        robot->limits.qMax[i] =
             2.0 * ROBOT_PI;

        robot->limits.qdMax[i] =
            ROBOT_PI;
    }


    /*
     * MATLAB:
     *
     *      robot.limits.qddMax = [];
     *
     * No acceleration limits are defined yet.
     */
    robot->limits.qddMaxDefined =
        false;

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        robot->limits.qddMax[i] =
            0.0;
    }


    /* ========================================================================
     * 6. REFERENCE CONFIGURATIONS
     * ========================================================================
     */

    /*
     * Mathematical zero configuration:
     *
     *      zeros(robot.dof,1)
     */
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        robot->configuration.zero[i] =
            0.0;
    }


    /*
     * MATLAB:
     *
     *      robot.configuration.home = [];
     *
     * Home configuration is intentionally undefined.
     */
    robot->configuration.homeDefined =
        false;

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        robot->configuration.home[i] =
            0.0;
    }


    /* ========================================================================
     * 7. FRAME DEFINITIONS
     * ========================================================================
     */

    robot->frames.base =
        "Base";

    robot->frames.flange =
        "Flange";

    robot->frames.tcp =
        "TCP";


    /* ========================================================================
     * 8. TOOL / TCP DEFINITION
     * ========================================================================
     *
     * MATLAB:
     *
     *      robot.tool.T_F_TCP = eye(4)
     *
     * Therefore the TCP currently coincides with the UR5 flange.
     * ========================================================================
     */

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            if (row == column)
            {
                robot->tool.T_F_TCP[row][column] =
                    1.0;
            }
            else
            {
                robot->tool.T_F_TCP[row][column] =
                    0.0;
            }
        }
    }


    /* ========================================================================
     * 9. COMMERCIAL REFERENCE PERFORMANCE
     * ========================================================================
     */

    robot->performance.reach =
        0.850;

    robot->performance.payload =
        5.0;

    robot->performance.repeatability =
        0.0001;

    robot->performance.maxJointSpeed =
        ROBOT_PI;


    /* ========================================================================
     * 10. SOURCE METADATA
     * ========================================================================
     */

    robot->source.geometry =
        "Universal Robots official DH parameter documentation";

    robot->source.performance =
        "Universal Robots original UR5 technical specification";

    robot->source.parameterStatus =
        "Commercial reference data";
}