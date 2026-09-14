#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdbool.h>

#define ROBOT_DOF 6
#define ROBOT_PI  3.14159265358979323846


/* ============================================================================
 * ROBOT CONFIGURATION DATA STRUCTURES
 * ============================================================================
 *
 * C equivalent of:
 *
 *      +config/UR5.m
 *
 * This file contains robot DATA only.
 *
 * It must not perform:
 *
 *      - Forward kinematics
 *      - Inverse kinematics
 *      - Jacobian calculations
 *      - Dynamics
 *      - Plotting
 *      - Simulation
 *
 * Project conventions:
 *
 *      Length              : metre [m]
 *      Joint angle         : radian [rad]
 *      Joint velocity      : rad/s
 *      Coordinate system   : right-handed
 *      Joint vector        : 6 elements
 *      DH convention       : Standard Denavit-Hartenberg
 *
 * ============================================================================
 */


/* ============================================================================
 * 1. ROBOT IDENTITY
 * ============================================================================
 */

typedef struct
{
    const char *name;
    const char *manufacturer;
    const char *role;

    int dof;

} RobotIdentity;


/* ============================================================================
 * 2. PROJECT CONVENTIONS
 * ============================================================================
 */

typedef struct
{
    const char *coordinateSystem;

    const char *lengthUnit;
    const char *angleUnit;
    const char *angularVelocityUnit;

    const char *jointVectorShape;

    const char *transformNotation;

} RobotConventions;


/* ============================================================================
 * 3. JOINT DEFINITIONS
 * ============================================================================
 */

typedef struct
{
    const char *names[ROBOT_DOF];

    /*
     * All six UR5 joints are revolute.
     */
    const char *type[ROBOT_DOF];

    /*
     * MATLAB:
     *
     *      robot.joints.order = (1:robot.dof).'
     *
     * Stored here as:
     *
     *      {1, 2, 3, 4, 5, 6}
     */
    int order[ROBOT_DOF];

} RobotJoints;


/* ============================================================================
 * 4. STANDARD DH KINEMATIC PARAMETERS
 * ============================================================================
 */

typedef struct
{
    const char *convention;

    /*
     * Standard DH parameters.
     *
     * For each revolute joint:
     *
     *      theta_i = q_i + thetaOffset_i
     *
     * Units:
     *
     *      a, d        : metre [m]
     *      alpha       : radian [rad]
     *      thetaOffset : radian [rad]
     */
    double a[ROBOT_DOF];
    double d[ROBOT_DOF];
    double alpha[ROBOT_DOF];
    double thetaOffset[ROBOT_DOF];

} RobotKinematics;


/* ============================================================================
 * 5. JOINT LIMITS
 * ============================================================================
 */

typedef struct
{
    /*
     * Joint-position limits [rad].
     */
    double qMin[ROBOT_DOF];
    double qMax[ROBOT_DOF];

    /*
     * Maximum joint velocity [rad/s].
     */
    double qdMax[ROBOT_DOF];

    /*
     * MATLAB source contains:
     *
     *      robot.limits.qddMax = [];
     *
     * Therefore no acceleration limits are currently defined.
     */
    bool qddMaxDefined;

    double qddMax[ROBOT_DOF];

} RobotLimits;


/* ============================================================================
 * 6. REFERENCE CONFIGURATIONS
 * ============================================================================
 */

typedef struct
{
    /*
     * Mathematical zero configuration [rad].
     */
    double zero[ROBOT_DOF];

    /*
     * MATLAB source contains:
     *
     *      robot.configuration.home = [];
     *
     * Therefore home is not currently defined.
     */
    bool homeDefined;

    double home[ROBOT_DOF];

} RobotConfiguration;


/* ============================================================================
 * 7. FRAME DEFINITIONS
 * ============================================================================
 */

typedef struct
{
    const char *base;
    const char *flange;
    const char *tcp;

} RobotFrames;


/* ============================================================================
 * 8. TOOL / TCP DEFINITION
 * ============================================================================
 */

typedef struct
{
    /*
     * Homogeneous transformation:
     *
     *      T_F_TCP
     *
     * Pose of TCP relative to flange.
     *
     * Stored row-major:
     *
     *      T_F_TCP[row][column]
     */
    double T_F_TCP[4][4];

} RobotTool;


/* ============================================================================
 * 9. COMMERCIAL REFERENCE PERFORMANCE
 * ============================================================================
 */

typedef struct
{
    double reach;          /* [m] */
    double payload;        /* [kg] */
    double repeatability;  /* [m] */
    double maxJointSpeed;  /* [rad/s] */

} RobotPerformance;


/* ============================================================================
 * 10. SOURCE METADATA
 * ============================================================================
 */

typedef struct
{
    const char *geometry;
    const char *performance;
    const char *parameterStatus;

} RobotSource;


/* ============================================================================
 * COMPLETE ROBOT CONFIGURATION
 * ============================================================================
 */

typedef struct
{
    RobotIdentity identity;

    RobotConventions conventions;

    RobotJoints joints;

    RobotKinematics kinematics;

    RobotLimits limits;

    RobotConfiguration configuration;

    RobotFrames frames;

    RobotTool tool;

    RobotPerformance performance;

    RobotSource source;

} RobotConfig;


/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

/*
 * Initialize a RobotConfig structure with the canonical UR5 configuration
 * contained in the original MATLAB config.UR5() file.
 */
void robot_config_init_ur5(RobotConfig *robot);


#endif /* ROBOT_CONFIG_H */