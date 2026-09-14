#ifndef CONTROL_FK_H
#define CONTROL_FK_H

#include "../Config/robot_config.h"


/* ============================================================================
 * FORWARD KINEMATICS
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      controlFK.m
 *
 * Computes:
 *
 *      T_B_TCP
 *
 * = pose of the TCP expressed in the Base frame.
 *
 *
 * MATLAB equivalent:
 *
 *      T_B_TCP = controlFK(robot, q)
 *
 *
 * Inputs:
 *
 *      robot
 *          Shared robot configuration.
 *
 *      q
 *          Joint configuration [rad].
 *
 *          q[0] = q1
 *          q[1] = q2
 *          ...
 *          q[5] = q6
 *
 *
 * Output:
 *
 *      T_B_TCP
 *          4x4 homogeneous transformation matrix.
 *
 *          Stored in C as:
 *
 *              T_B_TCP[row][column]
 *
 * ============================================================================
 */

void control_fk(
    const RobotConfig *robot,
    const double q[ROBOT_DOF],
    double T_B_TCP[4][4]
);


#endif /* CONTROL_FK_H */