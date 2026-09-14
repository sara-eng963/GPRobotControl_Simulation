#ifndef CONTROL_JACOBIAN_H
#define CONTROL_JACOBIAN_H

#include "../Config/robot_config.h"


/* ============================================================================
 * GEOMETRIC JACOBIAN
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      controlJacobian.m
 *
 *
 * Convention:
 *
 *      [ v_B     ]
 *      [ omega_B ] = J * q_dot
 *
 * where:
 *
 *      v_B
 *          TCP linear velocity expressed in the Base frame.
 *
 *      omega_B
 *          TCP angular velocity expressed in the Base frame.
 *
 *
 * Input:
 *
 *      robot
 *          Shared robot configuration.
 *
 *      q
 *          Joint configuration [rad].
 *
 *
 * Output:
 *
 *      J
 *          6x6 geometric Jacobian.
 *
 *          Rows 0..2:
 *              linear velocity contribution.
 *
 *          Rows 3..5:
 *              angular velocity contribution.
 *
 *          Column i:
 *              contribution of joint i.
 *
 * ============================================================================
 */

void control_jacobian(
    const RobotConfig *robot,
    const double q[ROBOT_DOF],
    double J[ROBOT_DOF][ROBOT_DOF]
);


#endif /* CONTROL_JACOBIAN_H */