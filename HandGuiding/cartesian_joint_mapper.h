#ifndef HANDGUIDING_CARTESIAN_JOINT_MAPPER_H
#define HANDGUIDING_CARTESIAN_JOINT_MAPPER_H

#include <stdbool.h>
#include "../ControlCore/Config/robot_config.h"

/* Input twist: [vx,vy,vz,wx,wy,wz] at TCP, expressed in BASE.
 * Model/limits/tuning approval is external; there are NO physical defaults. */
typedef struct {
    bool model_verified;
    double damping_lambda;                  /* DLS damping, > 0 */
    double minimum_sigma;                   /* minimum Jacobian singular value */
    double maximum_step_s;                  /* maximum accepted loop duration */
    double joint_acceleration_limit[ROBOT_DOF]; /* rad/s^2 */
    double tracking_error_limit[ROBOT_DOF];     /* rad */
} CartesianJointMapperConfig;

typedef struct {
    bool enabled;
    double position_ref[ROBOT_DOF];         /* rad */
    double velocity_ref[ROBOT_DOF];         /* rad/s */
} CartesianJointMapperState;

typedef struct {
    bool valid;
    bool limited;                           /* constraint altered velocity */
    double position_ref[ROBOT_DOF];
    double velocity_ref[ROBOT_DOF];
    double sigma_min;                       /* mixed Jacobian units */
} CartesianJointMapperOutput;

bool cartesian_joint_mapper_config_valid(
    const CartesianJointMapperConfig *config, const RobotConfig *robot);
void cartesian_joint_mapper_disable(CartesianJointMapperState *state);

/* Must be called with fresh measured joint angles and verified robot model.
 * Marking a synthetic fixture as verified never commissions real hardware. */
bool cartesian_joint_mapper_enable(
    CartesianJointMapperState *state, const CartesianJointMapperConfig *config,
    const RobotConfig *robot, const double measured_q[ROBOT_DOF]);

/* Damped least squares using existing ControlCore Jacobian and solver.
 * Enforces speed, acceleration, position and feedback-tracking constraints.
 * Any failure clears output and state; caller MUST inhibit guidance, not
 * interpret an invalid output as a motor hold/stop/safety command.
 * No hardware access. Input/output/state storage must not overlap. */
bool cartesian_joint_mapper_step(
    CartesianJointMapperState *state, const CartesianJointMapperConfig *config,
    const RobotConfig *robot, const double measured_q[ROBOT_DOF],
    const double twist_base_tcp[ROBOT_DOF], double period_s,
    CartesianJointMapperOutput *output);
#endif
