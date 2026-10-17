#include "cartesian_joint_mapper.h"

#include "../ControlCore/Kinematics/control_jacobian.h"
#include "../ControlCore/Math/matrix6.h"
#include "../ControlCore/Analysis/singularity.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* NOTE: Software-only planner; no FreeRTOS or CANopen/motion authorization. */
static bool model_valid(const RobotConfig *r)
{
    if (r == NULL || r->identity.dof != ROBOT_DOF) return false;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        if (!isfinite(r->limits.qMin[i]) || !isfinite(r->limits.qMax[i]) ||
            r->limits.qMin[i] >= r->limits.qMax[i] ||
            !isfinite(r->limits.qdMax[i]) || r->limits.qdMax[i] <= 0.0 ||
            !isfinite(r->kinematics.a[i]) || !isfinite(r->kinematics.d[i]) ||
            !isfinite(r->kinematics.alpha[i]) ||
            !isfinite(r->kinematics.thetaOffset[i]) ||
            (r->limits.qddMaxDefined &&
             (!isfinite(r->limits.qddMax[i]) || r->limits.qddMax[i] <= 0.0)))
            return false;
    }
    for (size_t i = 0U; i < 4U; ++i)
        for (size_t j = 0U; j < 4U; ++j)
            if (!isfinite(r->tool.T_F_TCP[i][j])) return false;
    for (size_t j = 0U; j < 3U; ++j)
        if (fabs(r->tool.T_F_TCP[3][j]) > 1e-9) return false;
    return fabs(r->tool.T_F_TCP[3][3] - 1.0) <= 1e-9;
}

bool cartesian_joint_mapper_config_valid(
    const CartesianJointMapperConfig *c, const RobotConfig *r)
{
    if (c == NULL || !c->model_verified || !model_valid(r) ||
        !isfinite(c->damping_lambda) || c->damping_lambda <= 0.0 ||
        !isfinite(c->minimum_sigma) || c->minimum_sigma <= 0.0 ||
        !isfinite(c->maximum_step_s) || c->maximum_step_s <= 0.0)
        return false;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        if (!isfinite(c->joint_acceleration_limit[i]) ||
            c->joint_acceleration_limit[i] <= 0.0 ||
            !isfinite(c->tracking_error_limit[i]) ||
            c->tracking_error_limit[i] <= 0.0 ||
            (r->limits.qddMaxDefined &&
             c->joint_acceleration_limit[i] > r->limits.qddMax[i]))
            return false;
    }
    return true;
}

void cartesian_joint_mapper_disable(CartesianJointMapperState *s)
{
    if (s != NULL) memset(s, 0, sizeof(*s));
}

bool cartesian_joint_mapper_enable(
    CartesianJointMapperState *s, const CartesianJointMapperConfig *c,
    const RobotConfig *r, const double q[ROBOT_DOF])
{
    if (s == NULL) return false;
    cartesian_joint_mapper_disable(s);
    if (!cartesian_joint_mapper_config_valid(c, r) || q == NULL) return false;
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
        if (!isfinite(q[i]) || q[i] < r->limits.qMin[i] ||
            q[i] > r->limits.qMax[i]) return false;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) s->position_ref[i] = q[i];
    s->enabled = true;
    return true;
}

bool cartesian_joint_mapper_step(
    CartesianJointMapperState *s, const CartesianJointMapperConfig *c,
    const RobotConfig *r, const double q[ROBOT_DOF],
    const double twist[ROBOT_DOF], double dt,
    CartesianJointMapperOutput *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (s == NULL || !s->enabled || out == NULL || q == NULL ||
        twist == NULL || !cartesian_joint_mapper_config_valid(c, r) ||
        !isfinite(dt) || dt <= 0.0 || dt > c->maximum_step_s)
        goto failure;

    for (size_t i = 0U; i < ROBOT_DOF; ++i)
        if (!isfinite(q[i]) || !isfinite(twist[i]) ||
            !isfinite(s->position_ref[i]) || !isfinite(s->velocity_ref[i]) ||
            q[i] < r->limits.qMin[i] || q[i] > r->limits.qMax[i] ||
            s->position_ref[i] < r->limits.qMin[i] ||
            s->position_ref[i] > r->limits.qMax[i] ||
            fabs(s->velocity_ref[i]) > r->limits.qdMax[i] ||
            fabs(s->position_ref[i] - q[i]) > c->tracking_error_limit[i])
            goto failure;

    double J[ROBOT_DOF][ROBOT_DOF];
    control_jacobian(r, q, J);
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
        for (size_t j = 0U; j < ROBOT_DOF; ++j)
            if (!isfinite(J[i][j])) goto failure;

    double sigma = 0.0;
    if (!singularity_sigma_min((const double (*)[ROBOT_DOF])J, &sigma) ||
        !isfinite(sigma) || sigma < c->minimum_sigma) goto failure;

    /* Damped least squares: (J^T J + lambda^2 I) qdot = J^T twist. */
    double A[ROBOT_DOF][ROBOT_DOF] = {{0}};
    double b[ROBOT_DOF] = {0};
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        for (size_t row = 0U; row < ROBOT_DOF; ++row) {
            b[i] += J[row][i] * twist[row];
            for (size_t j = 0U; j < ROBOT_DOF; ++j)
                A[i][j] += J[row][i] * J[row][j];
        }
        A[i][i] += c->damping_lambda * c->damping_lambda;
    }
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        if (!isfinite(b[i])) goto failure;
        for (size_t j = 0U; j < ROBOT_DOF; ++j)
            if (!isfinite(A[i][j])) goto failure;
    }
    double qd_requested[ROBOT_DOF] = {0};
    if (!matrix6_solve((const double (*)[ROBOT_DOF])A, b, qd_requested))
        goto failure;

    double next_q[ROBOT_DOF], next_qd[ROBOT_DOF];
    bool limited = false;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        double vel = qd_requested[i];
        const double v_max = r->limits.qdMax[i];
        const double dv_max = c->joint_acceleration_limit[i] * dt;
        if (!isfinite(vel) || !isfinite(dv_max) || dv_max <= 0.0)
            goto failure;
        if (vel > v_max) { vel = v_max; limited = true; }
        else if (vel < -v_max) { vel = -v_max; limited = true; }
        const double prev = s->velocity_ref[i];
        if (vel - prev > dv_max) { vel = prev + dv_max; limited = true; }
        else if (vel - prev < -dv_max) {
            vel = prev - dv_max; limited = true;
        }
        const double position = s->position_ref[i] + dt * vel;
        if (!isfinite(position) || !isfinite(vel) ||
            fabs(vel) > v_max ||
            position < r->limits.qMin[i] || position > r->limits.qMax[i] ||
            fabs(position - q[i]) > c->tracking_error_limit[i])
            goto failure;
        next_q[i] = position;
        next_qd[i] = vel;
    }
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        s->position_ref[i] = out->position_ref[i] = next_q[i];
        s->velocity_ref[i] = out->velocity_ref[i] = next_qd[i];
    }
    out->sigma_min = sigma;
    out->limited = limited;
    out->valid = true;
    return true;

failure:
    cartesian_joint_mapper_disable(s);
    return false;
}
