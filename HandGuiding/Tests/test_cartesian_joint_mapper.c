#include "../cartesian_joint_mapper.h"
#include "../../ControlCore/Kinematics/control_jacobian.h"
#include "../../ControlCore/Math/matrix6.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const double q0[ROBOT_DOF] = {0.3, -1.0, 1.2, -0.5, 0.7, 0.1};
static CartesianJointMapperConfig config(void)
{
    CartesianJointMapperConfig c = {0};
    c.model_verified = true; /* SYNTHETIC TEST ONLY, NOT REAL ARM */
    c.damping_lambda = 0.02;
    c.minimum_sigma = 1e-10;
    c.maximum_step_s = 0.004;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        c.joint_acceleration_limit[i] = 1000.0;
        c.tracking_error_limit[i] = 0.5;
    }
    return c;
}
static void cleared(const CartesianJointMapperState *s,
                    const CartesianJointMapperOutput *o)
{
    assert(!s->enabled);
    assert(!o->valid);
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        assert(s->position_ref[i] == 0.0);
        assert(s->velocity_ref[i] == 0.0);
        assert(o->position_ref[i] == 0.0);
        assert(o->velocity_ref[i] == 0.0);
    }
}
static bool close_to(double a, double b)
{
    return isfinite(a) && fabs(a-b) < 1e-9;
}
int main(void)
{
    RobotConfig robot;
    robot_config_init_ur5(&robot);
    CartesianJointMapperConfig c = config();
    CartesianJointMapperState s = {0};
    CartesianJointMapperOutput o = {0};
    const double dt = 0.002;
    double zero[ROBOT_DOF] = {0};
    double twist[ROBOT_DOF] = {0.005, -0.006, 0.004, 0.001, -0.002, 0.003};

    /* No model approval, no motion; and initial reference = measurements. */
    c.model_verified = false;
    assert(!cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    assert(!s.enabled);
    c.model_verified = true;
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
        assert(close_to(s.position_ref[i], q0[i]) && s.velocity_ref[i] == 0.0);

    assert(cartesian_joint_mapper_step(&s, &c, &robot, q0, zero, dt, &o));
    assert(o.valid && !o.limited);
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
        assert(close_to(o.position_ref[i], q0[i]) && o.velocity_ref[i] == 0.0);

    /* Compare against independently assembled DLS reference using existing
     * ControlCore solver; no velocity or acceleration cap should activate. */
    double J[ROBOT_DOF][ROBOT_DOF];
    control_jacobian(&robot, q0, J);
    double A[ROBOT_DOF][ROBOT_DOF] = {{0}}, b[ROBOT_DOF] = {0};
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        for (size_t k = 0U; k < ROBOT_DOF; ++k) {
            b[i] += J[k][i]*twist[k];
            for (size_t j = 0U; j < ROBOT_DOF; ++j)
                A[i][j] += J[k][i]*J[k][j];
        }
        A[i][i] += c.damping_lambda*c.damping_lambda;
    }
    double expected[ROBOT_DOF] = {0};
    assert(matrix6_solve((const double (*)[ROBOT_DOF])A,b,expected));
    assert(cartesian_joint_mapper_step(&s, &c, &robot, q0, twist, dt, &o));
    assert(o.valid && !o.limited && o.sigma_min >= c.minimum_sigma);
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        assert(close_to(o.velocity_ref[i], expected[i]));
        assert(close_to(o.position_ref[i], q0[i] + dt*expected[i]));
    }

    /* Independent standard-DH / NumPy reference, NOT computed using the
     * project's C Jacobian or Gaussian solver. Synthetic UR5 test pose,
     * q0, twist, lambda=0.02; no limits active. */
    const double expected_qd_oracle[ROBOT_DOF] = {
        0.010956515426733, -0.005043844988967, -0.003047721650016,
        0.013482091444020, 0.007490334769058, -0.004172802304265
    };
    const double expected_twist_oracle[ROBOT_DOF] = {
        0.005002313739467, -0.005981098645427, 0.003984381108398,
        0.000988592531718, -0.001995988341245, 0.003006309940476
    };
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        assert(fabs(o.velocity_ref[i] - expected_qd_oracle[i]) < 1e-9);
        assert(fabs(o.achieved_twist_base_tcp[i] -
                    expected_twist_oracle[i]) < 1e-9);
        assert(fabs(o.twist_residual_base_tcp[i] -
                    (twist[i] - expected_twist_oracle[i])) < 1e-9);
    }

    /* Enforce velocity limits without silently asserting exact Cartesian
     * tracking. Configure a synthetic low limit for an axis with motion. */
    size_t moving_axis = 0U;
    for (size_t i = 1U; i < ROBOT_DOF; ++i)
        if (fabs(expected[i]) > fabs(expected[moving_axis])) moving_axis = i;
    assert(fabs(expected[moving_axis]) > 1e-5);
    robot.limits.qdMax[moving_axis] = fabs(expected[moving_axis])*0.1;
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    assert(cartesian_joint_mapper_step(&s, &c, &robot, q0, twist, dt, &o));
    assert(o.limited && o.valid);
    assert(fabs(o.velocity_ref[moving_axis]) <= robot.limits.qdMax[moving_axis]);
    assert(moving_axis == 3U); /* Oracle's largest unconstrained joint speed. */
    const double expected_limited_twist[ROBOT_DOF] = {
        0.004135765428541, -0.006249153450034, 0.004938372214176,
        -0.002597214873070, 0.009595952174336, 0.003006309940476
    };
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        assert(fabs(o.achieved_twist_base_tcp[i] -
                    expected_limited_twist[i]) < 1e-9);
        assert(fabs(o.twist_residual_base_tcp[i] -
                    (twist[i] - expected_limited_twist[i])) < 1e-9);
    }
    /* Limiting one joint can reverse desired angular X direction. No
     * undocumented acceptance threshold or automatic re-scaling yet. */
    assert(twist[3] > 0.0 && o.achieved_twist_base_tcp[3] < 0.0);
    robot_config_init_ur5(&robot);

    /* Acceleration bound applies from zero; change of sign is bounded too. */
    c.joint_acceleration_limit[moving_axis] = 0.01;
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    assert(cartesian_joint_mapper_step(&s, &c, &robot, q0, twist, dt, &o));
    assert(o.limited);
    assert(fabs(o.velocity_ref[moving_axis]) <= 0.01*dt + 1e-12);
    double old_v = o.velocity_ref[moving_axis];
    for (size_t i = 0U; i < ROBOT_DOF; ++i) twist[i] = -twist[i];
    assert(cartesian_joint_mapper_step(&s, &c, &robot, q0, twist, dt, &o));
    assert(fabs(o.velocity_ref[moving_axis]-old_v) <= 0.01*dt + 1e-12);
    c = config();

    /* Position boundary reached: reject, not secretly wrap or clip. */
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    for (size_t i = 0U; i < ROBOT_DOF; ++i) twist[i] = -twist[i];
    if (expected[moving_axis] > 0.0)
        robot.limits.qMax[moving_axis] = q0[moving_axis] + 1e-12;
    else
        robot.limits.qMin[moving_axis] = q0[moving_axis] - 1e-12;
    assert(!cartesian_joint_mapper_step(&s, &c, &robot, q0, twist, dt, &o));
    cleared(&s, &o);
    robot_config_init_ur5(&robot);

    /* Singularity threshold rejection: any sigma below configured margin. */
    c.minimum_sigma = 1e6; /* synthetic rejection threshold */
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    assert(!cartesian_joint_mapper_step(&s, &c, &robot, q0, zero, dt, &o));
    cleared(&s, &o);
    c = config();

    /* Feedback tracking-error watchdog, timing fault, NaN and bad robot. */
    c.tracking_error_limit[moving_axis] = 1e-9;
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    assert(!cartesian_joint_mapper_step(&s, &c, &robot, q0, twist, dt, &o));
    cleared(&s, &o);
    c = config();
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    assert(!cartesian_joint_mapper_step(&s, &c, &robot, q0, zero, 0.005, &o));
    cleared(&s, &o);
    assert(cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    zero[5] = NAN;
    assert(!cartesian_joint_mapper_step(&s, &c, &robot, q0, zero, dt, &o));
    cleared(&s, &o);
    zero[5] = 0.0;
    robot.identity.dof = 7;
    assert(!cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    assert(!s.enabled);

    /* Standalone mapper must reject finite non-rigid TCP rotations too.
     * The upstream wrench layer already enforces this condition. */
    robot_config_init_ur5(&robot);
    robot.tool.T_F_TCP[0][0] = 1.1; /* scale, not rotation */
    assert(!cartesian_joint_mapper_config_valid(&c, &robot));
    assert(!cartesian_joint_mapper_enable(&s, &c, &robot, q0));
    robot.tool.T_F_TCP[0][0] = -1.0; /* reflection, det=-1 */
    assert(!cartesian_joint_mapper_config_valid(&c, &robot));
    robot.tool.T_F_TCP[0][0] = 1.0;
    assert(cartesian_joint_mapper_config_valid(&c, &robot));

    puts("PASS: Stage 4D standalone Jacobian/DLS, lifecycle and joint constraints");
    return 0;
}
