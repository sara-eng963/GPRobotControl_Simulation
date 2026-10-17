#include "../wrench_frame.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void check(const double actual[6], const double expected[6])
{
    for (size_t i = 0U; i < 6U; ++i)
        assert(isfinite(actual[i]) && fabs(actual[i] - expected[i]) < 1e-10);
}
static void zero(const double actual[6])
{
    const double expected[6] = {0};
    check(actual, expected);
}
static void synthetic_robot(RobotConfig *robot)
{
    robot_config_init_ur5(robot);
    /* All joints coincident: R_B_F = Rz(q1+q2+...+q6).
     * Synthetic fixture only; not a physical robot geometry. */
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        robot->kinematics.a[i] = 0.0;
        robot->kinematics.d[i] = 0.0;
        robot->kinematics.alpha[i] = 0.0;
        robot->kinematics.thetaOffset[i] = 0.0;
    }
    /* RotX(90 deg), flange->TCP offset 10 cm along flange X. */
    const double tool[4][4] = {
        {1.0, 0.0, 0.0, 0.1},
        {0.0, 0.0, -1.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    for (size_t i = 0U; i < 4U; ++i)
        for (size_t j = 0U; j < 4U; ++j)
            robot->tool.T_F_TCP[i][j] = tool[i][j];
}
int main(void)
{
    const double root_half = sqrt(0.5);
    SensorConfig sensor = {
        .geometry_verified = true,
        .flange_T_sensor = {
            .translation_m = {0.2, 0.0, 0.0},
            .rotation_wxyz = {root_half, 0.0, 0.0, root_half}
        },
        .sensor_T_grip = {
            .translation_m = {0.0, 0.3, 0.0},
            .rotation_wxyz = {1.0, 0.0, 0.0, 0.0}
        }
    };
    RobotConfig robot;
    synthetic_robot(&robot);
    double q[ROBOT_DOF] = {0};
    q[0] = acos(-1.0)/2.0;
    const FtSensorBias bias = {
        .force_N = {2.0, 0.0, 0.0},
        .torque_Nm = {0.0, 0.0, 0.1}, .valid = true
    };
    FtSensorSample sample = {
        .force_N = {3.0, 0.0, 0.0},
        .torque_Nm = {0.0, 0.0, 0.4},
        .received_time_us = 1000U, .valid = true
    };
    double out[6] = {0};
    /* f_S=(1,0,0), tau_S=(0,0,0.3) -> f_F=(0,1,0),
     * tau_F=(0,0,0.5). Shift F->TCP: tau=(0,0,0.4).
     * R_B_F=Rz(90), despite rotated tool Rx(90). */
    assert(wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out));
    const double expected1[6] = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.4};
    check(out, expected1);
    /* Different joint angle changes base axes but not wrench moment arm. */
    q[0] = 0.0;
    assert(wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out));
    const double expected2[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.4};
    check(out, expected2);
    /* Same reference point at sensor origin: no extra TCP point shift. */
    robot.tool.T_F_TCP[0][3] = 0.2;
    assert(wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out));
    const double expected3[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 0.3};
    check(out, expected3);
    robot.tool.T_F_TCP[0][3] = 0.1;

    /* Failure never produces a valid stale output. */
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, false, out)); zero(out);
    sensor.geometry_verified = false;
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out)); zero(out);
    sensor.geometry_verified = true;
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 2500U, 1000U,
        &sensor, &bias, &robot, q, true, out)); zero(out);
    q[1] = NAN;
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out)); zero(out);
    q[1] = 0.0;
    q[0] = robot.limits.qMax[0] + 0.1;
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out)); zero(out);
    q[0] = 0.0;
    robot.tool.T_F_TCP[0][0] = -1.0; /* reflected frame, det=-1 */
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out)); zero(out);
    robot.tool.T_F_TCP[0][0] = 1.0;
    robot.identity.dof = 7;
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out)); zero(out);
    robot.identity.dof = ROBOT_DOF;
    sample.force_N[0] = INFINITY;
    assert(!wrench_frame_sensor_to_base_tcp(&sample, 1100U, 1000U,
        &sensor, &bias, &robot, q, true, out)); zero(out);
    puts("PASS: Batch 4B2 sensor-to-base/TCP rigid-frame and failure checks");
    return 0;
}
