#include "wrench_frame.h"

#include "../ControlCore/Kinematics/control_fk.h"

#include <math.h>
#include <stddef.h>

/* Validate that a claimed homogeneous transform contains a true rigid
 * rotation (orthonormal, right-handed) rather than merely finite entries. */
static bool rigid_transform_valid(const double T[4][4])
{
    if (T == NULL) return false;
    for (size_t i = 0U; i < 4U; ++i) {
        for (size_t j = 0U; j < 4U; ++j)
            if (!isfinite(T[i][j])) return false;
    }
    for (size_t j = 0U; j < 3U; ++j)
        if (fabs(T[3][j]) > 1e-6) return false;
    if (fabs(T[3][3] - 1.0) > 1e-6) return false;

    for (size_t i = 0U; i < 3U; ++i) {
        for (size_t j = 0U; j < 3U; ++j) {
            double dot = 0.0;
            for (size_t k = 0U; k < 3U; ++k)
                dot += T[i][k] * T[j][k];
            if (!isfinite(dot) || fabs(dot - (i == j ? 1.0 : 0.0)) > 1e-6)
                return false;
        }
    }
    const double det =
        T[0][0]*(T[1][1]*T[2][2] - T[1][2]*T[2][1]) -
        T[0][1]*(T[1][0]*T[2][2] - T[1][2]*T[2][0]) +
        T[0][2]*(T[1][0]*T[2][1] - T[1][1]*T[2][0]);
    return isfinite(det) && fabs(det - 1.0) <= 1e-6;
}

static bool robot_pose_inputs_valid(const RobotConfig *robot,
                                    const double measured[ROBOT_DOF])
{
    if (robot == NULL || measured == NULL ||
        robot->identity.dof != ROBOT_DOF ||
        !rigid_transform_valid(robot->tool.T_F_TCP)) return false;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        if (!isfinite(measured[i]) ||
            !isfinite(robot->limits.qMin[i]) ||
            !isfinite(robot->limits.qMax[i]) ||
            robot->limits.qMin[i] >= robot->limits.qMax[i] ||
            measured[i] < robot->limits.qMin[i] ||
            measured[i] > robot->limits.qMax[i] ||
            !isfinite(robot->kinematics.a[i]) ||
            !isfinite(robot->kinematics.d[i]) ||
            !isfinite(robot->kinematics.alpha[i]) ||
            !isfinite(robot->kinematics.thetaOffset[i])) return false;
    }
    return true;
}

bool wrench_frame_sensor_to_base_tcp(
    const FtSensorSample *sample, uint32_t now_us, uint32_t max_age_us,
    const SensorConfig *sensor_config, const FtSensorBias *bias,
    const RobotConfig *robot, const double measured_joint_rad[ROBOT_DOF],
    bool robot_model_verified, double wrench_base_tcp[6])
{
    if (wrench_base_tcp == NULL) return false;
    for (size_t i = 0U; i < 6U; ++i) wrench_base_tcp[i] = 0.0;
    if (!robot_model_verified ||
        !robot_pose_inputs_valid(robot, measured_joint_rad)) return false;

    /* Stage 4B1 validates sample, tare, T_F_S, and handles S->F point shift. */
    double wrench_F[6] = {0};
    if (!ft_sensor_wrench_in_flange(sample, now_us, max_age_us,
                                    sensor_config, bias, wrench_F)) return false;

    /* Existing ControlCore FK provides T_B_TCP for the measured robot pose. */
    double T_B_TCP[4][4];
    control_fk(robot, measured_joint_rad, T_B_TCP);
    if (!rigid_transform_valid((const double (*)[4])T_B_TCP)) return false;

    /* R_B_F = R_B_TCP * R_F_TCP^T; this matters when tool frame is rotated. */
    double R_B_F[3][3] = {{0}};
    for (size_t i = 0U; i < 3U; ++i)
        for (size_t j = 0U; j < 3U; ++j)
            for (size_t k = 0U; k < 3U; ++k)
                R_B_F[i][j] +=
                    T_B_TCP[i][k] * robot->tool.T_F_TCP[j][k];

    /* Shift wrench reference from flange origin F to TCP origin:
     * tau_TCP^F = tau_F^F - p_F_TCP x f_F. Both currently expressed in F.
     * Sensor grip torque is already measured; never add p_S_G x f again. */
    const double p[3] = {
        robot->tool.T_F_TCP[0][3],
        robot->tool.T_F_TCP[1][3],
        robot->tool.T_F_TCP[2][3]
    };
    const double tau_TCP_F[3] = {
        wrench_F[3] - (p[1]*wrench_F[2] - p[2]*wrench_F[1]),
        wrench_F[4] - (p[2]*wrench_F[0] - p[0]*wrench_F[2]),
        wrench_F[5] - (p[0]*wrench_F[1] - p[1]*wrench_F[0])
    };
    double result[6] = {0};
    for (size_t i = 0U; i < 3U; ++i) {
        for (size_t j = 0U; j < 3U; ++j) {
            result[i] += R_B_F[i][j] * wrench_F[j];
            result[i+3U] += R_B_F[i][j] * tau_TCP_F[j];
        }
    }
    for (size_t i = 0U; i < 6U; ++i)
        if (!isfinite(result[i])) return false;
    for (size_t i = 0U; i < 6U; ++i) wrench_base_tcp[i] = result[i];
    return true;
}
