#include "virtual_ft_sensor.h"

#include "../../ControlCore/Kinematics/control_fk.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static void cross(const double a[3], const double b[3], double c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

bool virtual_ft_sensor_sample(
    const double wrench_base_tcp[6], const SensorConfig *sensor,
    const RobotConfig *robot, const double measured_q[ROBOT_DOF],
    uint32_t sample_time_us, FtSensorSample *sample)
{
    if (sample == NULL) return false;
    memset(sample, 0, sizeof(*sample));
    if (wrench_base_tcp == NULL || robot == NULL || measured_q == NULL ||
        !sensor_config_validate(sensor) || robot->identity.dof != ROBOT_DOF)
        return false;
    for (size_t i = 0U; i < 6U; ++i)
        if (!isfinite(wrench_base_tcp[i]) || !isfinite(measured_q[i]) ||
            measured_q[i] < robot->limits.qMin[i] ||
            measured_q[i] > robot->limits.qMax[i]) return false;

    double T_B_TCP[4][4];
    control_fk(robot, measured_q, T_B_TCP);
    double R_B_F[3][3] = {{0}}, R_F_S[3][3];
    const double *q = sensor->flange_T_sensor.rotation_wxyz;
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    R_F_S[0][0]=1-2*(y*y+z*z); R_F_S[0][1]=2*(x*y-z*w);
    R_F_S[0][2]=2*(x*z+y*w);
    R_F_S[1][0]=2*(x*y+z*w); R_F_S[1][1]=1-2*(x*x+z*z);
    R_F_S[1][2]=2*(y*z-x*w);
    R_F_S[2][0]=2*(x*z-y*w); R_F_S[2][1]=2*(y*z+x*w);
    R_F_S[2][2]=1-2*(x*x+y*y);
    /* R_B_F = R_B_TCP * R_F_TCP^T. TCP and measured q match forward path. */
    for (size_t i=0; i<3; ++i) {
        for (size_t j=0; j<3; ++j) {
            if (!isfinite(T_B_TCP[i][j])) return false;
            for (size_t k=0; k<3; ++k)
                R_B_F[i][j] += T_B_TCP[i][k] * robot->tool.T_F_TCP[j][k];
        }
    }

    double f_F[3]={0}, t_TCP_F[3]={0}, t_F[3], arm[3];
    for (size_t i=0; i<3; ++i) {
        for (size_t j=0; j<3; ++j) {
            f_F[i] += R_B_F[j][i]*wrench_base_tcp[j];
            t_TCP_F[i] += R_B_F[j][i]*wrench_base_tcp[j+3];
        }
    }
    const double p_TCP[3] = {
        robot->tool.T_F_TCP[0][3],
        robot->tool.T_F_TCP[1][3],
        robot->tool.T_F_TCP[2][3]};
    cross(p_TCP,f_F,arm);
    for(size_t i=0; i<3; ++i) t_F[i]=t_TCP_F[i]+arm[i];
    const double *p_S=sensor->flange_T_sensor.translation_m;
    cross(p_S,f_F,arm);
    double sensor_f[3]={0}, sensor_t[3]={0};
    for (size_t i=0; i<3; ++i) {
        for (size_t j=0; j<3; ++j) {
            sensor_f[i] += R_F_S[j][i]*f_F[j];
            sensor_t[i] += R_F_S[j][i]*(t_F[j]-arm[j]);
        }
        if (!isfinite(sensor_f[i]) || !isfinite(sensor_t[i]))
            return false;
    }
    for(size_t i=0; i<3; ++i) {
        sample->force_N[i]=sensor_f[i];
        sample->torque_Nm[i]=sensor_t[i];
    }
    sample->received_time_us=sample_time_us;
    sample->valid=true;
    return true;
}
