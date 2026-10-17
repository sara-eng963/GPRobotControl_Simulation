#include "ft_sensor.h"

#include <math.h>
#include <stddef.h>

bool ft_sensor_sample_usable(
    const FtSensorSample *sample,
    uint32_t now_us,
    uint32_t max_age_us)
{
    if (sample == NULL || !sample->valid ||
        max_age_us == 0U || max_age_us >= UINT32_C(0x80000000)) {
        return false;
    }
    for (size_t i = 0U; i < 3U; ++i) {
        if (!isfinite(sample->force_N[i]) ||
            !isfinite(sample->torque_Nm[i])) {
            return false;
        }
    }
    /* Both timestamps must share one monotonic clock and a short horizon. */
    return (uint32_t)(now_us - sample->received_time_us) <= max_age_us;
}

bool ft_sensor_capture_bias(
    const FtSensorSample *sample, uint32_t now_us, uint32_t max_age_us,
    FtSensorBias *bias)
{
    if (bias == NULL) return false;
    bias->valid = false;
    for (size_t i = 0U; i < 3U; ++i) {
        bias->force_N[i] = 0.0;
        bias->torque_Nm[i] = 0.0;
    }
    if (!ft_sensor_sample_usable(sample, now_us, max_age_us)) return false;
    for (size_t i = 0U; i < 3U; ++i) {
        bias->force_N[i] = sample->force_N[i];
        bias->torque_Nm[i] = sample->torque_Nm[i];
    }
    bias->valid = true;
    return true;
}

bool ft_sensor_wrench_in_flange(
    const FtSensorSample *sample, uint32_t now_us, uint32_t max_age_us,
    const SensorConfig *config, const FtSensorBias *bias,
    double wrench_flange[6])
{
    if (wrench_flange == NULL) return false;
    for (size_t i = 0U; i < 6U; ++i) wrench_flange[i] = 0.0;

    if (!ft_sensor_sample_usable(sample, now_us, max_age_us) ||
        !sensor_config_validate(config) || bias == NULL || !bias->valid) {
        return false;
    }

    double f_S[3], tau_S[3];
    for (size_t i = 0U; i < 3U; ++i) {
        if (!isfinite(bias->force_N[i]) ||
            !isfinite(bias->torque_Nm[i])) return false;
        f_S[i] = sample->force_N[i] - bias->force_N[i];
        tau_S[i] = sample->torque_Nm[i] - bias->torque_Nm[i];
        if (!isfinite(f_S[i]) || !isfinite(tau_S[i])) return false;
    }

    const double *const q = config->flange_T_sensor.rotation_wxyz;
    const double norm = sqrt(q[0]*q[0] + q[1]*q[1] +
                             q[2]*q[2] + q[3]*q[3]);
    if (!isfinite(norm) || norm <= 0.0) return false;
    const double w = q[0]/norm, x = q[1]/norm;
    const double y = q[2]/norm, z = q[3]/norm;
    const double R[3][3] = {
        {1.0-2.0*(y*y+z*z), 2.0*(x*y-z*w), 2.0*(x*z+y*w)},
        {2.0*(x*y+z*w), 1.0-2.0*(x*x+z*z), 2.0*(y*z-x*w)},
        {2.0*(x*z-y*w), 2.0*(y*z+x*w), 1.0-2.0*(x*x+y*y)}
    };
    double f_F[3] = {0.0, 0.0, 0.0};
    double tau_F[3] = {0.0, 0.0, 0.0};
    for (size_t i = 0U; i < 3U; ++i) {
        for (size_t j = 0U; j < 3U; ++j) {
            f_F[i] += R[i][j]*f_S[j];
            tau_F[i] += R[i][j]*tau_S[j];
        }
    }

    /* tau_F at flange origin = R_F_S*tau_S + p_F_S cross (R_F_S*f_S).
     * The physical grip lever arm is already represented in tau_S.
     * Do not add p_S_G cross f_S a second time. */
    const double *const p = config->flange_T_sensor.translation_m;
    tau_F[0] += p[1]*f_F[2] - p[2]*f_F[1];
    tau_F[1] += p[2]*f_F[0] - p[0]*f_F[2];
    tau_F[2] += p[0]*f_F[1] - p[1]*f_F[0];
    for (size_t i = 0U; i < 3U; ++i) {
        if (!isfinite(f_F[i]) || !isfinite(tau_F[i])) return false;
    }
    for (size_t i = 0U; i < 3U; ++i) {
        wrench_flange[i] = f_F[i];
        wrench_flange[i+3U] = tau_F[i];
    }
    return true;
}
