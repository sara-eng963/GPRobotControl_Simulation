#include "../ft_sensor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void zero_on_fail(const double wrench[6])
{
    for (size_t i = 0U; i < 6U; ++i) assert(wrench[i] == 0.0);
}

int main(void)
{
    const double root_half = sqrt(0.5);
    /* Purely synthetic fixture: 90-degree Z rotation, arbitrary offset. */
    SensorConfig cfg = {
        .geometry_verified = true,
        .flange_T_sensor = {
            .translation_m = {0.2, 0.1, 0.3},
            .rotation_wxyz = {root_half, 0.0, 0.0, root_half}
        },
        .sensor_T_grip = {
            .translation_m = {0.0, 0.15, 0.0},
            .rotation_wxyz = {1.0, 0.0, 0.0, 0.0}
        }
    };
    FtSensorSample s = {
        .force_N = {0.5, 1.0, -3.0},
        .torque_Nm = {0.2, -0.1, 0.3},
        .received_time_us = 1000U, .valid = true
    };
    FtSensorBias bias = {0};
    double out[6] = {0};
    assert(ft_sensor_capture_bias(&s, 1100U, 1000U, &bias));
    assert(ft_sensor_wrench_in_flange(&s, 1100U, 1000U, &cfg, &bias, out));
    for (size_t i = 0U; i < 6U; ++i) assert(fabs(out[i]) < 1e-12);

    /* Increment at S: +X force 2 N, +X torque 1 Nm.
     * Rz(90) yields f_F=(0,2,0), tau_F=(0,1,0) before point shift.
     * p_F_S x f_F = (-0.6,0,0.4) Nm. */
    s.force_N[0] += 2.0;
    s.torque_Nm[0] += 1.0;
    assert(ft_sensor_wrench_in_flange(&s, 1100U, 1000U, &cfg, &bias, out));
    const double expected[6] = {0.0, 2.0, 0.0, -0.6, 1.0, 0.4};
    for (size_t i = 0U; i < 6U; ++i)
        assert(fabs(out[i] - expected[i]) < 1e-12);

    cfg.geometry_verified = false;
    assert(!ft_sensor_wrench_in_flange(&s, 1100U, 1000U, &cfg, &bias, out));
    zero_on_fail(out);
    cfg.geometry_verified = true;

    assert(!ft_sensor_wrench_in_flange(&s, 3000U, 1000U, &cfg, &bias, out));
    zero_on_fail(out);

    bias.force_N[0] = NAN;
    assert(!ft_sensor_wrench_in_flange(&s, 1100U, 1000U, &cfg, &bias, out));
    zero_on_fail(out);
    bias.force_N[0] = 0.5;
    bias.valid = false;
    assert(!ft_sensor_wrench_in_flange(&s, 1100U, 1000U, &cfg, &bias, out));
    zero_on_fail(out);

    s.valid = false;
    assert(!ft_sensor_capture_bias(&s, 1100U, 1000U, &bias));
    assert(!bias.valid);
    puts("PASS: Batch 4B1 wrench rotation, point shift, tare and fault rejection");
    return 0;
}
