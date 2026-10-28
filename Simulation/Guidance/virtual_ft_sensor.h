#ifndef VIRTUAL_FT_SENSOR_H
#define VIRTUAL_FT_SENSOR_H

#include "../../HandGuiding/ft_sensor.h"
#include "../../ControlCore/Config/robot_config.h"

/* Inverse of the existing sensor->flange->base/TCP wrench transform.
 * Converts user-requested [F_B,T_B] ABOUT TCP to raw (unbiased) sensor
 * measurement at S, preserving the flange/TCP and flange/sensor moment
 * shifts. Does NOT add grip-to-sensor torque twice. Simulation ONLY. */
bool virtual_ft_sensor_sample(
    const double wrench_base_tcp[6],
    const SensorConfig *sensor,
    const RobotConfig *robot,
    const double measured_q[ROBOT_DOF],
    uint32_t sample_time_us,
    FtSensorSample *sample);
#endif
