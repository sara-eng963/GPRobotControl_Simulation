#ifndef HANDGUIDING_WRENCH_FRAME_H
#define HANDGUIDING_WRENCH_FRAME_H

#include "ft_sensor.h"
#include "../ControlCore/Config/robot_config.h"

/* A checked sensor-frame wrench, expressed in base B about the robot TCP.
 * Output order: [Fx,Fy,Fz,Tx,Ty,Tz] in [N,N,N,N*m,N*m,N*m].
 * Uses T_F_S from SensorConfig and T_F_TCP from RobotConfig, and FK at the
 * measured joint position. No operator-grip lever arm is added twice.
 * 'robot_model_verified' is a commissioning gate owned by the caller; it is
 * not evidence that this software has verified or certified the geometry.
 * Clears output on failure. Failure must inhibit downstream guidance.
 * No sensor acquisition, motor I/O, gravity compensation, or filtering.
 */
bool wrench_frame_sensor_to_base_tcp(
    const FtSensorSample *sample,
    uint32_t now_us,
    uint32_t max_age_us,
    const SensorConfig *sensor_config,
    const FtSensorBias *bias,
    const RobotConfig *robot,
    const double measured_joint_rad[ROBOT_DOF],
    bool robot_model_verified,
    double wrench_base_tcp[6]);

#endif
