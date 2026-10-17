#ifndef HANDGUIDING_FT_SENSOR_H
#define HANDGUIDING_FT_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "sensor_config.h"

/* Forces in N, moments in N*m, expressed about the sensor reference S. */
typedef struct {
    double force_N[3];
    double torque_Nm[3];
    /* Monotonic MCU timestamp when one complete sample was received. */
    uint32_t received_time_us;
    bool valid;
} FtSensorSample;

/* Static sensor-frame offset captured while the handle is unloaded.
 * This is not orientation-dependent gravity or inertial compensation. */
typedef struct {
    double force_N[3];
    double torque_Nm[3];
    bool valid;
} FtSensorBias;

bool ft_sensor_sample_usable(
    const FtSensorSample *sample, uint32_t now_us, uint32_t max_age_us);

/* Explicit tare request. On failure, invalidate the previous bias. */
bool ft_sensor_capture_bias(
    const FtSensorSample *sample, uint32_t now_us, uint32_t max_age_us,
    FtSensorBias *bias);

/* Return [Fx,Fy,Fz,Tx,Ty,Tz] in flange frame F, about the flange origin.
 * Uses flange_T_sensor, T_F_S; output is zeroed on any failure.
 * An independent caller must subsequently transform this wrench to the
 * appropriate base/TCP control reference point before using the Jacobian.
 * Not permitted for physical guidance while geometry_verified is false. */
bool ft_sensor_wrench_in_flange(
    const FtSensorSample *sample, uint32_t now_us, uint32_t max_age_us,
    const SensorConfig *config, const FtSensorBias *bias,
    double wrench_flange[6]);

#endif /* HANDGUIDING_FT_SENSOR_H */
