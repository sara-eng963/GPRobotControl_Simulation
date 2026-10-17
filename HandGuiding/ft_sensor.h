
#ifndef HANDGUIDING_FT_SENSOR_H
#define HANDGUIDING_FT_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Six-axis force/torque measurement.
 *
 * Reference frame:
 *   Sensor measurement frame (S).
 *
 * Units:
 *   Force  : Newtons (N)
 *   Torque : Newton-meters (N.m)
 */
typedef struct {
    double force_N[3];       /* Fx, Fy, Fz */
    double torque_Nm[3];     /* Tx, Ty, Tz */

    /* Monotonic MCU timestamp when complete
     * measurement was received, in microseconds.
     */
    uint32_t received_time_us;

    /* True only if acquisition-level checks passed. */
    bool valid;

} FtSensorSample;

/*
 * Checks numerical validity and measurement freshness.
 *
 * now_us and received_time_us must use the same
 * monotonic microsecond clock.
 *
 * max_age_us is configured by the caller.
 */
bool ft_sensor_sample_usable(
    const FtSensorSample *sample,
    uint32_t now_us,
    uint32_t max_age_us
);

#endif /* HANDGUIDING_FT_SENSOR_H */
