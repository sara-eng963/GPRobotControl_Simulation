
#ifndef HANDGUIDING_SENSOR_CONFIG_H
#define HANDGUIDING_SENSOR_CONFIG_H

#include <stdbool.h>

/* Pose of a child frame expressed relative to its parent frame. */
typedef struct {
    double translation_m[3];
    double rotation_wxyz[4]; /* Unit quaternion [w, x, y, z]. */
} SensorTransform;

typedef struct {
    bool geometry_verified;

    /* Sensor frame S relative to robot end-effector flange F. */
    SensorTransform flange_T_sensor;

    /* Operator grip frame G relative to sensor frame S. */
    SensorTransform sensor_T_grip;
} SensorConfig;

/* Physical design is not finalized: intentionally unconfigured. */
extern const SensorConfig g_sensor_config;

bool sensor_config_validate(const SensorConfig *config);

#endif /* HANDGUIDING_SENSOR_CONFIG_H */
