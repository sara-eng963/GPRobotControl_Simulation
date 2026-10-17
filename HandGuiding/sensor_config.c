
#include "sensor_config.h"

#include <math.h>
#include <stddef.h>


/*
 * HAND-GUIDING SENSOR GEOMETRY CONFIGURATION
 *
 * Update these values after the mechanical design is finalized.
 * Translation: meters
 * Rotation: quaternion [w, x, y, z]
 *
 * IMPORTANT:
 * Keep geometry_verified = false until all dimensions
 * and orientations are confirmed.
 */
const SensorConfig g_sensor_config = {
    .geometry_verified = false,

    /* Robot flange frame -> F/T sensor measurement frame */
    .flange_T_sensor = {
        .translation_m = {0.0, 0.0, 0.0},  /* TODO: X, Y, Z */
        .rotation_wxyz = {0.0, 0.0, 0.0, 0.0} /* TODO: w, x, y, z */
    },

    /* F/T sensor frame -> operator grip frame */
    .sensor_T_grip = {
        .translation_m = {0.0, 0.0, 0.0},  /* TODO: X, Y, Z */
        .rotation_wxyz = {0.0, 0.0, 0.0, 0.0} /* TODO: w, x, y, z */
    }
};


static bool transform_valid(const SensorTransform *transform)
{
    if (transform == NULL) {
        return false;
    }

    for (size_t i = 0U; i < 3U; ++i) {
        if (!isfinite(transform->translation_m[i])) {
            return false;
        }
    }

    double norm_squared = 0.0;

    for (size_t i = 0U; i < 4U; ++i) {
        const double component = transform->rotation_wxyz[i];
        if (!isfinite(component)) {
            return false;
        }
        norm_squared += component * component;
    }

    return isfinite(norm_squared) &&
           fabs(norm_squared - 1.0) <= 1e-4;
}

bool sensor_config_validate(const SensorConfig *config)
{
    return config != NULL &&
           config->geometry_verified &&
           transform_valid(&config->flange_T_sensor) &&
           transform_valid(&config->sensor_T_grip);
}
