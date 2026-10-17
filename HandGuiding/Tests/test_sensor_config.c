
#include "../sensor_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    assert(!sensor_config_validate(NULL));
    assert(!sensor_config_validate(&g_sensor_config));

    /* Synthetic fixture only; these are NOT real handle dimensions. */
    SensorConfig config = {
        .geometry_verified = true,

        .flange_T_sensor = {
            .translation_m = {0.12, -0.03, 0.04},
            .rotation_wxyz = {1.0, 0.0, 0.0, 0.0}
        },

        .sensor_T_grip = {
            .translation_m = {0.0, 0.08, 0.0},
            .rotation_wxyz = {1.0, 0.0, 0.0, 0.0}
        }
    };

    assert(sensor_config_validate(&config));

    /* Reject unverified geometry. */
    config.geometry_verified = false;
    assert(!sensor_config_validate(&config));
    config.geometry_verified = true;

    /* Reject invalid quaternion. */
    config.sensor_T_grip.rotation_wxyz[0] = 0.0;
    assert(!sensor_config_validate(&config));
    config.sensor_T_grip.rotation_wxyz[0] = 1.0;

    /* Reject NaN translation. */
    config.flange_T_sensor.translation_m[0] = NAN;
    assert(!sensor_config_validate(&config));
    config.flange_T_sensor.translation_m[0] = 0.12;

    /* Reject infinite rotation component. */
    config.flange_T_sensor.rotation_wxyz[1] = INFINITY;
    assert(!sensor_config_validate(&config));

    puts("PASS: Batch 4A.1 simplified sensor geometry configuration");
    return 0;
}
