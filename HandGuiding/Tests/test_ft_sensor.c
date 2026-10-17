
#include "../ft_sensor.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

int main(void)
{
    /* Synthetic measurement for testing only. */
    FtSensorSample sample = {
        .force_N = {5.0, -2.0, 1.0},
        .torque_Nm = {0.1, 0.0, -0.2},
        .received_time_us = 100000U,
        .valid = true
    };

    /* Test threshold, not a real sensor specification. */
    const uint32_t max_age_us = 5000U;

    /* Valid and fresh. */
    assert(ft_sensor_sample_usable(
        &sample, 102000U, max_age_us));

    /* Exactly at freshness boundary. */
    assert(ft_sensor_sample_usable(
        &sample, 105000U, max_age_us));

    /* Stale measurement. */
    assert(!ft_sensor_sample_usable(
        &sample, 105001U, max_age_us));

    /* Invalid acquisition status. */
    sample.valid = false;
    assert(!ft_sensor_sample_usable(
        &sample, 102000U, max_age_us));
    sample.valid = true;

    /* Invalid force value. */
    sample.force_N[0] = NAN;
    assert(!ft_sensor_sample_usable(
        &sample, 102000U, max_age_us));
    sample.force_N[0] = 5.0;

    /* Invalid torque value. */
    sample.torque_Nm[2] = INFINITY;
    assert(!ft_sensor_sample_usable(
        &sample, 102000U, max_age_us));
    sample.torque_Nm[2] = -0.2;

    /* Missing measurement or invalid threshold. */
    assert(!ft_sensor_sample_usable(
        NULL, 102000U, max_age_us));
    assert(!ft_sensor_sample_usable(
        &sample, 102000U, 0U));
    assert(!ft_sensor_sample_usable(
        &sample, 102000U, UINT32_MAX));

    /* Timestamp apparently in the future. */
    assert(!ft_sensor_sample_usable(
        &sample, 99999U, max_age_us));

    /* 32-bit timestamp rollover: elapsed = 201 us. */
    sample.received_time_us = UINT32_MAX - 100U;

    assert(ft_sensor_sample_usable(
        &sample, 100U, 201U));

    assert(!ft_sensor_sample_usable(
        &sample, 100U, 200U));

    puts("PASS: Batch 4A.2 F/T sample validation");
    return 0;
}
