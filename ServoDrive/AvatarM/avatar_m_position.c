#include "avatar_m_position.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool avatar_m_position_scale_default(
    AvatarMPositionScale *scale,
    double reduction_ratio
)
{
    if (
        scale == NULL ||
        !isfinite(reduction_ratio) ||
        reduction_ratio <= 0.0
    )
    {
        return false;
    }

    scale->units_per_motor_rev =
        AVATAR_M_MANUAL_UNITS_PER_MOTOR_REV;

    scale->reduction_ratio =
        reduction_ratio;

    scale->direction =
        1;

    scale->zero_offset_units =
        0;

    return true;
}

bool avatar_m_position_scale_valid(
    const AvatarMPositionScale *scale
)
{
    return
        scale != NULL &&
        isfinite(scale->units_per_motor_rev) &&
        scale->units_per_motor_rev > 0.0 &&
        isfinite(scale->reduction_ratio) &&
        scale->reduction_ratio > 0.0 &&
        (
            scale->direction == 1 ||
            scale->direction == -1
        );
}

bool avatar_m_joint_rad_to_position_units(
    const AvatarMPositionScale *scale,
    double joint_rad,
    int32_t *position_units
)
{
    if (
        !avatar_m_position_scale_valid(scale) ||
        !isfinite(joint_rad) ||
        position_units == NULL
    )
    {
        return false;
    }

    const double units_per_joint_rev =
        scale->units_per_motor_rev *
        scale->reduction_ratio;

    const double delta_units =
        (
            joint_rad /
            (2.0 * M_PI)
        ) *
        units_per_joint_rev *
        (double)scale->direction;

    const double absolute_units =
        (double)scale->zero_offset_units +
        delta_units;

    if (
        !isfinite(absolute_units) ||
        absolute_units < (double)INT32_MIN ||
        absolute_units > (double)INT32_MAX
    )
    {
        return false;
    }

    *position_units =
        (int32_t)llround(
            absolute_units
        );

    return true;
}

bool avatar_m_position_units_to_joint_rad(
    const AvatarMPositionScale *scale,
    int32_t position_units,
    double *joint_rad
)
{
    if (
        !avatar_m_position_scale_valid(scale) ||
        joint_rad == NULL
    )
    {
        return false;
    }

    const double units_per_joint_rev =
        scale->units_per_motor_rev *
        scale->reduction_ratio;

    const double delta_units =
        (double)position_units -
        (double)scale->zero_offset_units;

    *joint_rad =
        (
            delta_units /
            units_per_joint_rev
        ) *
        (2.0 * M_PI) *
        (double)scale->direction;

    return
        isfinite(*joint_rad);
}
