#ifndef AVATAR_M_POSITION_H
#define AVATAR_M_POSITION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Position conversion for an AVATAR M actuator installed as one robot joint.
 *
 * The CANopen manual states a 15-bit single-turn absolute encoder and
 * 32768 position units per motor revolution. The integrated joint module also
 * contains a reducer, so robot joint radians are NOT equal to motor radians.
 *
 * Keep reduction ratio, joint direction and mechanical zero explicit per axis.
 * This avoids carrying the old A6-EC 131072-units/rev assumption into AVATAR.
 */

#define AVATAR_M_MANUAL_UNITS_PER_MOTOR_REV 32768.0

typedef struct
{
    double units_per_motor_rev;
    double reduction_ratio;

    /* +1 or -1: robot-positive joint direction relative to motor-positive. */
    int direction;

    /*
     * Raw 0x6064/0x607A value corresponding to robot joint angle q = 0.
     * This is installation/calibration specific.
     */
    int32_t zero_offset_units;

} AvatarMPositionScale;

/*
 * Initialize the scale with the manual encoder scale and caller-supplied
 * reducer ratio. Direction defaults to +1 and zero offset to 0.
 */
bool avatar_m_position_scale_default(
    AvatarMPositionScale *scale,
    double reduction_ratio
);

bool avatar_m_position_scale_valid(
    const AvatarMPositionScale *scale
);

bool avatar_m_joint_rad_to_position_units(
    const AvatarMPositionScale *scale,
    double joint_rad,
    int32_t *position_units
);

bool avatar_m_position_units_to_joint_rad(
    const AvatarMPositionScale *scale,
    int32_t position_units,
    double *joint_rad
);

#ifdef __cplusplus
}
#endif

#endif /* AVATAR_M_POSITION_H */
