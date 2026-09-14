#ifndef S_CURVE_TIME_SCALING_H
#define S_CURVE_TIME_SCALING_H


#include "../Math/control_types.h"

#include <stdbool.h>
#include <stddef.h>


#define S_CURVE_SEGMENTS 7


/* ============================================================================
 * S-CURVE PROFILE INFORMATION
 * ============================================================================
 *
 * Direct equivalent of MATLAB's:
 *
 *      info.type
 *      info.T
 *      info.dt
 *      info.requestedVMax
 *      info.requestedAMax
 *      info.requestedJMax
 *      info.vPeak
 *      info.aPeak
 *      info.jPeak
 *      info.tJerk
 *      info.tConstantAcceleration
 *      info.tConstantVelocity
 *      info.segmentDurations
 * ============================================================================
 */

typedef struct
{
    const char *type;


    real_t T;

    real_t dt;


    real_t requestedVMax;

    real_t requestedAMax;

    real_t requestedJMax;


    real_t vPeak;

    real_t aPeak;

    real_t jPeak;


    real_t tJerk;

    real_t tConstantAcceleration;

    real_t tConstantVelocity;


    real_t segmentDurations[S_CURVE_SEGMENTS];

} SCurveInfo;


/* ============================================================================
 * SYMMETRIC JERK-LIMITED S-CURVE
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      sCurveTimeScaling.m
 *
 *
 * Inputs:
 *
 *      vMax
 *          Maximum normalized path velocity [1/s].
 *
 *      aMax
 *          Maximum normalized path acceleration [1/s^2].
 *
 *      jMax
 *          Maximum normalized path jerk [1/s^3].
 *
 *      dt
 *          Sampling period [s].
 *
 *
 * Outputs:
 *
 *      t
 *      s
 *      sDot
 *      sDDot
 *      sDDDot
 *
 *      count
 *          Number of generated samples.
 *
 *      info
 *          Profile information.
 *
 * ============================================================================
 */

bool s_curve_time_scaling(
    real_t vMax,
    real_t aMax,
    real_t jMax,
    real_t dt,
    real_t *t,
    real_t *s,
    real_t *sDot,
    real_t *sDDot,
    real_t *sDDDot,
    size_t capacity,
    size_t *count,
    SCurveInfo *info
);


#endif /* S_CURVE_TIME_SCALING_H */