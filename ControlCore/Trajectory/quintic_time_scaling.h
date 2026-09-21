#ifndef QUINTIC_TIME_SCALING_H
#define QUINTIC_TIME_SCALING_H


#include "../Math/control_types.h"

#include <stdbool.h>
#include <stddef.h>


/* ============================================================================
 * QUINTIC PROFILE INFORMATION
 * ============================================================================
 *
 * Streaming equivalent of MATLAB:
 *
 *      quinticTimeScaling.m
 *
 * The profile stores only the information required to evaluate the
 * quintic trajectory analytically at any requested time.
 *
 * No trajectory-sized arrays are allocated.
 * ============================================================================
 */

typedef struct
{
    const char *type;

    real_t T;
    real_t dt;

    real_t peakPathVelocity;
    real_t peakPathAcceleration;
    real_t peakPathJerk;

} QuinticInfo;


/* ============================================================================
 * ONE QUINTIC SAMPLE
 * ============================================================================
 */

typedef struct
{
    real_t t;

    real_t s;
    real_t sDot;
    real_t sDDot;
    real_t sDDDot;

} QuinticSample;


/* ============================================================================
 * STREAMING QUINTIC PROFILE
 * ============================================================================
 */

typedef struct
{
    QuinticInfo info;

    size_t sampleCount;

} QuinticProfile;


/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

/*
 * Initialize the analytical quintic profile.
 *
 * This is the streaming equivalent of:
 *
 *      [t, s, sDot, sDDot, sDDDot] =
 *          quinticTimeScaling(T, dt);
 *
 * but no full arrays are created.
 */
bool quintic_time_scaling_init(
    real_t T,
    real_t dt,
    QuinticProfile *profile
);


/*
 * Return the number of discrete samples corresponding to:
 *
 *      0:dt:T
 *
 * with T explicitly appended when dt does not divide T exactly.
 */
size_t quintic_time_scaling_sample_count(
    const QuinticProfile *profile
);


/*
 * Return the exact timestamp associated with one discrete sample index.
 */
bool quintic_time_scaling_sample_time(
    const QuinticProfile *profile,
    size_t sampleIndex,
    real_t *time
);


/*
 * Evaluate the quintic profile analytically at any time.
 */
bool quintic_time_scaling_evaluate(
    const QuinticProfile *profile,
    real_t time,
    QuinticSample *sample
);


/*
 * Convenience function for discrete streaming.
 */
bool quintic_time_scaling_evaluate_index(
    const QuinticProfile *profile,
    size_t sampleIndex,
    QuinticSample *sample
);


#endif /* QUINTIC_TIME_SCALING_H */