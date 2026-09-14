#ifndef S_CURVE_PROFILE_H
#define S_CURVE_PROFILE_H

#include "s_curve_time_scaling.h"

#include <stdbool.h>
#include <stddef.h>

/*
 * Streaming/evaluable representation of the same symmetric jerk-limited
 * S-curve used by s_curve_time_scaling().
 *
 * Unlike s_curve_time_scaling(), this module does NOT allocate or generate
 * arrays for the full motion duration. It stores only the seven-segment
 * analytic profile and evaluates any requested timestamp on demand.
 */

typedef struct
{
    real_t t;
    real_t s;
    real_t sDot;
    real_t sDDot;
    real_t sDDDot;

} SCurveSample;


typedef struct
{
    SCurveInfo info;

    real_t segmentStartTime[S_CURVE_SEGMENTS];
    real_t segmentEndTime[S_CURVE_SEGMENTS];

    real_t sStart[S_CURVE_SEGMENTS];
    real_t vStart[S_CURVE_SEGMENTS];
    real_t aStart[S_CURVE_SEGMENTS];

    real_t segmentJerk[S_CURVE_SEGMENTS];

    size_t sampleCount;

} SCurveProfile;


/*
 * Build the seven-segment analytic profile.
 *
 * No trajectory-sized arrays are required.
 */
bool s_curve_profile_init(
    real_t vMax,
    real_t aMax,
    real_t jMax,
    real_t dt,
    SCurveProfile *profile
);


/*
 * Number of 1/dt samples that reproduce the legacy sampling rule:
 *
 *     0:dt:T
 *
 * with the exact final time T appended when needed.
 */
size_t s_curve_profile_sample_count(
    const SCurveProfile *profile
);


/*
 * Return the exact timestamp associated with a sample index.
 *
 * The final index always maps exactly to profile->info.T.
 */
bool s_curve_profile_sample_time(
    const SCurveProfile *profile,
    size_t sampleIndex,
    real_t *time
);


/*
 * Evaluate s, sDot, sDDot and sDDDot analytically at one timestamp.
 */
bool s_curve_profile_evaluate(
    const SCurveProfile *profile,
    real_t time,
    SCurveSample *sample
);


/*
 * Convenience helper:
 * evaluate directly by discrete sample index.
 */
bool s_curve_profile_evaluate_index(
    const SCurveProfile *profile,
    size_t sampleIndex,
    SCurveSample *sample
);

#endif /* S_CURVE_PROFILE_H */
