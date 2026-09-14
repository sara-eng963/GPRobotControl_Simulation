#include "s_curve_profile.h"

#include <math.h>
#include <string.h>


bool s_curve_profile_init(
    real_t vMax,
    real_t aMax,
    real_t jMax,
    real_t dt,
    SCurveProfile *profile
)
{
    if (
        vMax <= 0.0 ||
        aMax <= 0.0 ||
        jMax <= 0.0 ||
        dt <= 0.0 ||
        profile == NULL
    )
    {
        return false;
    }

    memset(
        profile,
        0,
        sizeof(*profile)
    );

    const real_t D =
        1.0;

    real_t vAtAMax =
        aMax * aMax /
        jMax;

    real_t distanceToReachAMax =
        2.0 *
        aMax * aMax * aMax /
        (jMax * jMax);

    real_t vPeak;
    real_t aPeak;

    real_t tJ;
    real_t tA;
    real_t tV;


    /* ========================================================================
     * SAME PROFILE SELECTION MATH AS s_curve_time_scaling()
     * ========================================================================
     */

    if (vMax < vAtAMax)
    {
        real_t tJForVMax =
            sqrt(
                vMax /
                jMax
            );

        real_t distanceForVMax =
            2.0 *
            vMax *
            tJForVMax;

        if (D >= distanceForVMax)
        {
            vPeak =
                vMax;

            tJ =
                tJForVMax;

            tA =
                0.0;

            aPeak =
                jMax *
                tJ;

            tV =
                (
                    D -
                    distanceForVMax
                ) /
                vPeak;
        }
        else
        {
            vPeak =
                pow(
                    D *
                    sqrt(jMax) /
                    2.0,
                    2.0 / 3.0
                );

            tJ =
                sqrt(
                    vPeak /
                    jMax
                );

            tA =
                0.0;

            tV =
                0.0;

            aPeak =
                jMax *
                tJ;
        }
    }
    else
    {
        tJ =
            aMax /
            jMax;

        real_t distanceForVMax =
            vMax *
            (
                tJ +
                vMax / aMax
            );

        if (D >= distanceForVMax)
        {
            vPeak =
                vMax;

            aPeak =
                aMax;

            tA =
                vPeak / aMax
                -
                tJ;

            tV =
                (
                    D -
                    distanceForVMax
                ) /
                vPeak;
        }
        else if (
            D >=
            distanceToReachAMax
        )
        {
            aPeak =
                aMax;

            tV =
                0.0;

            real_t term =
                aMax * aMax /
                jMax;

            vPeak =
                (
                    -term
                    +
                    sqrt(
                        term * term
                        +
                        4.0 *
                        aMax *
                        D
                    )
                )
                /
                2.0;

            tA =
                vPeak / aMax
                -
                tJ;
        }
        else
        {
            vPeak =
                pow(
                    D *
                    sqrt(jMax) /
                    2.0,
                    2.0 / 3.0
                );

            tJ =
                sqrt(
                    vPeak /
                    jMax
                );

            tA =
                0.0;

            tV =
                0.0;

            aPeak =
                jMax *
                tJ;
        }
    }


    real_t segmentDuration[S_CURVE_SEGMENTS] =
    {
        tJ,
        tA,
        tJ,
        tV,
        tJ,
        tA,
        tJ
    };

    real_t segmentJerk[S_CURVE_SEGMENTS] =
    {
         jMax,
         0.0,
        -jMax,
         0.0,
        -jMax,
         0.0,
         jMax
    };


    real_t T =
        0.0;

    for (int k = 0;
         k < S_CURVE_SEGMENTS;
         k++)
    {
        T +=
            segmentDuration[k];
    }


    /* ========================================================================
     * PRECOMPUTE THE STATE AT EACH SEGMENT START
     * ========================================================================
     */

    real_t currentTime =
        0.0;

    real_t currentS =
        0.0;

    real_t currentV =
        0.0;

    real_t currentA =
        0.0;

    for (int k = 0;
         k < S_CURVE_SEGMENTS;
         k++)
    {
        profile->segmentStartTime[k] =
            currentTime;

        profile->sStart[k] =
            currentS;

        profile->vStart[k] =
            currentV;

        profile->aStart[k] =
            currentA;

        profile->segmentJerk[k] =
            segmentJerk[k];

        real_t h =
            segmentDuration[k];

        real_t j =
            segmentJerk[k];

        real_t nextS =
            currentS
            +
            currentV * h
            +
            0.5 *
            currentA *
            h * h
            +
            (1.0 / 6.0) *
            j *
            h * h * h;

        real_t nextV =
            currentV
            +
            currentA * h
            +
            0.5 *
            j *
            h * h;

        real_t nextA =
            currentA
            +
            j * h;

        currentS =
            nextS;

        currentV =
            nextV;

        currentA =
            nextA;

        currentTime +=
            h;

        profile->segmentEndTime[k] =
            currentTime;
    }


    /* ========================================================================
     * STORE PUBLIC PROFILE INFORMATION
     * ========================================================================
     */

    profile->info.type =
        "S-Curve";

    profile->info.T =
        T;

    profile->info.dt =
        dt;

    profile->info.requestedVMax =
        vMax;

    profile->info.requestedAMax =
        aMax;

    profile->info.requestedJMax =
        jMax;

    profile->info.vPeak =
        vPeak;

    profile->info.aPeak =
        aPeak;

    profile->info.jPeak =
        jMax;

    profile->info.tJerk =
        tJ;

    profile->info.tConstantAcceleration =
        tA;

    profile->info.tConstantVelocity =
        tV;

    for (int k = 0;
         k < S_CURVE_SEGMENTS;
         k++)
    {
        profile->info.segmentDurations[k] =
            segmentDuration[k];
    }


    /* ========================================================================
     * REPRODUCE LEGACY SAMPLE COUNT WITHOUT ALLOCATING SAMPLE ARRAYS
     * ========================================================================
     */

    size_t sampleCount =
        0;

    real_t ti =
        0.0;

    while (ti <= T)
    {
        sampleCount++;

        ti =
            (real_t)sampleCount *
            dt;
    }

    if (sampleCount == 0)
    {
        return false;
    }

    real_t lastRegularTime =
        (real_t)(sampleCount - 1U) *
        dt;

    if (lastRegularTime < T)
    {
        sampleCount++;
    }

    profile->sampleCount =
        sampleCount;

    return true;
}


size_t s_curve_profile_sample_count(
    const SCurveProfile *profile
)
{
    if (profile == NULL)
    {
        return 0;
    }

    return profile->sampleCount;
}


bool s_curve_profile_sample_time(
    const SCurveProfile *profile,
    size_t sampleIndex,
    real_t *time
)
{
    if (
        profile == NULL ||
        time == NULL ||
        sampleIndex >= profile->sampleCount ||
        profile->sampleCount == 0
    )
    {
        return false;
    }

    /*
     * Always force the last discrete sample to the exact analytical end time,
     * matching the legacy "append T when needed" behavior.
     */
    if (
        sampleIndex ==
        profile->sampleCount - 1U
    )
    {
        *time =
            profile->info.T;

        return true;
    }

    *time =
        (real_t)sampleIndex *
        profile->info.dt;

    return true;
}


bool s_curve_profile_evaluate(
    const SCurveProfile *profile,
    real_t time,
    SCurveSample *sample
)
{
    if (
        profile == NULL ||
        sample == NULL ||
        profile->sampleCount == 0
    )
    {
        return false;
    }

    real_t t =
        time;

    if (t < 0.0)
    {
        t =
            0.0;
    }

    if (t > profile->info.T)
    {
        t =
            profile->info.T;
    }


    int segmentIndex =
        S_CURVE_SEGMENTS - 1;

    for (int k = 0;
         k < S_CURVE_SEGMENTS;
         k++)
    {
        if (
            t <=
            profile->segmentEndTime[k] +
            1e-12
        )
        {
            segmentIndex =
                k;

            break;
        }
    }


    real_t tau =
        t -
        profile->segmentStartTime[segmentIndex];

    real_t j =
        profile->segmentJerk[segmentIndex];


    sample->t =
        t;

    sample->sDDot =
        profile->aStart[segmentIndex]
        +
        j * tau;

    sample->sDot =
        profile->vStart[segmentIndex]
        +
        profile->aStart[segmentIndex] *
        tau
        +
        0.5 *
        j *
        tau * tau;

    sample->s =
        profile->sStart[segmentIndex]
        +
        profile->vStart[segmentIndex] *
        tau
        +
        0.5 *
        profile->aStart[segmentIndex] *
        tau * tau
        +
        (1.0 / 6.0) *
        j *
        tau * tau * tau;

    sample->sDDDot =
        j;


    /*
     * Match the legacy endpoint forcing exactly.
     */
    if (t <= 0.0)
    {
        sample->s =
            0.0;

        sample->sDot =
            0.0;

        sample->sDDot =
            0.0;
    }

    if (t >= profile->info.T)
    {
        sample->s =
            1.0;

        sample->sDot =
            0.0;

        sample->sDDot =
            0.0;
    }

    return true;
}


bool s_curve_profile_evaluate_index(
    const SCurveProfile *profile,
    size_t sampleIndex,
    SCurveSample *sample
)
{
    real_t time;

    if (
        !s_curve_profile_sample_time(
            profile,
            sampleIndex,
            &time
        )
    )
    {
        return false;
    }

    return
        s_curve_profile_evaluate(
            profile,
            time,
            sample
        );
}
