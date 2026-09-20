#include "quintic_time_scaling.h"

#include <math.h>
#include <string.h>


bool quintic_time_scaling_init(
    real_t T,
    real_t dt,
    QuinticProfile *profile
)
{
    if (
        T <= 0.0 ||
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


    profile->info.type =
        "Quintic";

    profile->info.T =
        T;

    profile->info.dt =
        dt;


    /* ========================================================================
     * REPRODUCE MATLAB SAMPLE COUNT
     * ========================================================================
     *
     * MATLAB:
     *
     *      t = 0:dt:T;
     *
     *      if t(end) < T
     *          t = [t T];
     *      end
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


    /* ========================================================================
     * CALCULATE PROFILE PEAKS
     * ========================================================================
     *
     * MATLAB:
     *
     *      info.peakPathVelocity =
     *          max(abs(sDot));
     *
     *      info.peakPathAcceleration =
     *          max(abs(sDDot));
     *
     *      info.peakPathJerk =
     *          max(abs(sDDDot));
     *
     * We reproduce that behavior without storing the arrays.
     */

    real_t peakVelocity =
        0.0;

    real_t peakAcceleration =
        0.0;

    real_t peakJerk =
        0.0;


    for (
        size_t i = 0;
        i < sampleCount;
        i++
    )
    {
        QuinticSample sample;


        if (
            !quintic_time_scaling_evaluate_index(
                profile,
                i,
                &sample
            )
        )
        {
            return false;
        }


        real_t velocity =
            fabs(sample.sDot);

        real_t acceleration =
            fabs(sample.sDDot);

        real_t jerk =
            fabs(sample.sDDDot);


        if (velocity > peakVelocity)
        {
            peakVelocity =
                velocity;
        }


        if (acceleration > peakAcceleration)
        {
            peakAcceleration =
                acceleration;
        }


        if (jerk > peakJerk)
        {
            peakJerk =
                jerk;
        }
    }


    profile->info.peakPathVelocity =
        peakVelocity;

    profile->info.peakPathAcceleration =
        peakAcceleration;

    profile->info.peakPathJerk =
        peakJerk;


    return true;
}


size_t quintic_time_scaling_sample_count(
    const QuinticProfile *profile
)
{
    if (profile == NULL)
    {
        return 0;
    }


    return
        profile->sampleCount;
}


bool quintic_time_scaling_sample_time(
    const QuinticProfile *profile,
    size_t sampleIndex,
    real_t *time
)
{
    if (
        profile == NULL ||
        time == NULL ||
        profile->sampleCount == 0 ||
        sampleIndex >= profile->sampleCount
    )
    {
        return false;
    }


    /*
     * MATLAB explicitly includes the exact final time T.
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


bool quintic_time_scaling_evaluate(
    const QuinticProfile *profile,
    real_t time,
    QuinticSample *sample
)
{
    if (
        profile == NULL ||
        sample == NULL ||
        profile->sampleCount == 0 ||
        profile->info.T <= 0.0
    )
    {
        return false;
    }


    real_t t =
        time;


    /*
     * Clamp evaluation time to the valid trajectory interval.
     */
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


    const real_t T =
        profile->info.T;


    /* ========================================================================
     * NORMALIZED TIME
     * ========================================================================
     *
     * MATLAB:
     *
     *      tau = t / T;
     */

    real_t tau =
        t / T;


    real_t tau2 =
        tau * tau;

    real_t tau3 =
        tau2 * tau;

    real_t tau4 =
        tau3 * tau;

    real_t tau5 =
        tau4 * tau;


    sample->t =
        t;


    /* ========================================================================
     * QUINTIC PATH POSITION
     * ========================================================================
     *
     * MATLAB:
     *
     *      s =
     *          10*tau.^3
     *        - 15*tau.^4
     *        +  6*tau.^5;
     */

    sample->s =
        10.0 * tau3
        -
        15.0 * tau4
        +
        6.0 * tau5;


    /* ========================================================================
     * PATH VELOCITY
     * ========================================================================
     *
     * MATLAB:
     *
     *      sDot =
     *          (30*tau.^2
     *          -60*tau.^3
     *          +30*tau.^4) / T;
     */

    sample->sDot =
        (
            30.0 * tau2
            -
            60.0 * tau3
            +
            30.0 * tau4
        )
        /
        T;


    /* ========================================================================
     * PATH ACCELERATION
     * ========================================================================
     *
     * MATLAB:
     *
     *      sDDot =
     *          (60*tau
     *          -180*tau.^2
     *          +120*tau.^3) / T^2;
     */

    sample->sDDot =
        (
            60.0 * tau
            -
            180.0 * tau2
            +
            120.0 * tau3
        )
        /
        (T * T);


    /* ========================================================================
     * PATH JERK
     * ========================================================================
     *
     * MATLAB:
     *
     *      sDDDot =
     *          (60
     *          -360*tau
     *          +360*tau.^2) / T^3;
     */

    sample->sDDDot =
        (
            60.0
            -
            360.0 * tau
            +
            360.0 * tau2
        )
        /
        (T * T * T);


    /* ========================================================================
     * FORCE EXACT MATLAB ENDPOINT CONDITIONS
     * ========================================================================
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


    if (t >= T)
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


bool quintic_time_scaling_evaluate_index(
    const QuinticProfile *profile,
    size_t sampleIndex,
    QuinticSample *sample
)
{
    real_t time;


    if (
        !quintic_time_scaling_sample_time(
            profile,
            sampleIndex,
            &time
        )
    )
    {
        return false;
    }


    return
        quintic_time_scaling_evaluate(
            profile,
            time,
            sample
        );
}