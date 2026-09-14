#include "s_curve_time_scaling.h"

#include <math.h>
#include <stddef.h>


/* ============================================================================
 * SYMMETRIC JERK-LIMITED S-CURVE TIME SCALING
 * ============================================================================
 *
 * Direct C translation of:
 *
 *      sCurveTimeScaling.m
 *
 * Normalized path distance:
 *
 *      D = 1
 *
 * Up to seven stages:
 *
 *      1. +jMax
 *      2.  0 jerk, +a
 *      3. -jMax
 *      4.  0 jerk, constant velocity
 *      5. -jMax
 *      6.  0 jerk, -a
 *      7. +jMax
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
)
{
    /* ========================================================================
     * VALIDATE INPUTS
     * ========================================================================
     *
     * MATLAB:
     *
     *      if ~isscalar(vMax) || vMax <= 0
     *      ...
     */

    if (
        vMax <= 0.0 ||
        aMax <= 0.0 ||
        jMax <= 0.0 ||
        dt <= 0.0 ||
        t == NULL ||
        s == NULL ||
        sDot == NULL ||
        sDDot == NULL ||
        sDDDot == NULL ||
        count == NULL ||
        info == NULL ||
        capacity == 0
    )
    {
        return false;
    }


    /* ========================================================================
     * NORMALIZED PATH DISTANCE
     * ========================================================================
     *
     * MATLAB:
     *
     *      D = 1;
     */

    const real_t D =
        1.0;


    /* ========================================================================
     * DETERMINE WHICH LIMITS CAN ACTUALLY BE REACHED
     * ========================================================================
     */

    /*
     * MATLAB:
     *
     *      vAtAMax =
     *          aMax^2 / jMax;
     */

    real_t vAtAMax =
        aMax * aMax /
        jMax;


    /*
     * MATLAB:
     *
     *      distanceToReachAMax =
     *          2*aMax^3/jMax^2;
     */

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
     * CASE A
     *
     * Requested maximum velocity is reached BEFORE aMax would be reached.
     * ========================================================================
     */

    if (vMax < vAtAMax)
    {
        /*
         * MATLAB:
         *
         *      tJForVMax =
         *          sqrt(vMax/jMax);
         */

        real_t tJForVMax =
            sqrt(
                vMax /
                jMax
            );


        /*
         * MATLAB:
         *
         *      distanceForVMax =
         *          2*vMax*tJForVMax;
         */

        real_t distanceForVMax =
            2.0 *
            vMax *
            tJForVMax;


        if (D >= distanceForVMax)
        {
            /* --------------------------------------------------------------
             * Reach vMax, but NOT aMax.
             * --------------------------------------------------------------
             */

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
                )
                /
                vPeak;
        }
        else
        {
            /* --------------------------------------------------------------
             * Too short to reach vMax.
             *
             * Pure four-jerk-segment profile.
             * --------------------------------------------------------------
             *
             * MATLAB:
             *
             * vPeak =
             *      (D*sqrt(jMax)/2)^(2/3);
             */

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


    /* ========================================================================
     * CASE B
     *
     * aMax can potentially be reached before vMax.
     * ========================================================================
     */

    else
    {
        /*
         * MATLAB:
         *
         *      tJ =
         *          aMax/jMax;
         */

        tJ =
            aMax /
            jMax;


        /*
         * MATLAB:
         *
         * distanceForVMax =
         *      vMax *
         *      (tJ + vMax/aMax);
         */

        real_t distanceForVMax =
            vMax *
            (
                tJ +
                vMax / aMax
            );


        if (D >= distanceForVMax)
        {
            /* --------------------------------------------------------------
             * Full seven-stage profile.
             * --------------------------------------------------------------
             */

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
                )
                /
                vPeak;
        }
        else if (
            D >=
            distanceToReachAMax
        )
        {
            /* --------------------------------------------------------------
             * aMax reached, vMax not reached.
             * --------------------------------------------------------------
             */

            aPeak =
                aMax;


            tV =
                0.0;


            /*
             * MATLAB:
             *
             * vPeak =
             * (
             *     -aMax^2/jMax
             *     +
             *     sqrt(
             *         (aMax^2/jMax)^2
             *         +
             *         4*aMax*D
             *     )
             * ) / 2;
             */

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
            /* --------------------------------------------------------------
             * Too short to reach either aMax or vMax.
             * --------------------------------------------------------------
             */

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


    /* ========================================================================
     * DEFINE SEVEN SEGMENT DURATIONS
     * ========================================================================
     *
     * MATLAB:
     *
     * segmentDuration = [
     *      tJ
     *      tA
     *      tJ
     *      tV
     *      tJ
     *      tA
     *      tJ
     * ];
     */

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


    /*
     * MATLAB:
     *
     * segmentJerk = [
     *       jMax
     *       0
     *      -jMax
     *       0
     *      -jMax
     *       0
     *       jMax
     * ];
     */

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


    /* ========================================================================
     * TOTAL MOTION DURATION
     * ========================================================================
     *
     * MATLAB:
     *
     *      T =
     *          sum(segmentDuration);
     */

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
     * GENERATE TIME SAMPLES
     * ========================================================================
     *
     * MATLAB:
     *
     *      t = 0:dt:T;
     *
     *      if t(end) < T
     *
     *          t = [t T];
     *
     *      end
     * ========================================================================
     */

    size_t sampleCount =
        0;


    real_t ti =
        0.0;


    while (ti <= T)
    {
        if (sampleCount >= capacity)
        {
            return false;
        }


        t[sampleCount] =
            ti;


        sampleCount++;


        ti =
            (real_t)sampleCount *
            dt;
    }


    if (sampleCount == 0)
    {
        return false;
    }


    /*
     * Ensure exact final time is included.
     */
    if (
        t[sampleCount - 1] <
        T
    )
    {
        if (sampleCount >= capacity)
        {
            return false;
        }


        t[sampleCount] =
            T;


        sampleCount++;
    }


    /* ========================================================================
     * PRECOMPUTE STATE AT START OF EVERY SEGMENT
     * ========================================================================
     */

    real_t segmentStartTime[S_CURVE_SEGMENTS];

    real_t sStart[S_CURVE_SEGMENTS];

    real_t vStart[S_CURVE_SEGMENTS];

    real_t aStart[S_CURVE_SEGMENTS];


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
        /*
         * MATLAB:
         *
         *      segmentStartTime(k) =
         *          currentTime;
         *
         *      sStart(k) =
         *          currentS;
         *
         *      vStart(k) =
         *          currentV;
         *
         *      aStart(k) =
         *          currentA;
         */

        segmentStartTime[k] =
            currentTime;


        sStart[k] =
            currentS;


        vStart[k] =
            currentV;


        aStart[k] =
            currentA;


        real_t h =
            segmentDuration[k];


        real_t j =
            segmentJerk[k];


        /*
         * MATLAB:
         *
         * nextS =
         *      currentS
         *      + currentV*h
         *      + 0.5*currentA*h^2
         *      + (1/6)*j*h^3;
         */

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


        /*
         * MATLAB:
         *
         * nextV =
         *      currentV
         *      + currentA*h
         *      + 0.5*j*h^2;
         */

        real_t nextV =
            currentV
            +
            currentA * h
            +
            0.5 *
            j *
            h * h;


        /*
         * MATLAB:
         *
         * nextA =
         *      currentA
         *      + j*h;
         */

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
    }


    /* ========================================================================
     * SEGMENT END TIMES
     * ========================================================================
     *
     * MATLAB:
     *
     *      segmentEndTime =
     *          segmentStartTime +
     *          segmentDuration;
     */

    real_t segmentEndTime[S_CURVE_SEGMENTS];


    for (int k = 0;
         k < S_CURVE_SEGMENTS;
         k++)
    {
        segmentEndTime[k] =
            segmentStartTime[k]
            +
            segmentDuration[k];
    }


    /* ========================================================================
     * EVALUATE TRAJECTORY AT EVERY TIME SAMPLE
     * ========================================================================
     */

    for (size_t i = 0;
         i < sampleCount;
         i++)
    {
        real_t currentSampleTime =
            t[i];


        /* --------------------------------------------------------------------
         * MATLAB:
         *
         *      k = find(
         *          ti <= segmentEndTime + 1e-12,
         *          1,
         *          'first'
         *      );
         * --------------------------------------------------------------------
         */

        int segmentIndex =
            S_CURVE_SEGMENTS - 1;


        for (int k = 0;
             k < S_CURVE_SEGMENTS;
             k++)
        {
            if (
                currentSampleTime <=
                segmentEndTime[k] +
                1e-12
            )
            {
                segmentIndex =
                    k;

                break;
            }
        }


        /* --------------------------------------------------------------------
         * Time relative to segment beginning
         * --------------------------------------------------------------------
         */

        real_t tau =
            currentSampleTime
            -
            segmentStartTime[segmentIndex];


        real_t j =
            segmentJerk[segmentIndex];


        /* --------------------------------------------------------------------
         * Acceleration
         *
         * MATLAB:
         *
         * sDDot(i) =
         *      aStart(k) +
         *      j*tau;
         * --------------------------------------------------------------------
         */

        sDDot[i] =
            aStart[segmentIndex]
            +
            j * tau;


        /* --------------------------------------------------------------------
         * Velocity
         *
         * MATLAB:
         *
         * sDot(i) =
         *      vStart(k)
         *      + aStart(k)*tau
         *      + 0.5*j*tau^2;
         * --------------------------------------------------------------------
         */

        sDot[i] =
            vStart[segmentIndex]
            +
            aStart[segmentIndex] *
            tau
            +
            0.5 *
            j *
            tau * tau;


        /* --------------------------------------------------------------------
         * Position
         *
         * MATLAB:
         *
         * s(i) =
         *      sStart(k)
         *      + vStart(k)*tau
         *      + 0.5*aStart(k)*tau^2
         *      + (1/6)*j*tau^3;
         * --------------------------------------------------------------------
         */

        s[i] =
            sStart[segmentIndex]
            +
            vStart[segmentIndex] *
            tau
            +
            0.5 *
            aStart[segmentIndex] *
            tau * tau
            +
            (1.0 / 6.0) *
            j *
            tau * tau * tau;


        /* --------------------------------------------------------------------
         * Jerk
         * --------------------------------------------------------------------
         */

        sDDDot[i] =
            j;
    }


    /* ========================================================================
     * FORCE EXACT ENDPOINT CONDITIONS
     * ========================================================================
     *
     * MATLAB:
     *
     *      s(1)   = 0;
     *      s(end) = 1;
     *
     *      sDot(1)   = 0;
     *      sDot(end) = 0;
     *
     *      sDDot(1)   = 0;
     *      sDDot(end) = 0;
     */

    s[0] =
        0.0;


    s[sampleCount - 1] =
        1.0;


    sDot[0] =
        0.0;


    sDot[sampleCount - 1] =
        0.0;


    sDDot[0] =
        0.0;


    sDDot[sampleCount - 1] =
        0.0;


    /* ========================================================================
     * PROFILE INFORMATION
     * ========================================================================
     */

    info->type =
        "S-Curve";


    info->T =
        T;


    info->dt =
        dt;


    info->requestedVMax =
        vMax;


    info->requestedAMax =
        aMax;


    info->requestedJMax =
        jMax;


    info->vPeak =
        vPeak;


    info->aPeak =
        aPeak;


    info->jPeak =
        jMax;


    info->tJerk =
        tJ;


    info->tConstantAcceleration =
        tA;


    info->tConstantVelocity =
        tV;


    for (int k = 0;
         k < S_CURVE_SEGMENTS;
         k++)
    {
        info->segmentDurations[k] =
            segmentDuration[k];
    }


    *count =
        sampleCount;


    return true;
}