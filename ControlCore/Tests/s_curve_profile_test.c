#include "s_curve_profile.h"
#include "s_curve_time_scaling.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define LEGACY_CAPACITY 7000

static real_t legacyT[LEGACY_CAPACITY];
static real_t legacyS[LEGACY_CAPACITY];
static real_t legacySDot[LEGACY_CAPACITY];
static real_t legacySDDot[LEGACY_CAPACITY];
static real_t legacySDDDot[LEGACY_CAPACITY];

static void fail(
    const char *message
)
{
    printf("TEST FAILED: %s\n", message);
    exit(1);
}

static real_t max_value(
    real_t a,
    real_t b
)
{
    return
        a > b
        ? a
        : b;
}

int main(void)
{
    /*
     * Validated 0.5 m reference trajectory:
     *
     * physical speed = 0.10 m/s
     * physical accel = 0.25 m/s^2
     * physical jerk  = 1.00 m/s^3
     *
     * Normalized by L = 0.5 m:
     *
     * vMax = 0.2 1/s
     * aMax = 0.5 1/s^2
     * jMax = 2.0 1/s^3
     */

    const real_t vMax =
        0.2;

    const real_t aMax =
        0.5;

    const real_t jMax =
        2.0;

    const real_t dt =
        0.001;


    size_t legacyCount =
        0;

    SCurveInfo legacyInfo;

    if (
        !s_curve_time_scaling(
            vMax,
            aMax,
            jMax,
            dt,
            legacyT,
            legacyS,
            legacySDot,
            legacySDDot,
            legacySDDDot,
            LEGACY_CAPACITY,
            &legacyCount,
            &legacyInfo
        )
    )
    {
        fail(
            "legacy s_curve_time_scaling() failed"
        );
    }


    SCurveProfile profile;

    if (
        !s_curve_profile_init(
            vMax,
            aMax,
            jMax,
            dt,
            &profile
        )
    )
    {
        fail(
            "s_curve_profile_init() failed"
        );
    }


    size_t streamingCount =
        s_curve_profile_sample_count(
            &profile
        );

    if (streamingCount != legacyCount)
    {
        printf(
            "Legacy count:    %zu\n"
            "Streaming count: %zu\n",
            legacyCount,
            streamingCount
        );

        fail(
            "sample count mismatch"
        );
    }


    real_t maxTimeError =
        0.0;

    real_t maxSError =
        0.0;

    real_t maxSDotError =
        0.0;

    real_t maxSDDotError =
        0.0;

    real_t maxSDDDotError =
        0.0;


    for (size_t i = 0;
         i < legacyCount;
         i++)
    {
        SCurveSample sample;

        if (
            !s_curve_profile_evaluate_index(
                &profile,
                i,
                &sample
            )
        )
        {
            fail(
                "streaming sample evaluation failed"
            );
        }

        maxTimeError =
            max_value(
                maxTimeError,
                fabs(
                    sample.t -
                    legacyT[i]
                )
            );

        maxSError =
            max_value(
                maxSError,
                fabs(
                    sample.s -
                    legacyS[i]
                )
            );

        maxSDotError =
            max_value(
                maxSDotError,
                fabs(
                    sample.sDot -
                    legacySDot[i]
                )
            );

        maxSDDotError =
            max_value(
                maxSDDotError,
                fabs(
                    sample.sDDot -
                    legacySDDot[i]
                )
            );

        maxSDDDotError =
            max_value(
                maxSDDDotError,
                fabs(
                    sample.sDDDot -
                    legacySDDDot[i]
                )
            );
    }


    const real_t tolerance =
        1e-12;

    if (
        maxTimeError > tolerance ||
        maxSError > tolerance ||
        maxSDotError > tolerance ||
        maxSDDotError > tolerance ||
        maxSDDDotError > tolerance
    )
    {
        fail(
            "streaming evaluator does not match legacy S-curve"
        );
    }


    printf("\nSTREAMING S-CURVE TEST PASSED\n");
    printf("Samples:             %zu\n", streamingCount);
    printf("Duration:            %.9f s\n", profile.info.T);
    printf("Max t error:         %.3e\n", maxTimeError);
    printf("Max s error:         %.3e\n", maxSError);
    printf("Max sDot error:      %.3e\n", maxSDotError);
    printf("Max sDDot error:     %.3e\n", maxSDDotError);
    printf("Max sDDDot error:    %.3e\n\n", maxSDDDotError);

    return 0;
}
