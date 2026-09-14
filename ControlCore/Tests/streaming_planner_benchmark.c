#define _POSIX_C_SOURCE 200809L

#include "single_segment_line.h"
#include "single_segment_line_stream.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define GEOMETRY_CAPACITY 512U

static Vec3 rawGeometry[GEOMETRY_CAPACITY];
static Vec3 arcGeometry[GEOMETRY_CAPACITY];

static real_t lOriginal[GEOMETRY_CAPACITY];
static real_t lArc[GEOMETRY_CAPACITY];

static ADLSInfo ikScratch;


static double elapsed_us(
    const struct timespec *start,
    const struct timespec *end
)
{
    return
        (double)(end->tv_sec - start->tv_sec) * 1000000.0
        +
        (double)(end->tv_nsec - start->tv_nsec) / 1000.0;
}


static void fail(
    const char *message
)
{
    printf("BENCHMARK FAILED: %s\n", message);
    exit(1);
}


int main(void)
{
    RobotConfig robot;

    robot_config_init_ur5(
        &robot
    );


    /*
     * Same validated reachable A -> B path.
     *
     * Use 0.05 m/s so the test contains >10,000 samples and exercises
     * sustained streaming generation for a meaningful duration.
     */
    SingleLineRequest request =
        single_line_matlab_reference_request();

    request.desiredTCPSpeed =
        0.05;

    request.dt =
        0.001;


    SingleLineStreamWorkspace workspace;

    memset(
        &workspace,
        0,
        sizeof(workspace)
    );

    workspace.geometryCapacity =
        GEOMETRY_CAPACITY;

    workspace.rawGeometry =
        rawGeometry;

    workspace.arcGeometry =
        arcGeometry;

    workspace.lOriginal =
        lOriginal;

    workspace.lArc =
        lArc;

    workspace.ikScratch =
        &ikScratch;


    SingleLineStream stream;


    struct timespec initStart;
    struct timespec initEnd;

    clock_gettime(
        CLOCK_MONOTONIC,
        &initStart
    );


    if (
        !single_line_stream_init(
            &stream,
            &robot,
            &request,
            &workspace
        )
    )
    {
        fail(
            "single_line_stream_init() failed"
        );
    }


    clock_gettime(
        CLOCK_MONOTONIC,
        &initEnd
    );


    const size_t expectedSamples =
        single_line_stream_sample_count(
            &stream
        );


    size_t generated =
        0;

    size_t over1ms =
        0;

    double maxSampleUs =
        0.0;

    double totalSampleUs =
        0.0;


    struct timespec totalStart;
    struct timespec totalEnd;

    clock_gettime(
        CLOCK_MONOTONIC,
        &totalStart
    );


    for (;;)
    {
        SingleLineStreamSample sample;

        struct timespec sampleStart;
        struct timespec sampleEnd;

        clock_gettime(
            CLOCK_MONOTONIC,
            &sampleStart
        );


        bool produced =
            single_line_stream_next(
                &stream,
                &sample
            );


        clock_gettime(
            CLOCK_MONOTONIC,
            &sampleEnd
        );


        if (!produced)
        {
            if (
                single_line_stream_is_finished(
                    &stream
                )
            )
            {
                break;
            }


            printf(
                "Stream stopped with status: %s\n",
                single_line_stream_status_string(
                    single_line_stream_status(
                        &stream
                    )
                )
            );

            fail(
                "stream stopped before completion"
            );
        }


        double sampleUs =
            elapsed_us(
                &sampleStart,
                &sampleEnd
            );


        totalSampleUs +=
            sampleUs;


        if (
            sampleUs >
            maxSampleUs
        )
        {
            maxSampleUs =
                sampleUs;
        }


        if (
            sampleUs >
            1000.0
        )
        {
            over1ms++;
        }


        generated++;
    }


    clock_gettime(
        CLOCK_MONOTONIC,
        &totalEnd
    );


    if (
        generated !=
        expectedSamples
    )
    {
        fail(
            "generated sample count mismatch"
        );
    }


    double initializationUs =
        elapsed_us(
            &initStart,
            &initEnd
        );


    double wallTimeUs =
        elapsed_us(
            &totalStart,
            &totalEnd
        );


    double averageSampleUs =
        generated > 0
        ? totalSampleUs / (double)generated
        : 0.0;


    double samplesPerSecond =
        wallTimeUs > 0.0
        ? (double)generated * 1000000.0 / wallTimeUs
        : 0.0;


    double realtimeMargin =
        averageSampleUs > 0.0
        ? 1000.0 / averageSampleUs
        : 0.0;


    printf(
        "\nSTREAMING PLANNER BENCHMARK COMPLETE\n"
    );

    printf(
        "Trajectory samples:      %zu\n",
        generated
    );

    printf(
        "Trajectory duration:     %.6f s\n",
        stream.profile.info.T
    );

    printf(
        "Profile/geometry init:   %.3f ms\n",
        initializationUs / 1000.0
    );

    printf(
        "Generation wall time:    %.3f ms\n",
        wallTimeUs / 1000.0
    );

    printf(
        "Average q sample time:   %.3f us\n",
        averageSampleUs
    );

    printf(
        "Worst observed sample:   %.3f us\n",
        maxSampleUs
    );

    printf(
        "Samples slower than 1ms: %zu / %zu\n",
        over1ms,
        generated
    );

    printf(
        "Generation throughput:   %.1f samples/s\n",
        samplesPerSecond
    );

    printf(
        "Average 1kHz margin:     %.2fx\n\n",
        realtimeMargin
    );


    /*
     * This is a benchmark, not a hard pass/fail real-time certification.
     * WSL/Linux scheduling can introduce occasional host jitter.
     */
    if (
        samplesPerSecond <
        1000.0
    )
    {
        printf(
            "WARNING: average planner throughput is below 1 kHz.\n"
            "A finite streaming buffer would eventually underrun.\n\n"
        );
    }
    else
    {
        printf(
            "Average planner throughput exceeds the 1 kHz consumer rate.\n"
            "Use the worst-case timing to choose a practical look-ahead buffer.\n\n"
        );
    }


    return 0;
}
