#include "../Config/robot_config.h"
#include "../Pipeline/single_segment_circular.h"
#include "../Pipeline/single_segment_circular_stream.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_GEOMETRY_CAPACITY 1024U

static Vec3 raw_geometry[TEST_GEOMETRY_CAPACITY];
static Vec3 arc_geometry[TEST_GEOMETRY_CAPACITY];
static real_t l_original[TEST_GEOMETRY_CAPACITY];
static real_t l_arc[TEST_GEOMETRY_CAPACITY];
static ADLSInfo ik_scratch;

static SingleCircularStreamWorkspace make_workspace(void)
{
    SingleCircularStreamWorkspace workspace =
    {
        .geometryCapacity = TEST_GEOMETRY_CAPACITY,
        .rawGeometry = raw_geometry,
        .arcGeometry = arc_geometry,
        .lOriginal = l_original,
        .lArc = l_arc,
        .ikScratch = &ik_scratch
    };

    return workspace;
}

static int run_case(
    const char *name,
    const RobotConfig *robot,
    const SingleCircularRequest *request,
    SingleCircularStreamWorkspace *workspace
)
{
    SingleCircularStream stream;

    if (
        !single_circular_stream_init(
            &stream,
            robot,
            request,
            workspace
        )
    )
    {
        fprintf(
            stderr,
            "%s init FAILED: %s\n",
            name,
            single_circular_stream_status_string(
                single_circular_stream_status(&stream)
            )
        );
        return 1;
    }

    size_t expected = single_circular_stream_sample_count(&stream);
    size_t generated = 0;
    SingleCircularStreamSample first;
    SingleCircularStreamSample last;
    bool have_first = false;

    while (!single_circular_stream_is_finished(&stream))
    {
        SingleCircularStreamSample sample;

        if (!single_circular_stream_next(&stream, &sample))
        {
            fprintf(
                stderr,
                "%s generation FAILED: %s\n",
                name,
                single_circular_stream_status_string(
                    single_circular_stream_status(&stream)
                )
            );
            return 1;
        }

        if (!have_first)
        {
            first = sample;
            have_first = true;
        }

        last = sample;
        generated++;

        if (
            !isfinite(sample.t) ||
            !isfinite(sample.tcpSpeed) ||
            !isfinite(sample.positionError) ||
            !isfinite(sample.orientationError)
        )
        {
            fprintf(stderr, "%s produced non-finite diagnostics.\n", name);
            return 1;
        }
    }

    if (!have_first || generated != expected)
    {
        fprintf(
            stderr,
            "%s sample count mismatch: generated=%zu expected=%zu\n",
            name,
            generated,
            expected
        );
        return 1;
    }

    real_t first_error =
        sqrt(
            pow(first.pDesired.v[0] - request->point1.v[0], 2.0) +
            pow(first.pDesired.v[1] - request->point1.v[1], 2.0) +
            pow(first.pDesired.v[2] - request->point1.v[2], 2.0)
        );

    Vec3 expected_end =
        request->type == CIRCULAR_SEGMENT_FULL_CIRCLE
            ? request->point1
            : request->point3;

    real_t last_error =
        sqrt(
            pow(last.pDesired.v[0] - expected_end.v[0], 2.0) +
            pow(last.pDesired.v[1] - expected_end.v[1], 2.0) +
            pow(last.pDesired.v[2] - expected_end.v[2], 2.0)
        );

    if (first_error > 1e-8 || last_error > 1e-6)
    {
        fprintf(
            stderr,
            "%s endpoint check FAILED: first=%.3e last=%.3e\n",
            name,
            first_error,
            last_error
        );
        return 1;
    }

    printf("\n============================================================\n");
    printf(" %s STREAMING TEST PASS\n", name);
    printf("============================================================\n");
    printf("samples:             %zu\n", generated);
    printf("duration:            %.6f s\n", stream.profile.info.T);
    printf("path length:         %.6f m\n", stream.segmentLength);
    printf("peak TCP speed:      %.6f m/s\n", stream.peakTCPSpeed);
    printf("max position error:  %.3e m\n", stream.maxPositionError);
    printf("max rotation error:  %.3e rad\n", stream.maxOrientationError);
    printf("max IK iterations:   %d\n", stream.maxIKIterations);

    return 0;
}

int main(void)
{
    RobotConfig robot;
    robot_config_init_ur5(&robot);

    SingleCircularStreamWorkspace workspace = make_workspace();

    SingleCircularRequest arc = single_arc_matlab_reference_request();

    if (run_case("CIRCULAR ARC", &robot, &arc, &workspace) != 0)
    {
        return 1;
    }

    SingleCircularRequest circle = single_full_circle_matlab_reference_request();

    if (run_case("FULL CIRCLE", &robot, &circle, &workspace) != 0)
    {
        return 1;
    }

    printf("\nCircular streaming reference tests completed successfully.\n");
    return 0;
}
