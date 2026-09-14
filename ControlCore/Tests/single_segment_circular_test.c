#include "../Config/robot_config.h"
#include "../Pipeline/single_segment_circular.h"

#include <stdio.h>


#define TEST_GEOMETRY_CAPACITY   1024
#define TEST_TRAJECTORY_CAPACITY 12000


/* Geometry workspace */
static Vec3 rawGeometry[TEST_GEOMETRY_CAPACITY];
static Vec3 arcGeometry[TEST_GEOMETRY_CAPACITY];
static real_t lOriginal[TEST_GEOMETRY_CAPACITY];
static real_t lArc[TEST_GEOMETRY_CAPACITY];

/* Temporary S-curve workspace */
static real_t tempT[TEST_TRAJECTORY_CAPACITY];
static real_t tempS[TEST_TRAJECTORY_CAPACITY];
static real_t tempSDot[TEST_TRAJECTORY_CAPACITY];
static real_t tempSDDot[TEST_TRAJECTORY_CAPACITY];
static real_t tempSDDDot[TEST_TRAJECTORY_CAPACITY];

static ADLSInfo ikScratch;

/* Output trajectory */
static real_t trajectoryT[TEST_TRAJECTORY_CAPACITY];
static real_t trajectoryS[TEST_TRAJECTORY_CAPACITY];
static real_t trajectorySDot[TEST_TRAJECTORY_CAPACITY];
static real_t trajectorySDDot[TEST_TRAJECTORY_CAPACITY];
static real_t trajectorySDDDot[TEST_TRAJECTORY_CAPACITY];
static real_t trajectoryArcPosition[TEST_TRAJECTORY_CAPACITY];
static real_t trajectoryTCPSpeed[TEST_TRAJECTORY_CAPACITY];

static Vec3 trajectoryPosition[TEST_TRAJECTORY_CAPACITY];
static Quat trajectoryQuaternion[TEST_TRAJECTORY_CAPACITY];

static JointVector trajectoryQ[TEST_TRAJECTORY_CAPACITY];
static JointVector trajectoryQDot[TEST_TRAJECTORY_CAPACITY];
static JointVector trajectoryQDDot[TEST_TRAJECTORY_CAPACITY];

static int trajectoryIKIterations[TEST_TRAJECTORY_CAPACITY];
static real_t trajectoryPositionError[TEST_TRAJECTORY_CAPACITY];
static real_t trajectoryOrientationError[TEST_TRAJECTORY_CAPACITY];


static SingleCircularWorkspace make_workspace(void)
{
    SingleCircularWorkspace workspace =
    {
        .geometryCapacity = TEST_GEOMETRY_CAPACITY,
        .rawGeometry = rawGeometry,
        .arcGeometry = arcGeometry,
        .lOriginal = lOriginal,
        .lArc = lArc,

        .timeCapacity = TEST_TRAJECTORY_CAPACITY,
        .tempT = tempT,
        .tempS = tempS,
        .tempSDot = tempSDot,
        .tempSDDot = tempSDDot,
        .tempSDDDot = tempSDDDot,

        .ikScratch = &ikScratch
    };

    return workspace;
}


static SingleCircularTrajectory make_trajectory(void)
{
    SingleCircularTrajectory trajectory =
    {
        .capacity = TEST_TRAJECTORY_CAPACITY,
        .count = 0,

        .t = trajectoryT,
        .s = trajectoryS,
        .sDot = trajectorySDot,
        .sDDot = trajectorySDDot,
        .sDDDot = trajectorySDDDot,

        .arcPosition = trajectoryArcPosition,
        .tcpSpeed = trajectoryTCPSpeed,

        .pDesired = trajectoryPosition,
        .quatDesired = trajectoryQuaternion,

        .qPath = trajectoryQ,
        .qDot = trajectoryQDot,
        .qDDot = trajectoryQDDot,

        .ikIterations = trajectoryIKIterations,
        .positionError = trajectoryPositionError,
        .orientationError = trajectoryOrientationError
    };

    return trajectory;
}


static void print_report(
    const char *name,
    const SingleCircularReport *report
)
{
    printf("\n============================================================\n");
    printf(" %s\n", name);
    printf("============================================================\n");

    printf("samples:                %zu\n", report->samples);
    printf("path length:            %.6f m\n", report->pathLength);
    printf("duration:               %.6f s\n", report->duration);
    printf("peak TCP speed:         %.6f m/s\n", report->peakTCPSpeed);

    printf("circle center:          [%.6f %.6f %.6f] m\n",
        report->geometry.center.v[0],
        report->geometry.center.v[1],
        report->geometry.center.v[2]);

    printf("radius:                 %.6f m\n", report->geometry.radius);
    printf("sweep:                  %.6f rad\n", report->geometry.thetaTotal);

    printf("max position error:     %.3e m\n", report->maxPositionError);
    printf("max orientation error:  %.3e rad\n", report->maxOrientationError);
    printf("max IK iterations:      %d\n", report->maxIKIterations);

    printf("position limits:        %s\n",
        report->positionLimitsPass ? "PASS" : "FAIL");

    printf("velocity limits:        %s\n",
        report->velocityLimitsPass ? "PASS" : "FAIL");
}


int main(void)
{
    RobotConfig robot;
    robot_config_init_ur5(&robot);

    SingleCircularWorkspace workspace = make_workspace();
    SingleCircularTrajectory trajectory = make_trajectory();
    SingleCircularReport report;

    SingleCircularRequest arcRequest =
        single_arc_matlab_reference_request();

    if (
        !plan_single_segment_arc(
            &robot,
            &arcRequest,
            &workspace,
            &trajectory,
            &report
        )
    )
    {
        fprintf(stderr, "Circular-arc pipeline FAILED.\n");
        return 1;
    }

    print_report("C SINGLE-SEGMENT CIRCULAR ARC RESULT", &report);

    SingleCircularRequest circleRequest =
        single_full_circle_matlab_reference_request();

    trajectory.count = 0;

    if (
        !plan_single_segment_full_circle(
            &robot,
            &circleRequest,
            &workspace,
            &trajectory,
            &report
        )
    )
    {
        fprintf(stderr, "Full-circle pipeline FAILED.\n");
        return 1;
    }

    print_report("C SINGLE-SEGMENT FULL CIRCLE RESULT", &report);

    printf("\nCircular trajectory reference tests completed successfully.\n");

    return 0;
}