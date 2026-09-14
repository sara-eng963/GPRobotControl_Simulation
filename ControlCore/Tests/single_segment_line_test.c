#include "../Config/robot_config.h"

#include "../Pipeline/single_segment_line.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


/* ============================================================================
 * TEST BUFFER CAPACITIES
 * ============================================================================
 *
 * These are intentionally larger than the current MATLAB test requires.
 *
 * Current MATLAB reference test:
 *
 *      raw geometry points : 100
 *      path length         : 0.50 m
 *      arc spacing         : 0.005 m
 *
 * so the arc-length path is only about 101 points.
 *
 * The larger capacities give us room for future experiments.
 * ============================================================================
 */

#define TEST_GEOMETRY_CAPACITY   512
#define TEST_TRAJECTORY_CAPACITY 7000


/* ============================================================================
 * STATIC WORKSPACE
 * ============================================================================
 *
 * STATIC is important here.
 *
 * ADLSInfo contains large history arrays, and trajectory buffers can also
 * become large. Keeping them static prevents putting tens of kilobytes on
 * the normal function stack.
 *
 * This becomes even more important later under FreeRTOS / STM32.
 * ============================================================================
 */


/* ----------------------------------------------------------------------------
 * Geometry workspace
 * ----------------------------------------------------------------------------
 */

static Vec3 rawGeometry[
    TEST_GEOMETRY_CAPACITY
];

static Vec3 arcGeometry[
    TEST_GEOMETRY_CAPACITY
];

static real_t lOriginal[
    TEST_GEOMETRY_CAPACITY
];

static real_t lArc[
    TEST_GEOMETRY_CAPACITY
];


/* ----------------------------------------------------------------------------
 * Temporary S-curve workspace
 * ----------------------------------------------------------------------------
 */

static real_t tempT[
    TEST_TRAJECTORY_CAPACITY
];

static real_t tempS[
    TEST_TRAJECTORY_CAPACITY
];

static real_t tempSDot[
    TEST_TRAJECTORY_CAPACITY
];

static real_t tempSDDot[
    TEST_TRAJECTORY_CAPACITY
];

static real_t tempSDDDot[
    TEST_TRAJECTORY_CAPACITY
];


/* ----------------------------------------------------------------------------
 * ADLS workspace
 * ----------------------------------------------------------------------------
 */

static ADLSInfo ikScratch;


/* ============================================================================
 * FINAL TRAJECTORY STORAGE
 * ============================================================================
 */

static real_t trajectoryT[
    TEST_TRAJECTORY_CAPACITY
];

static real_t trajectoryS[
    TEST_TRAJECTORY_CAPACITY
];

static real_t trajectorySDot[
    TEST_TRAJECTORY_CAPACITY
];

static real_t trajectorySDDot[
    TEST_TRAJECTORY_CAPACITY
];

static real_t trajectorySDDDot[
    TEST_TRAJECTORY_CAPACITY
];


static real_t trajectoryArcPosition[
    TEST_TRAJECTORY_CAPACITY
];

static real_t trajectoryTCPSpeed[
    TEST_TRAJECTORY_CAPACITY
];


static Vec3 trajectoryPosition[
    TEST_TRAJECTORY_CAPACITY
];

static Quat trajectoryQuaternion[
    TEST_TRAJECTORY_CAPACITY
];


static JointVector trajectoryQ[
    TEST_TRAJECTORY_CAPACITY
];

static JointVector trajectoryQDot[
    TEST_TRAJECTORY_CAPACITY
];

static JointVector trajectoryQDDot[
    TEST_TRAJECTORY_CAPACITY
];


static int trajectoryIKIterations[
    TEST_TRAJECTORY_CAPACITY
];


static real_t trajectoryPositionError[
    TEST_TRAJECTORY_CAPACITY
];

static real_t trajectoryOrientationError[
    TEST_TRAJECTORY_CAPACITY
];


/* ============================================================================
 * RAD -> DEG
 * ============================================================================
 */

static double radians_to_degrees(
    double radians
)
{
    return
        radians *
        180.0 /
        ROBOT_PI;
}


/* ============================================================================
 * WRITE GENERATED TRAJECTORY TO CSV
 * ============================================================================
 *
 * This is extremely useful for comparing:
 *
 *      MATLAB output
 *
 * against:
 *
 *      C output
 *
 * sample-by-sample.
 * ============================================================================
 */

static bool write_trajectory_csv(
    const char *filename,
    const SingleLineTrajectory *trajectory
)
{
    if (
        filename == NULL ||
        trajectory == NULL
    )
    {
        return false;
    }


    FILE *file =
        fopen(
            filename,
            "w"
        );


    if (file == NULL)
    {
        return false;
    }


    /* ------------------------------------------------------------------------
     * Header
     * ------------------------------------------------------------------------
     */

    fprintf(
        file,
        "sample,"
        "time,"
        "s,"
        "sDot,"
        "sDDot,"
        "sDDDot,"
        "arcLength,"
        "tcpSpeed,"
        "px,"
        "py,"
        "pz,"
        "qw,"
        "qx,"
        "qy,"
        "qz,"
        "q1,"
        "q2,"
        "q3,"
        "q4,"
        "q5,"
        "q6,"
        "qd1,"
        "qd2,"
        "qd3,"
        "qd4,"
        "qd5,"
        "qd6,"
        "positionError,"
        "orientationError,"
        "ikIterations\n"
    );


    /* ------------------------------------------------------------------------
     * Samples
     * ------------------------------------------------------------------------
     */

    for (size_t i = 0;
         i < trajectory->count;
         i++)
    {
        fprintf(
            file,

            "%zu,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"

            "%.12f,"
            "%.12f,"
            "%.12f,"

            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"

            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"

            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"
            "%.12f,"

            "%.12e,"
            "%.12e,"

            "%d\n",

            i,

            trajectory->t[i],

            trajectory->s[i],

            trajectory->sDot[i],

            trajectory->sDDot[i],

            trajectory->sDDDot[i],

            trajectory->arcPosition[i],

            trajectory->tcpSpeed[i],

            trajectory->pDesired[i].v[0],
            trajectory->pDesired[i].v[1],
            trajectory->pDesired[i].v[2],

            trajectory->quatDesired[i].w,
            trajectory->quatDesired[i].x,
            trajectory->quatDesired[i].y,
            trajectory->quatDesired[i].z,

            trajectory->qPath[i].q[0],
            trajectory->qPath[i].q[1],
            trajectory->qPath[i].q[2],
            trajectory->qPath[i].q[3],
            trajectory->qPath[i].q[4],
            trajectory->qPath[i].q[5],

            trajectory->qDot[i].q[0],
            trajectory->qDot[i].q[1],
            trajectory->qDot[i].q[2],
            trajectory->qDot[i].q[3],
            trajectory->qDot[i].q[4],
            trajectory->qDot[i].q[5],

            trajectory->positionError[i],

            trajectory->orientationError[i],

            trajectory->ikIterations[i]
        );
    }


    fclose(file);


    return true;
}


/* ============================================================================
 * PRINT TEST REPORT
 * ============================================================================
 */

static void print_report(
    const SingleLineReport *report
)
{
    printf("\n");

    printf(
        "============================================================\n"
    );

    printf(
        " C SINGLE-SEGMENT LINE PIPELINE RESULT\n"
    );

    printf(
        "============================================================\n"
    );


    printf(
        "Waypoint A:               "
        "[%.6f %.6f %.6f] m\n",

        report->waypointA.v[0],
        report->waypointA.v[1],
        report->waypointA.v[2]
    );


    printf(
        "Waypoint B:               "
        "[%.6f %.6f %.6f] m\n",

        report->waypointB.v[0],
        report->waypointB.v[1],
        report->waypointB.v[2]
    );


    printf(
        "Path length:              %.6f m\n",
        report->pathLength
    );


    printf(
        "Raw geometry points:      %zu\n",
        report->rawGeometryPoints
    );


    printf(
        "Arc geometry points:      %zu\n",
        report->arcGeometryPoints
    );


    printf(
        "Trajectory samples:       %zu\n",
        report->samples
    );


    printf(
        "Duration:                 %.6f s\n",
        report->duration
    );


    printf(
        "Peak TCP speed:           %.6f m/s\n",
        report->peakTCPSpeed
    );


    printf(
        "\n"
    );


    printf(
        "Maximum position error:   %.6e m\n",
        report->maxPositionError
    );


    printf(
        "Maximum orientation error: %.6e rad\n",
        report->maxOrientationError
    );


    printf(
        "Maximum IK iterations:    %d\n",
        report->maxIKIterations
    );


    printf(
        "\n"
    );


    printf(
        "Joint position limits:    %s\n",
        report->positionLimitsPass
            ? "PASS"
            : "FAIL"
    );


    printf(
        "Joint velocity limits:    %s\n",
        report->velocityLimitsPass
            ? "PASS"
            : "FAIL"
    );


    printf(
        "\n"
    );


    for (int joint = 0;
         joint < ROBOT_DOF;
         joint++)
    {
        printf(
            "J%d | peak vel %.6f rad/s | "
            "largest step %.6f deg\n",

            joint + 1,

            report->peakJointVelocity[joint],

            radians_to_degrees(
                report->maxJointStep[joint]
            )
        );
    }


    printf(
        "\n"
    );


    printf(
        "S-Curve profile:          %s\n",
        report->profile.type
    );


    printf(
        "Peak normalized velocity: %.6f 1/s\n",
        report->profile.vPeak
    );


    printf(
        "Peak normalized accel:    %.6f 1/s^2\n",
        report->profile.aPeak
    );


    printf(
        "Normalized jerk:          %.6f 1/s^3\n",
        report->profile.jPeak
    );


    printf(
        "\n"
    );


    printf(
        "S-Curve segment times:\n"
    );


    for (int segment = 0;
         segment < S_CURVE_SEGMENTS;
         segment++)
    {
        printf(
            "  Segment %d: %.6f s\n",

            segment + 1,

            report
                ->profile
                .segmentDurations[segment]
        );
    }


    printf(
        "============================================================\n"
    );
}


/* ============================================================================
 * MAIN TEST
 * ============================================================================
 */

int main(void)
{
    /* ========================================================================
     * 1. CREATE ROBOT CONFIGURATION
     * ========================================================================
     */

    RobotConfig robot;


    robot_config_init_ur5(
        &robot
    );


    /* ========================================================================
     * 2. CREATE THE SAME TEST CASE AS MATLAB
     * ========================================================================
     *
     * This gives us:
     *
     *      qStart
     *      B = A + [0.5 0 0]
     *      15 deg yaw/pitch/roll
     *      100 geometry points
     *      0.005 m arc spacing
     *      0.10 m/s speed
     *      0.25 m/s^2 acceleration
     *      1.00 m/s^3 jerk
     *      dt = 0.05 s
     */

    SingleLineRequest request =
        single_line_matlab_reference_request();


    /* ========================================================================
     * 3. CONNECT WORKSPACE BUFFERS
     * ========================================================================
     */

    SingleLineWorkspace workspace =
    {
        .geometryCapacity =
            TEST_GEOMETRY_CAPACITY,


        .rawGeometry =
            rawGeometry,


        .arcGeometry =
            arcGeometry,


        .lOriginal =
            lOriginal,


        .lArc =
            lArc,


        .timeCapacity =
            TEST_TRAJECTORY_CAPACITY,


        .tempT =
            tempT,


        .tempS =
            tempS,


        .tempSDot =
            tempSDot,


        .tempSDDot =
            tempSDDot,


        .tempSDDDot =
            tempSDDDot,


        .ikScratch =
            &ikScratch
    };


    /* ========================================================================
     * 4. CONNECT FINAL TRAJECTORY BUFFERS
     * ========================================================================
     */

    SingleLineTrajectory trajectory =
    {
        .capacity =
            TEST_TRAJECTORY_CAPACITY,


        .count =
            0,


        .t =
            trajectoryT,


        .s =
            trajectoryS,


        .sDot =
            trajectorySDot,


        .sDDot =
            trajectorySDDot,


        .sDDDot =
            trajectorySDDDot,


        .arcPosition =
            trajectoryArcPosition,


        .tcpSpeed =
            trajectoryTCPSpeed,


        .pDesired =
            trajectoryPosition,


        .quatDesired =
            trajectoryQuaternion,


        .qPath =
            trajectoryQ,


        .qDot =
            trajectoryQDot,


        .qDDot =
            trajectoryQDDot,


        .ikIterations =
            trajectoryIKIterations,


        .positionError =
            trajectoryPositionError,


        .orientationError =
            trajectoryOrientationError
    };


    /* ========================================================================
     * 5. CREATE REPORT
     * ========================================================================
     */

    SingleLineReport report;


    /* ========================================================================
     * 6. RUN COMPLETE PIPELINE
     * ========================================================================
     */

    printf(
        "\nRunning C single-segment line pipeline...\n"
    );


    bool success =
        plan_single_segment_line(
            &robot,
            &request,
            &workspace,
            &trajectory,
            &report
        );


    /* ========================================================================
     * 7. CHECK PIPELINE RESULT
     * ========================================================================
     */

    if (!success)
    {
        fprintf(
            stderr,
            "\n"
            "============================================================\n"
            " TEST FAILED\n"
            "============================================================\n"
            "The single-segment line pipeline did not complete.\n"
            "\n"
            "Check:\n"
            "  - buffer capacities\n"
            "  - FK\n"
            "  - Jacobian\n"
            "  - ADLS IK\n"
            "  - arc-length generation\n"
            "  - S-curve generation\n"
            "============================================================\n"
        );


        return EXIT_FAILURE;
    }


    /* ========================================================================
     * 8. PRINT SAME KIND OF DIAGNOSTICS AS MATLAB
     * ========================================================================
     */

    print_report(
        &report
    );


    /* ========================================================================
     * 9. WRITE CSV FOR MATLAB <-> C COMPARISON
     * ========================================================================
     */

    const char *csvFilename =
        "single_segment_line_c.csv";


    if (
        write_trajectory_csv(
            csvFilename,
            &trajectory
        )
    )
    {
        printf(
            "\nTrajectory CSV written to:\n"
            "  %s\n",
            csvFilename
        );
    }
    else
    {
        fprintf(
            stderr,
            "\nWarning: could not write trajectory CSV.\n"
        );
    }


    /* ========================================================================
     * 10. BASIC AUTOMATIC ACCEPTANCE CHECKS
     * ========================================================================
     *
     * These are deliberately simple.
     *
     * The detailed numerical parity check against MATLAB comes next.
     * ========================================================================
     */


    if (!report.positionLimitsPass)
    {
        fprintf(
            stderr,
            "\nTEST FAILED: joint position limits violated.\n"
        );


        return EXIT_FAILURE;
    }


    if (!report.velocityLimitsPass)
    {
        fprintf(
            stderr,
            "\nTEST FAILED: joint velocity limits violated.\n"
        );


        return EXIT_FAILURE;
    }


    /*
     * Same order of magnitude as the ADLS Cartesian tolerance.
     *
     * Do not demand bit-identical MATLAB values because MATLAB's SVD and
     * linear algebra implementation differ from our small C routines.
     */
    if (
        report.maxPositionError >
        2e-4
    )
    {
        fprintf(
            stderr,
            "\n"
            "TEST FAILED: maximum Cartesian position error too large.\n"
        );


        return EXIT_FAILURE;
    }


    if (
        report.maxOrientationError >
        2e-4
    )
    {
        fprintf(
            stderr,
            "\n"
            "TEST FAILED: maximum orientation error too large.\n"
        );


        return EXIT_FAILURE;
    }


    /* ========================================================================
     * TEST PASSED
     * ========================================================================
     */

    printf(
        "\n"
        "============================================================\n"
        " TEST PASSED\n"
        "============================================================\n"
        "The modular C implementation successfully generated the\n"
        "single straight-line Cartesian trajectory.\n"
        "============================================================\n"
    );


    return EXIT_SUCCESS;
}