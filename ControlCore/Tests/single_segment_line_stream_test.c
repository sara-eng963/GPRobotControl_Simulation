#include "single_segment_line.h"
#include "single_segment_line_stream.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TRAJECTORY_CAPACITY 7000
#define GEOMETRY_CAPACITY   512


/* ============================================================================
 * LEGACY FULL-TRAJECTORY STORAGE
 * ============================================================================
 */

static Vec3 legacyRawGeometry[GEOMETRY_CAPACITY];
static Vec3 legacyArcGeometry[GEOMETRY_CAPACITY];

static real_t legacyLOriginal[GEOMETRY_CAPACITY];
static real_t legacyLArc[GEOMETRY_CAPACITY];

static real_t legacyTempT[TRAJECTORY_CAPACITY];
static real_t legacyTempS[TRAJECTORY_CAPACITY];
static real_t legacyTempSDot[TRAJECTORY_CAPACITY];
static real_t legacyTempSDDot[TRAJECTORY_CAPACITY];
static real_t legacyTempSDDDot[TRAJECTORY_CAPACITY];

static ADLSInfo legacyIKScratch;

static real_t legacyT[TRAJECTORY_CAPACITY];
static real_t legacyS[TRAJECTORY_CAPACITY];
static real_t legacySDot[TRAJECTORY_CAPACITY];
static real_t legacySDDot[TRAJECTORY_CAPACITY];
static real_t legacySDDDot[TRAJECTORY_CAPACITY];

static real_t legacyArcPosition[TRAJECTORY_CAPACITY];
static real_t legacyTCPSpeed[TRAJECTORY_CAPACITY];

static Vec3 legacyPosition[TRAJECTORY_CAPACITY];
static Quat legacyQuaternion[TRAJECTORY_CAPACITY];

static JointVector legacyQ[TRAJECTORY_CAPACITY];
static JointVector legacyQDot[TRAJECTORY_CAPACITY];
static JointVector legacyQDDot[TRAJECTORY_CAPACITY];

static int legacyIKIterations[TRAJECTORY_CAPACITY];

static real_t legacyPositionError[TRAJECTORY_CAPACITY];
static real_t legacyOrientationError[TRAJECTORY_CAPACITY];


/* ============================================================================
 * STREAMING FIXED-SIZE WORKSPACE
 * ============================================================================
 */

static Vec3 streamRawGeometry[GEOMETRY_CAPACITY];
static Vec3 streamArcGeometry[GEOMETRY_CAPACITY];

static real_t streamLOriginal[GEOMETRY_CAPACITY];
static real_t streamLArc[GEOMETRY_CAPACITY];

static ADLSInfo streamIKScratch;


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
    RobotConfig robot;

    robot_config_init_ur5(
        &robot
    );


    SingleLineRequest request =
        single_line_matlab_reference_request();


    /* ========================================================================
     * RUN EXISTING VALIDATED FULL PLANNER
     * ========================================================================
     */

    SingleLineWorkspace legacyWorkspace;

    memset(
        &legacyWorkspace,
        0,
        sizeof(legacyWorkspace)
    );

    legacyWorkspace.geometryCapacity =
        GEOMETRY_CAPACITY;

    legacyWorkspace.rawGeometry =
        legacyRawGeometry;

    legacyWorkspace.arcGeometry =
        legacyArcGeometry;

    legacyWorkspace.lOriginal =
        legacyLOriginal;

    legacyWorkspace.lArc =
        legacyLArc;

    legacyWorkspace.timeCapacity =
        TRAJECTORY_CAPACITY;

    legacyWorkspace.tempT =
        legacyTempT;

    legacyWorkspace.tempS =
        legacyTempS;

    legacyWorkspace.tempSDot =
        legacyTempSDot;

    legacyWorkspace.tempSDDot =
        legacyTempSDDot;

    legacyWorkspace.tempSDDDot =
        legacyTempSDDDot;

    legacyWorkspace.ikScratch =
        &legacyIKScratch;


    SingleLineTrajectory legacyTrajectory;

    memset(
        &legacyTrajectory,
        0,
        sizeof(legacyTrajectory)
    );

    legacyTrajectory.capacity =
        TRAJECTORY_CAPACITY;

    legacyTrajectory.t =
        legacyT;

    legacyTrajectory.s =
        legacyS;

    legacyTrajectory.sDot =
        legacySDot;

    legacyTrajectory.sDDot =
        legacySDDot;

    legacyTrajectory.sDDDot =
        legacySDDDot;

    legacyTrajectory.arcPosition =
        legacyArcPosition;

    legacyTrajectory.tcpSpeed =
        legacyTCPSpeed;

    legacyTrajectory.pDesired =
        legacyPosition;

    legacyTrajectory.quatDesired =
        legacyQuaternion;

    legacyTrajectory.qPath =
        legacyQ;

    legacyTrajectory.qDot =
        legacyQDot;

    legacyTrajectory.qDDot =
        legacyQDDot;

    legacyTrajectory.ikIterations =
        legacyIKIterations;

    legacyTrajectory.positionError =
        legacyPositionError;

    legacyTrajectory.orientationError =
        legacyOrientationError;


    SingleLineReport legacyReport;


    if (
        !plan_single_segment_line(
            &robot,
            &request,
            &legacyWorkspace,
            &legacyTrajectory,
            &legacyReport
        )
    )
    {
        fail(
            "legacy full planner failed"
        );
    }


    /* ========================================================================
     * INITIALIZE STREAMING PLANNER
     * ========================================================================
     */

    SingleLineStreamWorkspace streamWorkspace;

    memset(
        &streamWorkspace,
        0,
        sizeof(streamWorkspace)
    );

    streamWorkspace.geometryCapacity =
        GEOMETRY_CAPACITY;

    streamWorkspace.rawGeometry =
        streamRawGeometry;

    streamWorkspace.arcGeometry =
        streamArcGeometry;

    streamWorkspace.lOriginal =
        streamLOriginal;

    streamWorkspace.lArc =
        streamLArc;

    streamWorkspace.ikScratch =
        &streamIKScratch;


    SingleLineStream stream;


    if (
        !single_line_stream_init(
            &stream,
            &robot,
            &request,
            &streamWorkspace
        )
    )
    {
        printf(
            "Stream init status: %s\n",
            single_line_stream_status_string(
                single_line_stream_status(
                    &stream
                )
            )
        );

        fail(
            "stream initialization failed"
        );
    }


    if (
        single_line_stream_sample_count(
            &stream
        ) !=
        legacyTrajectory.count
    )
    {
        printf(
            "Legacy samples: %zu\n"
            "Stream samples: %zu\n",
            legacyTrajectory.count,
            single_line_stream_sample_count(
                &stream
            )
        );

        fail(
            "sample-count mismatch"
        );
    }


    /* ========================================================================
     * COMPARE EVERY GENERATED SAMPLE
     * ========================================================================
     */

    real_t maxTimeError =
        0.0;

    real_t maxSError =
        0.0;

    real_t maxPositionComponentError =
        0.0;

    real_t maxQuaternionComponentError =
        0.0;

    real_t maxJointError =
        0.0;

    real_t maxFKPositionErrorDifference =
        0.0;

    real_t maxFKOrientationErrorDifference =
        0.0;


    size_t generated =
        0;


    SingleLineStreamSample sample;


    while (
        single_line_stream_next(
            &stream,
            &sample
        )
    )
    {
        size_t i =
            sample.index;


        maxTimeError =
            max_value(
                maxTimeError,
                fabs(
                    sample.t -
                    legacyTrajectory.t[i]
                )
            );


        maxSError =
            max_value(
                maxSError,
                fabs(
                    sample.s -
                    legacyTrajectory.s[i]
                )
            );


        for (int axis = 0;
             axis < 3;
             axis++)
        {
            maxPositionComponentError =
                max_value(
                    maxPositionComponentError,
                    fabs(
                        sample.pDesired.v[axis] -
                        legacyTrajectory
                            .pDesired[i]
                            .v[axis]
                    )
                );
        }


        real_t quatErrors[4] =
        {
            fabs(
                sample.quatDesired.w -
                legacyTrajectory.quatDesired[i].w
            ),
            fabs(
                sample.quatDesired.x -
                legacyTrajectory.quatDesired[i].x
            ),
            fabs(
                sample.quatDesired.y -
                legacyTrajectory.quatDesired[i].y
            ),
            fabs(
                sample.quatDesired.z -
                legacyTrajectory.quatDesired[i].z
            )
        };


        for (int component = 0;
             component < 4;
             component++)
        {
            maxQuaternionComponentError =
                max_value(
                    maxQuaternionComponentError,
                    quatErrors[component]
                );
        }


        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            maxJointError =
                max_value(
                    maxJointError,
                    fabs(
                        sample.q.q[joint] -
                        legacyTrajectory
                            .qPath[i]
                            .q[joint]
                    )
                );
        }


        maxFKPositionErrorDifference =
            max_value(
                maxFKPositionErrorDifference,
                fabs(
                    sample.positionError -
                    legacyTrajectory.positionError[i]
                )
            );


        maxFKOrientationErrorDifference =
            max_value(
                maxFKOrientationErrorDifference,
                fabs(
                    sample.orientationError -
                    legacyTrajectory.orientationError[i]
                )
            );


        if (
            sample.ikIterations !=
            legacyTrajectory.ikIterations[i]
        )
        {
            fail(
                "IK iteration-count mismatch"
            );
        }


        generated++;
    }


    if (
        !single_line_stream_is_finished(
            &stream
        )
    )
    {
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


    if (
        generated !=
        legacyTrajectory.count
    )
    {
        fail(
            "generated sample count mismatch"
        );
    }


    const real_t tolerance =
        1e-12;


    if (
        maxTimeError > tolerance ||
        maxSError > tolerance ||
        maxPositionComponentError > tolerance ||
        maxQuaternionComponentError > tolerance ||
        maxJointError > tolerance ||
        maxFKPositionErrorDifference > tolerance ||
        maxFKOrientationErrorDifference > tolerance
    )
    {
        fail(
            "streaming line pipeline differs from legacy planner"
        );
    }


    printf(
        "\nSTREAMING SINGLE-LINE PIPELINE TEST PASSED\n"
    );

    printf(
        "Samples:                    %zu\n",
        generated
    );

    printf(
        "Duration:                   %.9f s\n",
        stream.profile.info.T
    );

    printf(
        "Max time error:             %.3e\n",
        maxTimeError
    );

    printf(
        "Max s error:                %.3e\n",
        maxSError
    );

    printf(
        "Max Cartesian component err:%.3e\n",
        maxPositionComponentError
    );

    printf(
        "Max quaternion component err:%.3e\n",
        maxQuaternionComponentError
    );

    printf(
        "Max joint error:            %.3e rad\n",
        maxJointError
    );

    printf(
        "Max FK position diff:       %.3e m\n",
        maxFKPositionErrorDifference
    );

    printf(
        "Max FK orientation diff:    %.3e rad\n",
        maxFKOrientationErrorDifference
    );

    printf(
        "Peak TCP speed:             %.6f m/s\n",
        stream.peakTCPSpeed
    );

    printf(
        "Max IK iterations:          %d\n\n",
        stream.maxIKIterations
    );


    return 0;
}
