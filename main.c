#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "robot_config.h"
#include "single_segment_line_stream.h"
#include "single_segment_circular_stream.h"
#include "trajectory_buffer.h"
#include "math3d.h"

#include "ethercat_master.h"
#include "cia402.h"
#include "a6ec_drive.h"

/* ============================================================================
 * LIVE CONTROLLER CONFIGURATION
 * ============================================================================
 */

#define MATLAB_PORT     5005
#define MATLAB_IP       "172.18.160.1"

#define HMI_PORT        5006
#define HMI_STATUS_PORT 5007
#define HMI_STATUS_IP   "127.0.0.1"

#define NUM_AXES        6
#define CYCLE_TIME_NS   1000000U

#define HMI_PACKET_MAGIC                     0x52425432U /* RBT2 */
#define HMI_COMMAND_PLAN_AND_RUN_LINE        1U
#define HMI_COMMAND_STOP                     2U
#define HMI_COMMAND_PLAN_AND_RUN_ARC         3U
#define HMI_COMMAND_PLAN_AND_RUN_FULL_CIRCLE 4U

#define HMI_POSE_VALUES                 6U
#define HMI_LINE_PLAN_WORD_COUNT       18U
#define HMI_CIRCULAR_PLAN_WORD_COUNT   24U
#define HMI_MAX_PACKET_WORD_COUNT      HMI_CIRCULAR_PLAN_WORD_COUNT
#define HMI_STOP_WORD_COUNT             3U

#define HMI_STATUS_PACKET_MAGIC 0x53544132U /* STA2 */
#define HMI_STATUS_WORD_COUNT   13U

#define GEOMETRY_CAPACITY          512U
#define TRAJECTORY_BUFFER_CAPACITY 256U
#define TRAJECTORY_PREFILL_SAMPLES 128U

/* ============================================================================
 * LIVE PATH / MOTION STATE
 * ============================================================================
 */

typedef enum
{
    LIVE_PATH_LINE = 0,
    LIVE_PATH_ARC,
    LIVE_PATH_FULL_CIRCLE
} LivePathType;


typedef enum
{
    MOTION_WAITING_FOR_PLAN = 0,
    MOTION_PLANNING,
    MOTION_PREPOSITION,
    MOTION_RUNNING,
    MOTION_FINISHED,
    MOTION_HOLD
} MotionState;


typedef struct
{
    LivePathType pathType;
    uint32_t sequence;

    float waypointA[HMI_POSE_VALUES];
    float waypointB[HMI_POSE_VALUES];
    float waypointC[HMI_POSE_VALUES];

    float tcpSpeed;
    float tcpAccel;
    float tcpJerk;

    JointVector qSeed;
} TrajectoryPlanCommand;


typedef struct
{
    bool positionLimitsPass;
    bool velocityLimitsPass;

    size_t samples;
    real_t duration;
    real_t pathLength;
    real_t peakTCPSpeed;
    real_t maxPositionError;
    real_t maxOrientationError;
    int maxIKIterations;

    real_t peakJointVelocity[ROBOT_DOF];
    real_t maxJointStep[ROBOT_DOF];
} StreamingPlanReport;


typedef struct
{
    real_t t;
    real_t tcpSpeed;
    JointVector q;
    int ikIterations;
    real_t positionError;
    real_t orientationError;
} LiveStreamSample;


typedef struct
{
    LivePathType type;
    SingleLineStream line;
    SingleCircularStream circular;
} LiveTrajectoryStream;

/* ============================================================================
 * CONTROLCORE STORAGE
 * ============================================================================
 */

static RobotConfig trajectory_robot;

static Vec3 trajectory_raw_geometry[GEOMETRY_CAPACITY];
static Vec3 trajectory_arc_geometry[GEOMETRY_CAPACITY];
static real_t trajectory_l_original[GEOMETRY_CAPACITY];
static real_t trajectory_l_arc[GEOMETRY_CAPACITY];
static ADLSInfo trajectory_ik_scratch;

static SingleLineStreamWorkspace line_stream_workspace;
static SingleCircularStreamWorkspace circular_stream_workspace;
static LiveTrajectoryStream execution_stream;

static JointVector trajectory_buffer_storage[TRAJECTORY_BUFFER_CAPACITY];
static TrajectoryBuffer trajectory_buffer;
static StreamingPlanReport trajectory_report;

static volatile size_t trajectory_index = 0;
static volatile size_t trajectory_total_samples = 0;
static volatile size_t trajectory_executed_samples = 0;

static volatile bool trajectory_ready = false;
static volatile bool trajectory_generation_finished = false;
static volatile bool planner_cancel_requested = false;
static volatile bool trajectory_underrun = false;

static JointVector trajectory_start_sample;
static volatile bool trajectory_have_start_sample = false;

static JointVector trajectory_final_sample;
static volatile bool trajectory_have_final_sample = false;

static volatile MotionState motion_state = MOTION_WAITING_FOR_PLAN;
static volatile bool hmi_hold_requested = true;
static volatile bool planner_busy = false;
static volatile LivePathType active_path_type = LIVE_PATH_LINE;

static QueueHandle_t trajectory_request_queue = NULL;
static volatile sig_atomic_t stop_requested = 0;

/* ============================================================================
 * SMALL HELPERS
 * ============================================================================
 */

 static double wrap_to_pi(double angle)
{
    while (angle > ROBOT_PI)
        angle -= 2.0 * ROBOT_PI;

    while (angle < -ROBOT_PI)
        angle += 2.0 * ROBOT_PI;

    return angle;
}

static void handle_sigint(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}


static real_t degrees_to_radians_main(real_t degrees)
{
    return degrees * ROBOT_PI / 180.0;
}


static float network_word_to_float(uint32_t network_word)
{
    uint32_t host_bits = ntohl(network_word);
    float value;
    memcpy(&value, &host_bits, sizeof(value));
    return value;
}


static const char *motion_state_name(MotionState state)
{
    switch (state)
    {
        case MOTION_WAITING_FOR_PLAN: return "WAITING";
        case MOTION_PLANNING:         return "PLANNING";
        case MOTION_PREPOSITION:      return "PREPOSITION";
        case MOTION_RUNNING:          return "RUNNING";
        case MOTION_FINISHED:         return "FINISHED";
        case MOTION_HOLD:             return "HOLD";
        default:                      return "UNKNOWN";
    }
}


static const char *live_path_name(LivePathType type)
{
    switch (type)
    {
        case LIVE_PATH_LINE:        return "STRAIGHT LINE";
        case LIVE_PATH_ARC:         return "CIRCULAR ARC";
        case LIVE_PATH_FULL_CIRCLE: return "FULL CIRCLE";
        default:                    return "UNKNOWN";
    }
}


static Quat pose_orientation_from_hmi(const float pose[HMI_POSE_VALUES])
{
    Mat3 rotation =
        eul_zyx(
            degrees_to_radians_main((real_t)pose[3]),
            degrees_to_radians_main((real_t)pose[4]),
            degrees_to_radians_main((real_t)pose[5])
        );

    return rotm_to_quat(rotation);
}


static Vec3 pose_position_from_hmi(const float pose[HMI_POSE_VALUES])
{
    return (Vec3)
    {{
        (real_t)pose[0],
        (real_t)pose[1],
        (real_t)pose[2]
    }};
}

/* ============================================================================
 * GENERIC LINE / CIRCULAR STREAM ADAPTER
 * ============================================================================
 */

static bool live_stream_init(
    LiveTrajectoryStream *stream,
    const TrajectoryPlanCommand *command
)
{
    if (stream == NULL || command == NULL)
    {
        return false;
    }

    memset(stream, 0, sizeof(*stream));
    stream->type = command->pathType;

    if (command->pathType == LIVE_PATH_LINE)
    {
        SingleLineRequest request;
        memset(&request, 0, sizeof(request));

        request.qSeed =
    command.qSeed;

for (int joint = 0; joint < ROBOT_DOF; joint++)
{
    request.qSeed.q[joint] =
        wrap_to_pi(request.qSeed.q[joint]);
}
        request.startPosition = pose_position_from_hmi(command->waypointA);
        request.endPosition = pose_position_from_hmi(command->waypointB);
        request.startOrientation = pose_orientation_from_hmi(command->waypointA);
        request.endOrientation = pose_orientation_from_hmi(command->waypointB);
        request.numGeometryPointsPerSegment = 100;
        request.arcLengthSpacing = 0.005;
        request.desiredTCPSpeed = (real_t)command->tcpSpeed;
        request.desiredTCPAccel = (real_t)command->tcpAccel;
        request.desiredTCPJerk = (real_t)command->tcpJerk;
        request.dt = 0.001;
        adls_default_parameters(&request.ikParameters);

        return
            single_line_stream_init(
                &stream->line,
                &trajectory_robot,
                &request,
                &line_stream_workspace
            );
    }

    SingleCircularRequest request;
    memset(&request, 0, sizeof(request));

    request.type =
        command->pathType == LIVE_PATH_ARC
            ? CIRCULAR_SEGMENT_ARC
            : CIRCULAR_SEGMENT_FULL_CIRCLE;

    request.qSeed =
    command.qSeed;

for (int joint = 0; joint < ROBOT_DOF; joint++)
{
    request.qSeed.q[joint] =
        wrap_to_pi(request.qSeed.q[joint]);
}
    request.point1 = pose_position_from_hmi(command->waypointA);
    request.point2 = pose_position_from_hmi(command->waypointB);
    request.point3 = pose_position_from_hmi(command->waypointC);
    request.direction = +1;
    request.startOrientation = pose_orientation_from_hmi(command->waypointA);
    request.endOrientation = pose_orientation_from_hmi(command->waypointC);
    request.numGeometryPointsPerSegment =
        command->pathType == LIVE_PATH_ARC ? 200U : 400U;
    request.arcLengthSpacing = 0.005;
    request.desiredTCPSpeed = (real_t)command->tcpSpeed;
    request.desiredTCPAccel = (real_t)command->tcpAccel;
    request.desiredTCPJerk = (real_t)command->tcpJerk;
    request.dt = 0.001;
    adls_default_parameters(&request.ikParameters);

    return
        single_circular_stream_init(
            &stream->circular,
            &trajectory_robot,
            &request,
            &circular_stream_workspace
        );
}


static size_t live_stream_sample_count(const LiveTrajectoryStream *stream)
{
    if (stream == NULL) return 0;

    return
        stream->type == LIVE_PATH_LINE
            ? single_line_stream_sample_count(&stream->line)
            : single_circular_stream_sample_count(&stream->circular);
}


static size_t live_stream_samples_generated(const LiveTrajectoryStream *stream)
{
    if (stream == NULL) return 0;

    return
        stream->type == LIVE_PATH_LINE
            ? single_line_stream_samples_generated(&stream->line)
            : single_circular_stream_samples_generated(&stream->circular);
}


static real_t live_stream_duration(const LiveTrajectoryStream *stream)
{
    if (stream == NULL) return 0.0;

    return
        stream->type == LIVE_PATH_LINE
            ? stream->line.profile.info.T
            : stream->circular.profile.info.T;
}


static real_t live_stream_path_length(const LiveTrajectoryStream *stream)
{
    if (stream == NULL) return 0.0;

    return
        stream->type == LIVE_PATH_LINE
            ? stream->line.segmentLength
            : stream->circular.segmentLength;
}


static bool live_stream_is_finished(const LiveTrajectoryStream *stream)
{
    if (stream == NULL) return false;

    return
        stream->type == LIVE_PATH_LINE
            ? single_line_stream_is_finished(&stream->line)
            : single_circular_stream_is_finished(&stream->circular);
}


static const char *live_stream_status_string(const LiveTrajectoryStream *stream)
{
    if (stream == NULL) return "INVALID_ARGUMENT";

    return
        stream->type == LIVE_PATH_LINE
            ? single_line_stream_status_string(
                  single_line_stream_status(&stream->line)
              )
            : single_circular_stream_status_string(
                  single_circular_stream_status(&stream->circular)
              );
}


static bool live_stream_next(
    LiveTrajectoryStream *stream,
    LiveStreamSample *sample
)
{
    if (stream == NULL || sample == NULL)
    {
        return false;
    }

    memset(sample, 0, sizeof(*sample));

    if (stream->type == LIVE_PATH_LINE)
    {
        SingleLineStreamSample line_sample;

        if (!single_line_stream_next(&stream->line, &line_sample))
        {
            return false;
        }

        sample->t = line_sample.t;
        sample->tcpSpeed = line_sample.tcpSpeed;
        sample->q = line_sample.q;
        sample->ikIterations = line_sample.ikIterations;
        sample->positionError = line_sample.positionError;
        sample->orientationError = line_sample.orientationError;
        return true;
    }

    SingleCircularStreamSample circular_sample;

    if (!single_circular_stream_next(&stream->circular, &circular_sample))
    {
        return false;
    }

    sample->t = circular_sample.t;
    sample->tcpSpeed = circular_sample.tcpSpeed;
    sample->q = circular_sample.q;
    sample->ikIterations = circular_sample.ikIterations;
    sample->positionError = circular_sample.positionError;
    sample->orientationError = circular_sample.orientationError;
    return true;
}

/* ============================================================================
 * HMI STATUS + COMMAND RECEIVE
 * ============================================================================
 */

static void send_hmi_status(
    int socket_fd,
    const struct sockaddr_in *status_address,
    int wkc,
    int expected_wkc,
    uint32_t last_sequence
)
{
    if (socket_fd < 0 || status_address == NULL)
    {
        return;
    }

    uint32_t packet[HMI_STATUS_WORD_COUNT];

    packet[0] = htonl(HMI_STATUS_PACKET_MAGIC);
    packet[1] = htonl((uint32_t)motion_state);
    packet[2] = htonl(last_sequence);
    packet[3] = htonl((uint32_t)trajectory_index);
    packet[4] = htonl((uint32_t)trajectory_total_samples);
    packet[5] = htonl((uint32_t)wkc);
    packet[6] = htonl((uint32_t)expected_wkc);

    for (int slave = 1; slave <= NUM_AXES; slave++)
    {
        A6ECPDOFeedback feedback;
        a6ec_read_feedback(slave, &feedback);
        packet[7 + (slave - 1)] = htonl((uint32_t)feedback.statusword);
    }

    sendto(
        socket_fd,
        packet,
        sizeof(packet),
        MSG_DONTWAIT,
        (const struct sockaddr *)status_address,
        sizeof(*status_address)
    );
}


static bool hmi_plan_values_valid(const TrajectoryPlanCommand *plan)
{
    if (plan == NULL) return false;

    for (int i = 0; i < (int)HMI_POSE_VALUES; i++)
    {
        if (
            !isfinite(plan->waypointA[i]) ||
            !isfinite(plan->waypointB[i]) ||
            !isfinite(plan->waypointC[i])
        )
        {
            return false;
        }
    }

    return
        isfinite(plan->tcpSpeed) &&
        isfinite(plan->tcpAccel) &&
        isfinite(plan->tcpJerk) &&
        plan->tcpSpeed > 0.0f &&
        plan->tcpAccel > 0.0f &&
        plan->tcpJerk > 0.0f;
}


static void reset_for_new_plan(void)
{
    planner_cancel_requested = false;
    trajectory_buffer_reset(&trajectory_buffer);
    trajectory_ready = false;
    trajectory_generation_finished = false;
    trajectory_underrun = false;
    trajectory_have_start_sample = false;
    trajectory_have_final_sample = false;
    trajectory_total_samples = 0;
    trajectory_executed_samples = 0;
    trajectory_index = 0;
    hmi_hold_requested = false;
    planner_busy = true;
    motion_state = MOTION_PLANNING;
}


static void print_received_plan(const TrajectoryPlanCommand *plan)
{
    printf(
        "\n"
        "============================================================\n"
        " HMI %s PLAN RECEIVED #%u\n"
        "============================================================\n",
        live_path_name(plan->pathType),
        plan->sequence
    );

    printf(
        "A: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n",
        plan->waypointA[0], plan->waypointA[1], plan->waypointA[2],
        plan->waypointA[3], plan->waypointA[4], plan->waypointA[5]
    );

    if (plan->pathType == LIVE_PATH_LINE)
    {
        printf(
            "B: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n",
            plan->waypointB[0], plan->waypointB[1], plan->waypointB[2],
            plan->waypointB[3], plan->waypointB[4], plan->waypointB[5]
        );
    }
    else
    {
        printf(
            "B: p=[%.6f %.6f %.6f] m | via/geometry point\n",
            plan->waypointB[0], plan->waypointB[1], plan->waypointB[2]
        );

        printf(
            "C: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n",
            plan->waypointC[0], plan->waypointC[1], plan->waypointC[2],
            plan->waypointC[3], plan->waypointC[4], plan->waypointC[5]
        );
    }

    printf(
        "TCP: speed=%.4f m/s | accel=%.4f m/s^2 | jerk=%.4f m/s^3\n"
        "Controller: HOLD while ControlCore validates and buffers\n"
        "============================================================\n",
        plan->tcpSpeed,
        plan->tcpAccel,
        plan->tcpJerk
    );

    fflush(stdout);
}


static void receive_hmi_commands(
    int command_socket,
    const JointVector *current_q,
    uint32_t *last_sequence
)
{
    if (command_socket < 0 || current_q == NULL || last_sequence == NULL)
    {
        return;
    }

    for (;;)
    {
        uint32_t packet[HMI_MAX_PACKET_WORD_COUNT];

        ssize_t received =
            recvfrom(
                command_socket,
                packet,
                sizeof(packet),
                MSG_DONTWAIT,
                NULL,
                NULL
            );

        if (received < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }

            perror("HMI recvfrom");
            break;
        }

        if (received < (ssize_t)(HMI_STOP_WORD_COUNT * sizeof(uint32_t)))
        {
            printf("Ignoring HMI packet that is too short: %zd bytes\n", received);
            continue;
        }

        uint32_t magic = ntohl(packet[0]);
        uint32_t command = ntohl(packet[1]);
        uint32_t sequence = ntohl(packet[2]);

        if (magic != HMI_PACKET_MAGIC)
        {
            printf("Ignoring HMI packet with invalid magic\n");
            continue;
        }

        *last_sequence = sequence;

        if (command == HMI_COMMAND_STOP)
        {
            const ssize_t expected =
                (ssize_t)(HMI_STOP_WORD_COUNT * sizeof(uint32_t));

            if (received != expected)
            {
                printf(
                    "Ignoring STOP packet with unexpected size: %zd bytes (expected %zd)\n",
                    received,
                    expected
                );
                continue;
            }

            taskENTER_CRITICAL();
            hmi_hold_requested = true;
            planner_cancel_requested = true;
            trajectory_ready = false;
            trajectory_generation_finished = false;
            trajectory_buffer_reset(&trajectory_buffer);
            motion_state = MOTION_HOLD;
            taskEXIT_CRITICAL();

            printf(
                "HMI command #%u: STOP -> HOLD CURRENT POSITION\n",
                sequence
            );
            fflush(stdout);
            continue;
        }

        bool is_line = command == HMI_COMMAND_PLAN_AND_RUN_LINE;
        bool is_arc = command == HMI_COMMAND_PLAN_AND_RUN_ARC;
        bool is_circle = command == HMI_COMMAND_PLAN_AND_RUN_FULL_CIRCLE;

        if (!is_line && !is_arc && !is_circle)
        {
            printf(
                "Ignoring unknown HMI command %u (sequence %u)\n",
                command,
                sequence
            );
            fflush(stdout);
            continue;
        }

        const size_t expected_words =
            is_line
                ? HMI_LINE_PLAN_WORD_COUNT
                : HMI_CIRCULAR_PLAN_WORD_COUNT;

        const ssize_t expected_bytes =
            (ssize_t)(expected_words * sizeof(uint32_t));

        if (received != expected_bytes)
        {
            printf(
                "Ignoring PLAN packet with unexpected size: %zd bytes (expected %zd)\n",
                received,
                expected_bytes
            );
            continue;
        }

        MotionState state = motion_state;

        if (
            planner_busy ||
            state == MOTION_PLANNING ||
            state == MOTION_PREPOSITION ||
            state == MOTION_RUNNING
        )
        {
            printf(
                "HMI PLAN #%u rejected: controller is %s. Press STOP before submitting a new path.\n",
                sequence,
                motion_state_name(state)
            );
            fflush(stdout);
            continue;
        }

        TrajectoryPlanCommand plan;
        memset(&plan, 0, sizeof(plan));

        plan.sequence = sequence;
        plan.pathType =
            is_line
                ? LIVE_PATH_LINE
                : is_arc
                    ? LIVE_PATH_ARC
                    : LIVE_PATH_FULL_CIRCLE;

        int index = 3;

        for (int i = 0; i < (int)HMI_POSE_VALUES; i++)
        {
            plan.waypointA[i] = network_word_to_float(packet[index++]);
        }

        for (int i = 0; i < (int)HMI_POSE_VALUES; i++)
        {
            plan.waypointB[i] = network_word_to_float(packet[index++]);
        }

        if (is_line)
        {
            memcpy(plan.waypointC, plan.waypointB, sizeof(plan.waypointC));
        }
        else
        {
            for (int i = 0; i < (int)HMI_POSE_VALUES; i++)
            {
                plan.waypointC[i] = network_word_to_float(packet[index++]);
            }
        }

        plan.tcpSpeed = network_word_to_float(packet[index++]);
        plan.tcpAccel = network_word_to_float(packet[index++]);
        plan.tcpJerk = network_word_to_float(packet[index++]);
        plan.qSeed = *current_q;

        if (!hmi_plan_values_valid(&plan))
        {
            printf(
                "HMI PLAN #%u rejected: invalid Cartesian/profile values\n",
                sequence
            );
            fflush(stdout);
            continue;
        }

        taskENTER_CRITICAL();
        reset_for_new_plan();
        active_path_type = plan.pathType;
        taskEXIT_CRITICAL();

        if (
            xQueueSend(
                trajectory_request_queue,
                &plan,
                0
            ) != pdPASS
        )
        {
            motion_state = MOTION_WAITING_FOR_PLAN;
            hmi_hold_requested = true;
            planner_busy = false;

            printf(
                "HMI PLAN #%u rejected: planner queue is busy\n",
                sequence
            );
            fflush(stdout);
            continue;
        }

        print_received_plan(&plan);
    }
}

/* ============================================================================
 * STREAMING VALIDATION / PRODUCER
 * ============================================================================
 */

static bool planner_is_cancelled(void)
{
    bool cancelled;

    taskENTER_CRITICAL();
    cancelled = planner_cancel_requested || stop_requested;
    taskEXIT_CRITICAL();

    return cancelled;
}


static void update_velocity_validation(
    StreamingPlanReport *report,
    int joint,
    real_t velocity
)
{
    real_t magnitude = fabs(velocity);

    if (magnitude > report->peakJointVelocity[joint])
    {
        report->peakJointVelocity[joint] = magnitude;
    }

    if (magnitude > trajectory_robot.limits.qdMax[joint])
    {
        report->velocityLimitsPass = false;
    }
}


static bool validate_streaming_trajectory(
    const TrajectoryPlanCommand *command,
    StreamingPlanReport *report
)
{
    if (command == NULL || report == NULL)
    {
        return false;
    }

    memset(report, 0, sizeof(*report));
    report->positionLimitsPass = true;
    report->velocityLimitsPass = true;

    LiveTrajectoryStream validation_stream;

    if (!live_stream_init(&validation_stream, command))
    {
        printf(
            "Streaming validation init failed for %s: %s\n",
            live_path_name(command->pathType),
            live_stream_status_string(&validation_stream)
        );
        return false;
    }

    report->samples = live_stream_sample_count(&validation_stream);
    report->duration = live_stream_duration(&validation_stream);
    report->pathLength = live_stream_path_length(&validation_stream);

    LiveStreamSample previous_previous;
    LiveStreamSample previous;
    bool have_previous_previous = false;
    bool have_previous = false;
    size_t generated = 0;

    for (;;)
    {
        if (planner_is_cancelled())
        {
            return false;
        }

        LiveStreamSample sample;

        if (!live_stream_next(&validation_stream, &sample))
        {
            if (live_stream_is_finished(&validation_stream))
            {
                break;
            }

            printf(
                "Streaming validation stopped for %s: %s\n",
                live_path_name(command->pathType),
                live_stream_status_string(&validation_stream)
            );
            return false;
        }

        generated++;

        if (sample.tcpSpeed > report->peakTCPSpeed)
            report->peakTCPSpeed = sample.tcpSpeed;

        if (sample.positionError > report->maxPositionError)
            report->maxPositionError = sample.positionError;

        if (sample.orientationError > report->maxOrientationError)
            report->maxOrientationError = sample.orientationError;

        if (sample.ikIterations > report->maxIKIterations)
            report->maxIKIterations = sample.ikIterations;

        for (int joint = 0; joint < ROBOT_DOF; joint++)
        {
            if (
                sample.q.q[joint] < trajectory_robot.limits.qMin[joint] ||
                sample.q.q[joint] > trajectory_robot.limits.qMax[joint]
            )
            {
                report->positionLimitsPass = false;
            }

            if (have_previous)
            {
                real_t step =
                    fabs(sample.q.q[joint] - previous.q.q[joint]);

                if (step > report->maxJointStep[joint])
                    report->maxJointStep[joint] = step;
            }
        }

        if (have_previous && !have_previous_previous)
        {
            real_t dt_first = sample.t - previous.t;

            if (fabs(dt_first) > 1e-15)
            {
                for (int joint = 0; joint < ROBOT_DOF; joint++)
                {
                    update_velocity_validation(
                        report,
                        joint,
                        (sample.q.q[joint] - previous.q.q[joint]) / dt_first
                    );
                }
            }
        }
        else if (have_previous && have_previous_previous)
        {
            real_t dt_central = sample.t - previous_previous.t;

            if (fabs(dt_central) > 1e-15)
            {
                for (int joint = 0; joint < ROBOT_DOF; joint++)
                {
                    update_velocity_validation(
                        report,
                        joint,
                        (sample.q.q[joint] - previous_previous.q.q[joint]) /
                            dt_central
                    );
                }
            }
        }

        if (have_previous)
        {
            previous_previous = previous;
            have_previous_previous = true;
        }

        previous = sample;
        have_previous = true;
    }

    if (generated != report->samples)
    {
        return false;
    }

    if (have_previous && have_previous_previous)
    {
        real_t dt_last = previous.t - previous_previous.t;

        if (fabs(dt_last) > 1e-15)
        {
            for (int joint = 0; joint < ROBOT_DOF; joint++)
            {
                update_velocity_validation(
                    report,
                    joint,
                    (previous.q.q[joint] - previous_previous.q.q[joint]) /
                        dt_last
                );
            }
        }
    }

    return
        report->samples > 0 &&
        report->positionLimitsPass &&
        report->velocityLimitsPass;
}


static void abort_streaming_plan(bool preserve_hold_state)
{
    taskENTER_CRITICAL();

    trajectory_buffer_reset(&trajectory_buffer);
    trajectory_ready = false;
    trajectory_generation_finished = false;
    trajectory_have_start_sample = false;
    trajectory_have_final_sample = false;
    trajectory_total_samples = 0;
    trajectory_executed_samples = 0;
    trajectory_index = 0;
    hmi_hold_requested = true;
    planner_busy = false;
    motion_state =
        preserve_hold_state
            ? MOTION_HOLD
            : MOTION_WAITING_FOR_PLAN;

    taskEXIT_CRITICAL();
}


static void print_validated_plan(
    const TrajectoryPlanCommand *command,
    size_t queued_samples
)
{
    printf(
        "\n"
        "============================================================\n"
        " CONTROLCORE %s PLAN #%u VALIDATED + BUFFERED\n"
        "============================================================\n"
        "Waypoint A:        [%.6f %.6f %.6f] m\n",
        live_path_name(command->pathType),
        command->sequence,
        command->waypointA[0],
        command->waypointA[1],
        command->waypointA[2]
    );

    if (command->pathType == LIVE_PATH_LINE)
    {
        printf(
            "Waypoint B:        [%.6f %.6f %.6f] m\n",
            command->waypointB[0],
            command->waypointB[1],
            command->waypointB[2]
        );
    }
    else
    {
        printf(
            "Waypoint B/via:    [%.6f %.6f %.6f] m\n"
            "Waypoint C:        [%.6f %.6f %.6f] m\n",
            command->waypointB[0],
            command->waypointB[1],
            command->waypointB[2],
            command->waypointC[0],
            command->waypointC[1],
            command->waypointC[2]
        );
    }

    printf(
        "Total samples:     %zu\n"
        "Duration:          %.6f s\n"
        "Path length:       %.6f m\n"
        "Peak TCP speed:    %.6f m/s\n"
        "Max pos error:     %.6e m\n"
        "Max rot error:     %.6e rad\n"
        "Joint pos limits:  PASS\n"
        "Joint vel limits:  PASS\n"
        "Execution buffer:  %zu / %u samples\n"
        "Next state:        %s\n"
        "============================================================\n",
        trajectory_report.samples,
        trajectory_report.duration,
        trajectory_report.pathLength,
        trajectory_report.peakTCPSpeed,
        trajectory_report.maxPositionError,
        trajectory_report.maxOrientationError,
        queued_samples,
        (unsigned)TRAJECTORY_BUFFER_CAPACITY,
        motion_state_name(motion_state)
    );

    fflush(stdout);
}


static void TrajectoryPlannerTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        TrajectoryPlanCommand command;

        if (
            xQueueReceive(
                trajectory_request_queue,
                &command,
                portMAX_DELAY
            ) != pdTRUE
        )
        {
            continue;
        }

        printf(
            "\nValidating HMI %s #%u with streaming ControlCore...\n",
            live_path_name(command.pathType),
            command.sequence
        );
        fflush(stdout);

        bool validation_success =
            validate_streaming_trajectory(
                &command,
                &trajectory_report
            );

        if (planner_is_cancelled())
        {
            abort_streaming_plan(true);
            printf("Planner #%u cancelled -> HOLD\n", command.sequence);
            fflush(stdout);
            continue;
        }

        if (!validation_success)
        {
            printf(
                "\n"
                "============================================================\n"
                " CONTROLCORE %s PLAN #%u FAILED VALIDATION\n"
                "============================================================\n"
                "The robot remains in HOLD.\n"
                "Joint position limits: %s\n"
                "Joint velocity limits: %s\n"
                "Check circular geometry, reachability, orientation, IK and limits.\n"
                "============================================================\n",
                live_path_name(command.pathType),
                command.sequence,
                trajectory_report.positionLimitsPass ? "PASS" : "FAIL",
                trajectory_report.velocityLimitsPass ? "PASS" : "FAIL"
            );
            fflush(stdout);
            abort_streaming_plan(false);
            continue;
        }

        if (!live_stream_init(&execution_stream, &command))
        {
            printf(
                "Execution stream init failed for %s: %s\n",
                live_path_name(command.pathType),
                live_stream_status_string(&execution_stream)
            );
            abort_streaming_plan(false);
            continue;
        }

        const size_t total_samples =
            live_stream_sample_count(&execution_stream);

        taskENTER_CRITICAL();
        trajectory_total_samples = total_samples;
        trajectory_executed_samples = 0;
        trajectory_index = 0;
        trajectory_generation_finished = false;
        trajectory_have_start_sample = false;
        trajectory_have_final_sample = false;
        trajectory_ready = false;
        trajectory_underrun = false;
        taskEXIT_CRITICAL();

        bool ready_announced = false;
        bool producer_failed = false;

        for (;;)
        {
            if (planner_is_cancelled())
            {
                break;
            }

            bool buffer_full;

            taskENTER_CRITICAL();
            buffer_full = trajectory_buffer_is_full(&trajectory_buffer);
            taskEXIT_CRITICAL();

            if (buffer_full)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            LiveStreamSample generated_sample;

            if (!live_stream_next(&execution_stream, &generated_sample))
            {
                if (live_stream_is_finished(&execution_stream))
                {
                    break;
                }

                printf(
                    "Execution stream failed for %s: %s\n",
                    live_path_name(command.pathType),
                    live_stream_status_string(&execution_stream)
                );
                producer_failed = true;
                break;
            }

            bool generated_final_sample =
                live_stream_samples_generated(&execution_stream) >= total_samples;

            bool publish_ready = false;
            size_t queued_samples = 0;

            taskENTER_CRITICAL();

            if (planner_cancel_requested || stop_requested)
            {
                taskEXIT_CRITICAL();
                break;
            }

            if (!trajectory_have_start_sample)
            {
                trajectory_start_sample = generated_sample.q;
                trajectory_have_start_sample = true;
            }

            if (!trajectory_buffer_push(&trajectory_buffer, &generated_sample.q))
            {
                taskEXIT_CRITICAL();
                producer_failed = true;
                printf("ERROR: trajectory buffer push failed unexpectedly\n");
                break;
            }

            if (generated_final_sample)
            {
                trajectory_final_sample = generated_sample.q;
                trajectory_have_final_sample = true;
                trajectory_generation_finished = true;
            }

            queued_samples = trajectory_buffer_count(&trajectory_buffer);

            if (
                !trajectory_ready &&
                trajectory_have_start_sample &&
                (
                    queued_samples >= TRAJECTORY_PREFILL_SAMPLES ||
                    trajectory_generation_finished
                )
            )
            {
                trajectory_ready = true;
                publish_ready = true;

                if (!hmi_hold_requested && motion_state != MOTION_HOLD)
                {
                    motion_state = MOTION_PREPOSITION;
                }
            }

            taskEXIT_CRITICAL();

            if (publish_ready && !ready_announced)
            {
                ready_announced = true;
                print_validated_plan(&command, queued_samples);
            }
        }

        if (planner_is_cancelled())
        {
            abort_streaming_plan(true);
            printf("Planner #%u cancelled -> HOLD\n", command.sequence);
            fflush(stdout);
            continue;
        }

        if (producer_failed)
        {
            printf(
                "\n"
                "============================================================\n"
                " CONTROLCORE PRODUCER #%u FAILED\n"
                "============================================================\n"
                "The robot is forced to HOLD.\n"
                "============================================================\n",
                command.sequence
            );
            fflush(stdout);
            abort_streaming_plan(false);
            continue;
        }

        taskENTER_CRITICAL();

        if (
            trajectory_generation_finished &&
            !trajectory_ready &&
            trajectory_have_start_sample
        )
        {
            trajectory_ready = true;

            if (!hmi_hold_requested && motion_state != MOTION_HOLD)
            {
                motion_state = MOTION_PREPOSITION;
            }
        }

        planner_busy = false;
        taskEXIT_CRITICAL();

        printf(
            "%s producer #%u complete: %zu samples generated.\n",
            live_path_name(command.pathType),
            command.sequence,
            total_samples
        );
        fflush(stdout);
    }
}

/* ============================================================================
 * STORAGE INITIALIZATION
 * ============================================================================
 */

static void initialize_trajectory_storage(void)
{
    robot_config_init_ur5(&trajectory_robot);

    line_stream_workspace.geometryCapacity = GEOMETRY_CAPACITY;
    line_stream_workspace.rawGeometry = trajectory_raw_geometry;
    line_stream_workspace.arcGeometry = trajectory_arc_geometry;
    line_stream_workspace.lOriginal = trajectory_l_original;
    line_stream_workspace.lArc = trajectory_l_arc;
    line_stream_workspace.ikScratch = &trajectory_ik_scratch;

    circular_stream_workspace.geometryCapacity = GEOMETRY_CAPACITY;
    circular_stream_workspace.rawGeometry = trajectory_raw_geometry;
    circular_stream_workspace.arcGeometry = trajectory_arc_geometry;
    circular_stream_workspace.lOriginal = trajectory_l_original;
    circular_stream_workspace.lArc = trajectory_l_arc;
    circular_stream_workspace.ikScratch = &trajectory_ik_scratch;

    if (
        !trajectory_buffer_init(
            &trajectory_buffer,
            trajectory_buffer_storage,
            TRAJECTORY_BUFFER_CAPACITY
        )
    )
    {
        printf("ERROR: could not initialize trajectory execution buffer\n");
        exit(1);
    }

    trajectory_index = 0;
    trajectory_total_samples = 0;
    trajectory_executed_samples = 0;
    trajectory_ready = false;
    trajectory_generation_finished = false;
    trajectory_have_start_sample = false;
    trajectory_have_final_sample = false;
    trajectory_underrun = false;
    planner_cancel_requested = false;
    motion_state = MOTION_WAITING_FOR_PLAN;
    hmi_hold_requested = true;
    planner_busy = false;
    active_path_type = LIVE_PATH_LINE;
}

/* ============================================================================
 * ETHERCAT TASK
 * ============================================================================
 */

static void EtherCATTask(void *pvParameters)
{
    (void)pvParameters;

    int telemetry_socket = -1;
    int command_socket = -1;

    printf("EtherCAT task started\n");

    EtherCATMasterConfig ethercat_config =
    {
        .interfaceName = "ecatA",
        .expectedSlaveCount = NUM_AXES,
        .cycleTimeNs = CYCLE_TIME_NS
    };

    if (
        !ethercat_master_init(&ethercat_config) ||
        !ethercat_master_open()
    )
    {
        exit(1);
    }

    int slave_count = ethercat_master_scan();

    if (slave_count <= 0 || slave_count != NUM_AXES)
    {
        ethercat_master_close();
        exit(1);
    }

    ethercat_master_map_pdos();

    printf("\nSetting all drives to CSP...\n");

    for (int slave = 1; slave <= NUM_AXES; slave++)
    {
        int8_t mode_readback = 0;

        if (a6ec_set_csp_mode(slave, &mode_readback))
        {
            printf(
                "Slave %d: CSP write OK | 0x6060 readback = %d\n",
                slave,
                mode_readback
            );
        }
        else
        {
            printf("Slave %d: ERROR configuring 0x6060\n", slave);

            while (ethercat_master_has_error())
            {
                printf("    SOEM: %s\n", ethercat_master_pop_error_string());
            }
        }
    }

    ethercat_master_configure_distributed_clocks();
    ethercat_master_wait_for_safe_op();
    ethercat_master_exchange();

    if (!ethercat_master_request_operational())
    {
        printf("ERROR: EtherCAT bus did not reach OPERATIONAL\n");
        ethercat_master_close();
        exit(1);
    }

    int expected_wkc = ethercat_master_expected_wkc();

    printf(
        "\nStarting 6-axis CSP + line/arc/circle HMI trajectory controller\n"
        "Press Ctrl+C for clean disable\n\n"
    );

    int print_counter = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t cycle_period = pdMS_TO_TICKS(1);

    telemetry_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (telemetry_socket < 0)
    {
        printf("ERROR: Could not create telemetry UDP socket\n");
    }

    struct sockaddr_in matlab_addr;
    memset(&matlab_addr, 0, sizeof(matlab_addr));
    matlab_addr.sin_family = AF_INET;
    matlab_addr.sin_port = htons(MATLAB_PORT);

    if (inet_pton(AF_INET, MATLAB_IP, &matlab_addr.sin_addr) != 1)
    {
        printf("ERROR: Invalid MATLAB IP address\n");
    }

    uint32_t telemetry_counter = 0;

    printf("MATLAB telemetry -> %s:%d\n", MATLAB_IP, MATLAB_PORT);

    command_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (command_socket < 0)
    {
        perror("ERROR: Could not create HMI command socket");
        if (telemetry_socket >= 0) close(telemetry_socket);
        ethercat_master_close();
        exit(1);
    }

    struct sockaddr_in command_addr;
    memset(&command_addr, 0, sizeof(command_addr));
    command_addr.sin_family = AF_INET;
    command_addr.sin_port = htons(HMI_PORT);
    command_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (
        bind(
            command_socket,
            (struct sockaddr *)&command_addr,
            sizeof(command_addr)
        ) < 0
    )
    {
        perror("ERROR: Could not bind HMI command socket");
        close(command_socket);
        if (telemetry_socket >= 0) close(telemetry_socket);
        ethercat_master_close();
        exit(1);
    }

    printf(
        "Cartesian HMI commands <- UDP :%d | RBT2 line/arc/full-circle\n",
        HMI_PORT
    );
    printf("Controller is HOLDING current position and waiting for PLAN + RUN.\n");

    struct sockaddr_in hmi_status_address;
    memset(&hmi_status_address, 0, sizeof(hmi_status_address));
    hmi_status_address.sin_family = AF_INET;
    hmi_status_address.sin_port = htons(HMI_STATUS_PORT);

    if (
        inet_pton(
            AF_INET,
            HMI_STATUS_IP,
            &hmi_status_address.sin_addr
        ) != 1
    )
    {
        printf("ERROR: Invalid HMI status IP address\n");
    }

    printf(
        "HMI live state -> %s:%d | protocol STA2\n",
        HMI_STATUS_IP,
        HMI_STATUS_PORT
    );

    uint32_t hmi_status_counter = 0;
    uint32_t last_hmi_sequence = 0;

    for (;;)
    {
        int wkc;
        bool all_operation_enabled = true;
        JointVector current_q;

        for (int slave = 1; slave <= NUM_AXES; slave++)
        {
            A6ECPDOFeedback feedback;
            a6ec_read_feedback(slave, &feedback);

            if (
                cia402_get_state(feedback.statusword) !=
                CIA402_STATE_OPERATION_ENABLED
            )
            {
                all_operation_enabled = false;
            }

            current_q.q[slave - 1] =
                a6ec_position_units_to_joint_rad(feedback.actualPosition);
        }

        bool all_at_trajectory_start =
            trajectory_ready && trajectory_have_start_sample;

        receive_hmi_commands(
            command_socket,
            &current_q,
            &last_hmi_sequence
        );

        JointVector cycle_trajectory_sample;
        bool cycle_has_trajectory_sample = false;
        bool buffer_underrun_now = false;

        /*
         * Peek first. The sample is removed only after a successful EtherCAT
         * exchange, so an isolated bad WKC cannot silently skip q[k].
         */
        if (
            !stop_requested &&
            !hmi_hold_requested &&
            all_operation_enabled &&
            motion_state == MOTION_RUNNING &&
            trajectory_ready
        )
        {
            taskENTER_CRITICAL();

            if (
                trajectory_buffer_peek(
                    &trajectory_buffer,
                    &cycle_trajectory_sample
                )
            )
            {
                cycle_has_trajectory_sample = true;
            }
            else if (!trajectory_generation_finished)
            {
                trajectory_underrun = true;
                trajectory_ready = false;
                hmi_hold_requested = true;
                planner_cancel_requested = true;
                motion_state = MOTION_HOLD;
                buffer_underrun_now = true;
            }

            taskEXIT_CRITICAL();
        }

        if (buffer_underrun_now)
        {
            printf(
                "\n"
                "============================================================\n"
                " TRAJECTORY BUFFER UNDERRUN\n"
                "============================================================\n"
                "Execution forced to HOLD.\n"
                "Executed samples: %zu / %zu\n"
                "============================================================\n",
                trajectory_executed_samples,
                trajectory_total_samples
            );
            fflush(stdout);
        }

        for (int slave = 1; slave <= NUM_AXES; slave++)
        {
            A6ECPDOFeedback feedback;
            a6ec_read_feedback(slave, &feedback);

            uint16_t drive_state = cia402_get_state(feedback.statusword);

            uint16_t controlword =
                stop_requested
                    ? cia402_get_disable_controlword(drive_state)
                    : cia402_get_enable_controlword(drive_state);

            const int joint = slave - 1;
            int32_t actual_position = feedback.actualPosition;
            int32_t start_position = actual_position;

            if (trajectory_ready && trajectory_have_start_sample)
            {
                start_position =
                    a6ec_joint_rad_to_position_units(
                        trajectory_start_sample.q[joint]
                    );

                long long start_error =
                    (long long)actual_position -
                    (long long)start_position;

                if (llabs(start_error) > 5)
                {
                    all_at_trajectory_start = false;
                }
            }
            else
            {
                all_at_trajectory_start = false;
            }

            int32_t target_position = actual_position;

            if (
                stop_requested ||
                !all_operation_enabled ||
                hmi_hold_requested
            )
            {
                target_position = actual_position;
            }
            else if (
                motion_state == MOTION_PREPOSITION &&
                trajectory_have_start_sample
            )
            {
                target_position = start_position;
            }
            else if (
                motion_state == MOTION_RUNNING &&
                cycle_has_trajectory_sample
            )
            {
                target_position =
                    a6ec_joint_rad_to_position_units(
                        cycle_trajectory_sample.q[joint]
                    );
            }
            else if (
                motion_state == MOTION_FINISHED &&
                trajectory_have_final_sample
            )
            {
                target_position =
                    a6ec_joint_rad_to_position_units(
                        trajectory_final_sample.q[joint]
                    );
            }

            A6ECPDOCommand command =
            {
                .controlword = controlword,
                .targetPosition = target_position
            };

            a6ec_write_command(slave, &command);
        }

        wkc = ethercat_master_exchange();

        bool cycle_sample_committed = false;
        bool cycle_sample_is_final = false;

        if (
            wkc >= expected_wkc &&
            motion_state == MOTION_RUNNING &&
            cycle_has_trajectory_sample &&
            trajectory_ready &&
            !hmi_hold_requested &&
            !stop_requested
        )
        {
            JointVector consumed;

            taskENTER_CRITICAL();

            if (trajectory_buffer_pop(&trajectory_buffer, &consumed))
            {
                trajectory_index = trajectory_executed_samples;
                trajectory_executed_samples++;
                cycle_sample_committed = true;

                if (
                    trajectory_generation_finished &&
                    trajectory_buffer_is_empty(&trajectory_buffer) &&
                    trajectory_executed_samples >= trajectory_total_samples
                )
                {
                    cycle_sample_is_final = true;
                }
            }

            taskEXIT_CRITICAL();
        }

        if (
            !stop_requested &&
            !hmi_hold_requested &&
            trajectory_ready &&
            wkc >= expected_wkc
        )
        {
            if (
                motion_state == MOTION_PREPOSITION &&
                all_operation_enabled &&
                all_at_trajectory_start
            )
            {
                trajectory_index = 0;
                trajectory_executed_samples = 0;
                motion_state = MOTION_RUNNING;

                printf(
                    "\n"
                    "============================================================\n"
                    " CONTROLCORE %s STREAMING EXECUTION START\n"
                    "============================================================\n"
                    "Samples:       %zu\n"
                    "dt:            0.001 s\n"
                    "Duration:      %.6f s\n"
                    "FIFO capacity: %u samples\n"
                    "============================================================\n",
                    live_path_name(active_path_type),
                    trajectory_total_samples,
                    trajectory_report.duration,
                    (unsigned)TRAJECTORY_BUFFER_CAPACITY
                );
                fflush(stdout);
            }
            else if (
                motion_state == MOTION_RUNNING &&
                cycle_sample_committed &&
                cycle_sample_is_final
            )
            {
                motion_state = MOTION_FINISHED;

                printf(
                    "\n"
                    "============================================================\n"
                    " CONTROLCORE %s STREAMING EXECUTION COMPLETE\n"
                    "============================================================\n"
                    "Final sample: %zu / %zu\n"
                    "Duration:     %.6f s\n"
                    "============================================================\n",
                    live_path_name(active_path_type),
                    trajectory_executed_samples,
                    trajectory_total_samples,
                    trajectory_report.duration
                );
                fflush(stdout);
            }
        }

        if (++hmi_status_counter >= 100)
        {
            hmi_status_counter = 0;
            send_hmi_status(
                command_socket,
                &hmi_status_address,
                wkc,
                expected_wkc,
                last_hmi_sequence
            );
        }

        if (++telemetry_counter >= 20)
        {
            telemetry_counter = 0;
            int32_t actual_positions[NUM_AXES];

            for (int slave = 1; slave <= NUM_AXES; slave++)
            {
                A6ECPDOFeedback feedback;
                a6ec_read_feedback(slave, &feedback);
                actual_positions[slave - 1] = feedback.actualPosition;
            }

            if (telemetry_socket >= 0)
            {
                sendto(
                    telemetry_socket,
                    actual_positions,
                    sizeof(actual_positions),
                    0,
                    (struct sockaddr *)&matlab_addr,
                    sizeof(matlab_addr)
                );
            }
        }

        if (wkc < expected_wkc)
        {
            printf(
                "\nWARNING: EtherCAT communication problem! WKC=%d expected=%d\n",
                wkc,
                expected_wkc
            );
            fflush(stdout);
        }

        if (++print_counter >= 1000)
        {
            print_counter = 0;

            printf(
                "WKC=%d/%d | STATE=%s | PATH=%s",
                wkc,
                expected_wkc,
                motion_state_name(motion_state),
                live_path_name(active_path_type)
            );

            for (int slave = 1; slave <= NUM_AXES; slave++)
            {
                A6ECPDOCommand command;
                A6ECPDOFeedback feedback;
                a6ec_read_command(slave, &command);
                a6ec_read_feedback(slave, &feedback);

                printf(
                    " | J%d T=%d A=%d SW=0x%04X",
                    slave,
                    command.targetPosition,
                    feedback.actualPosition,
                    feedback.statusword
                );
            }

            printf("\n");
            fflush(stdout);
        }

        if (stop_requested)
        {
            int all_disabled = 1;

            for (int slave = 1; slave <= NUM_AXES; slave++)
            {
                A6ECPDOFeedback feedback;
                a6ec_read_feedback(slave, &feedback);

                if (
                    cia402_get_state(feedback.statusword) !=
                    CIA402_STATE_SWITCH_ON_DISABLED
                )
                {
                    all_disabled = 0;
                }
            }

            if (all_disabled)
            {
                printf("\nAll drives are Switch On Disabled\n");
                printf("Closing EtherCAT...\n");

                if (telemetry_socket >= 0) close(telemetry_socket);
                if (command_socket >= 0) close(command_socket);

                ethercat_master_close();
                printf("EtherCAT closed cleanly\n");
                exit(0);
            }
        }

        vTaskDelayUntil(&last_wake_time, cycle_period);
    }
}

/* ============================================================================
 * MAIN
 * ============================================================================
 */

int main(void)
{
    initialize_trajectory_storage();

    trajectory_request_queue =
        xQueueCreate(1, sizeof(TrajectoryPlanCommand));

    if (trajectory_request_queue == NULL)
    {
        printf("Failed to create trajectory planner queue\n");
        return 1;
    }

    signal(SIGINT, handle_sigint);

    printf("FreeRTOS starting\n");
    printf(
        "No trajectory is preloaded. Waiting for RBT2 line/arc/full-circle PLAN + RUN.\n"
    );
    fflush(stdout);

    BaseType_t planner_result =
        xTaskCreate(
            TrajectoryPlannerTask,
            "Planner",
            3072,
            NULL,
            tskIDLE_PRIORITY,
            NULL
        );

    if (planner_result != pdPASS)
    {
        printf("Failed to create trajectory planner task\n");
        return 1;
    }

    BaseType_t ethercat_result =
        xTaskCreate(
            EtherCATTask,
            "EtherCAT",
            4096,
            NULL,
            tskIDLE_PRIORITY + 1,
            NULL
        );

    if (ethercat_result != pdPASS)
    {
        printf("Failed to create EtherCAT task\n");
        return 1;
    }

    vTaskStartScheduler();
    return 0;
}
