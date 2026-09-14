/*
 * ============================================================================
 *  FreeRTOS + SOEM + EtherCAT + CiA-402 + MATLAB TEST PROGRAM
 * ============================================================================
 *
 *  PURPOSE
 *  -------
 *  This program is a PC-side test of the control architecture that will later
 *  be used with the real 6-axis robot.
 *
 *  The software stack is:
 *
 *      main()
 *        |
 *        v
 *      FreeRTOS scheduler
 *        |
 *        v
 *      EtherCATTask()
 *        |
 *        v
 *      SOEM  (Simple Open EtherCAT Master)
 *        |
 *        v
 *      EtherCAT process data
 *        |
 *        v
 *      6 simulated CiA-402 servo drives
 *
 *  In parallel, actual joint positions are sent by UDP to MATLAB so MATLAB can
 *  visualize the robot motion.
 *
 *
 *  IMPORTANT: WHAT IS REAL AND WHAT IS SIMULATED?
 *  ------------------------------------------------
 *  REAL SOFTWARE:
 *      - FreeRTOS scheduling
 *      - our C application logic
 *      - SOEM EtherCAT master library
 *      - CiA-402 state-machine logic
 *      - PDO communication logic
 *      - UDP telemetry to MATLAB
 *
 *  SIMULATED:
 *      - Ethernet/EtherCAT physical link
 *      - A6-EC servo drives
 *      - motor/encoder behavior
 *
 *
 *  MAIN EXECUTION FLOW
 *  -------------------
 *
 *      main()
 *        |
 *        +--> install Ctrl+C handler
 *        |
 *        +--> create EtherCATTask
 *        |
 *        +--> start FreeRTOS scheduler
 *                |
 *                v
 *          EtherCATTask()
 *                |
 *                +--> open EtherCAT interface
 *                +--> discover 6 slaves
 *                +--> map PDOs
 *                +--> set CSP mode
 *                +--> configure Distributed Clocks
 *                +--> enter SAFE-OP
 *                +--> enter OPERATIONAL
 *                +--> calculate expected WKC
 *                |
 *                v
 *          1 ms cyclic loop
 *                |
 *                +--> read Statusword
 *                +--> calculate Controlword
 *                +--> write Target Position
 *                +--> send EtherCAT PDOs
 *                +--> receive EtherCAT PDOs
 *                +--> check WKC
 *                +--> send Actual Position to MATLAB
 *                +--> wait until next 1 ms cycle
 *
 *
 *  NOTE ABOUT PDO OFFSETS
 *  ----------------------
 *  This test accesses PDOs using byte offsets such as:
 *
 *      outputs + 0
 *      outputs + 2
 *      inputs  + 2
 *      inputs  + 4
 *
 *  Those offsets are valid ONLY because they match the PDO layout configured
 *  by the current simulator/slave definition.
 *
 *  When moving to the real A6-EC servo drive, verify the REAL PDO mapping from
 *  the A6-EC manual / ESI file before keeping these offsets.
 *
 * ============================================================================
 */

#include <stdio.h>      /* printf(), fflush()                            */
#include <stdlib.h>     /* exit()                                        */
#include <string.h>     /* memset(), memcpy()                            */
#include <stdint.h>     /* uint8_t, uint16_t, int32_t, etc.             */
#include <signal.h>     /* signal(), SIGINT                              */
#include <sys/socket.h> /* socket(), sendto()                            */
#include <arpa/inet.h>  /* sockaddr_in, htons(), inet_pton()            */
#include <unistd.h>     /* close()                                       */
#include <errno.h>      /* errno, EAGAIN, EWOULDBLOCK                     */
#include <stdbool.h>
#include <math.h>

/*
 * FreeRTOS headers.
 *
 * FreeRTOS is NOT the EtherCAT driver.
 * Its job here is to schedule EtherCATTask periodically.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*
 * SOEM = Simple Open EtherCAT Master.
 *
 * SOEM provides the EtherCAT-master functions used below:
 *      ecx_init()
 *      ecx_config_init()
 *      ecx_config_map_group()
 *      ecx_SDOwrite()
 *      ecx_send_processdata()
 *      ...
 */
#include "robot_config.h"
#include "single_segment_line.h"
#include "single_segment_line_stream.h"
#include "trajectory_buffer.h"
#include "math3d.h"

#include "ethercat_master.h"
#include "cia402.h"
#include "a6ec_drive.h"

/* ============================================================================
 *                              CONFIGURATION
 * ============================================================================
 */

/*
 * MATLAB UDP port.
 *
 * The C program sends joint-position feedback to this port.
 * MATLAB must listen on the same port.
 */
#define MATLAB_PORT 5005

/*
 * Mock HMI command port.
 *
 *      HMI -> controller : UDP 5006
 *      controller -> MATLAB : UDP 5005
 */
#define HMI_PORT        5006
#define HMI_STATUS_PORT 5007
#define HMI_STATUS_IP   "127.0.0.1"

/*
 * HMI protocol used by the Cartesian raylib HMI.
 *
 * "RBT2" plan packet:
 *
 *      18 x uint32 words = 72 bytes
 *
 *      magic, command, sequence,
 *      A.x, A.y, A.z, A.yaw, A.pitch, A.roll,
 *      B.x, B.y, B.z, B.yaw, B.pitch, B.roll,
 *      TCP speed, TCP acceleration, TCP jerk
 *
 * All float values are transmitted as IEEE-754 float bit patterns in
 * network byte order.
 *
 * STOP packet:
 *
 *      magic, command, sequence = 12 bytes
 */
#define HMI_PACKET_MAGIC                 0x52425432U   /* ASCII "RBT2" */
#define HMI_COMMAND_PLAN_AND_RUN_LINE   1U
#define HMI_COMMAND_STOP                2U

#define HMI_POSE_VALUES                 6U
#define HMI_PLAN_FLOAT_COUNT            15U
#define HMI_PLAN_WORD_COUNT             18U
#define HMI_STOP_WORD_COUNT             3U

/*
 * Controller -> HMI live-status packet.
 *
 * 13 uint32 words = 52 bytes:
 *
 *      magic "STA2"
 *      MotionState
 *      last HMI sequence
 *      trajectory index
 *      trajectory sample count
 *      actual WKC
 *      expected WKC
 *      six raw CiA-402 Statuswords
 */
#define HMI_STATUS_PACKET_MAGIC         0x53544132U   /* ASCII "STA2" */
#define HMI_STATUS_WORD_COUNT           13U

/*
 * MATLAB/Windows-side IP address as visible from Linux/WSL.
 *
 * This is NOT related to EtherCAT.
 * It is only used for the separate UDP visualization link.
 */
#define MATLAB_IP "172.18.160.1"

/* Robot contains six joints / six servo drives. */
#define NUM_AXES 6

/*
 * Desired EtherCAT Distributed-Clock SYNC0 period:
 *
 *      1,000,000 ns = 1 ms
 *
 * This requests one synchronization event every millisecond.
 */
#define CYCLE_TIME_NS 1000000U


/* ============================================================================
 * CONTROLCORE STREAMING TRAJECTORY
 * ============================================================================
 *
 * The live controller does NOT store the complete time-sampled trajectory.
 *
 * PlannerTask generates exact 1 ms JointVector samples into a fixed-size FIFO.
 * EtherCATTask consumes exactly one JointVector per 1 ms CSP cycle.
 *
 * Total trajectory duration is therefore independent of FIFO capacity.
 * ============================================================================
 */

#define GEOMETRY_CAPACITY          512U
#define TRAJECTORY_BUFFER_CAPACITY 256U
#define TRAJECTORY_PREFILL_SAMPLES 128U

/* Robot model. */
static RobotConfig trajectory_robot;

/* Fixed-size Cartesian geometry workspace. */
static Vec3 trajectory_raw_geometry[GEOMETRY_CAPACITY];
static Vec3 trajectory_arc_geometry[GEOMETRY_CAPACITY];
static real_t trajectory_l_original[GEOMETRY_CAPACITY];
static real_t trajectory_l_arc[GEOMETRY_CAPACITY];
static ADLSInfo trajectory_ik_scratch;

static SingleLineStreamWorkspace trajectory_stream_workspace;
static SingleLineStream trajectory_stream;

/* Fixed-size execution FIFO: 256 samples = 256 ms at 1 kHz. */
static JointVector trajectory_buffer_storage[TRAJECTORY_BUFFER_CAPACITY];
static TrajectoryBuffer trajectory_buffer;

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

static StreamingPlanReport trajectory_report;

/* Execution progress / planner-executor shared state. */
static volatile size_t trajectory_index = 0;
static volatile size_t trajectory_total_samples = 0;
static volatile size_t trajectory_executed_samples = 0;

static volatile bool trajectory_ready = false;
static volatile bool trajectory_generation_finished = false;
static volatile bool planner_cancel_requested = false;
static volatile bool trajectory_underrun = false;

/* q[0] is retained for PREPOSITION without consuming the FIFO. */
static JointVector trajectory_start_sample;
static volatile bool trajectory_have_start_sample = false;

/* Final q is retained so FINISHED can hold the exact final reference. */
static JointVector trajectory_final_sample;
static volatile bool trajectory_have_final_sample = false;


/* ============================================================================
 * HMI TRAJECTORY REQUEST / EXECUTION STATE
 * ============================================================================
 */

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
    uint32_t sequence;

    float waypointA[HMI_POSE_VALUES];
    float waypointB[HMI_POSE_VALUES];

    float tcpSpeed;
    float tcpAccel;
    float tcpJerk;

    /*
     * Snapshot of the current six joint positions when PLAN + RUN is accepted.
     *
     * This is ONLY the initial ADLS seed. It does not define waypoint A.
     */
    JointVector qSeed;

} TrajectoryPlanCommand;


/*
 * One-element mailbox from the 1 ms EtherCAT task to the lower-priority
 * trajectory-planning task.
 */
static QueueHandle_t trajectory_request_queue = NULL;


/*
 * Robot starts by holding its current position and waiting for an HMI plan.
 */
static volatile MotionState motion_state =
    MOTION_WAITING_FOR_PLAN;


/*
 * HMI STOP means hold current measured position.
 *
 * Ctrl+C is separate and still performs the clean CiA-402 disable sequence.
 */
static volatile bool hmi_hold_requested = true;


/*
 * True from the instant an HMI plan is accepted until the planner finishes.
 *
 * This prevents a second PLAN request from being queued while an earlier one
 * is still being generated, including the case where STOP was pressed during
 * planning.
 */
static volatile bool planner_busy = false;

/*
 * Set to 1 when Ctrl+C is pressed.
 *
 * volatile:
 *      the value may change outside normal program flow because a signal
 *      handler modifies it.
 *
 * sig_atomic_t:
 *      a type safe to modify from a POSIX signal handler.
 */
static volatile sig_atomic_t stop_requested = 0;


/* ============================================================================
 *                             Ctrl+C HANDLER
 * ============================================================================
 */

/*
 * Called when Linux receives SIGINT, normally from Ctrl+C.
 *
 * IMPORTANT:
 * We do NOT immediately kill EtherCAT here.
 *
 * Instead, we only set stop_requested = 1.
 *
 * The normal 1 ms control loop then notices the request and walks every servo
 * backwards through the CiA-402 state machine for a clean shutdown.
 */
static void handle_sigint(int signal_number)
{
    /* We do not need the actual signal number. */
    (void)signal_number;

    /* Tell EtherCATTask to begin clean shutdown. */
    stop_requested = 1;
}


/*
 * HMI orientations are entered in degrees.
 */
static real_t degrees_to_radians_main(
    real_t degrees
)
{
    return
        degrees *
        ROBOT_PI /
        180.0;
}


/*
 * Convert one 32-bit network-order IEEE-754 float word back to host float.
 */
static float network_word_to_float(
    uint32_t network_word
)
{
    uint32_t host_bits =
        ntohl(network_word);

    float value;

    memcpy(
        &value,
        &host_bits,
        sizeof(value)
    );

    return value;
}


static const char *motion_state_name(
    MotionState state
)
{
    switch (state)
    {
        case MOTION_WAITING_FOR_PLAN:
            return "WAITING";

        case MOTION_PLANNING:
            return "PLANNING";

        case MOTION_PREPOSITION:
            return "PREPOSITION";

        case MOTION_RUNNING:
            return "RUNNING";

        case MOTION_FINISHED:
            return "FINISHED";

        case MOTION_HOLD:
            return "HOLD";

        default:
            return "UNKNOWN";
    }
}

/* ============================================================================
 * CONTROLLER -> HMI LIVE STATUS
 * ============================================================================
 *
 * Sent over localhost UDP :5007 at 10 Hz.
 *
 * This is telemetry only. MSG_DONTWAIT is used so HMI display traffic can
 * never stall the 1 ms EtherCAT loop.
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
    if (
        socket_fd < 0 ||
        status_address == NULL
    )
    {
        return;
    }


    uint32_t packet[
        HMI_STATUS_WORD_COUNT
    ];


    packet[0] =
        htonl(
            HMI_STATUS_PACKET_MAGIC
        );


    packet[1] =
        htonl(
            (uint32_t)motion_state
        );


    packet[2] =
        htonl(
            last_sequence
        );


    packet[3] =
        htonl(
            (uint32_t)trajectory_index
        );


    packet[4] =
        htonl(
            (uint32_t)trajectory_total_samples
        );


    packet[5] =
        htonl(
            (uint32_t)wkc
        );


    packet[6] =
        htonl(
            (uint32_t)expected_wkc
        );


    for (int slave = 1;
         slave <= NUM_AXES;
         slave++)
    {
        A6ECPDOFeedback feedback;

        a6ec_read_feedback(
            slave,
            &feedback
        );


        packet[
            7 +
            (slave - 1)
        ] =
            htonl(
                (uint32_t)feedback.statusword
            );
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


/* ============================================================================
 *                       CARTESIAN HMI UDP COMMAND RECEIVER
 * ============================================================================
 *
 * RBT2 PLAN + RUN packet:
 *
 *      72 bytes total
 *
 *      word 0  : magic = "RBT2"
 *      word 1  : command = PLAN_AND_RUN_LINE
 *      word 2  : sequence
 *      word 3  : A.x
 *      word 4  : A.y
 *      word 5  : A.z
 *      word 6  : A.yaw   [deg]
 *      word 7  : A.pitch [deg]
 *      word 8  : A.roll  [deg]
 *      word 9  : B.x
 *      word 10 : B.y
 *      word 11 : B.z
 *      word 12 : B.yaw   [deg]
 *      word 13 : B.pitch [deg]
 *      word 14 : B.roll  [deg]
 *      word 15 : TCP speed [m/s]
 *      word 16 : TCP accel [m/s^2]
 *      word 17 : TCP jerk  [m/s^3]
 *
 * RBT2 STOP packet:
 *
 *      12 bytes total:
 *      magic, command, sequence
 *
 * This function never performs trajectory planning itself.
 *
 * Planning is intentionally moved to a lower-priority FreeRTOS task so the
 * 1 ms EtherCAT loop keeps exchanging PDOs while ADLS generates thousands of
 * trajectory samples.
 * ============================================================================
 */

static void receive_hmi_commands(
    int command_socket,
    const JointVector *current_q,
    uint32_t *last_sequence
)
{
    if (
        command_socket < 0 ||
        current_q == NULL ||
        last_sequence == NULL
    )
    {
        return;
    }


    for (;;)
    {
        uint32_t packet[HMI_PLAN_WORD_COUNT];

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
            if (
                errno == EAGAIN ||
                errno == EWOULDBLOCK
            )
            {
                break;
            }

            perror("HMI recvfrom");
            break;
        }


        /*
         * We need at least magic + command + sequence before we can decode
         * anything useful.
         */
        if (
            received <
            (ssize_t)(
                HMI_STOP_WORD_COUNT *
                sizeof(uint32_t)
            )
        )
        {
            printf(
                "Ignoring HMI packet that is too short: %zd bytes\n",
                received
            );

            continue;
        }


        uint32_t magic =
            ntohl(packet[0]);

        uint32_t command =
            ntohl(packet[1]);

        uint32_t sequence =
            ntohl(packet[2]);


        if (magic != HMI_PACKET_MAGIC)
        {
            printf(
                "Ignoring HMI packet with invalid magic\n"
            );

            continue;
        }


        *last_sequence =
            sequence;


        /* --------------------------------------------------------------------
         * PLAN + RUN ABSOLUTE CARTESIAN LINE
         * --------------------------------------------------------------------
         */

        if (
            command ==
            HMI_COMMAND_PLAN_AND_RUN_LINE
        )
        {
            const ssize_t expected_bytes =
                (ssize_t)(
                    HMI_PLAN_WORD_COUNT *
                    sizeof(uint32_t)
                );


            if (received != expected_bytes)
            {
                printf(
                    "Ignoring PLAN packet with unexpected size: "
                    "%zd bytes (expected %zd)\n",
                    received,
                    expected_bytes
                );

                continue;
            }


            /*
             * Do not rewrite the trajectory buffers while they are actively
             * being consumed by the EtherCAT executor.
             *
             * To change trajectory during PREPOSITION/RUNNING:
             *
             *      press STOP first,
             *      then send the new PLAN + RUN request.
             */
            MotionState state =
                motion_state;

            if (
                planner_busy ||
                state == MOTION_PLANNING ||
                state == MOTION_PREPOSITION ||
                state == MOTION_RUNNING
            )
            {
                printf(
                    "HMI PLAN #%u rejected: controller is %s. "
                    "Press STOP before submitting a new path.\n",
                    sequence,
                    motion_state_name(state)
                );

                fflush(stdout);
                continue;
            }


            TrajectoryPlanCommand plan;

            memset(
                &plan,
                0,
                sizeof(plan)
            );


            plan.sequence =
                sequence;


            for (int value = 0;
                 value < (int)HMI_POSE_VALUES;
                 value++)
            {
                plan.waypointA[value] =
                    network_word_to_float(
                        packet[3 + value]
                    );

                plan.waypointB[value] =
                    network_word_to_float(
                        packet[
                            3 +
                            HMI_POSE_VALUES +
                            value
                        ]
                    );
            }


            plan.tcpSpeed =
                network_word_to_float(
                    packet[15]
                );

            plan.tcpAccel =
                network_word_to_float(
                    packet[16]
                );

            plan.tcpJerk =
                network_word_to_float(
                    packet[17]
                );


            /*
             * Reject NaN/Inf or invalid motion limits before handing anything
             * to ControlCore.
             */
            bool values_valid =
                true;

            for (int value = 0;
                 value < (int)HMI_POSE_VALUES;
                 value++)
            {
                if (
                    !isfinite(plan.waypointA[value]) ||
                    !isfinite(plan.waypointB[value])
                )
                {
                    values_valid =
                        false;
                }
            }

            if (
                !isfinite(plan.tcpSpeed) ||
                !isfinite(plan.tcpAccel) ||
                !isfinite(plan.tcpJerk) ||
                plan.tcpSpeed <= 0.0f ||
                plan.tcpAccel <= 0.0f ||
                plan.tcpJerk <= 0.0f
            )
            {
                values_valid =
                    false;
            }


            if (!values_valid)
            {
                printf(
                    "HMI PLAN #%u rejected: invalid Cartesian/profile values\n",
                    sequence
                );

                fflush(stdout);
                continue;
            }


            /*
             * Snapshot the actual joint configuration NOW.
             *
             * The new ControlCore API uses this only as the first ADLS seed.
             * Waypoint A itself comes directly from the absolute Cartesian HMI
             * values.
             */
            plan.qSeed =
                *current_q;


            /*
             * Publish controller state BEFORE queuing the request.
             *
             * The EtherCAT executor will therefore hold measured position while
             * the planner task builds qPath in the background.
             */
            taskENTER_CRITICAL();

            planner_cancel_requested =
                false;

            trajectory_buffer_reset(
                &trajectory_buffer
            );

            trajectory_ready =
                false;

            trajectory_generation_finished =
                false;

            trajectory_underrun =
                false;

            trajectory_have_start_sample =
                false;

            trajectory_have_final_sample =
                false;

            trajectory_total_samples =
                0;

            trajectory_executed_samples =
                0;

            trajectory_index =
                0;

            hmi_hold_requested =
                false;

            planner_busy =
                true;

            motion_state =
                MOTION_PLANNING;

            taskEXIT_CRITICAL();


            if (
                xQueueSend(
                    trajectory_request_queue,
                    &plan,
                    0
                ) != pdPASS
            )
            {
                motion_state =
                    MOTION_WAITING_FOR_PLAN;

                hmi_hold_requested =
                    true;

                planner_busy =
                    false;

                printf(
                    "HMI PLAN #%u rejected: planner queue is busy\n",
                    sequence
                );

                fflush(stdout);
                continue;
            }


            printf(
                "\n"
                "============================================================\n"
                " HMI CARTESIAN PLAN RECEIVED #%u\n"
                "============================================================\n"
                "A: p=[%.6f %.6f %.6f] m | "
                "YPR=[%.2f %.2f %.2f] deg\n"
                "B: p=[%.6f %.6f %.6f] m | "
                "YPR=[%.2f %.2f %.2f] deg\n"
                "TCP: speed=%.4f m/s | accel=%.4f m/s^2 | jerk=%.4f m/s^3\n"
                "Controller: HOLD while ControlCore plans in background\n"
                "============================================================\n",
                sequence,
                plan.waypointA[0],
                plan.waypointA[1],
                plan.waypointA[2],
                plan.waypointA[3],
                plan.waypointA[4],
                plan.waypointA[5],
                plan.waypointB[0],
                plan.waypointB[1],
                plan.waypointB[2],
                plan.waypointB[3],
                plan.waypointB[4],
                plan.waypointB[5],
                plan.tcpSpeed,
                plan.tcpAccel,
                plan.tcpJerk
            );

            fflush(stdout);
        }

        /* --------------------------------------------------------------------
         * STOP -> HOLD CURRENT POSITION
         * --------------------------------------------------------------------
         */

        else if (
            command ==
            HMI_COMMAND_STOP
        )
        {
            const ssize_t expected_bytes =
                (ssize_t)(
                    HMI_STOP_WORD_COUNT *
                    sizeof(uint32_t)
                );


            if (received != expected_bytes)
            {
                printf(
                    "Ignoring STOP packet with unexpected size: "
                    "%zd bytes (expected %zd)\n",
                    received,
                    expected_bytes
                );

                continue;
            }


            taskENTER_CRITICAL();

            hmi_hold_requested =
                true;

            planner_cancel_requested =
                true;

            trajectory_ready =
                false;

            trajectory_generation_finished =
                false;

            trajectory_buffer_reset(
                &trajectory_buffer
            );

            motion_state =
                MOTION_HOLD;

            taskEXIT_CRITICAL();


            printf(
                "HMI command #%u: STOP -> HOLD CURRENT POSITION\n",
                sequence
            );

            fflush(stdout);
        }

        else
        {
            printf(
                "Ignoring unknown HMI command %u (sequence %u)\n",
                command,
                sequence
            );

            fflush(stdout);
        }
    }
}


/* ============================================================================
 *                            MAIN ETHERCAT TASK
 * ============================================================================
 *
 * This is the time-critical FreeRTOS task that performs EtherCAT I/O:
 *
 *      - initializes SOEM
 *      - discovers slaves
 *      - maps PDOs
 *      - configures CSP
 *      - configures Distributed Clocks
 *      - moves EtherCAT to OPERATIONAL
 *      - executes the 1 ms cyclic control loop
 *      - sends feedback to MATLAB
 *      - shuts down cleanly
 */

/* ============================================================================
 * INITIALIZE STREAMING CONTROLCORE STORAGE
 * ============================================================================
 */

static void initialize_trajectory_storage(void)
{
    robot_config_init_ur5(
        &trajectory_robot
    );

    trajectory_stream_workspace.geometryCapacity =
        GEOMETRY_CAPACITY;

    trajectory_stream_workspace.rawGeometry =
        trajectory_raw_geometry;

    trajectory_stream_workspace.arcGeometry =
        trajectory_arc_geometry;

    trajectory_stream_workspace.lOriginal =
        trajectory_l_original;

    trajectory_stream_workspace.lArc =
        trajectory_l_arc;

    trajectory_stream_workspace.ikScratch =
        &trajectory_ik_scratch;

    if (
        !trajectory_buffer_init(
            &trajectory_buffer,
            trajectory_buffer_storage,
            TRAJECTORY_BUFFER_CAPACITY
        )
    )
    {
        printf(
            "ERROR: could not initialize trajectory execution buffer\n"
        );

        exit(1);
    }

    trajectory_index =
        0;

    trajectory_total_samples =
        0;

    trajectory_executed_samples =
        0;

    trajectory_ready =
        false;

    trajectory_generation_finished =
        false;

    trajectory_have_start_sample =
        false;

    trajectory_have_final_sample =
        false;

    trajectory_underrun =
        false;

    planner_cancel_requested =
        false;

    motion_state =
        MOTION_WAITING_FOR_PLAN;

    hmi_hold_requested =
        true;

    planner_busy =
        false;
}


/* ============================================================================
 * STREAMING-PLANNER HELPERS
 * ============================================================================
 */

static bool planner_is_cancelled(void)
{
    bool cancelled;

    taskENTER_CRITICAL();

    cancelled =
        planner_cancel_requested ||
        stop_requested;

    taskEXIT_CRITICAL();

    return cancelled;
}


static void update_velocity_validation(
    StreamingPlanReport *report,
    int joint,
    real_t velocity
)
{
    real_t magnitude =
        fabs(
            velocity
        );

    if (
        magnitude >
        report->peakJointVelocity[joint]
    )
    {
        report->peakJointVelocity[joint] =
            magnitude;
    }

    if (
        magnitude >
        trajectory_robot.limits.qdMax[joint]
    )
    {
        report->velocityLimitsPass =
            false;
    }
}


/*
 * Validate the ENTIRE requested path before execution starts.
 *
 * This is still memory-bounded: only a rolling three-sample window is used
 * for velocity checks. No trajectory-sized qPath[] exists.
 */
static bool validate_streaming_trajectory(
    const SingleLineRequest *request,
    StreamingPlanReport *report
)
{
    if (
        request == NULL ||
        report == NULL
    )
    {
        return false;
    }

    memset(
        report,
        0,
        sizeof(*report)
    );

    report->positionLimitsPass =
        true;

    report->velocityLimitsPass =
        true;

    SingleLineStream validation_stream;

    if (
        !single_line_stream_init(
            &validation_stream,
            &trajectory_robot,
            request,
            &trajectory_stream_workspace
        )
    )
    {
        printf(
            "Streaming validation init failed: %s\n",
            single_line_stream_status_string(
                single_line_stream_status(
                    &validation_stream
                )
            )
        );

        return false;
    }

    report->samples =
        single_line_stream_sample_count(
            &validation_stream
        );

    report->duration =
        validation_stream.profile.info.T;

    report->pathLength =
        validation_stream.segmentLength;

    SingleLineStreamSample previous_previous;
    SingleLineStreamSample previous;

    bool have_previous_previous =
        false;

    bool have_previous =
        false;

    size_t generated =
        0;

    for (;;)
    {
        if (
            planner_is_cancelled()
        )
        {
            return false;
        }

        SingleLineStreamSample sample;

        if (
            !single_line_stream_next(
                &validation_stream,
                &sample
            )
        )
        {
            if (
                single_line_stream_is_finished(
                    &validation_stream
                )
            )
            {
                break;
            }

            printf(
                "Streaming validation stopped: %s\n",
                single_line_stream_status_string(
                    single_line_stream_status(
                        &validation_stream
                    )
                )
            );

            return false;
        }

        generated++;

        if (
            sample.tcpSpeed >
            report->peakTCPSpeed
        )
        {
            report->peakTCPSpeed =
                sample.tcpSpeed;
        }

        if (
            sample.positionError >
            report->maxPositionError
        )
        {
            report->maxPositionError =
                sample.positionError;
        }

        if (
            sample.orientationError >
            report->maxOrientationError
        )
        {
            report->maxOrientationError =
                sample.orientationError;
        }

        if (
            sample.ikIterations >
            report->maxIKIterations
        )
        {
            report->maxIKIterations =
                sample.ikIterations;
        }

        for (int joint = 0;
             joint < ROBOT_DOF;
             joint++)
        {
            if (
                sample.q.q[joint] <
                    trajectory_robot.limits.qMin[joint] ||
                sample.q.q[joint] >
                    trajectory_robot.limits.qMax[joint]
            )
            {
                report->positionLimitsPass =
                    false;
            }

            if (
                have_previous
            )
            {
                real_t step =
                    fabs(
                        sample.q.q[joint] -
                        previous.q.q[joint]
                    );

                if (
                    step >
                    report->maxJointStep[joint]
                )
                {
                    report->maxJointStep[joint] =
                        step;
                }
            }
        }

        /*
         * Same gradient convention as the existing validated full planner:
         * first = forward, interior = central, last = backward.
         */
        if (
            have_previous &&
            !have_previous_previous
        )
        {
            real_t dt_first =
                sample.t -
                previous.t;

            if (
                fabs(dt_first) >
                1e-15
            )
            {
                for (int joint = 0;
                     joint < ROBOT_DOF;
                     joint++)
                {
                    update_velocity_validation(
                        report,
                        joint,
                        (
                            sample.q.q[joint] -
                            previous.q.q[joint]
                        ) /
                        dt_first
                    );
                }
            }
        }
        else if (
            have_previous &&
            have_previous_previous
        )
        {
            real_t dt_central =
                sample.t -
                previous_previous.t;

            if (
                fabs(dt_central) >
                1e-15
            )
            {
                for (int joint = 0;
                     joint < ROBOT_DOF;
                     joint++)
                {
                    update_velocity_validation(
                        report,
                        joint,
                        (
                            sample.q.q[joint] -
                            previous_previous.q.q[joint]
                        ) /
                        dt_central
                    );
                }
            }
        }

        if (
            have_previous
        )
        {
            previous_previous =
                previous;

            have_previous_previous =
                true;
        }

        previous =
            sample;

        have_previous =
            true;
    }

    if (
        generated !=
        report->samples
    )
    {
        return false;
    }

    /* Final sample: backward difference. */
    if (
        have_previous &&
        have_previous_previous
    )
    {
        real_t dt_last =
            previous.t -
            previous_previous.t;

        if (
            fabs(dt_last) >
            1e-15
        )
        {
            for (int joint = 0;
                 joint < ROBOT_DOF;
                 joint++)
            {
                update_velocity_validation(
                    report,
                    joint,
                    (
                        previous.q.q[joint] -
                        previous_previous.q.q[joint]
                    ) /
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


static void abort_streaming_plan(
    bool preserve_hold_state
)
{
    taskENTER_CRITICAL();

    trajectory_buffer_reset(
        &trajectory_buffer
    );

    trajectory_ready =
        false;

    trajectory_generation_finished =
        false;

    trajectory_have_start_sample =
        false;

    trajectory_have_final_sample =
        false;

    trajectory_total_samples =
        0;

    trajectory_executed_samples =
        0;

    trajectory_index =
        0;

    hmi_hold_requested =
        true;

    planner_busy =
        false;

    if (
        preserve_hold_state
    )
    {
        motion_state =
            MOTION_HOLD;
    }
    else
    {
        motion_state =
            MOTION_WAITING_FOR_PLAN;
    }

    taskEXIT_CRITICAL();
}


/* ============================================================================
 * LOWER-PRIORITY STREAMING CONTROLCORE PLANNER TASK
 * ============================================================================
 */

static void TrajectoryPlannerTask(
    void *pvParameters
)
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

        SingleLineRequest request;

        memset(
            &request,
            0,
            sizeof(request)
        );

        request.qSeed =
            command.qSeed;

        request.startPosition =
            (Vec3)
            {{
                (real_t)command.waypointA[0],
                (real_t)command.waypointA[1],
                (real_t)command.waypointA[2]
            }};

        Mat3 R_A =
            eul_zyx(
                degrees_to_radians_main(
                    (real_t)command.waypointA[3]
                ),
                degrees_to_radians_main(
                    (real_t)command.waypointA[4]
                ),
                degrees_to_radians_main(
                    (real_t)command.waypointA[5]
                )
            );

        request.startOrientation =
            rotm_to_quat(
                R_A
            );

        request.endPosition =
            (Vec3)
            {{
                (real_t)command.waypointB[0],
                (real_t)command.waypointB[1],
                (real_t)command.waypointB[2]
            }};

        Mat3 R_B =
            eul_zyx(
                degrees_to_radians_main(
                    (real_t)command.waypointB[3]
                ),
                degrees_to_radians_main(
                    (real_t)command.waypointB[4]
                ),
                degrees_to_radians_main(
                    (real_t)command.waypointB[5]
                )
            );

        request.endOrientation =
            rotm_to_quat(
                R_B
            );

        request.numGeometryPointsPerSegment =
            100;

        request.arcLengthSpacing =
            0.005;

        request.desiredTCPSpeed =
            (real_t)command.tcpSpeed;

        request.desiredTCPAccel =
            (real_t)command.tcpAccel;

        request.desiredTCPJerk =
            (real_t)command.tcpJerk;

        request.dt =
            0.001;

        adls_default_parameters(
            &request.ikParameters
        );

        printf(
            "\nValidating HMI Cartesian line #%u with streaming ControlCore...\n",
            command.sequence
        );

        fflush(stdout);

        bool validation_success =
            validate_streaming_trajectory(
                &request,
                &trajectory_report
            );

        if (
            planner_is_cancelled()
        )
        {
            abort_streaming_plan(
                true
            );

            printf(
                "Planner #%u cancelled -> HOLD\n",
                command.sequence
            );

            fflush(stdout);
            continue;
        }

        if (
            !validation_success
        )
        {
            printf(
                "\n"
                "============================================================\n"
                " CONTROLCORE PLAN #%u FAILED VALIDATION\n"
                "============================================================\n"
                "The robot remains in HOLD.\n"
                "Joint position limits: %s\n"
                "Joint velocity limits: %s\n"
                "Check reachability, orientation, IK convergence and limits.\n"
                "============================================================\n",
                command.sequence,
                trajectory_report.positionLimitsPass ? "PASS" : "FAIL",
                trajectory_report.velocityLimitsPass ? "PASS" : "FAIL"
            );

            fflush(stdout);

            abort_streaming_plan(
                false
            );

            continue;
        }

        /*
         * Deterministic second pass: generate the exact same q[k] sequence,
         * now feeding the fixed execution FIFO.
         */
        if (
            !single_line_stream_init(
                &trajectory_stream,
                &trajectory_robot,
                &request,
                &trajectory_stream_workspace
            )
        )
        {
            printf(
                "Execution stream init failed: %s\n",
                single_line_stream_status_string(
                    single_line_stream_status(
                        &trajectory_stream
                    )
                )
            );

            abort_streaming_plan(
                false
            );

            continue;
        }

        const size_t total_samples =
            single_line_stream_sample_count(
                &trajectory_stream
            );

        taskENTER_CRITICAL();

        trajectory_total_samples =
            total_samples;

        trajectory_executed_samples =
            0;

        trajectory_index =
            0;

        trajectory_generation_finished =
            false;

        trajectory_have_start_sample =
            false;

        trajectory_have_final_sample =
            false;

        trajectory_ready =
            false;

        trajectory_underrun =
            false;

        taskEXIT_CRITICAL();

        bool ready_announced =
            false;

        bool producer_failed =
            false;

        for (;;)
        {
            if (
                planner_is_cancelled()
            )
            {
                break;
            }

            bool buffer_full;

            taskENTER_CRITICAL();

            buffer_full =
                trajectory_buffer_is_full(
                    &trajectory_buffer
                );

            taskEXIT_CRITICAL();

            if (
                buffer_full
            )
            {
                vTaskDelay(
                    pdMS_TO_TICKS(1)
                );

                continue;
            }

            SingleLineStreamSample generated_sample;

            if (
                !single_line_stream_next(
                    &trajectory_stream,
                    &generated_sample
                )
            )
            {
                if (
                    single_line_stream_is_finished(
                        &trajectory_stream
                    )
                )
                {
                    break;
                }

                printf(
                    "Execution stream failed: %s\n",
                    single_line_stream_status_string(
                        single_line_stream_status(
                            &trajectory_stream
                        )
                    )
                );

                producer_failed =
                    true;

                break;
            }

            bool publish_ready =
                false;

            bool generated_final_sample =
                (
                    single_line_stream_samples_generated(
                        &trajectory_stream
                    ) >=
                    total_samples
                );

            size_t queued_samples =
                0;

            taskENTER_CRITICAL();

            /*
             * STOP might arrive while ADLS is calculating this sample.
             * Never publish a stale post-STOP reference.
             */
            if (
                planner_cancel_requested ||
                stop_requested
            )
            {
                taskEXIT_CRITICAL();
                break;
            }

            if (
                !trajectory_have_start_sample
            )
            {
                trajectory_start_sample =
                    generated_sample.q;

                trajectory_have_start_sample =
                    true;
            }

            if (
                !trajectory_buffer_push(
                    &trajectory_buffer,
                    &generated_sample.q
                )
            )
            {
                taskEXIT_CRITICAL();

                producer_failed =
                    true;

                printf(
                    "ERROR: trajectory buffer push failed unexpectedly\n"
                );

                break;
            }

            if (
                generated_final_sample
            )
            {
                trajectory_final_sample =
                    generated_sample.q;

                trajectory_have_final_sample =
                    true;

                trajectory_generation_finished =
                    true;
            }

            queued_samples =
                trajectory_buffer_count(
                    &trajectory_buffer
                );

            if (
                !trajectory_ready &&
                trajectory_have_start_sample &&
                (
                    queued_samples >=
                        TRAJECTORY_PREFILL_SAMPLES ||
                    trajectory_generation_finished
                )
            )
            {
                trajectory_ready =
                    true;

                publish_ready =
                    true;

                if (
                    !hmi_hold_requested &&
                    motion_state != MOTION_HOLD
                )
                {
                    motion_state =
                        MOTION_PREPOSITION;
                }
            }

            taskEXIT_CRITICAL();

            if (
                publish_ready &&
                !ready_announced
            )
            {
                ready_announced =
                    true;

                printf(
                    "\n"
                    "============================================================\n"
                    " CONTROLCORE PLAN #%u VALIDATED + BUFFERED\n"
                    "============================================================\n"
                    "Waypoint A:        [%.6f %.6f %.6f] m\n"
                    "Waypoint B:        [%.6f %.6f %.6f] m\n"
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
                    command.sequence,
                    command.waypointA[0],
                    command.waypointA[1],
                    command.waypointA[2],
                    command.waypointB[0],
                    command.waypointB[1],
                    command.waypointB[2],
                    trajectory_report.samples,
                    trajectory_report.duration,
                    trajectory_report.pathLength,
                    trajectory_report.peakTCPSpeed,
                    trajectory_report.maxPositionError,
                    trajectory_report.maxOrientationError,
                    queued_samples,
                    (unsigned)TRAJECTORY_BUFFER_CAPACITY,
                    motion_state_name(
                        motion_state
                    )
                );

                fflush(stdout);
            }
        }

        if (
            planner_is_cancelled()
        )
        {
            abort_streaming_plan(
                true
            );

            printf(
                "Planner #%u cancelled -> HOLD\n",
                command.sequence
            );

            fflush(stdout);
            continue;
        }

        if (
            producer_failed
        )
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

            abort_streaming_plan(
                false
            );

            continue;
        }

        taskENTER_CRITICAL();

        if (
            trajectory_generation_finished &&
            !trajectory_ready &&
            trajectory_have_start_sample
        )
        {
            trajectory_ready =
                true;

            if (
                !hmi_hold_requested &&
                motion_state != MOTION_HOLD
            )
            {
                motion_state =
                    MOTION_PREPOSITION;
            }
        }

        planner_busy =
            false;

        taskEXIT_CRITICAL();

        printf(
            "Trajectory producer #%u complete: %zu samples generated.\n",
            command.sequence,
            total_samples
        );

        fflush(stdout);
    }
}


static void EtherCATTask(void *pvParameters)
{
    /* No parameters were passed to this task. */
    (void)pvParameters;

    /*
     * UDP socket used only for MATLAB visualization.
     *
     * Keep it in this function's scope so both:
     *      - error cleanup
     *      - normal shutdown
     * can close it.
     *
     * -1 means the socket has not been created yet.
     */
    int telemetry_socket = -1;

    /* UDP socket that receives commands from the desktop Cartesian HMI. */
    int command_socket = -1;

    printf("EtherCAT task started\n");


    /* ========================================================================
     *  ETHERCAT MASTER STARTUP
     * ========================================================================
     *
     * Same tested startup sequence as before, now routed through EtherCATComm:
     *
     *      open ecatA
     *      -> discover exactly 6 slaves
     *      -> map PDOs
     *      -> configure A6-EC CSP mode
     *      -> configure Distributed Clocks / 1 ms SYNC0
     *      -> SAFE-OP
     *      -> first process-data exchange
     *      -> OPERATIONAL
     *      -> expected WKC
     * ========================================================================
     */

    EtherCATMasterConfig ethercat_config =
    {
        .interfaceName = "ecatA",
        .expectedSlaveCount = NUM_AXES,
        .cycleTimeNs = CYCLE_TIME_NS
    };


    if (
        !ethercat_master_init(
            &ethercat_config
        ) ||
        !ethercat_master_open()
    )
    {
        exit(1);
    }


    int slave_count =
        ethercat_master_scan();


    if (
        slave_count <= 0 ||
        slave_count != NUM_AXES
    )
    {
        ethercat_master_close();
        exit(1);
    }


    ethercat_master_map_pdos();


    /* ========================================================================
     *  CONFIGURE EVERY A6-EC DRIVE FOR CSP
     * ========================================================================
     */

    printf(
        "\nSetting all drives to CSP...\n"
    );


    for (int slave = 1;
         slave <= NUM_AXES;
         slave++)
    {
        int8_t mode_readback =
            0;


        if (
            a6ec_set_csp_mode(
                slave,
                &mode_readback
            )
        )
        {
            printf(
                "Slave %d: CSP write OK | "
                "0x6060 readback = %d\n",
                slave,
                mode_readback
            );
        }
        else
        {
            printf(
                "Slave %d: ERROR configuring 0x6060\n",
                slave
            );


            while (
                ethercat_master_has_error()
            )
            {
                printf(
                    "    SOEM: %s\n",
                    ethercat_master_pop_error_string()
                );
            }
        }
    }


    ethercat_master_configure_distributed_clocks();


    ethercat_master_wait_for_safe_op();


    /* Same initial process-data exchange used before OPERATIONAL. */
    ethercat_master_exchange();


    if (
        !ethercat_master_request_operational()
    )
    {
        printf(
            "ERROR: EtherCAT bus did not reach OPERATIONAL\n"
        );

        ethercat_master_close();
        exit(1);
    }


    int expected_wkc =
        ethercat_master_expected_wkc();


    /* ========================================================================
     *  8. PREPARE THE 1 ms CYCLIC CSP TEST
     * ========================================================================
     */

    printf(
        "\nStarting 6-axis CSP + Cartesian HMI trajectory controller\n"
    );

    printf(
        "Press Ctrl+C for clean disable\n\n"
    );


    /*
     * Used to print diagnostics every 1000 cycles ~= once per second.
     */
    int print_counter = 0;

    /*
     * FreeRTOS tick timestamp used by vTaskDelayUntil().
     *
     * vTaskDelayUntil() is preferable to simply calling vTaskDelay(1) because
     * it tries to maintain a periodic schedule relative to the previous wake
     * time instead of accumulating execution-time drift.
     */
    TickType_t last_wake_time =
        xTaskGetTickCount();

    /*
     * Desired FreeRTOS task period = 1 ms.
     *
     * This assumes the FreeRTOS tick configuration can represent 1 ms.
     */
    const TickType_t cycle_period =
        pdMS_TO_TICKS(1);


    /* ========================================================================
     *  MATLAB UDP TELEMETRY SETUP
     * ========================================================================
     *
     * This path is separate from EtherCAT.
     *
     * EtherCAT:
     *      controls the simulated servo drives.
     *
     * UDP:
     *      sends joint feedback to MATLAB for visualization.
     */

    /*
     * Create an IPv4 UDP socket.
     */
    telemetry_socket =
        socket(AF_INET, SOCK_DGRAM, 0);

    if (telemetry_socket < 0)
    {
        printf("ERROR: Could not create telemetry UDP socket\n");
    }

    /*
     * Destination address structure for MATLAB.
     */
    struct sockaddr_in matlab_addr;

    memset(
        &matlab_addr,
        0,
        sizeof(matlab_addr)
    );

    /* IPv4 */
    matlab_addr.sin_family = AF_INET;

    /*
     * UDP port numbers are stored in network byte order.
     */
    matlab_addr.sin_port =
        htons(MATLAB_PORT);

    /*
     * Convert MATLAB_IP from text:
     *
     *      "172.18.160.1"
     *
     * into the binary IPv4 address stored in matlab_addr.
     */
    if (inet_pton(
            AF_INET,
            MATLAB_IP,
            &matlab_addr.sin_addr
        ) != 1)
    {
        printf("ERROR: Invalid MATLAB IP address\n");
    }

    /*
     * We do NOT send to MATLAB every 1 ms.
     *
     * 20 EtherCAT cycles x 1 ms = 20 ms
     *
     * Therefore:
     *
     *      1 / 0.020 s = 50 Hz telemetry.
     */
    uint32_t telemetry_counter = 0;

    printf(
        "MATLAB telemetry -> %s:%d\n",
        MATLAB_IP,
        MATLAB_PORT
    );


    /* ========================================================================
     *  CARTESIAN HMI UDP COMMAND SETUP
     * ========================================================================
     *
     * hmi.c runs as a separate desktop process and sends commands to UDP 5006.
     *
     * The EtherCAT loop polls this socket with MSG_DONTWAIT, so the HMI can
     * never block the 1 ms control loop.
     */

    command_socket =
        socket(AF_INET, SOCK_DGRAM, 0);

    if (command_socket < 0)
    {
        perror("ERROR: Could not create HMI command socket");

        if (telemetry_socket >= 0)
        {
            close(telemetry_socket);
            telemetry_socket = -1;
        }

        ethercat_master_close();
        exit(1);
    }

    struct sockaddr_in command_addr;

    memset(
        &command_addr,
        0,
        sizeof(command_addr)
    );

    command_addr.sin_family = AF_INET;
    command_addr.sin_port = htons(HMI_PORT);
    command_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(
            command_socket,
            (struct sockaddr *)&command_addr,
            sizeof(command_addr)
        ) < 0)
    {
        perror("ERROR: Could not bind HMI command socket");

        close(command_socket);
        command_socket = -1;

        if (telemetry_socket >= 0)
        {
            close(telemetry_socket);
            telemetry_socket = -1;
        }

        ethercat_master_close();
        exit(1);
    }

    printf(
        "Cartesian HMI commands <- UDP :%d | protocol RBT2\n",
        HMI_PORT
    );

    printf(
        "Controller is HOLDING current position and waiting for "
        "PLAN + RUN A -> B.\n"
    );


    /*
     * Controller -> HMI live state-machine telemetry destination.
     *
     * The command socket is reused for these outgoing UDP packets.
     */
    struct sockaddr_in hmi_status_address;

    memset(
        &hmi_status_address,
        0,
        sizeof(hmi_status_address)
    );


    hmi_status_address.sin_family =
        AF_INET;

    hmi_status_address.sin_port =
        htons(
            HMI_STATUS_PORT
        );


    if (
        inet_pton(
            AF_INET,
            HMI_STATUS_IP,
            &hmi_status_address.sin_addr
        ) != 1
    )
    {
        printf(
            "ERROR: Invalid HMI status IP address\n"
        );
    }


    printf(
        "HMI live state -> %s:%d | protocol STA2\n",
        HMI_STATUS_IP,
        HMI_STATUS_PORT
    );


    uint32_t hmi_status_counter =
        0;


    uint32_t last_hmi_sequence =
        0;


    trajectory_index =
        0;


    /* ========================================================================
     *                          CYCLIC CONTROL LOOP
     * ========================================================================
     *
     * This loop is the heart of the test.
     *
     * Nominally once every 1 ms:
     *
     *      1. read previous Statusword
     *      2. calculate next Controlword
     *      3. select the current buffered ControlCore q sample
     *      4. convert radians -> A6 position units and write output PDO
     *      5. send EtherCAT process data
     *      6. receive EtherCAT process data
     *      7. inspect WKC
     *      8. send telemetry to MATLAB
     *      9. sleep until the next cycle
     */

    for (;;)
    {
        int wkc;


        /*
         * Read the previous cycle's drive states and actual positions.
         *
         * current_q is used only as the initial ADLS seed when a new PLAN
         * command is accepted.
         */
        bool all_operation_enabled =
            true;

        JointVector current_q;


        for (int slave = 1;
             slave <= NUM_AXES;
             slave++)
        {
            A6ECPDOFeedback feedback;

            a6ec_read_feedback(
                slave,
                &feedback
            );


            uint16_t drive_state =
                cia402_get_state(
                    feedback.statusword
                );


            if (
                drive_state !=
                CIA402_STATE_OPERATION_ENABLED
            )
            {
                all_operation_enabled =
                    false;
            }


            current_q.q[slave - 1] =
                a6ec_position_units_to_joint_rad(
                    feedback.actualPosition
                );
        }


        /*
         * PREPOSITION is allowed only after full-path validation and q[0]
         * publication.
         */
        bool all_at_trajectory_start =
            trajectory_ready &&
            trajectory_have_start_sample;


        /*
         * Poll RBT2 without blocking the 1 ms EtherCAT task.
         */
        receive_hmi_commands(
            command_socket,
            &current_q,
            &last_hmi_sequence
        );


        /*
         * Select exactly ONE trajectory sample for this complete six-axis
         * EtherCAT cycle. All six joints use the same k.
         */
        JointVector cycle_trajectory_sample;

        bool cycle_has_trajectory_sample =
            false;

        bool cycle_sample_is_final =
            false;

        bool buffer_underrun_now =
            false;

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
                trajectory_buffer_pop(
                    &trajectory_buffer,
                    &cycle_trajectory_sample
                )
            )
            {
                cycle_has_trajectory_sample =
                    true;

                trajectory_index =
                    trajectory_executed_samples;

                trajectory_executed_samples++;

                if (
                    trajectory_generation_finished &&
                    trajectory_buffer_is_empty(
                        &trajectory_buffer
                    ) &&
                    trajectory_executed_samples >=
                        trajectory_total_samples
                )
                {
                    cycle_sample_is_final =
                        true;
                }
            }
            else if (
                !trajectory_generation_finished
            )
            {
                trajectory_underrun =
                    true;

                trajectory_ready =
                    false;

                hmi_hold_requested =
                    true;

                planner_cancel_requested =
                    true;

                motion_state =
                    MOTION_HOLD;

                buffer_underrun_now =
                    true;
            }

            taskEXIT_CRITICAL();
        }

        if (
            buffer_underrun_now
        )
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


        /* ====================================================================
         *  PREPARE PDO COMMANDS FOR ALL SIX SERVO DRIVES
         * ====================================================================
         */

        for (int slave = 1;
             slave <= NUM_AXES;
             slave++)
        {
            /*
             * Read this A6-EC drive's current feedback from the mapped TPDO.
             * The byte offsets now live only in ServoDrive/A6EC/a6ec_pdo.*.
             */
            A6ECPDOFeedback feedback;

            a6ec_read_feedback(
                slave,
                &feedback
            );


            uint16_t drive_state =
                cia402_get_state(
                    feedback.statusword
                );


            uint16_t controlword;


            if (stop_requested)
            {
                controlword =
                    cia402_get_disable_controlword(
                        drive_state
                    );
            }
            else
            {
                controlword =
                    cia402_get_enable_controlword(
                        drive_state
                    );
            }


            const int joint =
                slave - 1;


            int32_t actual_position =
                feedback.actualPosition;


            int32_t start_position =
                actual_position;


            if (
                trajectory_ready &&
                trajectory_have_start_sample
            )
            {
                start_position =
                    a6ec_joint_rad_to_position_units(
                        trajectory_start_sample.q[joint]
                    );


                long long start_error =
                    (long long)actual_position -
                    (long long)start_position;


                if (
                    llabs(start_error) >
                    5
                )
                {
                    all_at_trajectory_start =
                        false;
                }
            }
            else
            {
                all_at_trajectory_start =
                    false;
            }


            int32_t target_position =
                actual_position;


            if (
                stop_requested ||
                !all_operation_enabled ||
                hmi_hold_requested
            )
            {
                target_position =
                    actual_position;
            }
            else
            {
                MotionState state =
                    motion_state;


                if (
                    state == MOTION_WAITING_FOR_PLAN ||
                    state == MOTION_PLANNING ||
                    state == MOTION_HOLD
                )
                {
                    target_position =
                        actual_position;
                }
                else if (
                    state == MOTION_PREPOSITION &&
                    trajectory_have_start_sample
                )
                {
                    /*
                     * Simulator-only pre-position to q[0].
                     * Final hardware should replace this with a planned safe
                     * point-to-point move.
                     */
                    target_position =
                        start_position;
                }
                else if (
                    state == MOTION_RUNNING &&
                    cycle_has_trajectory_sample
                )
                {
                    target_position =
                        a6ec_joint_rad_to_position_units(
                            cycle_trajectory_sample.q[joint]
                        );
                }
                else if (
                    state == MOTION_FINISHED &&
                    trajectory_have_final_sample
                )
                {
                    target_position =
                        a6ec_joint_rad_to_position_units(
                            trajectory_final_sample.q[joint]
                        );
                }
            }


            /*
             * Same command as before, now encoded by the A6-EC PDO module.
             */
            A6ECPDOCommand command =
            {
                .controlword = controlword,
                .targetPosition = target_position
            };


            a6ec_write_command(
                slave,
                &command
            );

        }


        /* ====================================================================
         *  EXCHANGE ETHERCAT PROCESS DATA
         * ====================================================================
         *
         * Up to this point we only MODIFIED local output memory.
         *
         * Nothing reaches the EtherCAT slaves until we call:
         *
         *      ecx_send_processdata()
         *
         * Then we receive the response with:
         *
         *      ecx_receive_processdata()
         */

        wkc =
            ethercat_master_exchange();


        /* ====================================================================
         *  CONTROLCORE STREAMING TRAJECTORY EXECUTION STATE
         * ====================================================================
         */

        if (
            !stop_requested &&
            !hmi_hold_requested &&
            trajectory_ready
        )
        {
            /*
             * PREPOSITION -> RUNNING.
             *
             * The FIFO is not consumed during PREPOSITION.
             */
            if (
                motion_state ==
                MOTION_PREPOSITION
            )
            {
                if (
                    all_operation_enabled &&
                    all_at_trajectory_start
                )
                {
                    trajectory_index =
                        0;

                    trajectory_executed_samples =
                        0;

                    motion_state =
                        MOTION_RUNNING;

                    printf(
                        "\n"
                        "============================================================\n"
                        " CONTROLCORE STREAMING EXECUTION START\n"
                        "============================================================\n"
                        "Samples:       %zu\n"
                        "dt:            0.001 s\n"
                        "Duration:      %.6f s\n"
                        "FIFO capacity: %u samples\n"
                        "============================================================\n",
                        trajectory_total_samples,
                        trajectory_report.duration,
                        (unsigned)TRAJECTORY_BUFFER_CAPACITY
                    );

                    fflush(stdout);
                }
            }
            else if (
                motion_state ==
                    MOTION_RUNNING &&
                cycle_has_trajectory_sample &&
                cycle_sample_is_final
            )
            {
                motion_state =
                    MOTION_FINISHED;

                printf(
                    "\n"
                    "============================================================\n"
                    " CONTROLCORE STREAMING EXECUTION COMPLETE\n"
                    "============================================================\n"
                    "Final sample: %zu / %zu\n"
                    "Duration:     %.6f s\n"
                    "============================================================\n",
                    trajectory_executed_samples,
                    trajectory_total_samples,
                    trajectory_report.duration
                );

                fflush(stdout);
            }
        }


        /* ====================================================================
         *  SEND LIVE CONTROLLER / CiA-402 STATE TO HMI AT 10 Hz
         * ====================================================================
         *
         * 100 EtherCAT cycles x 1 ms = 100 ms.
         */

        if (++hmi_status_counter >= 100)
        {
            hmi_status_counter =
                0;


            send_hmi_status(
                command_socket,
                &hmi_status_address,
                wkc,
                expected_wkc,
                last_hmi_sequence
            );
        }


        /* ====================================================================
         *  SEND JOINT FEEDBACK TO MATLAB AT ~50 Hz
         * ====================================================================
         */

        if (++telemetry_counter >= 20)
        {
            telemetry_counter = 0;

            /*
             * One signed 32-bit actual position for each of the six joints.
             *
             * MATLAB receives this 24-byte packet:
             *
             *      6 joints x 4 bytes = 24 bytes
             */
            int32_t actual_positions[NUM_AXES];

            for (int slave = 1;
                 slave <= NUM_AXES;
                 slave++)
            {
                A6ECPDOFeedback feedback;

                a6ec_read_feedback(
                    slave,
                    &feedback
                );


                actual_positions[slave - 1] =
                    feedback.actualPosition;
            }

            /*
             * Send the six raw int32 joint positions to MATLAB over UDP.
             *
             * There is currently:
             *      - no timestamp
             *      - no packet header
             *      - no angle conversion
             *
             * It is deliberately minimal for visualization testing.
             */
            sendto(
                telemetry_socket,
                actual_positions,
                sizeof(actual_positions),
                0,
                (struct sockaddr *)&matlab_addr,
                sizeof(matlab_addr)
            );
        }


        /* ====================================================================
         *  FAILURE-DETECTION TEST USING WKC
         * ====================================================================
         *
         * If actual WKC is lower than expected, not every expected EtherCAT
         * operation was successfully processed.
         */

        if (wkc < expected_wkc)
        {
            printf(
                "\nWARNING: EtherCAT "
                "communication problem! "
                "WKC=%d expected=%d\n",
                wkc,
                expected_wkc
            );

            fflush(stdout);
        }


        /* ====================================================================
         *  PRINT DIAGNOSTICS ONCE PER SECOND
         * ====================================================================
         */

        if (++print_counter >= 1000)
        {
            print_counter = 0;

            /*
             * Print overall EtherCAT health first.
             */
            printf(
                "WKC=%d/%d | STATE=%s",
                wkc,
                expected_wkc,
                motion_state_name(motion_state)
            );


            /*
             * Then print each joint:
             *
             *      T  = Target Position
             *      A  = Actual Position
             *      SW = Statusword
             */
            for (int slave = 1;
                 slave <= NUM_AXES;
                 slave++)
            {
                A6ECPDOCommand command;
                A6ECPDOFeedback feedback;


                a6ec_read_command(
                    slave,
                    &command
                );


                a6ec_read_feedback(
                    slave,
                    &feedback
                );


                int32_t target =
                    command.targetPosition;


                int32_t actual =
                    feedback.actualPosition;


                uint16_t status =
                    feedback.statusword;


                printf(
                    " | J%d T=%d A=%d "
                    "SW=0x%04X",
                    slave,
                    target,
                    actual,
                    status
                );
            }


            printf("\n");

            /*
             * Force buffered console output to appear immediately.
             */
            fflush(stdout);
        }


        /* ====================================================================
         *  CLEAN SHUTDOWN CHECK
         * ====================================================================
         *
         * Once Ctrl+C sets stop_requested:
         *
         *      cia402_get_disable_controlword()
         *
         * progressively walks every drive back toward:
         *
         *      Switch On Disabled = 0x0040
         *
         * We only terminate the application after ALL six drives report that
         * state.
         */

        if (stop_requested)
        {
            /*
             * Assume success until one drive proves otherwise.
             */
            int all_disabled = 1;


            for (int slave = 1;
                 slave <= NUM_AXES;
                 slave++)
            {
                A6ECPDOFeedback feedback;

                a6ec_read_feedback(
                    slave,
                    &feedback
                );


                uint16_t drive_state =
                    cia402_get_state(
                        feedback.statusword
                    );


                if (
                    drive_state !=
                    CIA402_STATE_SWITCH_ON_DISABLED
                )
                {
                    all_disabled = 0;
                }
            }


            /*
             * Only close EtherCAT after every drive confirms it is disabled.
             */
            if (all_disabled)
            {
                printf(
                    "\nAll drives are "
                    "Switch On Disabled\n"
                );

                printf(
                    "Closing EtherCAT...\n"
                );

                /* Close MATLAB telemetry socket if it was created. */
                if (telemetry_socket >= 0)
                {
                    close(telemetry_socket);
                    telemetry_socket = -1;
                }


                /* Close Cartesian-HMI command socket. */
                if (command_socket >= 0)
                {
                    close(command_socket);
                    command_socket = -1;
                }


                /*
                 * Disable SYNC0 on DC-capable slaves and close the EtherCAT
                 * master backend.
                 */
                ethercat_master_close();

                printf(
                    "EtherCAT closed cleanly\n"
                );

                /*
                 * End the whole PC test process.
                 */
                exit(0);
            }
        }


        /*
         * Wait until the next periodic FreeRTOS wake time.
         *
         * Nominally:
         *
         *      cycle 0
         *        |
         *        +---- 1 ms ----> cycle 1
         *                          |
         *                          +---- 1 ms ----> cycle 2
         *
         * This is what gives our PC test its 1 ms cyclic structure.
         *
         * IMPORTANT FOR THE REAL ROBOT:
         * A FreeRTOS/PC timing test is not automatically equivalent to a hard
         * real-time EtherCAT implementation. Real hardware timing, task
         * priority, network driver behavior, DC synchronization, jitter and
         * safety handling must still be validated.
         */
        vTaskDelayUntil(
            &last_wake_time,
            cycle_period
        );
    }
}


/* ============================================================================
 *                                  main()
 * ============================================================================
 *
 * main() itself does very little.
 *
 * Its job is:
 *
 *      1. install Ctrl+C handler
 *      2. create the EtherCAT FreeRTOS task
 *      3. start the FreeRTOS scheduler
 *
 * After vTaskStartScheduler(), EtherCATTask() does the actual work.
 */

int main(void)
{
    /*
     * Bind all ControlCore static buffers.
     *
     * No fixed trajectory is generated here anymore.
     */
    initialize_trajectory_storage();


    /*
     * One-element mailbox:
     *
     * EtherCAT task -> Planner task
     */
    trajectory_request_queue =
        xQueueCreate(
            1,
            sizeof(TrajectoryPlanCommand)
        );


    if (
        trajectory_request_queue ==
        NULL
    )
    {
        printf(
            "Failed to create trajectory planner queue\n"
        );

        return 1;
    }


    /*
     * Register Ctrl+C handler.
     */
    signal(
        SIGINT,
        handle_sigint
    );


    printf(
        "FreeRTOS starting\n"
    );

    printf(
        "No trajectory is preloaded. Waiting for RBT2 HMI PLAN + RUN.\n"
    );

    fflush(stdout);


    /*
     * Planner deliberately runs at lower priority than EtherCAT.
     *
     * This allows thousands of ADLS calculations to happen while the EtherCAT
     * task continues waking every millisecond to exchange PDOs.
     */
    BaseType_t planner_result =
        xTaskCreate(
            TrajectoryPlannerTask,
            "Planner",
            2048,
            NULL,
            tskIDLE_PRIORITY,
            NULL
        );


    if (planner_result != pdPASS)
    {
        printf(
            "Failed to create trajectory planner task\n"
        );

        return 1;
    }


    /*
     * EtherCAT cyclic task.
     */
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
        printf(
            "Failed to create EtherCAT task\n"
        );

        return 1;
    }


    vTaskStartScheduler();


    /*
     * Under normal operation the scheduler does not return.
     */
    return 0;
}

