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
#include "soem/soem.h"
#include "robot_config.h"
#include "single_segment_line.h"
#include "math3d.h"

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
 * CiA-402 mode value:
 *
 *      8 = CSP = Cyclic Synchronous Position
 *
 * In CSP, the EtherCAT master continuously sends a new target position every
 * control cycle.
 */
#define CSP_MODE 8

/*
 * Desired EtherCAT Distributed-Clock SYNC0 period:
 *
 *      1,000,000 ns = 1 ms
 *
 * This requests one synchronization event every millisecond.
 */
#define CYCLE_TIME_NS 1000000U


/* ============================================================================
 *                           GLOBAL ETHERCAT DATA
 * ============================================================================
 */

/*
 * SOEM context.
 *
 * SOEM stores its entire EtherCAT-master state here:
 *      - discovered slaves
 *      - slave states
 *      - process-data pointers
 *      - EtherCAT errors
 *      - group information
 *      - Distributed Clock information
 *      - etc.
 *
 * Because the "ecx_*" SOEM API is used, the context is passed explicitly to
 * most SOEM functions.
 */
static ecx_contextt soem_context;

/*
 * EtherCAT process-data memory.
 *
 * During PDO mapping, SOEM uses this array as the shared process-data area.
 *
 * Conceptually:
 *
 *      IOmap
 *      +------------------------------------------------------+
 *      | slave outputs | slave inputs | ...                   |
 *      +------------------------------------------------------+
 *
 * SOEM later makes each slave's .outputs and .inputs pointers point into the
 * correct locations inside this memory.
 */
static uint8_t IOmap[4096];

/* ============================================================================
 * CONTROLCORE SINGLE-SEGMENT TRAJECTORY
 * ============================================================================
 */

#define TRAJECTORY_CAPACITY 7000
#define GEOMETRY_CAPACITY   512


/* Robot model. */
static RobotConfig trajectory_robot;


/* --------------------------------------------------------------------------
 * Geometry workspace
 * --------------------------------------------------------------------------
 */

static Vec3 trajectory_raw_geometry[GEOMETRY_CAPACITY];
static Vec3 trajectory_arc_geometry[GEOMETRY_CAPACITY];

static real_t trajectory_l_original[GEOMETRY_CAPACITY];
static real_t trajectory_l_arc[GEOMETRY_CAPACITY];


/* --------------------------------------------------------------------------
 * S-curve workspace
 * --------------------------------------------------------------------------
 */

static real_t trajectory_temp_t[TRAJECTORY_CAPACITY];
static real_t trajectory_temp_s[TRAJECTORY_CAPACITY];
static real_t trajectory_temp_s_dot[TRAJECTORY_CAPACITY];
static real_t trajectory_temp_s_ddot[TRAJECTORY_CAPACITY];
static real_t trajectory_temp_s_dddot[TRAJECTORY_CAPACITY];


/* ADLS scratch data. */
static ADLSInfo trajectory_ik_scratch;


/* --------------------------------------------------------------------------
 * Final trajectory
 * --------------------------------------------------------------------------
 */

static real_t trajectory_t[TRAJECTORY_CAPACITY];
static real_t trajectory_s[TRAJECTORY_CAPACITY];
static real_t trajectory_s_dot[TRAJECTORY_CAPACITY];
static real_t trajectory_s_ddot[TRAJECTORY_CAPACITY];
static real_t trajectory_s_dddot[TRAJECTORY_CAPACITY];

static real_t trajectory_arc_position[TRAJECTORY_CAPACITY];
static real_t trajectory_tcp_speed[TRAJECTORY_CAPACITY];

static Vec3 trajectory_position[TRAJECTORY_CAPACITY];
static Quat trajectory_quaternion[TRAJECTORY_CAPACITY];

static JointVector trajectory_q[TRAJECTORY_CAPACITY];
static JointVector trajectory_q_dot[TRAJECTORY_CAPACITY];
static JointVector trajectory_q_ddot[TRAJECTORY_CAPACITY];

static int trajectory_ik_iterations[TRAJECTORY_CAPACITY];

static real_t trajectory_position_error[TRAJECTORY_CAPACITY];
static real_t trajectory_orientation_error[TRAJECTORY_CAPACITY];


static SingleLineWorkspace trajectory_workspace;

static SingleLineTrajectory trajectory;

static SingleLineReport trajectory_report;


/*
 * Number of the trajectory point currently being executed later.
 */
static size_t trajectory_index = 0;


/*
 * True once ControlCore has successfully planned the complete motion.
 */
static volatile bool trajectory_ready = false;


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


/* ============================================================================
 *                         ETHERCAT STATE PRINT HELPER
 * ============================================================================
 */

/*
 * EtherCAT itself has communication states:
 *
 *      INIT
 *        ->
 *      PRE-OP
 *        ->
 *      SAFE-OP
 *        ->
 *      OPERATIONAL
 *
 * These are NOT the same thing as the CiA-402 servo-drive states
 * (Switch On Disabled, Ready to Switch On, etc.).
 *
 * This helper only converts SOEM's EtherCAT state number to readable text.
 */
static const char *state_name(uint16_t state)
{
    switch (state)
    {
        case EC_STATE_INIT:
            return "INIT";

        case EC_STATE_PRE_OP:
            return "PRE-OP";

        case EC_STATE_SAFE_OP:
            return "SAFE-OP";

        case EC_STATE_OPERATIONAL:
            return "OPERATIONAL";

        /*
         * EtherCAT can report SAFE-OP together with an error flag.
         */
        case EC_STATE_SAFE_OP + EC_STATE_ERROR:
            return "SAFE-OP + ERROR";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================================
 *                         PDO BYTE-ORDER HELPERS
 * ============================================================================
 *
 * EtherCAT process data is transmitted little-endian.
 *
 * We therefore should not directly cast raw byte pointers into integer
 * pointers. These helper functions:
 *
 *      1. convert between CPU byte order and EtherCAT byte order
 *      2. copy the bytes safely using memcpy()
 *
 * This also avoids possible alignment problems.
 */


/*
 * Write unsigned 16-bit value into EtherCAT process data.
 *
 * Used for values such as the CiA-402 Controlword.
 */
static void write_u16(uint8_t *p, uint16_t value)
{
    /*
     * htoes()
     * = host-to-EtherCAT-short
     *
     * Converts the CPU representation to EtherCAT's 16-bit byte order.
     */
    uint16_t ethercat_value = htoes(value);

    /*
     * Copy exactly two bytes into the PDO memory.
     */
    memcpy(
        p,
        &ethercat_value,
        sizeof(ethercat_value)
    );
}


/*
 * Write signed 32-bit value into EtherCAT process data.
 *
 * Used here for target position.
 */
static void write_i32(uint8_t *p, int32_t value)
{
    /*
     * htoel()
     * = host-to-EtherCAT-long
     *
     * SOEM's helper works on uint32_t, so cast first.
     * The bit pattern remains the same for a signed two's-complement int32_t.
     */
    uint32_t ethercat_value =
        htoel((uint32_t)value);

    memcpy(
        p,
        &ethercat_value,
        sizeof(ethercat_value)
    );
}


/*
 * Read an unsigned 16-bit value from EtherCAT process data.
 *
 * Used here for the CiA-402 Statusword.
 */
static uint16_t read_u16(const uint8_t *p)
{
    uint16_t value;

    memcpy(
        &value,
        p,
        sizeof(value)
    );

    /*
     * etohs()
     * = EtherCAT-to-host-short.
     */
    return etohs(value);
}


/*
 * Read a signed 32-bit value from EtherCAT process data.
 *
 * Used here for actual position.
 */
static int32_t read_i32(const uint8_t *p)
{
    uint32_t value;

    memcpy(
        &value,
        p,
        sizeof(value)
    );

    /*
     * Convert EtherCAT byte order back to CPU byte order.
     */
    value = etohl(value);

    /*
     * Reinterpret the resulting 32-bit bit pattern as signed position data.
     */
    return (int32_t)value;
}

/* ============================================================================
 * ROBOT JOINT ANGLE -> A6 POSITION REFERENCE UNIT
 * ============================================================================
 *
 * A6-EC:
 *      17-bit encoder
 *      131072 counts / revolution
 *
 * Current KickCAT test:
 *      electronic gear = 1:1
 *
 * Therefore:
 *
 *      drive_units = q_rad * 131072 / (2*pi)
 *
 * NOTE:
 * The real robot will later need:
 *      - gearbox ratios
 *      - joint direction signs
 *      - mechanical zero offsets
 *
 * Those are deliberately NOT added yet.
 * ============================================================================
 */

#define A6_POSITION_UNITS_PER_REV 131072.0

static int32_t joint_rad_to_drive_units(
    real_t q_rad
)
{
    const double two_pi =
        6.28318530717958647692;

    return (int32_t)llround(
        q_rad *
        A6_POSITION_UNITS_PER_REV /
        two_pi
    );
}


/*
 * Inverse conversion used to seed IK from the current simulated drive
 * feedback.
 */
static real_t drive_units_to_joint_rad(
    int32_t drive_units
)
{
    const real_t two_pi =
        6.28318530717958647692;

    return
        ((real_t)drive_units) *
        two_pi /
        A6_POSITION_UNITS_PER_REV;
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
            trajectory_ready
                ? (uint32_t)trajectory.count
                : 0U
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
        uint8_t *inputs =
            soem_context
                .slavelist[slave]
                .inputs;


        uint16_t statusword =
            read_u16(
                inputs + 2
            );


        packet[
            7 +
            (slave - 1)
        ] =
            htonl(
                (uint32_t)statusword
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
 *                       CiA-402 ENABLE STATE MACHINE
 * ============================================================================
 *
 * EtherCAT communication being OPERATIONAL does NOT automatically mean the
 * servo motor is enabled.
 *
 * The servo itself follows the separate CiA-402 drive state machine:
 *
 *      Switch On Disabled
 *              |
 *              | Controlword 0x0006
 *              v
 *      Ready to Switch On
 *              |
 *              | Controlword 0x0007
 *              v
 *      Switched On
 *              |
 *              | Controlword 0x000F
 *              v
 *      Operation Enabled
 *
 * The current drive state is inferred from the Statusword.
 *
 * This function receives the decoded drive_state bits and returns the
 * Controlword that should be sent next.
 */

static uint16_t get_enable_controlword(
    uint16_t drive_state
)
{
    if (drive_state == 0x0040)
    {
        /*
         * Current state:
         *      Switch On Disabled
         *
         * Send:
         *      0x0006 = Shutdown command
         *
         * Desired next state:
         *      Ready to Switch On
         */
        return 0x0006;
    }

    if (drive_state == 0x0021)
    {
        /*
         * Current state:
         *      Ready to Switch On
         *
         * Send:
         *      0x0007 = Switch On
         *
         * Desired next state:
         *      Switched On
         */
        return 0x0007;
    }

    if (drive_state == 0x0023)
    {
        /*
         * Current state:
         *      Switched On
         *
         * Send:
         *      0x000F = Enable Operation
         *
         * Desired next state:
         *      Operation Enabled
         */
        return 0x000F;
    }

    if (drive_state == 0x0027)
    {
        /*
         * Current state:
         *      Operation Enabled
         *
         * Keep sending 0x000F so the drive remains enabled.
         */
        return 0x000F;
    }

    /*
     * Fallback:
     *
     * If we encounter an unhandled/non-fault state, start again with the
     * Shutdown command.
     *
     * NOTE:
     * Real hardware will need more complete handling for FAULT, QUICK STOP,
     * FAULT REACTION ACTIVE, etc.
     */
    return 0x0006;
}


/* ============================================================================
 *                       CiA-402 DISABLE STATE MACHINE
 * ============================================================================
 *
 * This performs the reverse sequence when Ctrl+C is pressed.
 *
 *      Operation Enabled
 *              |
 *              | 0x0007
 *              v
 *      Switched On
 *              |
 *              | 0x0006
 *              v
 *      Ready to Switch On
 *              |
 *              | 0x0000
 *              v
 *      Switch On Disabled
 *
 * The point is to shut the drives down cleanly instead of instantly exiting
 * while they are still enabled.
 */

static uint16_t get_disable_controlword(
    uint16_t drive_state
)
{
    if (drive_state == 0x0027)
    {
        /*
         * Operation Enabled -> Switched On
         */
        return 0x0007;
    }

    if (drive_state == 0x0023)
    {
        /*
         * Switched On -> Ready to Switch On
         */
        return 0x0006;
    }

    if (drive_state == 0x0021)
    {
        /*
         * Ready to Switch On -> Switch On Disabled
         */
        return 0x0000;
    }

    /*
     * Default shutdown request.
     */
    return 0x0000;
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
            trajectory_ready =
                false;

            trajectory_index =
                0;

            hmi_hold_requested =
                false;

            planner_busy =
                true;

            motion_state =
                MOTION_PLANNING;


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


            hmi_hold_requested =
                true;

            motion_state =
                MOTION_HOLD;


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
 * INITIALIZE CONTROLCORE TRAJECTORY STORAGE
 * ============================================================================
 *
 * This only binds the static workspace/output buffers.
 *
 * It does NOT create a trajectory.
 *
 * The controller starts in HOLD and waits for an RBT2 PLAN + RUN request from
 * hmi.c.
 * ============================================================================
 */

static void initialize_trajectory_storage(void)
{
    robot_config_init_ur5(
        &trajectory_robot
    );


    trajectory_workspace.geometryCapacity =
        GEOMETRY_CAPACITY;

    trajectory_workspace.rawGeometry =
        trajectory_raw_geometry;

    trajectory_workspace.arcGeometry =
        trajectory_arc_geometry;

    trajectory_workspace.lOriginal =
        trajectory_l_original;

    trajectory_workspace.lArc =
        trajectory_l_arc;


    trajectory_workspace.timeCapacity =
        TRAJECTORY_CAPACITY;

    trajectory_workspace.tempT =
        trajectory_temp_t;

    trajectory_workspace.tempS =
        trajectory_temp_s;

    trajectory_workspace.tempSDot =
        trajectory_temp_s_dot;

    trajectory_workspace.tempSDDot =
        trajectory_temp_s_ddot;

    trajectory_workspace.tempSDDDot =
        trajectory_temp_s_dddot;

    trajectory_workspace.ikScratch =
        &trajectory_ik_scratch;


    trajectory.capacity =
        TRAJECTORY_CAPACITY;

    trajectory.count =
        0;

    trajectory.t =
        trajectory_t;

    trajectory.s =
        trajectory_s;

    trajectory.sDot =
        trajectory_s_dot;

    trajectory.sDDot =
        trajectory_s_ddot;

    trajectory.sDDDot =
        trajectory_s_dddot;

    trajectory.arcPosition =
        trajectory_arc_position;

    trajectory.tcpSpeed =
        trajectory_tcp_speed;

    trajectory.pDesired =
        trajectory_position;

    trajectory.quatDesired =
        trajectory_quaternion;

    trajectory.qPath =
        trajectory_q;

    trajectory.qDot =
        trajectory_q_dot;

    trajectory.qDDot =
        trajectory_q_ddot;

    trajectory.ikIterations =
        trajectory_ik_iterations;

    trajectory.positionError =
        trajectory_position_error;

    trajectory.orientationError =
        trajectory_orientation_error;


    trajectory_index =
        0;

    trajectory_ready =
        false;

    motion_state =
        MOTION_WAITING_FOR_PLAN;

    hmi_hold_requested =
        true;

    planner_busy =
        false;
}


/* ============================================================================
 * LOWER-PRIORITY CONTROLCORE PLANNER TASK
 * ============================================================================
 *
 * The EtherCAT task must keep cycling every 1 ms.
 *
 * Sequential ADLS over thousands of points can take much longer than one
 * EtherCAT cycle, so trajectory generation must NOT execute inside the cyclic
 * EtherCAT loop.
 *
 * The EtherCAT task:
 *
 *      receives RBT2
 *          ->
 *      snapshots current q as qSeed
 *          ->
 *      queues TrajectoryPlanCommand
 *
 * This lower-priority task:
 *
 *      converts absolute A/B YPR to quaternions
 *          ->
 *      calls plan_single_segment_line()
 *          ->
 *      publishes qPath only after planning fully succeeds
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


        /*
         * qSeed only helps ADLS choose/converge to the first IK solution.
         *
         * It does NOT define Cartesian waypoint A.
         */
        request.qSeed =
            command.qSeed;


        /* --------------------------------------------------------------------
         * ABSOLUTE WAYPOINT A
         * --------------------------------------------------------------------
         */

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


        /* --------------------------------------------------------------------
         * ABSOLUTE WAYPOINT B
         * --------------------------------------------------------------------
         */

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


        /* --------------------------------------------------------------------
         * SAME VALIDATED CONTROLCORE SETTINGS
         * --------------------------------------------------------------------
         */

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

        /*
         * One planned sample maps one-to-one onto one 1 ms EtherCAT CSP cycle.
         */
        request.dt =
            0.001;


        adls_default_parameters(
            &request.ikParameters
        );


        printf(
            "\nPlanning HMI Cartesian line #%u with ControlCore...\n",
            command.sequence
        );

        fflush(stdout);


        bool success =
            plan_single_segment_line(
                &trajectory_robot,
                &request,
                &trajectory_workspace,
                &trajectory,
                &trajectory_report
            );


        /*
         * ControlCore reports joint-limit results separately from basic
         * generation success. Arbitrary HMI waypoints must not be executed if
         * either joint position or joint velocity limits fail.
         */
        if (
            success &&
            (
                trajectory.count == 0 ||
                !trajectory_report.positionLimitsPass ||
                !trajectory_report.velocityLimitsPass
            )
        )
        {
            success =
                false;
        }


        /*
         * Publish the newly generated trajectory only AFTER all ControlCore
         * validation has completed.
         */
        taskENTER_CRITICAL();

        if (success)
        {
            trajectory_index =
                0;

            trajectory_ready =
                true;


            /*
             * If STOP arrived while planning, preserve HOLD.
             *
             * Otherwise automatically enter the simulator pre-position phase,
             * then the executor will run qPath at exactly 1 ms/sample.
             */
            if (
                hmi_hold_requested ||
                motion_state == MOTION_HOLD
            )
            {
                motion_state =
                    MOTION_HOLD;
            }
            else
            {
                motion_state =
                    MOTION_PREPOSITION;
            }
        }
        else
        {
            trajectory.count =
                0;

            trajectory_ready =
                false;

            hmi_hold_requested =
                true;

            motion_state =
                MOTION_WAITING_FOR_PLAN;
        }


        planner_busy =
            false;

        taskEXIT_CRITICAL();


        if (!success)
        {
            printf(
                "\n"
                "============================================================\n"
                " CONTROLCORE PLAN #%u FAILED\n"
                "============================================================\n"
                "The robot remains in HOLD.\n"
                "Check reachability, orientation, IK convergence, motion limits,\n"
                "and trajectory buffer capacity.\n"
                "============================================================\n",
                command.sequence
            );

            fflush(stdout);
            continue;
        }


        printf(
            "\n"
            "============================================================\n"
            " CONTROLCORE PLAN #%u READY\n"
            "============================================================\n"
            "Waypoint A:     [%.6f %.6f %.6f] m\n"
            "Waypoint B:     [%.6f %.6f %.6f] m\n"
            "Samples:        %zu\n"
            "Duration:       %.6f s\n"
            "Path length:    %.6f m\n"
            "Peak TCP speed: %.6f m/s\n"
            "Max pos error:  %.6e m\n"
            "Max rot error:  %.6e rad\n"
            "Joint pos lim:  %s\n"
            "Joint vel lim:  %s\n"
            "Next state:     %s\n"
            "============================================================\n",
            command.sequence,
            command.waypointA[0],
            command.waypointA[1],
            command.waypointA[2],
            command.waypointB[0],
            command.waypointB[1],
            command.waypointB[2],
            trajectory.count,
            trajectory_report.duration,
            trajectory_report.pathLength,
            trajectory_report.peakTCPSpeed,
            trajectory_report.maxPositionError,
            trajectory_report.maxOrientationError,
            trajectory_report.positionLimitsPass ? "PASS" : "FAIL",
            trajectory_report.velocityLimitsPass ? "PASS" : "FAIL",
            motion_state_name(motion_state)
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
     *  RESET SOEM DATA STRUCTURES
     * ========================================================================
     */

    /*
     * Clear the SOEM context before use.
     *
     * This ensures no stale values exist in SOEM's internal structures.
     */
    memset(
        &soem_context,
        0,
        sizeof(soem_context)
    );

    /*
     * Clear the process-data buffer.
     */
    memset(
        IOmap,
        0,
        sizeof(IOmap)
    );


    /* ========================================================================
     *  1. OPEN THE ETHERCAT NETWORK INTERFACE
     * ========================================================================
     *
     * In the simulation, "ecatA" is the virtual EtherCAT interface.
     *
     * On real hardware, this would instead be the network interface connected
     * to the EtherCAT slave chain.
     */

    printf(
        "Opening SOEM on ecatA...\n"
    );

    /*
     * ecx_init():
     *
     * Ask SOEM to open the specified Ethernet interface for raw EtherCAT frame
     * communication.
     *
     * This does NOT yet discover slaves.
     */
    if (!ecx_init(
            &soem_context,
            "ecatA"
        ))
    {
        printf(
            "ERROR: Could not open ecatA\n"
        );

        /*
         * Stay alive instead of immediately killing the process.
         *
         * Because this is a FreeRTOS task, vTaskDelay() lets other tasks run
         * while this task sleeps.
         */
        for (;;)
        {
            vTaskDelay(
                pdMS_TO_TICKS(1000)
            );
        }
    }

    printf(
        "SOEM initialized successfully\n"
    );


    /* ========================================================================
     *  2. DISCOVER ETHERCAT SLAVES
     * ========================================================================
     */

    printf(
        "\nScanning EtherCAT bus...\n"
    );

    /*
     * ecx_config_init():
     *
     * SOEM scans the EtherCAT network and discovers all slaves.
     *
     * The returned value is the number of slaves found.
     *
     * SOEM also populates:
     *
     *      soem_context.slavelist[1]
     *      soem_context.slavelist[2]
     *      ...
     *
     * NOTE:
     * slavelist[0] is a special aggregate entry representing all slaves.
     */
    int slave_count =
        ecx_config_init(
            &soem_context
        );

    /*
     * No slaves means there is no useful EtherCAT system to control.
     */
    if (slave_count <= 0)
    {
        printf(
            "ERROR: No EtherCAT slaves found\n"
        );

        if (telemetry_socket >= 0)
        {
            close(telemetry_socket);
            telemetry_socket = -1;
        }

        /* Close the EtherCAT interface. */
        ecx_close(&soem_context);

        exit(1);
    }

    printf(
        "%d EtherCAT slave(s) found\n",
        slave_count
    );

    /*
     * Print the name reported by each EtherCAT slave.
     */
    for (int slave = 1;
         slave <= slave_count;
         slave++)
    {
        printf(
            "Slave %d: %s\n",
            slave,
            soem_context.slavelist[slave].name
        );
    }

    /*
     * This robot expects exactly six servo drives.
     *
     * Finding fewer or more than six indicates that the system topology does
     * not match the expected robot.
     */
    if (slave_count != NUM_AXES)
    {
        printf(
            "\nERROR: Expected %d slaves "
            "but found %d\n",
            NUM_AXES,
            slave_count
        );

        if (telemetry_socket >= 0)
        {
            close(telemetry_socket);
            telemetry_socket = -1;
        }

        ecx_close(&soem_context);

        exit(1);
    }


    /* ========================================================================
     *  3/4. MAP PDOs INTO IOmap
     * ========================================================================
     *
     * PDO = Process Data Object
     *
     * PDOs are the fast cyclic variables exchanged every EtherCAT cycle.
     *
     * Typical examples:
     *
     *      Master -> Drive:
     *          Controlword
     *          Target Position
     *
     *      Drive -> Master:
     *          Statusword
     *          Actual Position
     *
     * ecx_config_map_group() builds the process-data layout and assigns each
     * slave's:
     *
     *      slavelist[slave].outputs
     *      slavelist[slave].inputs
     *
     * pointers into IOmap.
     */

    printf(
        "\nMapping PDOs...\n"
    );

    int mapped_bytes =
        ecx_config_map_group(
            &soem_context,
            IOmap,
            0               /* EtherCAT group 0 */
        );

    printf(
        "Mapped bytes: %d\n",
        mapped_bytes
    );

    printf(
        "Outputs: %d bytes | "
        "Inputs: %d bytes\n",
        soem_context.grouplist[0].Obytes,
        soem_context.grouplist[0].Ibytes
    );


    /* ========================================================================
     *  CONFIGURE EVERY DRIVE FOR CSP
     * ========================================================================
     *
     * CiA-402 object:
     *
     *      0x6060 = Modes of Operation
     *
     * CSP mode number:
     *
     *      8
     *
     * Here we use SDO communication because this is configuration/startup
     * traffic, not the fast cyclic control loop.
     *
     * Recall:
     *
     *      SDO = setup / configuration / parameter access
     *      PDO = fast cyclic process data
     */

    printf(
        "\nSetting all drives to CSP...\n"
    );

    for (int slave = 1;
         slave <= NUM_AXES;
         slave++)
    {
        int8_t mode = CSP_MODE;

        /*
         * Write:
         *
         *      slave      = current drive
         *      index      = 0x6060
         *      subindex   = 0x00
         *      value      = 8 (CSP)
         */
        int sdo_wkc =
            ecx_SDOwrite(
                &soem_context,
                slave,
                0x6060,
                0x00,
                FALSE,
                sizeof(mode),
                &mode,
                EC_TIMEOUTRXM
            );

        /*
         * SDO write failed.
         */
        if (sdo_wkc <= 0)
        {
            printf(
                "Slave %d: ERROR writing 0x6060\n",
                slave
            );

            /*
             * SOEM stores protocol errors in an internal error list.
             * Print and drain that list.
             */
            while (soem_context.ecaterror)
            {
                printf(
                    "    SOEM: %s\n",
                    ecx_elist2string(
                        &soem_context
                    )
                );
            }

            /*
             * Skip verification for this slave and move to the next.
             */
            continue;
        }


        /*
         * Read 0x6060 back immediately to verify that the write succeeded.
         *
         * NOTE:
         * A real implementation may also inspect 0x6061
         * "Modes of Operation Display" to verify the active operating mode.
         */
        int8_t mode_readback = 0;

        int mode_size =
            sizeof(mode_readback);

        int read_wkc =
            ecx_SDOread(
                &soem_context,
                slave,
                0x6060,
                0x00,
                FALSE,
                &mode_size,
                &mode_readback,
                EC_TIMEOUTRXM
            );

        if (read_wkc > 0)
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
                "Slave %d: ERROR reading 0x6060\n",
                slave
            );

            while (soem_context.ecaterror)
            {
                printf(
                    "    SOEM: %s\n",
                    ecx_elist2string(
                        &soem_context
                    )
                );
            }
        }
    }


    /* ========================================================================
     *  5. CONFIGURE ETHERCAT DISTRIBUTED CLOCKS
     * ========================================================================
     *
     * Multi-axis robots need all drives to update at nearly the same instant.
     *
     * EtherCAT Distributed Clocks (DC) synchronize the slave clocks so their
     * cyclic actions can be aligned.
     */

    printf(
        "\nConfiguring Distributed Clocks...\n"
    );

    /*
     * ecx_configdc():
     *
     * Detect and configure Distributed Clock capable slaves.
     */
    boolean dc_found =
        ecx_configdc(
            &soem_context
        );

    printf(
        "DC-capable bus: %s\n",
        dc_found ? "YES" : "NO"
    );

    for (int slave = 1;
         slave <= NUM_AXES;
         slave++)
    {
        /*
         * hasdc tells us whether this slave supports EtherCAT Distributed
         * Clocks.
         */
        if (soem_context.slavelist[slave].hasdc)
        {
            printf(
                "Slave %d: DC supported "
                "-> requesting 1 ms SYNC0\n",
                slave
            );

            /*
             * ecx_dcsync0():
             *
             * Enable each slave's SYNC0 event.
             *
             * TRUE          = enable SYNC0
             * 1,000,000 ns  = 1 ms period
             * 0             = zero phase shift
             */
            ecx_dcsync0(
                &soem_context,
                slave,
                TRUE,
                CYCLE_TIME_NS,
                0
            );
        }
        else
        {
            printf(
                "Slave %d: no DC support\n",
                slave
            );
        }
    }


    /* ========================================================================
     *  6. WAIT FOR ETHERCAT SAFE-OP
     * ========================================================================
     *
     * IMPORTANT:
     *
     * EtherCAT SAFE-OP is a COMMUNICATION state.
     * It is different from the CiA-402 servo-drive state machine.
     *
     * In SAFE-OP, input process data can be exchanged, but outputs are not yet
     * fully active for normal operation.
     */

    printf(
        "\nWaiting for SAFE-OP...\n"
    );

    /*
     * Ask SOEM to wait until all slaves reach SAFE-OP.
     *
     * slave = 0 means "all slaves".
     */
    ecx_statecheck(
        &soem_context,
        0,
        EC_STATE_SAFE_OP,
        EC_TIMEOUTSTATE * 4
    );

    /*
     * Refresh SOEM's stored copy of every slave's EtherCAT state.
     */
    ecx_readstate(
        &soem_context
    );

    for (int slave = 1;
         slave <= NUM_AXES;
         slave++)
    {
        printf(
            "Slave %d: %s\n",
            slave,
            state_name(
                soem_context
                    .slavelist[slave]
                    .state
            )
        );
    }


    /* ========================================================================
     *  SEND FIRST PROCESS-DATA FRAME
     * ========================================================================
     *
     * Sending/receiving at least one valid process-data frame helps establish
     * process-data exchange before requesting OPERATIONAL.
     */

    ecx_send_processdata(
        &soem_context
    );

    ecx_receive_processdata(
        &soem_context,
        EC_TIMEOUTRET
    );


    /* ========================================================================
     *  7. REQUEST ETHERCAT OPERATIONAL STATE
     * ========================================================================
     *
     * Again:
     *
     *      EtherCAT OPERATIONAL
     *
     * is NOT the same as:
     *
     *      CiA-402 Operation Enabled.
     *
     * First we make the EtherCAT network OPERATIONAL.
     * Later, inside the cyclic loop, we enable each servo using Controlword.
     */

    printf(
        "\nRequesting OPERATIONAL...\n"
    );

    /*
     * slavelist[0] represents the complete slave group.
     *
     * Request OPERATIONAL for all slaves.
     */
    soem_context.slavelist[0].state =
        EC_STATE_OPERATIONAL;

    /*
     * Write the requested EtherCAT state onto the bus.
     */
    ecx_writestate(
        &soem_context,
        0
    );

    /*
     * Give the slaves multiple chances to enter OPERATIONAL.
     *
     * Process data continues to be exchanged while we wait.
     */
    for (int attempt = 0;
         attempt < 50;
         attempt++)
    {
        ecx_send_processdata(
            &soem_context
        );

        ecx_receive_processdata(
            &soem_context,
            EC_TIMEOUTRET
        );

        ecx_statecheck(
            &soem_context,
            0,
            EC_STATE_OPERATIONAL,
            EC_TIMEOUTSTATE / 10
        );

        if (soem_context
                .slavelist[0]
                .state ==
            EC_STATE_OPERATIONAL)
        {
            break;
        }
    }


    /*
     * Refresh states and print the final result.
     */
    ecx_readstate(
        &soem_context
    );

    for (int slave = 1;
         slave <= NUM_AXES;
         slave++)
    {
        printf(
            "Slave %d final state: %s "
            "(0x%02X)\n",
            slave,
            state_name(
                soem_context
                    .slavelist[slave]
                    .state
            ),
            soem_context
                .slavelist[slave]
                .state
        );
    }


    /* ========================================================================
     *  CALCULATE EXPECTED WKC
     * ========================================================================
     *
     * WKC = Working Counter.
     *
     * EtherCAT slaves increment the Working Counter when they successfully
     * process the parts of a frame addressed to them.
     *
     * Therefore:
     *
     *      correct WKC  -> expected slaves processed the process data
     *      low WKC      -> communication/slave/process-data problem
     *
     * SOEM's normal expected-WKC formula is:
     *
     *      outputsWKC * 2 + inputsWKC
     */

    int expected_wkc =
        (
            soem_context
                .grouplist[0]
                .outputsWKC * 2
        )
        +
        soem_context
            .grouplist[0]
            .inputsWKC;

    printf(
        "\nExpected WKC = %d\n",
        expected_wkc
    );


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

        ecx_close(&soem_context);
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

        ecx_close(&soem_context);
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
     *      3. select the current ControlCore qPath sample
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
            uint8_t *inputs =
                soem_context
                    .slavelist[slave]
                    .inputs;


            uint16_t statusword =
                read_u16(
                    inputs + 2
                );


            uint16_t drive_state =
                statusword &
                0x006F;


            if (drive_state != 0x0027)
            {
                all_operation_enabled =
                    false;
            }


            int32_t actual_position =
                read_i32(
                    inputs + 4
                );


            current_q.q[slave - 1] =
                drive_units_to_joint_rad(
                    actual_position
                );
        }


        /*
         * This can become true only after a complete valid trajectory exists.
         */
        bool all_at_trajectory_start =
            trajectory_ready &&
            trajectory.count > 0;


        /*
         * Poll RBT2 without blocking the 1 ms EtherCAT task.
         */
        receive_hmi_commands(
            command_socket,
            &current_q,
            &last_hmi_sequence
        );


        /* ====================================================================
         *  PREPARE PDO COMMANDS FOR ALL SIX SERVO DRIVES
         * ====================================================================
         */

        for (int slave = 1;
             slave <= NUM_AXES;
             slave++)
        {
            /*
             * After PDO mapping, SOEM gives us direct pointers to each slave's
             * output and input process-data regions.
             *
             * outputs:
             *      master -> servo
             *
             * inputs:
             *      servo -> master
             */
            uint8_t *outputs =
                soem_context
                    .slavelist[slave]
                    .outputs;

            uint8_t *inputs =
                soem_context
                    .slavelist[slave]
                    .inputs;


            /*
             * Read CiA-402 Statusword from this slave's input PDO.
             *
             * CURRENT SIMULATOR PDO LAYOUT:
             *
             *      inputs + 2 -> Statusword
             *
             * This offset comes from the simulator PDO layout and must be
             * rechecked against the real A6-EC mapping later.
             */
            uint16_t statusword =
                read_u16(
                    inputs + 2
                );

            /*
             * Mask the Statusword down to the bits used to identify the
             * important CiA-402 drive states.
             *
             * Examples after masking:
             *
             *      0x0040 = Switch On Disabled
             *      0x0021 = Ready to Switch On
             *      0x0023 = Switched On
             *      0x0027 = Operation Enabled
             */
            uint16_t drive_state =
                statusword & 0x006F;


            /*
             * Controlword that we will write to this drive's output PDO.
             */
            uint16_t controlword;


            /*
             * If Ctrl+C has been pressed:
             *
             *      walk DOWN the CiA-402 state machine.
             *
             * Otherwise:
             *
             *      walk UP the CiA-402 state machine toward Operation Enabled.
             */
            if (stop_requested)
            {
                controlword =
                    get_disable_controlword(
                        drive_state
                    );
            }
            else
            {
                controlword =
                    get_enable_controlword(
                        drive_state
                    );
            }


            /*
             * Write CiA-402 Controlword into the output PDO.
             *
             * CURRENT SIMULATOR PDO LAYOUT:
             *
             *      outputs + 0 -> Controlword
             */
            write_u16(
                outputs + 0,
                controlword
            );


            /*
             * ================================================================
             *  TARGET POSITION SELECTION
             * ================================================================
             *
             * WAITING / PLANNING / HOLD
             *      -> hold measured position
             *
             * PREPOSITION
             *      -> move simulated axes to qPath[0]
             *
             * RUNNING
             *      -> one precomputed qPath sample per 1 ms EtherCAT cycle
             *
             * FINISHED
             *      -> hold final qPath sample
             *
             * No interpolation is added here.
             */

            const int joint =
                slave - 1;


            int32_t actual_position =
                read_i32(
                    inputs + 4
                );


            /*
             * Avoid touching qPath[0] until the planner has fully published a
             * successful trajectory.
             */
            int32_t start_position =
                actual_position;


            if (
                trajectory_ready &&
                trajectory.count > 0
            )
            {
                start_position =
                    joint_rad_to_drive_units(
                        trajectory.qPath[0].q[joint]
                    );


                long long start_error =
                    (long long)actual_position -
                    (long long)start_position;


                if (llabs(start_error) > 5)
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


            if (stop_requested)
            {
                /*
                 * Ctrl+C shutdown:
                 * hold measured position while CiA-402 walks down.
                 */
                target_position =
                    actual_position;
            }
            else if (!all_operation_enabled)
            {
                target_position =
                    actual_position;
            }
            else if (hmi_hold_requested)
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
                    state == MOTION_PREPOSITION
                )
                {
                    /*
                     * SIMULATOR-ONLY pre-position stage.
                     *
                     * This is separate from the requested Cartesian line.
                     * Real hardware later needs a deliberately planned safe
                     * point-to-point move to waypoint A.
                     */
                    target_position =
                        start_position;
                }
                else if (
                    state == MOTION_RUNNING &&
                    trajectory_ready &&
                    trajectory.count > 0
                )
                {
                    target_position =
                        joint_rad_to_drive_units(
                            trajectory
                                .qPath[trajectory_index]
                                .q[joint]
                        );
                }
                else if (
                    state == MOTION_FINISHED &&
                    trajectory_ready &&
                    trajectory.count > 0
                )
                {
                    target_position =
                        joint_rad_to_drive_units(
                            trajectory
                                .qPath[trajectory.count - 1]
                                .q[joint]
                        );
                }
            }


            /*
             * CURRENT SIMULATOR PDO LAYOUT:
             *
             *      outputs + 2 -> Target Position
             */
            write_i32(
                outputs + 2,
                target_position
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

        ecx_send_processdata(
            &soem_context
        );

        /*
         * Receive slave input PDOs and obtain actual WKC.
         *
         * After this call, each slave's .inputs memory contains the latest
         * process data received from that slave.
         */
        wkc =
            ecx_receive_processdata(
                &soem_context,
                EC_TIMEOUTRET
            );


        /* ====================================================================
         *  CONTROLCORE TRAJECTORY EXECUTION STATE
         * ====================================================================
         *
         * trajectory_index advances ONCE per EtherCAT cycle, only after all
         * six axes used the same sample.
         * ====================================================================
         */

        if (
            !stop_requested &&
            !hmi_hold_requested &&
            trajectory_ready
        )
        {
            /*
             * PREPOSITION -> RUNNING
             *
             * The requested Cartesian line does not begin until all six
             * simulated axes have reached qPath[0].
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

                    motion_state =
                        MOTION_RUNNING;


                    printf(
                        "\n"
                        "============================================================\n"
                        " CONTROLCORE TRAJECTORY EXECUTION START\n"
                        "============================================================\n"
                        "Samples:  %zu\n"
                        "dt:       0.001 s\n"
                        "Duration: %.6f s\n"
                        "============================================================\n",
                        trajectory.count,
                        trajectory_report.duration
                    );

                    fflush(stdout);
                }
            }

            /*
             * RUNNING:
             *
             * one qPath point per 1 ms CSP cycle.
             */
            else if (
                motion_state ==
                MOTION_RUNNING
            )
            {
                if (
                    trajectory_index + 1 <
                    trajectory.count
                )
                {
                    trajectory_index++;
                }
                else
                {
                    motion_state =
                        MOTION_FINISHED;


                    printf(
                        "\n"
                        "============================================================\n"
                        " CONTROLCORE TRAJECTORY EXECUTION COMPLETE\n"
                        "============================================================\n"
                        "Final sample: %zu / %zu\n"
                        "Duration:     %.6f s\n"
                        "============================================================\n",
                        trajectory_index + 1,
                        trajectory.count,
                        trajectory_report.duration
                    );

                    fflush(stdout);
                }
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
                uint8_t *inputs =
                    soem_context
                        .slavelist[slave]
                        .inputs;

                /*
                 * Read each drive's Actual Position.
                 *
                 * CURRENT SIMULATOR OFFSET:
                 *
                 *      inputs + 4
                 */
                actual_positions[slave - 1] =
                    read_i32(inputs + 4);
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
                uint8_t *outputs =
                    soem_context
                        .slavelist[slave]
                        .outputs;

                uint8_t *inputs =
                    soem_context
                        .slavelist[slave]
                        .inputs;


                /*
                 * Read the target that OUR MASTER currently placed into the
                 * output PDO.
                 */
                int32_t target =
                    read_i32(
                        outputs + 2
                    );

                /*
                 * Read the actual position returned by the drive.
                 */
                int32_t actual =
                    read_i32(
                        inputs + 4
                    );

                /*
                 * Read full raw CiA-402 Statusword.
                 */
                uint16_t status =
                    read_u16(
                        inputs + 2
                    );


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
         *      get_disable_controlword()
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
                uint8_t *inputs =
                    soem_context
                        .slavelist[slave]
                        .inputs;


                uint16_t statusword =
                    read_u16(
                        inputs + 2
                    );

                uint16_t drive_state =
                    statusword & 0x006F;


                /*
                 * 0x0040 = CiA-402 Switch On Disabled.
                 */
                if (drive_state != 0x0040)
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

                /*
                 * Disable EtherCAT SYNC0 generation before closing.
                 */
                for (int slave = 1;
                     slave <= NUM_AXES;
                     slave++)
                {
                    if (soem_context
                            .slavelist[slave]
                            .hasdc)
                    {
                        ecx_dcsync0(
                            &soem_context,
                            slave,
                            FALSE,          /* disable SYNC0 */
                            CYCLE_TIME_NS,
                            0
                        );
                    }
                }


                /*
                 * Close MATLAB telemetry socket if it was created.
                 */
                if (telemetry_socket >= 0)
                {
                    close(telemetry_socket);
                    telemetry_socket = -1;
                }

                /*
                 * Close Cartesian-HMI command socket.
                 */
                if (command_socket >= 0)
                {
                    close(command_socket);
                    command_socket = -1;
                }

                /*
                 * Close SOEM's Ethernet/EtherCAT interface.
                 */
                ecx_close(
                    &soem_context
                );

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

