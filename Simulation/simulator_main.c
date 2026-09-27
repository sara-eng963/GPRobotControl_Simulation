#include "../HMI/hmi_protocol.h"

#include "../StateMachine/state_machine.h"

#include "../ControlCore/Config/robot_config.h"
#include "../ControlCore/Kinematics/adls_ik.h"
#include "../ControlCore/Kinematics/control_fk.h"
#include "../ControlCore/Trajectory/joint_trajectory.h"

#include "../EtherCATComm/ethercat_master.h"

#include "../ServoDrive/A6EC/a6ec_drive.h"
#include "../ServoDrive/CiA402/cia402.h"

#include "FreeRTOS.h"
#include "task.h"

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>


#define SIM_NUM_AXES                 ROBOT_DOF
#define SIM_CYCLE_TIME_NS            1000000U
#define SIM_PV_GEOMETRY_CAPACITY     512U
#define SIM_PV_STORAGE_CAPACITY      60000U

#define SIM_CONTROL_TASK_STACK_WORDS 4096U
#define SIM_CONTROL_TASK_PRIORITY    (tskIDLE_PRIORITY + 2U)

#define MATLAB_PORT                  5005
#define MATLAB_IP                    "172.18.160.1"

#define DEG2RAD(x) ((x) * ROBOT_PI / 180.0)


typedef enum
{
    GUIDANCE_ERR_NONE = 0,
    GUIDANCE_ERR_NOT_TEACHING,
    GUIDANCE_ERR_INHIBITED,
    GUIDANCE_ERR_FEEDBACK,
    GUIDANCE_ERR_IK,
    GUIDANCE_ERR_LIMIT,
    GUIDANCE_ERR_TRAJECTORY,
    GUIDANCE_ERR_DRIVE

} GuidanceError;


typedef struct
{
    PvExecutionSample samples[SIM_PV_STORAGE_CAPACITY];

    uint32_t sample_count;

    bool writing;
    bool committed;

    ValidatedTrajectory metadata;

} RamValidatedStorage;


typedef struct
{
    bool active;

    JointTrajectory trajectory;

    JointVector q_goal;

    GuidanceError error;

} SimGuidance;


typedef struct
{
    int command_socket;
    int telemetry_socket;

    struct sockaddr_in panel_status_address;
    struct sockaddr_in teaching_status_address;
    struct sockaddr_in matlab_address;

    uint32_t status_sequence;
    uint32_t last_command_sequence;

    HmiProgramSelection selected_program;

    bool estop_active;
    bool paused;

    bool start_teach_requested;
    bool approach_requested;

    bool approach_reset_requested;
    bool approach_home_requested;

    TeachingEvent pending_teaching_event;

    int last_wkc;

    bool robot_homed;

} SupervisorRuntime;


static volatile sig_atomic_t stop_requested =
    0;


static RobotConfig robot;
static EtherCATMasterConfig ethercat_config;
static HomingConfig homing_config;
static TeachingConfig teaching_config;

static PathValidationConfig path_validation_config;
static PathValidationServices path_validation_services;
static PathValidationWorkspace path_validation_workspace;

static Vec3 pv_raw_geometry[SIM_PV_GEOMETRY_CAPACITY];
static Vec3 pv_arc_geometry[SIM_PV_GEOMETRY_CAPACITY];
static real_t pv_l_original[SIM_PV_GEOMETRY_CAPACITY];
static real_t pv_l_arc[SIM_PV_GEOMETRY_CAPACITY];
static ADLSInfo pv_ik_scratch;

static RamValidatedStorage validated_storage;
static PathValidationStorage path_validation_storage;
static ValidatedTrajectory validated_trajectory;

static ApproachConfig approach_config;
static ApproachServices approach_services;

static StateMachine machine;
static StateMachineDependencies dependencies;
static StateMachineInputs inputs;

static SimGuidance guidance;
static ADLSInfo guidance_ik_info;


static void on_signal(
    int signal_number
)
{
    (void)signal_number;

    stop_requested =
        1;
}


static void sleep_1ms(void)
{
    const struct timespec delay =
    {
        .tv_sec = 0,
        .tv_nsec = 1000000L
    };

    nanosleep(
        &delay,
        NULL
    );
}


static uint32_t float_to_network_word(
    float value
)
{
    uint32_t bits =
        0U;

    memcpy(
        &bits,
        &value,
        sizeof(bits)
    );

    return
        htonl(bits);
}


static float network_word_to_float(
    uint32_t value
)
{
    const uint32_t bits =
        ntohl(value);

    float result =
        0.0F;

    memcpy(
        &result,
        &bits,
        sizeof(result)
    );

    return
        result;
}


/* ============================================================================
 * VALIDATED TRAJECTORY RAM STORAGE
 * ============================================================================
 */

static bool ram_storage_begin(
    void *context
)
{
    RamValidatedStorage *storage =
        (RamValidatedStorage *)context;

    if (storage == NULL)
    {
        return false;
    }

    storage->sample_count =
        0U;

    storage->writing =
        true;

    storage->committed =
        false;

    memset(
        &storage->metadata,
        0,
        sizeof(storage->metadata)
    );

    return true;
}


static bool ram_storage_write(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
)
{
    RamValidatedStorage *storage =
        (RamValidatedStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        !storage->writing ||
        sample_index >= SIM_PV_STORAGE_CAPACITY
    )
    {
        return false;
    }

    storage->samples[sample_index] =
        *sample;

    if (
        sample_index + 1U >
        storage->sample_count
    )
    {
        storage->sample_count =
            sample_index + 1U;
    }

    return true;
}


static bool ram_storage_commit(
    const ValidatedTrajectory *metadata,
    void *context
)
{
    RamValidatedStorage *storage =
        (RamValidatedStorage *)context;

    if (
        storage == NULL ||
        metadata == NULL ||
        !storage->writing ||
        metadata->sample_count > storage->sample_count
    )
    {
        return false;
    }

    storage->metadata =
        *metadata;

    storage->writing =
        false;

    storage->committed =
        true;

    return true;
}


static void ram_storage_abort(
    void *context
)
{
    RamValidatedStorage *storage =
        (RamValidatedStorage *)context;

    if (storage == NULL)
    {
        return;
    }

    storage->writing =
        false;

    storage->committed =
        false;

    storage->sample_count =
        0U;
}


static bool approach_read_validated_sample(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
)
{
    RamValidatedStorage *storage =
        (RamValidatedStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        !storage->committed ||
        sample_index >= storage->metadata.sample_count ||
        sample_index >= storage->sample_count
    )
    {
        return false;
    }

    *sample =
        storage->samples[sample_index];

    return true;
}


/* ============================================================================
 * CONFIGURATION
 * ============================================================================
 */

static void configure_robot(void)
{
    robot_config_init_ur5(
        &robot
    );

    /*
     * Same temporary simulation home already used by the HOMING / TEACHING
     * integration tests. It is NOT a final robot commissioning value.
     */
    robot.configuration.homeDefined =
        true;

    robot.configuration.home[0] =
        DEG2RAD(0.0);

    robot.configuration.home[1] =
        DEG2RAD(-90.0);

    robot.configuration.home[2] =
        DEG2RAD(90.0);

    robot.configuration.home[3] =
        DEG2RAD(0.0);

    robot.configuration.home[4] =
        DEG2RAD(0.0);

    robot.configuration.home[5] =
        DEG2RAD(0.0);
}


static void configure_state_dependencies(void)
{
    ethercat_config =
        (EtherCATMasterConfig)
        {
            .interfaceName = "ecatA",
            .expectedSlaveCount = SIM_NUM_AXES,
            .cycleTimeNs = SIM_CYCLE_TIME_NS
        };

    homing_config =
        (HomingConfig)
        {
            .duration = 5.0,
            .dt = 0.001,
            .positionTolerance = DEG2RAD(0.5),
            .requiredStableCycles = 20U,
            .maxVerificationCycles = 2000U
        };

    teaching_config =
        (TeachingConfig)
        {
            .default_speed_mps = 0.010F,
            .minimum_speed_mps = 0.001F,
            .maximum_speed_mps = 0.100F,
            .speed_step_mps = 0.001F,
            .minimum_point_separation_m = 0.002F,
            .collinearity_epsilon_m2 = 1.0e-10F
        };

    memset(
        &path_validation_config,
        0,
        sizeof(path_validation_config)
    );

    path_validation_config.default_tcp_speed_mps =
        0.010;

    path_validation_config.max_tcp_speed_mps =
        0.100;

    path_validation_config.max_tcp_acceleration_mps2 =
        0.250;

    path_validation_config.max_tcp_jerk_mps3 =
        1.000;

    path_validation_config.minimum_segment_length_m =
        0.002;

    path_validation_config.maximum_fk_position_error_m =
        0.001;

    path_validation_config.maximum_fk_orientation_error_rad =
        0.01;

    /*
     * No project singularity acceptance threshold has been commissioned yet.
     * A value of zero therefore disables rejection by this policy in the PC
     * HMI simulator without inventing a physical-machine safety threshold.
     */
    path_validation_config.minimum_singularity_sigma =
        0.0;

    path_validation_config.maximum_joint_step_rad =
        0.20;

    path_validation_config.maximum_position_quantization_error_rad =
        0.001;

    path_validation_config.geometry_points_per_segment =
        200U;

    path_validation_config.arc_length_spacing_m =
        0.005;

    adls_default_parameters(
        &path_validation_config.ik_parameters
    );

    path_validation_config.check_joint_acceleration =
        false;

    path_validation_config.require_collision_callback =
        false;

    memset(
        &path_validation_services,
        0,
        sizeof(path_validation_services)
    );

    path_validation_workspace =
        (PathValidationWorkspace)
        {
            .geometry_capacity = SIM_PV_GEOMETRY_CAPACITY,
            .raw_geometry = pv_raw_geometry,
            .arc_geometry = pv_arc_geometry,
            .l_original = pv_l_original,
            .l_arc = pv_l_arc,
            .ik_scratch = &pv_ik_scratch
        };

    memset(
        &validated_storage,
        0,
        sizeof(validated_storage)
    );

    path_validation_storage =
        (PathValidationStorage)
        {
            .begin = ram_storage_begin,
            .write_sample = ram_storage_write,
            .commit = ram_storage_commit,
            .abort = ram_storage_abort,
            .capacity_samples = SIM_PV_STORAGE_CAPACITY,
            .context = &validated_storage
        };

    memset(
        &validated_trajectory,
        0,
        sizeof(validated_trajectory)
    );

    memset(
        &approach_config,
        0,
        sizeof(approach_config)
    );

    approach_config.duration_safety_factor =
        1.10;

    approach_config.minimum_leg_duration_s =
        0.05;

    approach_config.maximum_leg_duration_s =
        10.0;

    approach_config.final_position_tolerance_rad =
        0.001;

    approach_config.following_error_limit_rad =
        0.10;

    approach_config.use_jerk_limits =
        false;

    approach_config.required_stable_cycles =
        5U;

    approach_config.maximum_verification_cycles =
        100U;

    approach_config.validation_samples_per_step =
        64U;

    approach_config.require_collision_check =
        false;

    memset(
        &approach_services,
        0,
        sizeof(approach_services)
    );

    approach_services.read_validated_sample =
        approach_read_validated_sample;

    approach_services.storage_context =
        &validated_storage;

    memset(
        &dependencies,
        0,
        sizeof(dependencies)
    );

    dependencies.ethercat_config =
        &ethercat_config;

    dependencies.robot =
        &robot;

    dependencies.homing_config =
        &homing_config;

    dependencies.teaching_config =
        &teaching_config;

    dependencies.path_validation_config =
        &path_validation_config;

    dependencies.path_validation_services =
        &path_validation_services;

    dependencies.path_validation_workspace =
        &path_validation_workspace;

    dependencies.path_validation_storage =
        &path_validation_storage;

    dependencies.validated_trajectory =
        &validated_trajectory;

    dependencies.approach_config =
        &approach_config;

    dependencies.approach_services =
        &approach_services;
}


/* ============================================================================
 * DRIVE / FK FEEDBACK
 * ============================================================================
 */

static bool pdo_ready(void)
{
    return
        ethercat_master_slave_inputs(1) != NULL &&
        ethercat_master_slave_outputs(1) != NULL;
}


static bool read_actual_joints(
    JointVector *q,
    int32_t actual_units[ROBOT_DOF]
)
{
    if (
        q == NULL ||
        !pdo_ready()
    )
    {
        return false;
    }

    for (
        int slave = 1;
        slave <= ROBOT_DOF;
        ++slave
    )
    {
        A6ECPDOFeedback feedback;

        a6ec_read_feedback(
            slave,
            &feedback
        );

        if (
            cia402_get_state(
                feedback.statusword
            ) !=
            CIA402_STATE_OPERATION_ENABLED
        )
        {
            return false;
        }

        q->q[slave - 1] =
            a6ec_position_units_to_joint_rad(
                feedback.actualPosition
            );

        if (actual_units != NULL)
        {
            actual_units[slave - 1] =
                feedback.actualPosition;
        }
    }

    return true;
}


static bool write_joint_target(
    const JointVector *q,
    int *wkc
)
{
    if (
        q == NULL ||
        !pdo_ready()
    )
    {
        return false;
    }

    for (
        int slave = 1;
        slave <= ROBOT_DOF;
        ++slave
    )
    {
        A6ECPDOFeedback feedback;

        a6ec_read_feedback(
            slave,
            &feedback
        );

        if (
            cia402_get_state(
                feedback.statusword
            ) !=
            CIA402_STATE_OPERATION_ENABLED
        )
        {
            return false;
        }

        const A6ECPDOCommand command =
        {
            .controlword =
                CIA402_CONTROLWORD_ENABLE_OPERATION,

            .targetPosition =
                a6ec_joint_rad_to_position_units(
                    q->q[slave - 1]
                )
        };

        a6ec_write_command(
            slave,
            &command
        );
    }

    const int actual_wkc =
        ethercat_master_exchange();

    if (wkc != NULL)
    {
        *wkc =
            actual_wkc;
    }

    const int expected =
        ethercat_master_expected_wkc();

    return
        expected > 0 &&
        actual_wkc >= expected;
}


static bool hold_current_position(
    int *wkc
)
{
    JointVector q;

    if (
        !read_actual_joints(
            &q,
            NULL
        )
    )
    {
        return false;
    }

    return
        write_joint_target(
            &q,
            wkc
        );
}


/* ============================================================================
 * SIMULATED HAND GUIDING
 * ============================================================================
 *
 * This is intentionally outside state_teaching.c.
 *
 * It substitutes only for the future physical admittance/manual-guidance
 * controller. Teaching itself still records the actual A6 feedback and
 * computes TCP pose through ControlCore FK exactly as it does in the real
 * architecture.
 * ============================================================================
 */

static bool guidance_plan_xyz(
    SimGuidance *sim,
    float x_m,
    float y_m,
    float z_m
)
{
    if (sim == NULL)
    {
        return false;
    }

    sim->error =
        GUIDANCE_ERR_NONE;

    if (
        machine.current_state !=
        ROBOT_STATE_TEACHING
    )
    {
        sim->error =
            GUIDANCE_ERR_NOT_TEACHING;

        return false;
    }

    SupervisorRuntime *runtime =
        NULL;

    (void)runtime;

    JointVector q_start;

    if (
        !read_actual_joints(
            &q_start,
            NULL
        )
    )
    {
        sim->error =
            GUIDANCE_ERR_FEEDBACK;

        return false;
    }

    double target_transform[4][4];

    control_fk(
        &robot,
        q_start.q,
        target_transform
    );

    target_transform[0][3] =
        (double)x_m;

    target_transform[1][3] =
        (double)y_m;

    target_transform[2][3] =
        (double)z_m;

    ADLSParameters parameters;

    adls_default_parameters(
        &parameters
    );

    double q_solution[ROBOT_DOF];

    if (
        !adls_ik(
            &robot,
            target_transform,
            q_start.q,
            &parameters,
            q_solution,
            &guidance_ik_info
        )
    )
    {
        sim->error =
            GUIDANCE_ERR_IK;

        return false;
    }

    real_t duration =
        0.25;

    for (
        int joint = 0;
        joint < ROBOT_DOF;
        ++joint
    )
    {
        if (
            !isfinite(q_solution[joint]) ||
            q_solution[joint] <
                robot.limits.qMin[joint] ||
            q_solution[joint] >
                robot.limits.qMax[joint]
        )
        {
            sim->error =
                GUIDANCE_ERR_LIMIT;

            return false;
        }

        sim->q_goal.q[joint] =
            q_solution[joint];

        const real_t dq =
            fabs(
                sim->q_goal.q[joint] -
                q_start.q[joint]
            );

        const real_t required =
            1.875 *
            dq /
            robot.limits.qdMax[joint] *
            1.20;

        if (required > duration)
        {
            duration =
                required;
        }
    }

    if (duration > 4.0)
    {
        duration =
            4.0;
    }

    if (
        !joint_trajectory_init(
            &sim->trajectory,
            &q_start,
            &sim->q_goal,
            duration,
            0.001
        )
    )
    {
        sim->error =
            GUIDANCE_ERR_TRAJECTORY;

        return false;
    }

    sim->active =
        true;

    return true;
}


static bool guidance_step(
    SimGuidance *sim,
    int *wkc
)
{
    if (sim == NULL)
    {
        return false;
    }

    if (!sim->active)
    {
        return
            hold_current_position(
                wkc
            );
    }

    JointTrajectorySample sample;

    if (
        !joint_trajectory_next(
            &sim->trajectory,
            &sample
        )
    )
    {
        if (
            joint_trajectory_is_finished(
                &sim->trajectory
            )
        )
        {
            sim->active =
                false;

            return
                write_joint_target(
                    &sim->q_goal,
                    wkc
                );
        }

        sim->error =
            GUIDANCE_ERR_TRAJECTORY;

        sim->active =
            false;

        return false;
    }

    if (
        !write_joint_target(
            &sample.q,
            wkc
        )
    )
    {
        sim->error =
            GUIDANCE_ERR_DRIVE;

        sim->active =
            false;

        return false;
    }

    if (
        joint_trajectory_is_finished(
            &sim->trajectory
        )
    )
    {
        sim->active =
            false;
    }

    return true;
}


/* ============================================================================
 * HMI UDP
 * ============================================================================
 */

static bool setup_udp(
    SupervisorRuntime *runtime
)
{
    if (runtime == NULL)
    {
        return false;
    }

    runtime->command_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (runtime->command_socket < 0)
    {
        perror("HMI controller command socket");
        return false;
    }

    int reuse =
        1;

    (void)setsockopt(
        runtime->command_socket,
        SOL_SOCKET,
        SO_REUSEADDR,
        &reuse,
        sizeof(reuse)
    );

    struct sockaddr_in command_address;

    memset(
        &command_address,
        0,
        sizeof(command_address)
    );

    command_address.sin_family =
        AF_INET;

    command_address.sin_port =
        htons(HMI_CONTROLLER_PORT);

    command_address.sin_addr.s_addr =
        htonl(INADDR_ANY);

    if (
        bind(
            runtime->command_socket,
            (struct sockaddr *)&command_address,
            sizeof(command_address)
        ) < 0
    )
    {
        perror("HMI controller bind");
        close(runtime->command_socket);
        runtime->command_socket = -1;
        return false;
    }

    memset(
        &runtime->panel_status_address,
        0,
        sizeof(runtime->panel_status_address)
    );

    runtime->panel_status_address.sin_family =
        AF_INET;

    runtime->panel_status_address.sin_port =
        htons(HMI_PANEL_STATUS_PORT);

    (void)inet_pton(
        AF_INET,
        "127.0.0.1",
        &runtime->panel_status_address.sin_addr
    );

    runtime->teaching_status_address =
        runtime->panel_status_address;

    runtime->teaching_status_address.sin_port =
        htons(HMI_TEACH_STATUS_PORT);

    runtime->telemetry_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    memset(
        &runtime->matlab_address,
        0,
        sizeof(runtime->matlab_address)
    );

    runtime->matlab_address.sin_family =
        AF_INET;

    runtime->matlab_address.sin_port =
        htons(MATLAB_PORT);

    (void)inet_pton(
        AF_INET,
        MATLAB_IP,
        &runtime->matlab_address.sin_addr
    );

    return true;
}


static TeachingEvent selected_program_event(
    HmiProgramSelection program
)
{
    switch (program)
    {
        case HMI_PROGRAM_LINE:
            return
                TEACH_EVENT_SELECT_LINE;

        case HMI_PROGRAM_ARC:
            return
                TEACH_EVENT_SELECT_ARC;

        case HMI_PROGRAM_CIRCLE:
            return
                TEACH_EVENT_SELECT_CIRCLE;

        default:
            return
                TEACH_EVENT_NONE;
    }
}


static void request_program(
    SupervisorRuntime *runtime,
    HmiProgramSelection program
)
{
    runtime->selected_program =
        program;

    if (
        machine.current_state ==
        ROBOT_STATE_TEACHING
    )
    {
        runtime->pending_teaching_event =
            selected_program_event(
                program
            );
    }
}


static void handle_hmi_command(
    SupervisorRuntime *runtime,
    HmiCommand command,
    uint32_t sequence,
    float arg0,
    float arg1,
    float arg2
)
{
    if (runtime == NULL)
    {
        return;
    }

    runtime->last_command_sequence =
        sequence;

    switch (command)
    {
        case HMI_CMD_SELECT_LINE:
            request_program(
                runtime,
                HMI_PROGRAM_LINE
            );
            break;

        case HMI_CMD_SELECT_ARC:
            request_program(
                runtime,
                HMI_PROGRAM_ARC
            );
            break;

        case HMI_CMD_SELECT_CIRCLE:
            request_program(
                runtime,
                HMI_PROGRAM_CIRCLE
            );
            break;

        case HMI_CMD_RECORD:
            if (
                machine.current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                runtime->pending_teaching_event =
                    TEACH_EVENT_RECORD_POINT;
            }
            break;

        case HMI_CMD_VALIDATE_PREVIEW:
            /*
             * One physical HMI button has two context-dependent actions:
             *
             *   TEACHING                  -> request Path Validation
             *   PATH_VALIDATION + VALID  -> request Preview / Approach
             *
             * The second action is deliberately unavailable until the exact
             * taught program has passed Path Validation.
             */
            if (
                machine.current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                runtime->pending_teaching_event =
                    TEACH_EVENT_VALIDATE_PATH;
            }
            else if (
                machine.current_state ==
                    ROBOT_STATE_PATH_VALIDATION
                &&
                machine.path_validation_outputs.report.result ==
                    PV_RESULT_VALID
            )
            {
                runtime->approach_requested =
                    true;
            }
            break;

        case HMI_CMD_SPEED_UP:
            if (
                machine.current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                runtime->pending_teaching_event =
                    TEACH_EVENT_SPEED_INCREASE;
            }
            break;

        case HMI_CMD_SPEED_DOWN:
            if (
                machine.current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                runtime->pending_teaching_event =
                    TEACH_EVENT_SPEED_DECREASE;
            }
            break;

        case HMI_CMD_SPEED_DEFAULT:
            if (
                machine.current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                runtime->pending_teaching_event =
                    TEACH_EVENT_SPEED_DEFAULT;
            }
            break;

        case HMI_CMD_START_REPLAY:
            /*
             * START begins a Teaching session from a healthy IDLE state.
             *
             * Keep this as a latched supervisor request. IDLE consumes
             * IDLE_COMMAND_TEACH through the existing global StateMachine;
             * we do not bypass state_idle.c or force the state directly.
             */
            if (
                machine.current_state ==
                    ROBOT_STATE_IDLE
                &&
                machine.idle.phase !=
                    IDLE_PHASE_FAILED
            )
            {
                runtime->start_teach_requested =
                    true;

                printf(
                    "HMI: START accepted -> request TEACHING\n"
                );

                fflush(
                    stdout
                );
            }
            else
            {
                printf(
                    "HMI: START ignored in state=%s idle_phase=%d idle_error=%d\n",
                    state_machine_state_name(
                        machine.current_state
                    ),
                    (int)machine.idle.phase,
                    (int)machine.idle.error
                );

                fflush(
                    stdout
                );
            }
            break;

        case HMI_CMD_PAUSE_TOGGLE:
            runtime->paused =
                !runtime->paused;

            break;

        case HMI_CMD_RESET:
            if (
                machine.current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                runtime->pending_teaching_event =
                    TEACH_EVENT_RESET;
            }
            else if (
                machine.current_state ==
                ROBOT_STATE_APPROACH
            )
            {
                runtime->approach_reset_requested =
                    true;
            }
            else if (
                machine.current_state ==
                ROBOT_STATE_PATH_VALIDATION
            )
            {
                state_path_validation_cancel(
                    &machine.path_validation
                );
            }
            break;

        case HMI_CMD_HOME:
            if (
                machine.current_state ==
                ROBOT_STATE_APPROACH
            )
            {
                runtime->approach_home_requested =
                    true;
            }
            else if (
                machine.current_state ==
                ROBOT_STATE_IDLE
            )
            {
                machine.previous_state =
                    machine.current_state;

                state_homing_enter(
                    &machine.homing
                );

                machine.current_state =
                    ROBOT_STATE_HOMING;

                runtime->robot_homed =
                    false;
            }
            break;

        case HMI_CMD_ESTOP_TOGGLE:
            runtime->estop_active =
                !runtime->estop_active;

            if (runtime->estop_active)
            {
                guidance.active =
                    false;
            }

            break;

        case HMI_CMD_SIM_GUIDANCE_POSE:
            if (
                runtime->estop_active ||
                runtime->paused
            )
            {
                guidance.error =
                    GUIDANCE_ERR_INHIBITED;

                break;
            }

            (void)guidance_plan_xyz(
                &guidance,
                arg0,
                arg1,
                arg2
            );

            break;

        case HMI_CMD_NONE:
        default:
            break;
    }
}


static void poll_hmi_commands(
    SupervisorRuntime *runtime
)
{
    if (
        runtime == NULL ||
        runtime->command_socket < 0
    )
    {
        return;
    }

    for (;;)
    {
        uint32_t packet[HMI_COMMAND_WORD_COUNT];

        const ssize_t received =
            recvfrom(
                runtime->command_socket,
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

            perror("HMI command recvfrom");
            break;
        }

        if (
            received !=
            (ssize_t)sizeof(packet)
        )
        {
            continue;
        }

        if (
            ntohl(packet[HMI_COMMAND_WORD_MAGIC]) !=
                HMI_COMMAND_MAGIC
            ||
            ntohl(packet[HMI_COMMAND_WORD_VERSION]) !=
                HMI_PROTOCOL_VERSION
        )
        {
            continue;
        }

        const HmiCommand command =
            (HmiCommand)
            ntohl(
                packet[HMI_COMMAND_WORD_COMMAND]
            );

        const uint32_t sequence =
            ntohl(
                packet[HMI_COMMAND_WORD_SEQUENCE]
            );

        handle_hmi_command(
            runtime,
            command,
            sequence,
            network_word_to_float(
                packet[HMI_COMMAND_WORD_ARG0]
            ),
            network_word_to_float(
                packet[HMI_COMMAND_WORD_ARG1]
            ),
            network_word_to_float(
                packet[HMI_COMMAND_WORD_ARG2]
            )
        );
    }
}


/* ============================================================================
 * STATUS + MATLAB TELEMETRY
 * ============================================================================
 */

static void capture_recorded_points(
    float points[HMI_MAX_RECORDED_POINTS][3],
    uint32_t *count
)
{
    if (
        points == NULL ||
        count == NULL
    )
    {
        return;
    }

    memset(
        points,
        0,
        sizeof(float) *
        HMI_MAX_RECORDED_POINTS *
        3U
    );

    *count =
        0U;

    const TaughtSegment *segment =
        NULL;

    if (
        machine.current_state ==
            ROBOT_STATE_TEACHING
        &&
        machine.teaching.captured_point_count > 0U
    )
    {
        segment =
            &machine.teaching.working_segment;
    }
    else if (
        machine.teaching.draft.segment_count > 0U
    )
    {
        segment =
            &machine.teaching.draft.segments[
                machine.teaching.draft.segment_count - 1U
            ];
    }

    if (segment == NULL)
    {
        return;
    }

    uint32_t available =
        segment->point_count;

    if (
        machine.current_state ==
            ROBOT_STATE_TEACHING
        &&
        machine.teaching.captured_point_count > 0U
    )
    {
        available =
            machine.teaching.captured_point_count;
    }

    if (available > HMI_MAX_RECORDED_POINTS)
    {
        available =
            HMI_MAX_RECORDED_POINTS;
    }

    for (
        uint32_t i = 0U;
        i < available;
        ++i
    )
    {
        points[i][0] =
            segment->points[i].position_m[0];

        points[i][1] =
            segment->points[i].position_m[1];

        points[i][2] =
            segment->points[i].position_m[2];
    }

    *count =
        available;
}


static void send_status(
    SupervisorRuntime *runtime
)
{
    if (
        runtime == NULL ||
        runtime->command_socket < 0
    )
    {
        return;
    }

    uint32_t packet[HMI_STATUS_WORD_COUNT];

    memset(
        packet,
        0,
        sizeof(packet)
    );

    runtime->status_sequence++;

    packet[HMI_STATUS_WORD_MAGIC] =
        htonl(HMI_STATUS_MAGIC);

    packet[HMI_STATUS_WORD_VERSION] =
        htonl(HMI_PROTOCOL_VERSION);

    packet[HMI_STATUS_WORD_SEQUENCE] =
        htonl(runtime->status_sequence);

    packet[HMI_STATUS_WORD_ROBOT_STATE] =
        htonl((uint32_t)machine.current_state);

    packet[HMI_STATUS_WORD_SELECTED_PROGRAM] =
        htonl((uint32_t)runtime->selected_program);

    packet[HMI_STATUS_WORD_ESTOP] =
        htonl(runtime->estop_active ? 1U : 0U);

    packet[HMI_STATUS_WORD_PAUSED] =
        htonl(runtime->paused ? 1U : 0U);

    packet[HMI_STATUS_WORD_GUIDANCE_ACTIVE] =
        htonl(guidance.active ? 1U : 0U);

    packet[HMI_STATUS_WORD_BOOT_PHASE] =
        htonl((uint32_t)machine.boot.phase);

    packet[HMI_STATUS_WORD_BOOT_ERROR] =
        htonl((uint32_t)machine.boot.error);

    packet[HMI_STATUS_WORD_HOMING_PHASE] =
        htonl((uint32_t)machine.homing.phase);

    packet[HMI_STATUS_WORD_HOMING_ERROR] =
        htonl((uint32_t)machine.homing.error);

    packet[HMI_STATUS_WORD_IDLE_PHASE] =
        htonl((uint32_t)machine.idle.phase);

    packet[HMI_STATUS_WORD_IDLE_ERROR] =
        htonl((uint32_t)machine.idle.error);

    packet[HMI_STATUS_WORD_TEACHING_PHASE] =
        htonl((uint32_t)machine.teaching_outputs.phase);

    packet[HMI_STATUS_WORD_TEACHING_ERROR] =
        htonl((uint32_t)machine.teaching_outputs.error);

    packet[HMI_STATUS_WORD_TEACHING_MESSAGE] =
        htonl(
            (uint32_t)
            machine.teaching_outputs.status_message_id
        );

    packet[HMI_STATUS_WORD_TEACHING_NEXT_POINT] =
        htonl(
            (uint32_t)
            machine.teaching_outputs.next_point_number
        );

    packet[HMI_STATUS_WORD_TEACHING_SEGMENT_COUNT] =
        htonl(
            (uint32_t)
            machine.teaching.draft.segment_count
        );

    packet[HMI_STATUS_WORD_TEACHING_SPEED] =
        float_to_network_word(
            machine.teaching_outputs.displayed_speed_mps
        );

    packet[HMI_STATUS_WORD_RECORD_ALLOWED] =
        htonl(
            machine.teaching_outputs.record_allowed
            ? 1U
            : 0U
        );

    packet[HMI_STATUS_WORD_VALIDATE_ALLOWED] =
        htonl(
            machine.teaching_outputs.validate_allowed
            ? 1U
            : 0U
        );

    packet[HMI_STATUS_WORD_PV_PHASE] =
        htonl(
            (uint32_t)
            machine.path_validation_outputs.report.phase
        );

    packet[HMI_STATUS_WORD_PV_RESULT] =
        htonl(
            (uint32_t)
            machine.path_validation_outputs.report.result
        );

    packet[HMI_STATUS_WORD_PV_ERROR] =
        htonl(
            (uint32_t)
            machine.path_validation_outputs.report.error
        );

    packet[HMI_STATUS_WORD_PV_PROGRESS] =
        float_to_network_word(
            (float)
            machine.path_validation_outputs.report.progress_0_to_1
        );

    packet[HMI_STATUS_WORD_APPROACH_PHASE] =
        htonl(
            (uint32_t)
            machine.approach_outputs.report.phase
        );

    packet[HMI_STATUS_WORD_APPROACH_RESULT] =
        htonl(
            (uint32_t)
            machine.approach_outputs.report.result
        );

    packet[HMI_STATUS_WORD_APPROACH_ERROR] =
        htonl(
            (uint32_t)
            machine.approach_outputs.report.error
        );

    packet[HMI_STATUS_WORD_APPROACH_PROGRESS] =
        float_to_network_word(
            (float)
            machine.approach_outputs.report.progress_0_to_1
        );

    JointVector actual_q;

    memset(
        &actual_q,
        0,
        sizeof(actual_q)
    );

    double tcp[4][4] =
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    if (
        read_actual_joints(
            &actual_q,
            NULL
        )
    )
    {
        control_fk(
            &robot,
            actual_q.q,
            tcp
        );
    }

    packet[HMI_STATUS_WORD_TCP_X] =
        float_to_network_word(
            (float)tcp[0][3]
        );

    packet[HMI_STATUS_WORD_TCP_Y] =
        float_to_network_word(
            (float)tcp[1][3]
        );

    packet[HMI_STATUS_WORD_TCP_Z] =
        float_to_network_word(
            (float)tcp[2][3]
        );

    for (
        int joint = 0;
        joint < ROBOT_DOF;
        ++joint
    )
    {
        packet[
            HMI_STATUS_WORD_Q1 +
            joint
        ] =
            float_to_network_word(
                (float)actual_q.q[joint]
            );
    }

    const int expected_wkc =
        ethercat_master_expected_wkc();

    const int shown_wkc =
        runtime->last_wkc > 0
        ? runtime->last_wkc
        : expected_wkc;

    packet[HMI_STATUS_WORD_WKC] =
        htonl(
            shown_wkc > 0
            ? (uint32_t)shown_wkc
            : 0U
        );

    packet[HMI_STATUS_WORD_EXPECTED_WKC] =
        htonl(
            expected_wkc > 0
            ? (uint32_t)expected_wkc
            : 0U
        );

    float recorded[HMI_MAX_RECORDED_POINTS][3];
    uint32_t recorded_count =
        0U;

    capture_recorded_points(
        recorded,
        &recorded_count
    );

    packet[HMI_STATUS_WORD_RECORDED_COUNT] =
        htonl(recorded_count);

    for (
        uint32_t point = 0U;
        point < HMI_MAX_RECORDED_POINTS;
        ++point
    )
    {
        for (
            uint32_t axis = 0U;
            axis < 3U;
            ++axis
        )
        {
            packet[
                HMI_STATUS_WORD_P1_X +
                point * 3U +
                axis
            ] =
                float_to_network_word(
                    recorded[point][axis]
                );
        }
    }

    packet[HMI_STATUS_WORD_LAST_COMMAND_SEQUENCE] =
        htonl(
            runtime->last_command_sequence
        );

    packet[HMI_STATUS_WORD_GUIDANCE_ERROR] =
        htonl(
            (uint32_t)guidance.error
        );

    (void)sendto(
        runtime->command_socket,
        packet,
        sizeof(packet),
        MSG_DONTWAIT,
        (const struct sockaddr *)
            &runtime->panel_status_address,
        sizeof(runtime->panel_status_address)
    );

    (void)sendto(
        runtime->command_socket,
        packet,
        sizeof(packet),
        MSG_DONTWAIT,
        (const struct sockaddr *)
            &runtime->teaching_status_address,
        sizeof(runtime->teaching_status_address)
    );
}


static void send_matlab_telemetry(
    SupervisorRuntime *runtime
)
{
    if (
        runtime == NULL ||
        runtime->telemetry_socket < 0 ||
        !pdo_ready()
    )
    {
        return;
    }

    int32_t actual_positions[ROBOT_DOF];

    JointVector q;

    if (
        !read_actual_joints(
            &q,
            actual_positions
        )
    )
    {
        return;
    }

    (void)sendto(
        runtime->telemetry_socket,
        actual_positions,
        sizeof(actual_positions),
        MSG_DONTWAIT,
        (const struct sockaddr *)
            &runtime->matlab_address,
        sizeof(runtime->matlab_address)
    );
}


/* ============================================================================
 * CLEAN SHUTDOWN
 * ============================================================================ */

static void disable_drives(void)
{
    if (!pdo_ready())
    {
        return;
    }

    for (
        int attempt = 0;
        attempt < 100;
        ++attempt
    )
    {
        bool all_disabled =
            true;

        for (
            int slave = 1;
            slave <= ROBOT_DOF;
            ++slave
        )
        {
            A6ECPDOFeedback feedback;

            a6ec_read_feedback(
                slave,
                &feedback
            );

            const uint16_t drive_state =
                cia402_get_state(
                    feedback.statusword
                );

            if (
                drive_state !=
                CIA402_STATE_SWITCH_ON_DISABLED
            )
            {
                all_disabled =
                    false;
            }

            const A6ECPDOCommand command =
            {
                .controlword =
                    cia402_get_disable_controlword(
                        drive_state
                    ),

                .targetPosition =
                    feedback.actualPosition
            };

            a6ec_write_command(
                slave,
                &command
            );
        }

        (void)ethercat_master_exchange();

        if (all_disabled)
        {
            break;
        }

        sleep_1ms();
    }
}


/* ============================================================================
 * MAIN
 * ============================================================================ */

static void SimulatorControlTask(
    void *pvParameters
)
{
    (void)pvParameters;

    configure_robot();

    configure_state_dependencies();

    memset(
        &inputs,
        0,
        sizeof(inputs)
    );

    inputs.path_validation_sample_budget =
        64U;

    if (
        !state_machine_init(
            &machine,
            1U
        )
    )
    {
        fprintf(
            stderr,
            "Could not initialize global state machine.\n"
        );

        exit(EXIT_FAILURE);
    }

    SupervisorRuntime runtime;

    memset(
        &runtime,
        0,
        sizeof(runtime)
    );

    runtime.command_socket =
        -1;

    runtime.telemetry_socket =
        -1;

    runtime.selected_program =
        HMI_PROGRAM_LINE;

    runtime.pending_teaching_event =
        TEACH_EVENT_NONE;

    if (!setup_udp(&runtime))
    {
        exit(EXIT_FAILURE);
    }

    memset(
        &guidance,
        0,
        sizeof(guidance)
    );

    RobotState last_state =
        machine.current_state;

    HomingPhase last_homing_phase =
        machine.homing.phase;

    uint32_t tick =
        0U;

    printf(
        "\n"
        "============================================================\n"
        " HMI STATE-MACHINE SIMULATOR\n"
        "============================================================\n"
        "Actual state modules: BOOT -> HOMING -> IDLE -> TEACHING\n"
        "                      -> PATH_VALIDATION -> APPROACH\n"
        "HMI command port:     %u\n"
        "Panel status port:    %u\n"
        "Teaching status port: %u\n"
        "MATLAB telemetry:     %s:%u (same int32 A6 format as main.c)\n"
        "============================================================\n",
        (unsigned)HMI_CONTROLLER_PORT,
        (unsigned)HMI_PANEL_STATUS_PORT,
        (unsigned)HMI_TEACH_STATUS_PORT,
        MATLAB_IP,
        (unsigned)MATLAB_PORT
    );

    TickType_t last_wake_time =
        xTaskGetTickCount();

    const TickType_t cycle_period =
        pdMS_TO_TICKS(1U);


    for (;;)
    {
        if (stop_requested)
        {
            break;
        }
        poll_hmi_commands(
            &runtime
        );

        /*
         * The simulated guidance controller owns motion only while TEACHING.
         * The actual Teaching state remains read-only with respect to motors.
         */
        if (
            machine.current_state ==
            ROBOT_STATE_TEACHING
        )
        {
            if (
                runtime.estop_active ||
                runtime.paused
            )
            {
                guidance.active =
                    false;

                (void)hold_current_position(
                    &runtime.last_wkc
                );
            }
            else
            {
                (void)guidance_step(
                    &guidance,
                    &runtime.last_wkc
                );
            }
        }
        else if (
            machine.current_state ==
            ROBOT_STATE_PATH_VALIDATION
        )
        {
            /*
             * Path Validation intentionally commands no drives.
             * Keep the already-enabled simulated robot stationary while the
             * CPU validates the frozen Teaching draft.
             */
            (void)hold_current_position(
                &runtime.last_wkc
            );
        }

        memset(
            &inputs,
            0,
            sizeof(inputs)
        );

        inputs.path_validation_sample_budget =
            64U;

        inputs.teaching_runtime.timestamp_ms =
            tick;

        inputs.teaching_runtime.calibration_version =
            1U;

        inputs.teaching_runtime.active_frame_id =
            1U;

        inputs.teaching_runtime.active_tool_id =
            1U;

        inputs.teaching_runtime.robot_motion_settled =
            !guidance.active;

        inputs.teaching_runtime.manual_guidance_active =
            machine.current_state ==
            ROBOT_STATE_TEACHING;

        inputs.teaching_runtime.motion_permitted =
            !runtime.estop_active &&
            !runtime.paused;

        inputs.teaching_runtime.estop_active =
            runtime.estop_active;

        inputs.teaching_runtime.protective_stop_active =
            false;

        inputs.teaching_runtime.global_fault_active =
            false;

        inputs.teaching_runtime.robot_homed =
            runtime.robot_homed;

        if (
            machine.current_state ==
                ROBOT_STATE_IDLE
            &&
            runtime.start_teach_requested
        )
        {
            inputs.idle_command =
                IDLE_COMMAND_TEACH;
        }

        if (
            machine.current_state ==
                ROBOT_STATE_TEACHING
            &&
            runtime.pending_teaching_event !=
                TEACH_EVENT_NONE
        )
        {
            inputs.teaching_event =
                runtime.pending_teaching_event;

            runtime.pending_teaching_event =
                TEACH_EVENT_NONE;
        }

        if (
            machine.current_state ==
                ROBOT_STATE_PATH_VALIDATION
            &&
            runtime.approach_requested
        )
        {
            inputs.approach_operation =
                APPROACH_OPERATION_PREVIEW;
        }
        else
        {
            inputs.approach_operation =
                APPROACH_OPERATION_NONE;
        }

        inputs.approach_control.motion_permission =
            !runtime.estop_active &&
            !runtime.paused;

        inputs.approach_control.pause_requested =
            runtime.paused;

        inputs.approach_control.protective_stop_active =
            false;

        inputs.approach_control.reset_requested =
            runtime.approach_reset_requested;

        inputs.approach_control.home_requested =
            runtime.approach_home_requested;

        inputs.approach_control.estop_active =
            runtime.estop_active;

        inputs.approach_control.external_fault_active =
            false;

        runtime.approach_reset_requested =
            false;

        runtime.approach_home_requested =
            false;

        const StateStepResult step_result =
            state_machine_step(
                &machine,
                &dependencies,
                &inputs
            );

        if (
            machine.current_state ==
                ROBOT_STATE_HOMING
            &&
            machine.homing.phase !=
                last_homing_phase
        )
        {
            printf(
                "HOMING: phase=%d error=%d axis=%d samples=%zu verify=%u stable=%u\n",
                (int)machine.homing.phase,
                (int)machine.homing.error,
                machine.homing.failedAxis,
                machine.homing.samplesSent,
                machine.homing.verificationCycles,
                machine.homing.stableCycles
            );

            fflush(
                stdout
            );

            last_homing_phase =
                machine.homing.phase;
        }

        if (
            machine.current_state !=
            last_state
        )
        {
            printf(
                "FSM: %s -> %s\n",
                state_machine_state_name(
                    last_state
                ),
                state_machine_state_name(
                    machine.current_state
                )
            );

            fflush(
                stdout
            );

            if (
                last_state ==
                    ROBOT_STATE_HOMING
                &&
                machine.current_state ==
                    ROBOT_STATE_IDLE
            )
            {
                runtime.robot_homed =
                    true;
            }

            if (
                machine.current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                runtime.start_teach_requested =
                    false;

                runtime.pending_teaching_event =
                    selected_program_event(
                        runtime.selected_program
                    );
            }

            if (
                machine.current_state ==
                ROBOT_STATE_APPROACH
            )
            {
                runtime.approach_requested =
                    false;
            }

            last_state =
                machine.current_state;
        }

        /*
         * Supervisor-side handling for the two explicit abort reasons already
         * exposed by Approach. This does not alter Approach's internal logic.
         */
        if (
            machine.current_state ==
                ROBOT_STATE_APPROACH
            &&
            step_result ==
                STATE_STEP_COMPLETE
            &&
            machine.approach.result ==
                APPROACH_RESULT_ABORTED
        )
        {
            if (
                machine.approach.error ==
                APPROACH_ERR_HOME_REQUESTED
            )
            {
                machine.previous_state =
                    machine.current_state;

                state_homing_enter(
                    &machine.homing
                );

                machine.current_state =
                    ROBOT_STATE_HOMING;

                runtime.robot_homed =
                    false;

                last_state =
                    machine.current_state;
            }
            else if (
                machine.approach.error ==
                APPROACH_ERR_RESET_REQUESTED
            )
            {
                machine.previous_state =
                    machine.current_state;

                state_idle_enter(
                    &machine.idle
                );

                machine.current_state =
                    ROBOT_STATE_IDLE;

                last_state =
                    machine.current_state;
            }
        }

        if (
            step_result ==
                STATE_STEP_FAILED
            &&
            machine.current_state ==
                ROBOT_STATE_BOOT
        )
        {
            fprintf(
                stderr,
                "BOOT failed. Error=%d axis=%d\n",
                (int)machine.boot.error,
                machine.boot.failedAxis
            );
        }

        if (++tick % 20U == 0U)
        {
            send_status(
                &runtime
            );

            send_matlab_telemetry(
                &runtime
            );
        }

        vTaskDelayUntil(
            &last_wake_time,
            cycle_period
        );
    }

    disable_drives();

    ethercat_master_close();

    if (runtime.telemetry_socket >= 0)
    {
        close(
            runtime.telemetry_socket
        );
    }

    if (runtime.command_socket >= 0)
    {
        close(
            runtime.command_socket
        );
    }

    vTaskDelete(
        NULL
    );
}


/* ============================================================================
 * PC SIMULATOR ENTRY POINT
 * ============================================================================
 *
 * The simulator deliberately uses the FreeRTOS POSIX port.  The future real
 * robot main.c will use the MCU/STM32 FreeRTOS port, but both entry points are
 * expected to drive the SAME StateMachine and state modules.
 *
 * Simulator-only services remain outside the state logic:
 *
 *      KickCAT/SOEM
 *      desktop HMI UDP
 *      simulated hand guidance
 *      host RAM validated-trajectory storage
 *      MATLAB telemetry
 * ============================================================================
 */

int main(void)
{
    signal(
        SIGINT,
        on_signal
    );

    signal(
        SIGTERM,
        on_signal
    );


    printf(
        "\n"
        "============================================================\n"
        " ROBOT PC SIMULATOR - FreeRTOS POSIX\n"
        "============================================================\n"
        "Entry point: Simulation/simulator_main.c\n"
        "State logic: shared StateMachine/States modules\n"
        "============================================================\n"
    );

    fflush(
        stdout
    );


    const BaseType_t task_result =
        xTaskCreate(
            SimulatorControlTask,
            "SimControl",
            SIM_CONTROL_TASK_STACK_WORDS,
            NULL,
            SIM_CONTROL_TASK_PRIORITY,
            NULL
        );


    if (task_result != pdPASS)
    {
        fprintf(
            stderr,
            "Failed to create simulator control task.\n"
        );

        return 1;
    }


    vTaskStartScheduler();


    /*
     * The scheduler should not return during normal simulator operation.
     */
    fprintf(
        stderr,
        "FreeRTOS scheduler stopped unexpectedly.\n"
    );

    return 1;
}
