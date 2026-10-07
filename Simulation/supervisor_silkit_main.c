#include "../StateMachine/supervisor_task.h"
#include "../HMI/hmi_task.h"
#include "../HMI/hmi_protocol.h"
#include "../CANComm/SILKit/silkit_can_backend.h"
#include "../CANComm/CANopen/canopen_master.h"
#include "../ServoDrive/CiA402/cia402.h"
#include "../ServoDrive/AvatarM/avatar_m_position.h"
#include "../ControlCore/Kinematics/control_fk.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define SIM_NUM_AXES 6
#define SIM_PV_GEOMETRY_CAPACITY 512
#define SIM_PV_STORAGE_CAPACITY 100000
#define MATLAB_STATUS_PORT 5005
#define DEG2RAD(x) ((x)*0.017453292519943295)
typedef struct {
    PvExecutionSample samples[SIM_PV_STORAGE_CAPACITY];
    uint32_t sample_count;
    bool writing, committed;
    ValidatedTrajectory metadata;
} RamValidatedStorage;
static RobotConfig robot;
/*
 * Active Supervisor simulation uses the CANopen / AVATAR stack end-to-end.
 * The removed EtherCAT/A6EC implementation is preserved on the archive branch.
 */
static CanBackend silkit_backend;
static CanopenMaster can_boot_master;
static AvatarMPositionScale avatar_position_scales[SIM_NUM_AXES];

static HomingConfig homing_config;
static TeachingConfig teaching_config;
static PathValidationConfig path_validation_config;
static PathValidationServices path_validation_services;
static PathValidationWorkspace path_validation_workspace;
static Vec3 pv_raw_geometry[SIM_PV_GEOMETRY_CAPACITY], pv_arc_geometry[SIM_PV_GEOMETRY_CAPACITY];
static real_t pv_l_original[SIM_PV_GEOMETRY_CAPACITY], pv_l_arc[SIM_PV_GEOMETRY_CAPACITY];
static ADLSInfo pv_ik_scratch, guidance_info;
static RamValidatedStorage validated_storage;
static PathValidationStorage path_validation_storage;
static ValidatedTrajectory validated_trajectory;
static ApproachConfig approach_config;
static ApproachServices approach_services;
static PathExecutionConfig execution_config = {1000, 5};
static PathExecutionServices execution_services;
static PausedServices paused_services;
static FaultServices fault_services;
static EmergencyStopServices emergency_services;
static HomingState retraction;
static bool wire_feed, estop, protective, injected_fault, communication_failure, guidance_pending;
static float guidance_xyz[3];
static unsigned guidance_error;
static bool homed;
static int udp=-1;
static int matlab_udp=-1;
static struct sockaddr_in matlab_status_address;
static bool matlab_status_ready;
static uint32_t last_sequence, status_sequence;

/*
 * Human-readable operator/debug event log.
 * Each HMI button press gets a monotonically increasing event number.
 * Meaningful controller outcomes are then tagged with the most recent event.
 */
static FILE *operator_event_log;
static bool operator_event_log_started;
static bool operator_event_log_failed;
static uint32_t operator_event_counter;
static uint32_t last_operator_event_id;

/* Sequence numbers belong to each GUI sender, not globally to both windows. */
static struct { uint32_t address, sequence; uint16_t port; bool used; } peers[8];
static SupervisorOutputSnapshot lamps;

static uint32_t sim_now_ms(void)
{
    return (uint32_t)(
        ((uint64_t)xTaskGetTickCount() * 1000ULL) /
        (uint64_t)configTICK_RATE_HZ
    );
}

static bool sim_can_communication_healthy(void)
{
    return
        !communication_failure &&
        canopen_master_all_heartbeats_operational(
            &can_boot_master
        );
}

static bool sim_can_drives_ready(void)
{
    return
        !communication_failure &&
        canopen_master_all_feedback_valid(
            &can_boot_master
        ) &&
        canopen_master_all_drives_operation_enabled(
            &can_boot_master
        );
}

static bool sim_read_joint(
    size_t axis,
    double *joint_rad
)
{
    if (
        joint_rad == NULL ||
        axis >= SIM_NUM_AXES
    )
    {
        return false;
    }

    const AvatarMDrive *drive =
        canopen_master_drive(
            &can_boot_master,
            axis
        );

    if (
        drive == NULL ||
        !drive->feedback_valid
    )
    {
        return false;
    }

    return avatar_m_position_units_to_joint_rad(
        &avatar_position_scales[axis],
        drive->feedback.actual_position,
        joint_rad
    );
}

static bool sim_read_joint_vector(
    double q[SIM_NUM_AXES]
)
{
    if (q == NULL)
    {
        return false;
    }

    for (size_t axis = 0U;
         axis < SIM_NUM_AXES;
         ++axis)
    {
        if (!sim_read_joint(
                axis,
                &q[axis]))
        {
            return false;
        }
    }

    return true;
}

static bool sim_hold_current(void)
{
    int32_t targets[SIM_NUM_AXES];

    for (size_t axis = 0U;
         axis < SIM_NUM_AXES;
         ++axis)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(
                &can_boot_master,
                axis
            );

        if (
            drive == NULL ||
            !drive->feedback_valid
        )
        {
            return false;
        }

        targets[axis] =
            drive->feedback.actual_position;
    }

    return canopen_master_send_target_cycle(
        &can_boot_master,
        targets,
        SIM_NUM_AXES
    );
}

static bool sim_apply_guided_joint_vector(
    const double q[SIM_NUM_AXES]
)
{
    if (q == NULL)
    {
        return false;
    }

    int32_t targets[SIM_NUM_AXES];

    for (size_t axis = 0U;
         axis < SIM_NUM_AXES;
         ++axis)
    {
        if (!avatar_m_joint_rad_to_position_units(
                &avatar_position_scales[axis],
                q[axis],
                &targets[axis]))
        {
            return false;
        }
    }

    /*
     * PC-only manual-guidance stand-in:
     * the Teaching window requests a Cartesian pose, but all resulting joint
     * commands still travel through the real CanopenMaster -> SIL Kit CAN1
     * transport -> six AVATAR SIL Kit nodes. No motor state is mutated here.
     */
    return canopen_master_send_target_cycle(
        &can_boot_master,
        targets,
        SIM_NUM_AXES
    );
}

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
    homing_config =
        (HomingConfig)
        {
            .duration = 5.0,
            .dt = 0.002,
            .positionTolerance = DEG2RAD(0.5),
            .requiredStableCycles = 20U,
            .maxVerificationCycles = 2000U
        };

    for (int i = 0; i < SIM_NUM_AXES; ++i)
    {
        if (!avatar_m_position_scale_default(
                &avatar_position_scales[i],
                50.0))
        {
            fprintf(stderr, "Invalid AVATAR simulation position scale.\n");
            exit(1);
        }
    }

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

    /*
     * Keep validation limits bit-for-bit aligned with Teaching's float
     * settings. A literal double 0.100 can be slightly smaller than the
     * promoted float 0.100F, which made the HMI's legal maximum speed get
     * rejected as PV_ERR_INVALID_PARAMETER.
     */
    path_validation_config.default_tcp_speed_mps =
        (real_t)teaching_config.default_speed_mps;

    path_validation_config.max_tcp_speed_mps =
        (real_t)teaching_config.maximum_speed_mps;

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


}

static bool relay(bool enable,void *ctx) { (void)ctx;wire_feed=enable;return true; }
static bool off(void *ctx) { return relay(false,ctx); }
static bool safe(void *ctx) { (void)ctx;return off(NULL); }
static bool hold(bool *stopped,void *ctx) {
    (void)ctx;
    if(stopped==NULL || !sim_hold_current())return false;
    *stopped=true;
    return true;
}
static bool retract_prepare(void *ctx) { (void)ctx;state_homing_enter(&retraction);return true; }
static StateStepResult retract_step(void *ctx) {
    (void)ctx;
    return state_homing_step(
        &retraction,
        &homing_config,
        &robot,
        &can_boot_master,
        avatar_position_scales,
        (uint32_t)(xTaskGetTickCount()*1000/configTICK_RATE_HZ)
    );
}
static bool clearance(void *ctx) {
    (void)ctx;
    double q[SIM_NUM_AXES];
    if(!sim_read_joint_vector(q))return false;
    for(int i=0;i<SIM_NUM_AXES;i++)
        if(fabs(q[i]-robot.configuration.home[i])>homing_config.positionTolerance)
            return false;
    /* Simulation home tolerance, NOT a collision or cell-clearance proof. */
    return true;
}
static bool read_inputs(void *ctx,SupervisorInputSnapshot *out) {
    (void)ctx;
    memset(out,0,sizeof(*out));
    out->valid=true;
    out->timestamp_ms=sim_now_ms();
    out->asserted[SUP_IO_ESTOP]=estop;
    out->asserted[SUP_IO_PROTECTIVE_STOP]=protective;
    double q[SIM_NUM_AXES];
    if(sim_read_joint_vector(q))
        for(int i=0;i<SIM_NUM_AXES;i++)
            out->asserted[SUP_IO_HOME_J1+i]=
                fabs(q[i]-robot.configuration.home[i])<homing_config.positionTolerance;
    return true;
}
static bool write_outputs(void *ctx,const SupervisorOutputSnapshot *out) { (void)ctx;lamps=*out;return true; }
static void post_required(SupervisorMessage *m) {
    if(!supervisor_task_post(m,0)) {
        /* A failed safety/runtime publication must not be silently ignored. */
        safe(NULL);fprintf(stderr,"Supervisor queue overflow in simulation adapter\n");exit(1);
    }
}
static void input_task(void *arg)
{
    (void)arg;bool old_estop=false,old_protective=false,old_fault=false;

    for(;;) {
        StateMachine machine;
        supervisor_task_get_state(&machine);
        if(machine.activeState==ROBOT_STATE_IDLE && machine.previousState==ROBOT_STATE_HOMING)homed=true;
        if(estop||protective||injected_fault||communication_failure) {homed=false;safe(NULL);}
        const bool drives_ready=sim_can_drives_ready();
        const bool communication_healthy=sim_can_communication_healthy();
        const bool motion_permitted=
            drives_ready&&communication_healthy&&!estop&&!protective&&!injected_fault;

        if(guidance_pending) {
            guidance_pending=false;guidance_error=2;
            if(machine.activeState==ROBOT_STATE_TEACHING && !estop && !protective && drives_ready) {
                double q[SIM_NUM_AXES],target[4][4],solution[SIM_NUM_AXES];
                if(sim_read_joint_vector(q)) {
                    control_fk(&robot,q,target);
                    for(int j=0;j<3;j++)target[j][3]=guidance_xyz[j];
                    ADLSParameters params;adls_default_parameters(&params);
                    if(adls_ik(&robot,target,q,&params,solution,&guidance_info) &&
                       sim_apply_guided_joint_vector(solution)) {
                        guidance_error=0;
                    } else {
                        guidance_error=1;
                        fprintf(stderr,"[GUIDANCE] IK/apply failed: target=[%.6f %.6f %.6f] m iterations=%d pos_error=%.6g m ori_error=%.6g rad\n", guidance_xyz[0],guidance_xyz[1],guidance_xyz[2],guidance_info.iterations,guidance_info.positionError,guidance_info.orientationError);
                    }
                }
            }
        }
        SupervisorMessage m={0};m.type=SUPERVISOR_MESSAGE_SAFETY;
        m.data.safety.statusValid=true;
        m.data.safety.drivesReady=drives_ready;
        m.data.safety.communicationHealthy=communication_healthy;
        m.data.safety.estopActive=estop;m.data.safety.protectiveStopActive=protective;
        m.data.safety.globalFaultActive=injected_fault||communication_failure;
        m.data.safety.motionPermitted=motion_permitted;
        m.data.safety.timestampMs=sim_now_ms();
        post_required(&m);
        SupervisorMessage event={0};event.type=SUPERVISOR_MESSAGE_EVENT;
        if(estop!=old_estop){event.data.event.type=estop?SUPERVISOR_EVENT_ESTOP_ASSERTED:SUPERVISOR_EVENT_ESTOP_RELEASED;post_required(&event);old_estop=estop;}
        if(protective!=old_protective){event.data.event.type=protective?SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED:SUPERVISOR_EVENT_PROTECTIVE_STOP_CLEARED;post_required(&event);old_protective=protective;}
        if((injected_fault||communication_failure)!=old_fault){
            if(injected_fault||communication_failure){event.data.event.type=SUPERVISOR_EVENT_FAULT_DETECTED;event.data.event.faultSeverity=ROBOT_FAULT_SEVERITY_RECOVERABLE;event.data.event.faultCode=0x9001;post_required(&event);}
            old_fault=injected_fault||communication_failure;
        }
        m.type=SUPERVISOR_MESSAGE_TEACHING_RUNTIME;
        memset(&m.data,0,sizeof(m.data));
        m.data.teaching_runtime.timestamp_ms=(uint32_t)(xTaskGetTickCount()*1000/configTICK_RATE_HZ);
        m.data.teaching_runtime.calibration_version=1;
        m.data.teaching_runtime.robot_homed=homed;
        m.data.teaching_runtime.manual_guidance_active=machine.activeState==ROBOT_STATE_TEACHING && !guidance_pending;
        m.data.teaching_runtime.robot_motion_settled=!guidance_pending;
        m.data.teaching_runtime.motion_permitted=motion_permitted;
        m.data.teaching_runtime.estop_active=estop;
        m.data.teaching_runtime.protective_stop_active=protective;
        m.data.teaching_runtime.global_fault_active=injected_fault||communication_failure;
        post_required(&m);
        m.type=SUPERVISOR_MESSAGE_APPROACH_CONTROL;memset(&m.data,0,sizeof(m.data));
        m.data.approach_control.motion_permission=motion_permitted;
        m.data.approach_control.estop_active=estop;m.data.approach_control.protective_stop_active=protective;
        m.data.approach_control.external_fault_active=injected_fault||communication_failure;
        post_required(&m);
        m.type=SUPERVISOR_MESSAGE_PATH_EXECUTION_INPUTS;memset(&m.data,0,sizeof(m.data));
        m.data.path_execution_inputs.now_ms=(uint32_t)(xTaskGetTickCount()*1000/configTICK_RATE_HZ);
        post_required(&m);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
static void operator_log_open(void)
{
    if (operator_event_log_started)
    {
        return;
    }

    operator_event_log_started = true;

    const char *path =
        getenv("SIM_EVENT_LOG");

    if (path == NULL || path[0] == '\0')
    {
        path =
            "operator_events.txt";
    }

    operator_event_log =
        fopen(
            path,
            "w"
        );

    if (operator_event_log == NULL)
    {
        perror("Cannot open operator event log");
        operator_event_log_failed = true;
        return;
    }

    fprintf(
        operator_event_log,
        "# GP Robot operator/debug event log\n"
        "# EVENT numbers are HMI/operator actions received by the controller.\n"
        "# OUTCOME lines show meaningful changes observed after the latest EVENT.\n"
        "# time_ms is monotonic FreeRTOS simulation time.\n\n"
    );

    fflush(
        operator_event_log
    );

    printf(
        "[LOGGER] operator events -> %s\n",
        path
    );
    fflush(stdout);
}

static const char *operator_button_name(
    uint32_t command,
    RobotStateId state
)
{
    switch (command)
    {
        case HMI_EVENT_SELECT_LINE:
            return "LINE";

        case HMI_EVENT_SELECT_ARC:
            return "CIRCULAR_ARC";

        case HMI_EVENT_SELECT_CIRCLE:
            return "CIRCLE";

        case HMI_EVENT_RECORD:
            return "RECORD";

        case HMI_EVENT_VALIDATE_PREVIEW:
            if (state == ROBOT_STATE_TEACHING)
            {
                return "VALIDATE";
            }

            if (state == ROBOT_STATE_IDLE)
            {
                return "PREVIEW";
            }

            return "VALIDATE/PREVIEW";

        case HMI_EVENT_SPEED_INCREASE:
            return "SPEED_PLUS";

        case HMI_EVENT_SPEED_DECREASE:
            return "SPEED_MINUS";

        case HMI_EVENT_SPEED_DEFAULT:
            return "DEFAULT_MODE";

        case HMI_EVENT_START:
            return "START";

        case HMI_EVENT_PAUSE:
            return "PAUSE";

        case HMI_EVENT_RESUME:
            return "RESUME";

        case HMI_EVENT_RESET:
            return "RESET";

        case HMI_EVENT_HOME:
            return "HOME";

        case HMI_PROTOCOL_SIM_ESTOP_TOGGLE:
            return "E_STOP";

        case 0x80000003U:
            return "SIM_FAULT";

        case 0x80000004U:
            return "PROTECTIVE_STOP";

        case 0x80000005U:
            return "COMMUNICATION_FAILURE";

        default:
            return "UNKNOWN";
    }
}

static const char *operator_program_name(
    unsigned program
)
{
    switch (program)
    {
        case HMI_PROGRAM_NONE:
            return "NONE";

        case HMI_PROGRAM_LINE:
            return "LINE";

        case HMI_PROGRAM_ARC:
            return "CIRCULAR_ARC";

        case HMI_PROGRAM_CIRCLE:
            return "CIRCLE";

        default:
            return "UNKNOWN_PROGRAM";
    }
}

static const char *operator_validation_result_name(
    unsigned result
)
{
    switch (result)
    {
        case PV_RESULT_NONE:
            return "NONE";

        case PV_RESULT_RUNNING:
            return "RUNNING";

        case PV_RESULT_VALID:
            return "VALID";

        case PV_RESULT_INVALID:
            return "INVALID";

        case PV_RESULT_CANCELLED:
            return "CANCELLED";

        default:
            return "UNKNOWN_RESULT";
    }
}

static void operator_log_press(
    uint32_t command,
    uint32_t hmi_sequence
)
{
    if (command == HMI_PROTOCOL_SIM_GUIDANCE_POSE)
    {
        /*
         * Teaching guidance packets can arrive continuously while dragging.
         * Do not flood the human-readable button/event log with them.
         */
        return;
    }

    operator_log_open();

    if (
        operator_event_log == NULL ||
        operator_event_log_failed
    )
    {
        return;
    }

    StateMachine machine;
    const bool have_state =
        supervisor_task_get_state(
            &machine
        );

    const RobotStateId state =
        have_state
            ? machine.activeState
            : ROBOT_STATE_COUNT;

    last_operator_event_id =
        ++operator_event_counter;

    fprintf(
        operator_event_log,
        "EVENT %03u | time_ms=%u | hmi_seq=%u | button=%s | state_before=%s\n",
        last_operator_event_id,
        (unsigned)(
            xTaskGetTickCount() *
            1000 /
            configTICK_RATE_HZ
        ),
        hmi_sequence,
        operator_button_name(
            command,
            state
        ),
        state_machine_state_name(
            state
        )
    );

    fflush(
        operator_event_log
    );
}

static void operator_log_outcomes(
    const SupervisorDiagnostics *s
)
{
    static bool snapshot_valid;
    static RobotStateId previous_state;
    static unsigned previous_geometry;
    static unsigned previous_points;
    static unsigned previous_validation_result;
    static unsigned previous_rejections;
    static unsigned previous_fault_code;
    static bool previous_preview_accepted;
    static bool previous_estop;
    static bool previous_wire_feed;
    static float previous_speed;

    if (s == NULL)
    {
        return;
    }

    operator_log_open();

    if (
        operator_event_log == NULL ||
        operator_event_log_failed
    )
    {
        return;
    }

    const uint32_t now =
        (uint32_t)(
            xTaskGetTickCount() *
            1000 /
            configTICK_RATE_HZ
        );

    if (!snapshot_valid)
    {
        snapshot_valid = true;
        previous_state = s->machine.activeState;
        previous_geometry = s->selected_geometry;
        previous_points = s->captured_points;
        previous_validation_result =
            s->validation.report.result;
        previous_rejections =
            s->machine.rejectedEventCount;
        previous_fault_code =
            s->machine.activeFaultCode;
        previous_preview_accepted =
            s->machine.previewAccepted;
        previous_estop = estop;
        previous_wire_feed = wire_feed;
        previous_speed =
            s->teaching.displayed_speed_mps;

        fprintf(
            operator_event_log,
            "STATUS    | time_ms=%u | startup_state=%s | CAN=%u/%u\n",
            now,
            state_machine_state_name(
                s->machine.activeState
            ),
            sim_can_drives_ready()
                ? SIM_NUM_AXES
                : 0U,
            SIM_NUM_AXES
        );

        fflush(
            operator_event_log
        );

        return;
    }

#define LOG_OUTCOME(...)                                                     \
    do                                                                       \
    {                                                                        \
        fprintf(                                                             \
            operator_event_log,                                              \
            "OUTCOME   | time_ms=%u | after_event=%03u | ",                 \
            now,                                                             \
            last_operator_event_id                                           \
        );                                                                   \
        fprintf(                                                             \
            operator_event_log,                                              \
            __VA_ARGS__                                                      \
        );                                                                   \
        fputc(                                                               \
            '\n',                                                            \
            operator_event_log                                               \
        );                                                                   \
        fflush(                                                              \
            operator_event_log                                               \
        );                                                                   \
    } while (0)

    if (s->machine.activeState != previous_state)
    {
        LOG_OUTCOME(
            "state=%s -> %s",
            state_machine_state_name(
                previous_state
            ),
            state_machine_state_name(
                s->machine.activeState
            )
        );

        previous_state =
            s->machine.activeState;
    }

    if (s->selected_geometry != previous_geometry)
    {
        LOG_OUTCOME(
            "program_selection=%s -> %s",
            operator_program_name(
                previous_geometry
            ),
            operator_program_name(
                (unsigned)s->selected_geometry
            )
        );

        previous_geometry =
            s->selected_geometry;
    }

    if (s->captured_points != previous_points)
    {
        LOG_OUTCOME(
            "recorded_points=%u -> %u",
            previous_points,
            s->captured_points
        );

        previous_points =
            s->captured_points;
    }

    if (
        s->validation.report.result !=
        previous_validation_result
    )
    {
        LOG_OUTCOME(
            "validation_result=%s -> %s error=%s(%u)",
            operator_validation_result_name(
                previous_validation_result
            ),
            operator_validation_result_name(
                (unsigned)s->validation.report.result
            ),
            state_path_validation_error_name(
                s->validation.report.error
            ),
            (unsigned)s->validation.report.error
        );

        previous_validation_result =
            s->validation.report.result;
    }

    if (
        s->machine.previewAccepted &&
        !previous_preview_accepted
    )
    {
        LOG_OUTCOME(
            "preview_complete=YES"
        );
    }

    previous_preview_accepted =
        s->machine.previewAccepted;

    if (
        s->machine.rejectedEventCount !=
        previous_rejections
    )
    {
        LOG_OUTCOME(
            "command_rejected total=%u last_event=%s",
            s->machine.rejectedEventCount,
            state_machine_event_name(
                s->machine.lastEvent
            )
        );

        previous_rejections =
            s->machine.rejectedEventCount;
    }

    if (
        s->machine.activeFaultCode !=
        previous_fault_code
    )
    {
        LOG_OUTCOME(
            "fault_code=0x%04X -> 0x%04X",
            previous_fault_code,
            s->machine.activeFaultCode
        );

        previous_fault_code =
            s->machine.activeFaultCode;
    }

    if (estop != previous_estop)
    {
        LOG_OUTCOME(
            "estop=%s",
            estop
                ? "ACTIVE"
                : "RELEASED"
        );

        previous_estop =
            estop;
    }

    if (wire_feed != previous_wire_feed)
    {
        LOG_OUTCOME(
            "wire_feed=%s",
            wire_feed
                ? "ON"
                : "OFF"
        );

        previous_wire_feed =
            wire_feed;
    }

    if (
        fabsf(
            s->teaching.displayed_speed_mps -
            previous_speed
        ) > 1.0e-7F
    )
    {
        LOG_OUTCOME(
            "teaching_speed=%.4f -> %.4f m/s",
            previous_speed,
            s->teaching.displayed_speed_mps
        );

        previous_speed =
            s->teaching.displayed_speed_mps;
    }

#undef LOG_OUTCOME
}

static float word_float(uint32_t w) { float f;memcpy(&f,&w,4);return f; }
static uint32_t float_word(float f) { uint32_t w;memcpy(&w,&f,4);return w; }
static bool udp_event(void *ctx,HmiEvent *event)
{
    (void)ctx;*event=HMI_EVENT_NONE;
    uint32_t words[HMI_COMMAND_WORD_COUNT+1];
    struct sockaddr_in peer={0};socklen_t peer_length=sizeof(peer);
    ssize_t n=recvfrom(udp,words,sizeof(words),MSG_DONTWAIT,(struct sockaddr*)&peer,&peer_length);
    if(n!=(ssize_t)(HMI_COMMAND_WORD_COUNT*4))return false;
    for(unsigned i=0;i<HMI_COMMAND_WORD_COUNT;i++)words[i]=ntohl(words[i]);
    if(words[0]!=HMI_COMMAND_MAGIC||words[1]!=HMI_PROTOCOL_VERSION)return false;
    /* Duplicate UDP events cannot repeat Record or a context-dependent button. */
    unsigned slot=8;
    for(unsigned i=0;i<8;i++)if(peers[i].used && peers[i].address==peer.sin_addr.s_addr && peers[i].port==peer.sin_port){slot=i;break;}
    if(slot==8)for(unsigned i=0;i<8;i++)if(!peers[i].used){slot=i;break;}
    if(slot==8)return false;
    if(peers[slot].used && (int32_t)(words[3]-peers[slot].sequence)<=0)return false;
    peers[slot].used=true;peers[slot].address=peer.sin_addr.s_addr;peers[slot].port=peer.sin_port;peers[slot].sequence=words[3];
    last_sequence=words[3];

    operator_log_press(
        words[2],
        words[3]
    );

    if(words[2]!=HMI_PROTOCOL_SIM_GUIDANCE_POSE)
        printf("[HMI RX] time_ms=%u command=%u sequence=%u (received, not yet acknowledged)\n",
            (unsigned)(xTaskGetTickCount()*1000/configTICK_RATE_HZ),words[2],words[3]);
    if(words[2]==HMI_PROTOCOL_SIM_ESTOP_TOGGLE){estop=!estop;return false;}
    if(words[2]==HMI_PROTOCOL_SIM_GUIDANCE_POSE){for(int i=0;i<3;i++)guidance_xyz[i]=word_float(words[4+i]);guidance_pending=true;return false;}
    /* Additive mock-input commands; the existing GUI need not emit them. */
    if(words[2]==0x80000003U){injected_fault=words[4]!=0;return false;}
    if(words[2]==0x80000004U){protective=words[4]!=0;return false;}
    if(words[2]==0x80000005U){communication_failure=words[4]!=0;return false;}
    if(!hmi_event_is_valid((HmiEvent)words[2]))return false;
    *event=(HmiEvent)words[2];return true;
}

/* PC diagnostics only: runs from the lower-priority HMI task.
 * Samples are decimated to 10 Hz; this is not a 1 ms motion recorder.
 * CSV open/write failure is reported once and never changes robot state.
 */
static void trace_state(const SupervisorDiagnostics *s)
{
    operator_log_outcomes(s);

    static bool started, file_failed;
    static FILE *csv;
    static uint32_t previous_ms;
    static RobotStateId previous_state = ROBOT_STATE_COUNT;
    uint32_t now = (uint32_t)(xTaskGetTickCount()*1000/configTICK_RATE_HZ);
    if (!started) {
        started=true;
        const char *path=getenv("SIM_TRACE_FILE");
        if (!path || !*path) path="simulation_trace.csv";
        csv=fopen(path,"w");
        if (!csv) { perror("Cannot open simulation trace"); file_failed=true; }
        else fprintf(csv,"time_ms,state,phase,error,mode,segments,points,next_point,job_speed_mps,record_allowed,validate_allowed,pv_result,pv_progress,pv_failed_segment,pv_failed_sample,pv_failed_joint,approach_progress,execution_sample,execution_count,wire_feed,guidance_error,motion_permitted,drives_ready,estop,protective,fault_code,reset_ack,preview_accepted,rejections,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad\n");
    }
    if (s->machine.activeState==previous_state && (uint32_t)(now-previous_ms)<100U) return;
    previous_state=s->machine.activeState;previous_ms=now;
    unsigned phase=0,error=0;
    double q[SIM_NUM_AXES]={0};
    (void)sim_read_joint_vector(q);
    const char *phase_name="SUPERVISORY",*error_name="NONE";
    switch(s->machine.activeState) {
    case ROBOT_STATE_BOOT: phase=s->boot.phase;error=s->boot.error;phase_name="BOOT_PHASE";break;
    case ROBOT_STATE_HOMING: phase=s->homing_phase;error=s->homing_error;phase_name="HOMING_PHASE";break;
    case ROBOT_STATE_TEACHING:
        phase=s->teaching.phase;error=s->teaching.error;
        phase_name=state_teaching_phase_name(s->teaching.phase);
        error_name=state_teaching_error_name(s->teaching.error);break;
    case ROBOT_STATE_PATH_VALIDATION:
        phase=s->validation.report.phase;error=s->validation.report.error;
        phase_name=state_path_validation_phase_name(s->validation.report.phase);
        error_name=state_path_validation_error_name(s->validation.report.error);break;
    case ROBOT_STATE_APPROACH:
        phase=s->approach.report.phase;error=s->approach.report.error;
        phase_name=state_approach_phase_name(s->approach.report.phase);
        error_name=state_approach_error_name(s->approach.report.error);break;
    case ROBOT_STATE_PATH_EXECUTION:
        phase=s->execution.phase;error=s->execution.error;
        phase_name=state_path_execution_phase_name(s->execution.phase);
        error_name=state_path_execution_error_name(s->execution.error);break;
    default: error=s->machine.activeFaultCode;break;
    }
    printf("[TRACE %u ms] %s phase=%s(%u) error=%s(%u) mode=%u seg=%u points=%u next=P%u speed=%.4f m/s record=%u validate=%u PV=%u/%.1f%% failed_seg/sample/joint=%u/%u/%u approach=%.1f%% sample=%u/%u relay=%u guidance_err=%u motion=%u drives=%u estop=%u protective=%u reset_ack=%u preview_ok=%u rejected=%u\n",
        now,state_machine_state_name(s->machine.activeState),phase_name,phase,error_name,error,
        s->machine.executionMode,s->segment_count,s->captured_points,s->teaching.next_point_number,
        s->teaching.displayed_speed_mps,s->teaching.record_allowed,s->teaching.validate_allowed,
        s->validation.report.result,100.0*s->validation.report.progress_0_to_1,
        s->validation.report.failed_segment,s->validation.report.failed_sample,s->validation.report.failed_joint,
        100.0*s->approach.report.progress_0_to_1,s->execution.sample_index,s->execution.sample_count,
        wire_feed,guidance_error,s->machine.safety.motionPermitted,s->machine.safety.drivesReady,
        estop,protective,s->machine.emergencyResetAcknowledged,s->machine.previewAccepted,s->machine.rejectedEventCount);
    if(csv && !file_failed) {
        int result=fprintf(csv,"%u,%s,%u,%u,%u,%u,%u,%u,%.6f,%u,%u,%u,%.6f,%u,%u,%u,%.6f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
            now,state_machine_state_name(s->machine.activeState),phase,error,s->machine.executionMode,
            s->segment_count,s->captured_points,s->teaching.next_point_number,s->teaching.displayed_speed_mps,
            s->teaching.record_allowed,s->teaching.validate_allowed,s->validation.report.result,
            s->validation.report.progress_0_to_1,s->validation.report.failed_segment,s->validation.report.failed_sample,
            s->validation.report.failed_joint,s->approach.report.progress_0_to_1,s->execution.sample_index,
            s->execution.sample_count,wire_feed,guidance_error,s->machine.safety.motionPermitted,
            s->machine.safety.drivesReady,estop,protective,s->machine.activeFaultCode,
            s->machine.emergencyResetAcknowledged,s->machine.previewAccepted,s->machine.rejectedEventCount,
            q[0],q[1],q[2],q[3],q[4],q[5]);
        if(result<0 || fflush(csv)!=0) { perror("Simulation trace write failed");file_failed=true; }
    }
}

static bool configure_matlab_status_destination(void)
{
    const char *ip =
        getenv("MATLAB_IP");

    if (ip == NULL || ip[0] == '\0')
    {
        ip = "127.0.0.1";
    }

    memset(
        &matlab_status_address,
        0,
        sizeof(matlab_status_address)
    );

    matlab_status_address.sin_family =
        AF_INET;

    matlab_status_address.sin_port =
        htons(MATLAB_STATUS_PORT);

    if (
        inet_pton(
            AF_INET,
            ip,
            &matlab_status_address.sin_addr
        ) != 1
    )
    {
        fprintf(
            stderr,
            "[MATLAB] invalid MATLAB_IP: %s\n",
            ip
        );

        matlab_status_ready =
            false;

        return false;
    }

    /*
     * Use a separate UDP socket for MATLAB. The HMI command socket is bound
     * specifically to 127.0.0.1:5010, so reusing it for a Windows/WSL host
     * destination can produce packets with an unusable loopback source.
     * An unbound socket lets Linux select the correct WSL interface/source.
     */
    matlab_udp =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (matlab_udp < 0)
    {
        perror("[MATLAB] UDP socket");
        matlab_status_ready =
            false;
        return false;
    }

    if (fcntl(
            matlab_udp,
            F_SETFL,
            O_NONBLOCK) < 0)
    {
        perror("[MATLAB] nonblocking UDP");
        close(matlab_udp);
        matlab_udp = -1;
        matlab_status_ready =
            false;
        return false;
    }

    matlab_status_ready =
        true;

    printf(
        "[MATLAB] live status -> %s:%u\n",
        ip,
        (unsigned)MATLAB_STATUS_PORT
    );
    fflush(stdout);

    return true;
}


static void udp_status(void *ctx,const SupervisorDiagnostics *s)
{
    (void)ctx;
    trace_state(s);
    uint32_t w[HMI_STATUS_WORD_COUNT]={0};
    w[0]=HMI_STATUS_MAGIC;w[1]=HMI_PROTOCOL_VERSION;w[2]=++status_sequence;
    w[3]=s->machine.activeState;w[4]=s->selected_geometry;w[5]=estop;
    w[6]=s->machine.activeState==ROBOT_STATE_PAUSED;
    w[7]=s->machine.activeState==ROBOT_STATE_TEACHING && guidance_pending && !estop;
    w[8]=s->boot.phase;w[9]=s->boot.error;w[10]=s->homing_phase;w[11]=s->homing_error;
    w[14]=s->teaching.phase;w[15]=s->teaching.error;w[16]=s->teaching.status_message_id;
    w[17]=s->teaching.next_point_number;w[18]=s->segment_count;
    w[19]=float_word(s->teaching.displayed_speed_mps);w[20]=s->teaching.record_allowed;w[21]=s->teaching.validate_allowed;
    w[22]=s->validation.report.phase;w[23]=s->validation.report.result;w[24]=s->validation.report.error;w[25]=float_word((float)s->validation.report.progress_0_to_1);
    w[26]=s->approach.report.phase;w[27]=s->approach.report.result;w[28]=s->approach.report.error;w[29]=float_word((float)s->approach.report.progress_0_to_1);
    w[30]=s->machine.executionMode==ROBOT_EXECUTION_PREVIEW && s->machine.activeState==ROBOT_STATE_PATH_EXECUTION;
    w[31]=s->machine.previewAccepted;w[32]=s->execution.error;
    w[33]=float_word(s->execution.sample_count?(float)s->execution.sample_index/s->execution.sample_count:0);
    double q[SIM_NUM_AXES]={0},T[4][4];
    (void)sim_read_joint_vector(q);
    for(int i=0;i<SIM_NUM_AXES;i++)w[37+i]=float_word((float)q[i]);
    control_fk(&robot,q,T);for(int i=0;i<3;i++)w[34+i]=float_word((float)T[i][3]);
    w[43]=sim_can_drives_ready()?SIM_NUM_AXES:0;w[44]=SIM_NUM_AXES;w[45]=s->captured_points;
    for(unsigned p=0;p<s->captured_points&&p<3;p++)for(int i=0;i<3;i++)w[46+p*3+i]=float_word(s->working_segment.points[p].position_m[i]);
    w[55]=last_sequence;w[56]=guidance_error;
    for(unsigned i=0;i<HMI_STATUS_WORD_COUNT;i++)w[i]=htonl(w[i]);
    struct sockaddr_in dst={0};dst.sin_family=AF_INET;dst.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    dst.sin_port=htons(HMI_PANEL_STATUS_PORT);(void)sendto(udp,w,sizeof(w),MSG_DONTWAIT,(struct sockaddr*)&dst,sizeof(dst));
    dst.sin_port=htons(HMI_TEACH_STATUS_PORT);(void)sendto(udp,w,sizeof(w),MSG_DONTWAIT,(struct sockaddr*)&dst,sizeof(dst));

    /*
     * MATLAB receives the same versioned status packet on its own port.
     * This keeps the visualizer synchronized with the exact Supervisor/HMI
     * state instead of maintaining a second telemetry data model.
     */
    if (matlab_status_ready)
    {
        const ssize_t matlab_sent =
            sendto(
            matlab_udp,
            w,
            sizeof(w),
            MSG_DONTWAIT,
            (struct sockaddr *)&matlab_status_address,
            sizeof(matlab_status_address)
        );

        static bool matlab_send_error_reported = false;

        if (
            matlab_sent < 0 &&
            !matlab_send_error_reported
        )
        {
            perror("[MATLAB] UDP send");
            matlab_send_error_reported = true;
        }
    }

    static RobotStateId old=ROBOT_STATE_COUNT;static bool old_relay;
    if(old!=s->machine.activeState || old_relay!=wire_feed) {
        printf("[SIM] state=%s wire_feed=%s PV=%s transitions=%u\n",state_machine_state_name(s->machine.activeState),wire_feed?"ON":"OFF",state_path_validation_error_name(s->validation.report.error),s->machine.transitionCount);
        fflush(stdout);old=s->machine.activeState;old_relay=wire_feed;
    }
}
static bool configure_silkit_can(void)
{
    const char *registry_uri =
        getenv("SILKIT_REGISTRY_URI");

    if (
        registry_uri == NULL ||
        registry_uri[0] == '\0'
    )
    {
        registry_uri =
            "silkit://localhost:8500";
    }

    const SilKitCanBackendConfig backend_config =
    {
        .participant_name = "RobotControllerSupervisor",
        .controller_name = "RobotCAN",
        .network_name = "CAN1",
        .registry_uri = registry_uri,
        .bitrate = 1000000U
    };

    if (!silkit_can_backend_create(
            &silkit_backend,
            &backend_config))
    {
        fprintf(
            stderr,
            "Could not create SIL Kit CAN backend. Is the registry running?\n"
        );

        return false;
    }

    if (!silkit_can_backend_wait_ready(
            &silkit_backend,
            5000U))
    {
        fprintf(
            stderr,
            "SIL Kit CAN backend did not become ready within 5 s.\n"
        );

        can_backend_close(
            &silkit_backend
        );

        return false;
    }

    const CanopenMasterConfig config =
    {
        .backend = &silkit_backend,
        .node_ids = {1U, 2U, 3U, 4U, 5U, 6U},
        .node_count = SIM_NUM_AXES,
        .heartbeat_timeout_ms = 300U,
        .sdo_timeout_ms = 50U
    };

    printf(
        "[SILKIT] Controller connected: %s, CAN1 @ 1 Mbit/s\n",
        registry_uri
    );
    fflush(stdout);

    return canopen_master_init(
        &can_boot_master,
        &config
    );
}

static bool start_system(void)
{
    configure_robot();configure_state_dependencies();

    if (!configure_silkit_can())
    {
        fprintf(stderr, "Could not initialize SIL Kit CANopen transport.\n");
        return false;
    }
    execution_services=(PathExecutionServices){approach_read_validated_sample,relay,retract_prepare,retract_step,clearance,hold,&validated_storage};
    paused_services=(PausedServices){hold,off,NULL};fault_services=(FaultServices){safe,NULL};emergency_services=(EmergencyStopServices){safe,NULL};
    SupervisorTaskConfig c={0};
    c.canopen_master=&can_boot_master;c.avatar_position_scales=avatar_position_scales;c.robot=&robot;c.homing_config=&homing_config;c.teaching_config=&teaching_config;
    c.validation_config=&path_validation_config;c.validation_services=&path_validation_services;
    c.validation_workspace=&path_validation_workspace;c.validation_storage=&path_validation_storage;c.validated_trajectory=&validated_trajectory;
    c.approach_config=&approach_config;c.approach_services=&approach_services;
    c.path_execution_config=&execution_config;c.path_execution_services=&execution_services;
    c.paused_services=&paused_services;c.fault_services=&fault_services;c.emergency_stop_services=&emergency_services;
    c.validation_sample_budget=4;c.first_program_id=1;c.period_ticks=pdMS_TO_TICKS(1);
    c.io=(SupervisorIoServices){NULL,read_inputs,write_outputs};
    HmiTaskConfig h={udp_event,udp_status,NULL,pdMS_TO_TICKS(10)};
    return supervisor_task_init(&c) && hmi_task_init(&h) &&
        supervisor_task_start(4) && hmi_task_start(2) &&
        xTaskCreate(input_task,"SimInputs",1024,NULL,3,NULL)==pdPASS;
}
int main(void)
{
    udp=socket(AF_INET,SOCK_DGRAM,0);
    struct sockaddr_in addr={0};addr.sin_family=AF_INET;addr.sin_port=htons(HMI_CONTROLLER_PORT);addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(udp<0 || bind(udp,(struct sockaddr*)&addr,sizeof(addr))<0){perror("GUI command port 5010");return 1;}
    (void)configure_matlab_status_destination();
    if(fcntl(udp,F_SETFL,O_NONBLOCK)<0 || !start_system()){fprintf(stderr,"Startup failed\n");return 1;}
    puts("REAL state modules + HMI task + SIL Kit CAN1 + six external AVATAR virtual nodes.\nGUI UDP ports: commands 5010, panel 5011, teaching 5012. MATLAB status: 5005.");
    vTaskStartScheduler();return 1;
}
