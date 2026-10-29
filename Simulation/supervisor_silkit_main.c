#include "../StateMachine/supervisor_task.h"
#include "../HMI/hmi_task.h"
#include "../HMI/hmi_protocol.h"
#include "../CANComm/SILKit/silkit_can_backend.h"
#include "../CANComm/CANopen/canopen_master.h"
#include "../ServoDrive/JointDrive/canopen_joint_drive_port.h"
#include "../ServoDrive/CiA402/cia402.h"
#include "../ServoDrive/AvatarM/avatar_m_position.h"
#include "../ControlCore/Kinematics/control_fk.h"
#include "Guidance/host_guidance_demo.h"
#include "../HandGuiding/guidance_command_gate.h"
#include "Storage/trajectory_prefetch.h"
#include "Timing/host_timing_trace.h"
#include "../Intercore/validation_storage_bridge.h"
#include "Renode/Storage/qspi_nor_validated_storage.h"
#include "Tests/w25q_nor_model.h"
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
/* Non-owning drive service for the standalone retraction callback. */
static JointDrivePort retraction_drive;
static AvatarMPositionScale avatar_position_scales[SIM_NUM_AXES];
static HostGuidanceDemo host_guidance_demo;
static bool host_guidance_enabled;
static bool virtual_ft_mode;
static bool virtual_motion_settled;
static VirtualFtReceiver virtual_ft_receiver;
/* Exact, already-configured host-only gate limits copied at init for
 * post-fault diagnostics. No access to the Supervisor's live gate state. */
static HandGuidingCommandGateConfig diagnostic_gate_limits;
static const char *host_guidance_trace_path;

static HomingConfig homing_config;
static TeachingConfig teaching_config;
static PathValidationConfig path_validation_config;
static PathValidationServices path_validation_services;
static PathValidationWorkspace path_validation_workspace;
static Vec3 pv_raw_geometry[SIM_PV_GEOMETRY_CAPACITY], pv_arc_geometry[SIM_PV_GEOMETRY_CAPACITY];
static real_t pv_l_original[SIM_PV_GEOMETRY_CAPACITY], pv_l_arc[SIM_PV_GEOMETRY_CAPACITY];
static ADLSInfo pv_ik_scratch, guidance_info;
static RamValidatedStorage validated_storage;
/* Host test ONLY: independent W25Q512JV model, not STM32 QUADSPI hardware. */
static QspiNorValidatedStorage host_flash_storage;
static Model host_flash_model;
static uint8_t *host_flash_image;
static PvExecutionSample host_flash_first_sample;
static bool host_flash_mode, host_flash_first_valid;
static TaughtProgram host_flash_validation_snapshot;
static TrajectoryPrefetch validated_prefetch;
static TaskHandle_t prefetch_task_handle;
/* Two-core target ownership model: M7 numerical validation worker only
 * enqueues storage requests; M4 owns the QSPI and durable commit backend.
 * POSIX simulation models the two roles with independent FreeRTOS tasks. */
static ValidationShared flash_validation_shared;
static ValidationM7Client flash_validation_client;
static ValidationM4Worker flash_validation_worker;
static TaskHandle_t flash_storage_task_handle;
static PathValidationStorage path_validation_storage;
static ValidatedTrajectory validated_trajectory;
static ApproachConfig approach_config;
static ApproachServices approach_services;
static PathExecutionConfig execution_config = {
    .controlled_stop_timeout_ms = 1000,
    .clearance_stable_cycles = 5,
    .feedback_max_age_us = 100000 /* Host-only scheduling allowance. */
};
static PathExecutionServices execution_services;
static PausedServices paused_services;
static FaultServices fault_services;
static EmergencyStopServices emergency_services;
static HomingState retraction;
static bool wire_feed, estop, protective, injected_fault, communication_failure;
typedef struct { float xyz[3]; } SimGuidancePose;
static QueueHandle_t guidance_pose_queue;
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
    SupervisorDriveSnapshot snapshot;
    return !communication_failure &&
        supervisor_task_get_drive_snapshot(&snapshot) &&
        snapshot.valid && snapshot.heartbeats_operational;
}

static bool sim_can_drives_ready(void)
{
    SupervisorDriveSnapshot snapshot;
    return !communication_failure &&
        supervisor_task_get_drive_snapshot(&snapshot) &&
        snapshot.valid && snapshot.drives_ready;
}

/* Non-owning tasks receive one consistent copy instead of dereferencing
 * CanopenMaster while Supervisor is polling and updating its drives. */
static bool sim_read_joint_vector(double q[SIM_NUM_AXES])
{
    SupervisorDriveSnapshot snapshot;
    if (q == NULL || !supervisor_task_get_drive_snapshot(&snapshot) ||
        !snapshot.valid) return false;
    for (size_t axis = 0U; axis < SIM_NUM_AXES; ++axis) {
        if (!snapshot.feedback_valid[axis] ||
            !avatar_m_position_units_to_joint_rad(
                &avatar_position_scales[axis],
                snapshot.actual_position_units[axis], &q[axis])) return false;
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
    SupervisorMessage request = {0};
    request.type = SUPERVISOR_MESSAGE_GUIDED_TARGET;
    memcpy(request.data.guided_target_units, targets, sizeof(targets));
    /* Queue acceptance is not permission to move: Supervisor gates it. */
    return supervisor_task_post(&request, 0U);
}

/* Simulated RAM source: physical STM32 firmware will substitute the W25Q
 * bulk read API. Only the prefill/worker invokes this function. */
static bool ram_prefetch_read_batch(uint32_t first, PvExecutionSample *out,
                                    uint32_t count, void *context)
{
    RamValidatedStorage *s = (RamValidatedStorage *)context;
    if (!s || !out || !s->committed || first > s->metadata.sample_count ||
        first > s->sample_count ||
        count > s->metadata.sample_count - first ||
        count > s->sample_count - first) return false;
    memcpy(out, &s->samples[first], (size_t)count * sizeof(*out));
    return true;
}

/* Validation entry is retried by the Supervisor every millisecond.
 * Worker acknowledges STOPPED after any currently in-flight flash read.
 * No sleep or wait on the control task. */
static bool prefetch_ready_for_validation(void *ignored)
{
    (void)ignored;
    /* Do not start a new validation while M4 still has a pending commit,
     * failed transaction, or in-flight cancellation to acknowledge. */
    if (host_flash_mode && !validation_m4_idle(&flash_validation_shared))
        return false;
    if (trajectory_prefetch_stopped(&validated_prefetch)) return true;
    if (!trajectory_prefetch_request_stop(&validated_prefetch)) return false;
    return trajectory_prefetch_stopped(&validated_prefetch);
}

/* Flash/backend callbacks are called ONLY from this M4-role task.
 * Never run this function from Supervisor's 1 kHz control loop. */
static void flash_storage_worker_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* Avoid flooding the trace with idle flash-worker polls. */
        const uint32_t head = __atomic_load_n(
            &flash_validation_shared.head, __ATOMIC_ACQUIRE);
        const uint32_t tail = __atomic_load_n(
            &flash_validation_shared.tail, __ATOMIC_ACQUIRE);
        const bool active = head != tail ||
            __atomic_load_n(&flash_validation_shared.cancel_request, __ATOMIC_ACQUIRE) ||
            __atomic_load_n(&flash_validation_shared.release_request, __ATOMIC_ACQUIRE);
        if (active) timing_begin(TIMING_FLASH);
        const bool result = validation_m4_step(&flash_validation_worker);
        if (active) timing_end(TIMING_FLASH, result);
        vTaskDelay(1);
    }
}

static void prefetch_worker_task(void *arg)
{
    (void)arg;
    for (;;) {
        const bool active = !trajectory_prefetch_stopped(&validated_prefetch);
        if (active) timing_begin(TIMING_PREFETCH);
        const bool result = trajectory_prefetch_worker_step(&validated_prefetch);
        if (active) timing_end(TIMING_PREFETCH, result);
        vTaskDelay(1);
    }
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

    /* Supervisor already waited nonblockingly for STOPPED at entry. */
    if (!trajectory_prefetch_stopped(&validated_prefetch)) return false;
    trajectory_prefetch_disarm(&validated_prefetch);

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

    /* This initial prefill is from PC RAM during VALIDATION COMMIT, never
     * from QSPI and never during active motion. Worker remains suspended. */
    if (!trajectory_prefetch_prepare(&validated_prefetch,
            ram_prefetch_read_batch, storage, metadata->sample_count) ||
        !trajectory_prefetch_refill(&validated_prefetch) ||
        !trajectory_prefetch_refill(&validated_prefetch)) {
        storage->committed = false;
        trajectory_prefetch_disarm(&validated_prefetch);
        return false;
    }
    return trajectory_prefetch_resume_worker(&validated_prefetch);
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
    /* Abort occurs during validation, when the entry gate holds the
     * prefetch worker STOPPED. Fail closed on any violated ownership rule;
     * never block the Supervisor or race an in-flight read. */
    if (!trajectory_prefetch_stopped(&validated_prefetch)) {
        fprintf(stderr, "[PREFETCH] Unexpected active worker on abort\n");
        exit(EXIT_FAILURE); /* PC simulation only. */
    }
    trajectory_prefetch_disarm(&validated_prefetch);

    storage->writing =
        false;

    storage->committed =
        false;

    storage->sample_count =
        0U;
}



/* Batch 6 host functional integration: path validation uses the same storage
 * APIs as the STM32 side. This synchronous write path is NOT a deployable
 * RTOS scheduling design. Move flash writes away from Supervisor on hardware. */
static bool host_flash_read_batch(uint32_t first, PvExecutionSample *out,
                                  uint32_t count, void *ctx)
{
    return qspi_nor_validated_storage_read_samples(first, out, count, ctx);
}

static bool host_flash_begin(void *ctx)
{
    if (!trajectory_prefetch_stopped(&validated_prefetch)) return false;
    trajectory_prefetch_disarm(&validated_prefetch);
    host_flash_first_valid=false;
    return qspi_nor_validated_storage_begin(ctx);
}

static bool host_flash_commit(const ValidatedTrajectory *metadata, void *ctx)
{
    QspiNorValidatedStorage *storage=(QspiNorValidatedStorage *)ctx;
    if (!qspi_nor_validated_storage_commit(metadata,storage)) return false;

    /* Reconstruct runtime state from persistent image, independently of
     * in-memory commit bookkeeping. The image survives the object reset. */
    QspiNorValidatedStorage recovered;
    if (!qspi_nor_validated_storage_init(&recovered,storage->base_address,
                                         storage->capacity_samples) ||
        !qspi_nor_validated_storage_load_committed(&recovered) ||
        recovered.scan_io_error_mask != 0U ||
        recovered.metadata.program_id != metadata->program_id ||
        recovered.metadata.artifact_crc != metadata->artifact_crc ||
        recovered.sample_count != metadata->sample_count)
        return false;
    *storage=recovered;
    /* APPROACH currently only reads sample zero, from the Supervisor task.
     * Cache it here; never perform direct flash I/O inside APPROACH. */
    if (!qspi_nor_validated_storage_read_sample(0,&host_flash_first_sample,storage))
        return false;
    host_flash_first_valid=true;

    if (!trajectory_prefetch_prepare(&validated_prefetch,host_flash_read_batch,
                                      storage,metadata->sample_count) ||
        !trajectory_prefetch_refill(&validated_prefetch) ||
        !trajectory_prefetch_refill(&validated_prefetch) ||
        !trajectory_prefetch_resume_worker(&validated_prefetch))
        return false;
    printf("[FLASH_SIM] committed+recovered program=%u samples=%u prefilled=%u\n",
           metadata->program_id,metadata->sample_count,
           trajectory_prefetch_buffered(&validated_prefetch));
    fflush(stdout);
    return true;
}

static void host_flash_abort(void *ctx)
{
    if (!trajectory_prefetch_stopped(&validated_prefetch)) {
        fprintf(stderr,"[FLASH_SIM] Unexpected active worker on abort\n");
        exit(EXIT_FAILURE); /* PC simulation: fail closed, never wait. */
    }
    trajectory_prefetch_disarm(&validated_prefetch);
    host_flash_first_valid=false;
    qspi_nor_validated_storage_abort(ctx);
}

/* Host-only fault injection; the FlashM4 task owns all flash callbacks.
 * No STM32 firmware or CANopen configuration uses these switches. */
static int host_flash_test_mode;
static bool host_flash_test_fired;
static bool host_flash_test_command(void *ctx, uint8_t op, uint32_t addr,
                                    const uint8_t *tx, size_t nt,
                                    uint8_t *rx, size_t nr)
{
    if (!host_flash_test_fired && host_flash_test_mode == 1 &&
        op == W25_CMD_ERASE4K4) {
        host_flash_test_fired = true;
        puts("[FLASH_TEST] delayed erase entered");
        fflush(stdout);
        /* Simulator-only artificial wait AFTER validation worker ownership;
         * controller must continue publishing status while erase is blocked. */
        vTaskDelay(pdMS_TO_TICKS(700));
    }
    if (!host_flash_test_fired && host_flash_test_mode == 2 &&
        op == W25_CMD_PROG4) {
        host_flash_test_fired = true;
        puts("[FLASH_TEST] injected page-program transport failure");
        fflush(stdout);
        return false;
    }
    return w25_model_command(ctx, op, addr, tx, nt, rx, nr);
}

static bool host_flash_initialize(void)
{
    /* Tests only: one-shot error/delay. Normal runs retain strict NOR model. */
    const char *test_mode = getenv("SIM_FLASH_TEST_MODE");
    host_flash_test_mode = test_mode != NULL && strcmp(test_mode,"cancel")==0
        ? 1 : (test_mode != NULL && strcmp(test_mode,"program_fail")==0 ? 2 : 0);
    host_flash_test_fired = false;
    host_flash_image=malloc(W25Q512JV_SIZE_BYTES);
    if (!host_flash_image) return false;
    w25_model_init(&host_flash_model,host_flash_image,W25Q512JV_SIZE_BYTES);
    host_flash_model.suppress_event_trace=true;
    W25Q512JVBus bus={host_flash_test_command,w25_model_time,w25_model_idle,
                      &host_flash_model};
    if (!qspi_nor_validated_storage_set_bus(&bus) ||
        !qspi_nor_validated_storage_init(&host_flash_storage,0U,
                                         SIM_PV_STORAGE_CAPACITY)) return false;
    puts("[FLASH_SIM] W25Q512JV host-model backend enabled (NOT STM32 hardware)");
    fflush(stdout);
    return true;
}

static bool approach_read_validated_sample(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
)
{
    if (host_flash_mode) {
        (void)context;
        if (sample == NULL || sample_index != 0U || !host_flash_first_valid)
            return false;
        *sample=host_flash_first_sample;
        return true;
    }
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


/* This callback executes inside PATH_EXECUTION. It accesses SRAM only;
 * all source reads are confined to prefill/worker code above. */
static bool execution_read_prefetched(uint32_t sample_index,
                                     PvExecutionSample *sample, void *ctx)
{
    (void)ctx;
    return trajectory_prefetch_take(&validated_prefetch, sample_index, sample);
}

static bool execution_skip_prefetched(uint32_t first, uint32_t count, void *ctx)
{
    (void)ctx;
    return trajectory_prefetch_skip(&validated_prefetch, first, count);
}

static bool execution_stream_ready(uint32_t next_index, uint32_t count, void *ctx)
{
    (void)ctx;
    return trajectory_prefetch_begin_execution(&validated_prefetch, next_index, count);
}

static void execution_end_stream(void *ctx)
{
    (void)ctx;
    /* Called in the Supervisor task after it stops dispatching execution,
     * including external transitions from execution/PAUSED into FAULT. */
    trajectory_prefetch_end_execution(&validated_prefetch);
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
        virtual_ft_mode ? 0.15 : DEG2RAD(0.0);

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
            .default_speed_mps = TEACHING_SPEED_PRESET_MEDIUM_MPS,
            .minimum_speed_mps = 0.001F,
            .maximum_speed_mps = 0.100F,
            .speed_low_mps = TEACHING_SPEED_PRESET_LOW_MPS,
            .speed_medium_mps = TEACHING_SPEED_PRESET_MEDIUM_MPS,
            .speed_high_mps = TEACHING_SPEED_PRESET_HIGH_MPS,
            .duplicate_position_tolerance_m = 0.002F,
            .duplicate_orientation_tolerance_rad = 0.010F,
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
        &retraction_drive,
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
        timing_begin(TIMING_SIM_INPUTS);
        /* Acquisition must not wait behind HMI printf/CSV/file flushing. */
        if (virtual_ft_mode) virtual_ft_receiver_poll(&virtual_ft_receiver);
        StateMachine machine;
        supervisor_task_get_state(&machine);
        bool settled = false;
        if (virtual_ft_mode) {
            HostGuidanceMotionStatus motion;
            settled = machine.activeState == ROBOT_STATE_TEACHING &&
                host_guidance_motion_status(&host_guidance_demo, &motion) &&
                motion.settled;
            taskENTER_CRITICAL();
            virtual_motion_settled = settled;
            taskEXIT_CRITICAL();
        }
        if(machine.activeState==ROBOT_STATE_IDLE && machine.previousState==ROBOT_STATE_HOMING)homed=true;
        if(estop||protective||injected_fault||communication_failure) {homed=false;safe(NULL);}
        const bool drives_ready=sim_can_drives_ready();
        const bool communication_healthy=sim_can_communication_healthy();
        const bool motion_permitted=
            drives_ready&&communication_healthy&&!estop&&!protective&&!injected_fault;

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
        /* Safety/fault messages must precede any guidance command in queue. */
        SimGuidancePose guidance_pose;
        const bool guidance_pending = guidance_pose_queue &&
            xQueueReceive(guidance_pose_queue, &guidance_pose, 0U) == pdPASS;
        if(guidance_pending) {guidance_error=2;
            if(machine.activeState==ROBOT_STATE_TEACHING && !estop && !protective && drives_ready) {
                double q[SIM_NUM_AXES],target[4][4],solution[SIM_NUM_AXES];
                if(sim_read_joint_vector(q)) {
                    control_fk(&robot,q,target);
                    for(int j=0;j<3;j++)target[j][3]=guidance_pose.xyz[j];
                    ADLSParameters params;adls_default_parameters(&params);
                    if(adls_ik(&robot,target,q,&params,solution,&guidance_info) &&
                       sim_apply_guided_joint_vector(solution)) {
                        guidance_error=0;
                    } else {
                        guidance_error=1;
                        fprintf(stderr,"[GUIDANCE] IK/apply failed: target=[%.6f %.6f %.6f] m iterations=%d pos_error=%.6g m ori_error=%.6g rad\n", guidance_pose.xyz[0],guidance_pose.xyz[1],guidance_pose.xyz[2],guidance_info.iterations,guidance_info.positionError,guidance_info.orientationError);
                    }
                }
            }
        }
        m.type=SUPERVISOR_MESSAGE_TEACHING_RUNTIME;
        memset(&m.data,0,sizeof(m.data));
        m.data.teaching_runtime.timestamp_ms=(uint32_t)(xTaskGetTickCount()*1000/configTICK_RATE_HZ);
        m.data.teaching_runtime.calibration_version=1;
        m.data.teaching_runtime.robot_homed=homed;
        m.data.teaching_runtime.manual_guidance_active=machine.activeState==ROBOT_STATE_TEACHING && !guidance_pending;
        m.data.teaching_runtime.robot_motion_settled=
            virtual_ft_mode ? settled : !guidance_pending;
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
        timing_end(TIMING_SIM_INPUTS, true);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#include "Robot/operator_transport.inc"

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
    if (!trajectory_prefetch_init(&validated_prefetch)) return false;
    /* One-slot FreeRTOS mailbox: GUI overwrites stale Cartesian requests. */
    guidance_pose_queue = xQueueCreate(1U, sizeof(SimGuidancePose));
    if (guidance_pose_queue == NULL) return false;
    const char *selected=getenv("SIM_STORAGE_BACKEND");
    host_flash_mode=selected != NULL && strcmp(selected,"flash")==0;
    if (host_flash_mode && !host_flash_initialize()) return false;
    /* Strict host opt-in only. Never wired into production STM32 firmware. */
    const char *guidance_opt = getenv("HAND_GUIDING_DRY_RUN");
    host_guidance_enabled = guidance_opt && strcmp(guidance_opt, "1") == 0;
    const char *dispatch_opt = getenv("HAND_GUIDING_SIM_MOTION");
    const bool sim_guidance_dispatch =
        dispatch_opt && strcmp(dispatch_opt, "1") == 0;
    const char *virtual_opt = getenv("HAND_GUIDING_VIRTUAL_FT");
    virtual_ft_mode = virtual_opt && strcmp(virtual_opt, "1") == 0;
    if (virtual_ft_mode && (!host_guidance_enabled ||
                            !sim_guidance_dispatch)) {
        fprintf(stderr, "[VFT] requires HAND_GUIDING_DRY_RUN=1 and "
                        "HAND_GUIDING_SIM_MOTION=1\\n");
        return false;
    }
    if (sim_guidance_dispatch && !host_guidance_enabled) {
        fprintf(stderr, "[4E3] HAND_GUIDING_SIM_MOTION requires "
                        "HAND_GUIDING_DRY_RUN=1\\n");
        return false;
    }
    /* Interactive home is nonsingular in the unmodified simulation model.
     * Teaching FK, guidance and MATLAB all see the actual J5 angle. */
    configure_robot();configure_state_dependencies();
    host_guidance_trace_path = getenv("HAND_GUIDING_TRACE_FILE");
    if (host_guidance_enabled && (!host_guidance_trace_path ||
                                  !*host_guidance_trace_path))
        host_guidance_trace_path = "guidance_dry_run.csv";
    if (host_guidance_enabled) {
        const char *fault = getenv("HAND_GUIDING_INJECT");
        HostGuidanceFaultKind kind = HOST_GUIDANCE_INJECT_NONE;
        if (fault && strcmp(fault, "stale_ft") == 0)
            kind = HOST_GUIDANCE_INJECT_STALE_FT;
        else if (fault && strcmp(fault, "bad_feedback") == 0)
            kind = HOST_GUIDANCE_INJECT_BAD_FEEDBACK;
        else if (fault && *fault && strcmp(fault, "none") != 0) {
            fprintf(stderr, "[4E2B] Invalid HAND_GUIDING_INJECT value\n");
            return false;
        }
        if (!host_guidance_init(&host_guidance_demo, &robot, kind, 20U))
            return false;
        if (virtual_ft_mode) {
            const VirtualFtLimits limits = {5.0, 1.0,
                                            VIRTUAL_FT_DEFAULT_TIMEOUT_US};
            if (!virtual_ft_receiver_init(&virtual_ft_receiver, &limits))
                return false;
            host_guidance_use_virtual_ft(&host_guidance_demo,
                                           &virtual_ft_receiver);
        }
        fprintf(stderr, "[4E2B] guidance candidate producer enabled; "
                        "fault=%s, trace=%s\n",
                fault ? fault : "none", host_guidance_trace_path);
    }
    if (host_flash_mode) {
        /* M4-private flash callbacks. The M7 validator sees ONLY the
         * nonblocking queue proxy; it cannot erase/write/QSPI directly. */
        const PathValidationStorage m4_flash_backend = {
            .begin=host_flash_begin,
            .write_sample=qspi_nor_validated_storage_write_sample,
            .commit=host_flash_commit,
            .abort=host_flash_abort,
            .capacity_samples=SIM_PV_STORAGE_CAPACITY,
            .context=&host_flash_storage
        };
        validation_shared_initialize(&flash_validation_shared);
        validation_m7_init(&flash_validation_client,
                              &flash_validation_shared);
        if (!validation_m4_init(&flash_validation_worker,
                                   &flash_validation_shared,
                                   &m4_flash_backend)) {
            fprintf(stderr, "[STARTUP] M4 flash storage bridge init failed\n");
            return false;
        }
        path_validation_storage =
            validation_m7_storage(&flash_validation_client,
                                     SIM_PV_STORAGE_CAPACITY);
        approach_services.storage_context=&host_flash_storage;
    }

    if (!configure_silkit_can())
    {
        fprintf(stderr, "Could not initialize SIL Kit CANopen transport.\n");
        return false;
    }
    /* Reuse the same CANopen coordinator as Supervisor; never reinterpret
     * a CanopenMaster pointer as JointDrivePort during retraction. */
    retraction_drive = canopen_joint_drive_port_make(&can_boot_master);
    execution_services=(PathExecutionServices){
        .read_sample=execution_read_prefetched, .set_wire_feed_enabled=relay,
        .prepare_retraction=retract_prepare, .step_retraction=retract_step,
        .clearance_verified=clearance, .controlled_stop=hold,
        .context=&validated_storage, .skip_samples=execution_skip_prefetched,
        .stream_ready=execution_stream_ready,
        .end_stream=execution_end_stream
    };
    paused_services=(PausedServices){hold,off,NULL};fault_services=(FaultServices){safe,NULL};emergency_services=(EmergencyStopServices){safe,NULL};
    SupervisorTaskConfig c={0};
    c.canopen_master=&can_boot_master;c.avatar_position_scales=avatar_position_scales;c.robot=&robot;c.homing_config=&homing_config;c.teaching_config=&teaching_config;
    c.validation_config=&path_validation_config;c.validation_services=&path_validation_services;
    c.validation_workspace=&path_validation_workspace;c.validation_storage=&path_validation_storage;c.validated_trajectory=&validated_trajectory;
    approach_services.controlled_stop=hold;
    approach_services.controlled_stop_timeout_ms=1000U;
    c.approach_config=&approach_config;c.approach_services=&approach_services;
    c.path_execution_config=&execution_config;c.path_execution_services=&execution_services;
    c.paused_services=&paused_services;c.fault_services=&fault_services;c.emergency_stop_services=&emergency_services;
    c.validation_sample_budget=4;c.first_program_id=1;
    c.period_ticks=pdMS_TO_TICKS(host_guidance_enabled ? 2 : 1);
    if (host_guidance_enabled)
        c.guidance_dry_run = host_guidance_port(&host_guidance_demo);
    if (sim_guidance_dispatch) {
        /* Host-only synthetic validation limits; NOT motion approval for
         * the actual custom robot. Used solely with SIL Kit virtual motors. */
        c.sim_guidance_dispatch_enabled = true;
        c.sim_guidance_command_limits.maximum_candidate_age_us = 1000U;
        for (size_t j = 0U; j < SIM_NUM_AXES; ++j) {
            c.sim_guidance_command_limits.max_step_rad[j] = 0.008;
            c.sim_guidance_command_limits.max_target_feedback_error_rad[j] = 0.1;
            c.sim_guidance_command_limits.max_quantization_error_rad[j] = 0.00001;
        }
        diagnostic_gate_limits = c.sim_guidance_command_limits;
        fprintf(stderr, "[4E3] SIMULATED MOTOR DISPATCH ENABLED, "
                        "host-only synthetic parameters; never hardware\\n");
    }
    c.validation_enter_ready=prefetch_ready_for_validation;
    /* Flash erase/program/CRC reads run only in low-priority task.
     * No persistent artifact is published until the worker commits it. */
    c.validation_run_in_worker=host_flash_mode;
    c.validation_worker_priority=1U;
    c.validation_worker_program=&host_flash_validation_snapshot;
    c.io=(SupervisorIoServices){NULL,read_inputs,write_outputs};
    HmiTaskConfig h={udp_event,udp_status,NULL,pdMS_TO_TICKS(10)};
    /* Give a precise startup failure instead of hiding which task
     * allocation failed behind a chained boolean expression. */
    if (!supervisor_task_init(&c)) {
        fprintf(stderr, "[STARTUP] supervisor_task_init failed\n");
        return false;
    }
    if (!hmi_task_init(&h)) {
        fprintf(stderr, "[STARTUP] hmi_task_init failed\n");
        return false;
    }
    if (!supervisor_task_start(4)) {
        fprintf(stderr, "[STARTUP] supervisor_task_start failed; "
                "FreeRTOS heap remaining=%zu bytes\n",
                (size_t)xPortGetFreeHeapSize());
        return false;
    }
    if (!hmi_task_start(2)) {
        fprintf(stderr, "[STARTUP] hmi_task_start failed; "
                "heap remaining=%zu bytes\n",
                (size_t)xPortGetFreeHeapSize());
        return false;
    }
    if (xTaskCreate(input_task,"SimInputs",1024,NULL,3,NULL)!=pdPASS) {
        fprintf(stderr, "[STARTUP] SimInputs allocation failed; "
                "heap remaining=%zu bytes\n",
                (size_t)xPortGetFreeHeapSize());
        return false;
    }
    if (host_flash_mode &&
        xTaskCreate(flash_storage_worker_task,"FlashM4",2048,NULL,1,
                    &flash_storage_task_handle)!=pdPASS) {
        fprintf(stderr, "[STARTUP] M4 flash worker allocation failed; "
                "heap remaining=%zu bytes\n", (size_t)xPortGetFreeHeapSize());
        return false;
    }
    if (xTaskCreate(prefetch_worker_task,"TrajPrefetch",1024,NULL,2,
                    &prefetch_task_handle)!=pdPASS) {
        fprintf(stderr, "[STARTUP] TrajPrefetch allocation failed; "
                "heap remaining=%zu bytes\n",
                (size_t)xPortGetFreeHeapSize());
        return false;
    }
    return true;
}
int main(void)
{
    udp=socket(AF_INET,SOCK_DGRAM,0);
    struct sockaddr_in addr={0};addr.sin_family=AF_INET;addr.sin_port=htons(HMI_CONTROLLER_PORT);addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(udp<0 || bind(udp,(struct sockaddr*)&addr,sizeof(addr))<0){perror("GUI command port 5010");return 1;}
    (void)configure_matlab_status_destination();
    if (!timing_init(getenv("HOST_TIMING_CSV"))) {
        fprintf(stderr, "[TIMING] CSV initialization failed\n");
        return 1;
    }
    if(fcntl(udp,F_SETFL,O_NONBLOCK)<0 || !start_system()){fprintf(stderr,"Startup failed\n");return 1;}
    puts("REAL state modules + HMI task + SIL Kit CAN1 + six external AVATAR virtual nodes.\nGUI UDP ports: commands 5010, panel 5011, teaching 5012. MATLAB status: 5005.");
    vTaskStartScheduler();return 1;
}
