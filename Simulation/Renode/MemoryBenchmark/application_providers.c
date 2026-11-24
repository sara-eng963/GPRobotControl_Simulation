/* Hardware boundary providers ONLY. Actual control, CANopen, F/T, validation,
 * NOR driver and playback routines are linked unchanged into this ARM image.
 * Both cores' service tasks share one Cortex-M7 kernel for this experiment. */
#include "application.h"
#include "../../../ServoDrive/AvatarM/Sim/avatar_m_node_sim.h"
#include "../../../ServoDrive/AvatarM/avatar_m_pdo.h"
#include "../../../Simulation/Tests/w25q_nor_model.h"
#include "../../../Simulation/Renode/Storage/qspi_nor_validated_storage.h"
#include "../../../HandGuiding/Tests/p05_mock_ft_protocol.h"
#include <string.h>
#include <math.h>

volatile uint32_t membench_application[MB_WORDS] = {[MB_MAGIC]=0x4150504D, [MB_VERSION]=1};
SupervisorTaskConfig mb_config;
ValidatedTrajectory mb_artifact;
HandGuidingDryRunState mb_guidance;
volatile bool mb_operator_pressed, mb_ft_stopped, mb_estop;
static RobotConfig robot;
static AvatarMPositionScale scales[6];
static CanopenMaster master;
static CanBackend backend;
static AvatarMNodeSim nodes[6];
static CanFrame rx[64];
static uint32_t rx_head, rx_tail, previous_tick, safety_sequence, producer_timestamp;
static FtSerialPort *serial;
static uint32_t ft_sequence;
static Model nor;
static QspiNorValidatedStorage storage;
static PathValidationStorage physical_storage;
static PvExecutionSample first_sample;
static bool cached;
#define STORAGE_SAMPLES 10000U
#define SLOT_SIZE ((((4096U+STORAGE_SAMPLES*24U)+4095U)/4096U)*4096U)
/* External device bytes live in Renode's external memory, NOT MCU .bss. */
#define NOR_BYTES ((uint8_t *)(uintptr_t)0x90000000)
static HomingConfig homing;
static TeachingConfig teaching;
static PathValidationConfig validation;
static PathValidationWorkspace workspace;
static Vec3 raw[512], arc[512];
static real_t lengths[512], arc_lengths[512];
static ADLSInfo ik_scratch;
static TaughtProgram worker_program;
static ValidationM7Client client;
static PathValidationStorage proxy;
static TrajectoryExecutionBinding binding;
static ApproachConfig approach;
static ApproachServices approach_services;
static PathExecutionConfig execution;
static PathExecutionServices physical_execution;
static PausedServices paused;
static FaultServices faults;
static EmergencyStopServices emergency;

void mb_fail(uint32_t line)
{
    membench_application[MB_FAILURE_LINE]=line;
    membench_application[MB_FAILURE_CALLER]=(uint32_t)__builtin_return_address(0);
    membench_application[MB_STATUS]=2;
    taskDISABLE_INTERRUPTS();
    for (;;) { __asm volatile("nop"); }
}

static void enqueue(const CanFrame *f)
{
    /* Like a finite hardware FIFO, discard arriving frames when fault service
     * has stopped normal CAN polling. Overflow during normal work still fails. */
    if (rx_tail-rx_head==64 && membench_application[MB_PHASE]==MB_FAULTS) return;
    MB_ASSERT(rx_tail-rx_head < 64);
    rx[rx_tail++ % 64]=*f;
}
static void feedback(unsigned i)
{
    CanFrame frame;
    MB_ASSERT(avatar_m_build_tpdo4((uint8_t)(i+1), nodes[i].actual_position, nodes[i].statusword, &frame));
    enqueue(&frame);
}
static CanBackendResult send_frame(void *unused, const CanFrame *f)
{
    (void)unused;
    if (f->id >= 0x501 && f->id <= 0x506) ++membench_application[MB_CAN_TARGETS];
    if (f->id == 0x80) ++membench_application[MB_CAN_SYNCS];
    for (unsigned i=0;i<6;i++) {
        CanFrame reply;
        const AvatarMNodeResult result=avatar_m_node_process_frame(&nodes[i],f,&reply);
        MB_ASSERT(result != AVATAR_M_NODE_INVALID_FRAME);
        if (result == AVATAR_M_NODE_RESPONSE) enqueue(&reply);
        if (f->id == 0 && (f->data[1] == 0 || f->data[1] == i+1)) {
            CanFrame heartbeat={.id=0x701+i, .dlc=1};
            heartbeat.data[0]=f->data[0]==1 ? 5 : (f->data[0]==0x82 || f->data[0]==0x81 ? 0 : 0x7F);
            enqueue(&heartbeat);
        }
        if (f->id==0x80 && nodes[i].active_target_valid) {
            /* Explicit deterministic perfect-following device, not mechanics. */
            avatar_m_node_set_actual_position(&nodes[i],nodes[i].active_target_position);
            feedback(i);
        }
    }
    return CAN_BACKEND_OK;
}
static CanBackendResult receive_frame(void *unused, CanFrame *f)
{
    (void)unused;
    if (rx_head==rx_tail) return CAN_BACKEND_WOULD_BLOCK;
    *f=rx[rx_head++ % 64];
    return CAN_BACKEND_OK;
}
static void close_bus(void *unused) { (void)unused; }
static uint32_t now_us(void *unused) { (void)unused; return xTaskGetTickCount()*1000U; }
static void board_post(const SupervisorMessage *message)
{
    const bool ok=supervisor_task_post(message,0);
    MB_ASSERT(ok || membench_application[MB_MAX_SUPERVISOR_QUEUE]==32);
}
static bool inputs(void *unused, SupervisorInputSnapshot *out)
{
    (void)unused;
    const uint32_t now=xTaskGetTickCount();
    producer_timestamp=now;
    *out=(SupervisorInputSnapshot){.timestamp_ms=now, .valid=true};
    out->asserted[SUP_IO_ESTOP]=mb_estop;
    ++membench_application[MB_SUPERVISOR_CYCLES];
    for (unsigned i=0;i<6;i++) {
        CanFrame heartbeat;
        if (avatar_m_node_tick_ms(&nodes[i],now-previous_tick,&heartbeat)) enqueue(&heartbeat);
        feedback(i);
    }
    previous_tick=now;
    /* Explicit synthetic safety producer; real production verifier stays
     * mandatory and all sequencing/freshness checks still execute. */
    SupervisorMessage message={.type=SUPERVISOR_MESSAGE_SAFETY};
    message.data.safety=(RobotSafetySnapshot){.statusValid=true,.motionPermitted=!mb_estop,
        .drivesReady=true,.communicationHealthy=true,.estopActive=mb_estop,.timestampMs=now};
    board_post(&message);
    message.type=SUPERVISOR_MESSAGE_TEACHING_RUNTIME;
    message.data.teaching_runtime=(TeachingRuntimeInputs){.timestamp_ms=now,
        .calibration_version=1,.active_frame_id=1,.active_tool_id=1,
        .robot_motion_settled=true,.manual_guidance_active=true,
        .motion_permitted=!mb_estop,.robot_homed=true};
    board_post(&message);
    message.type=SUPERVISOR_MESSAGE_APPROACH_CONTROL;
    message.data.approach_control=(ApproachControlInputs){.motion_permission=!mb_estop};
    board_post(&message);
    return true;
}
static bool outputs(void *unused, const SupervisorOutputSnapshot *out)
{ (void)unused; (void)out; return true; }
static bool verify(void *unused, const RobotSafetySnapshot *sample, uint32_t *session, uint32_t *sequence)
{
    (void)unused;
    *session=77; *sequence=++safety_sequence;
    return sample->timestampMs==producer_timestamp;
}
static bool issue(void *unused, uint32_t *session) { (void)unused; *session=745; return true; }
static bool board(void *unused, DriveRestartBoardSample *out)
{
    (void)unused;
    *out=(DriveRestartBoardSample){.valid=true,.operator_pressed=mb_operator_pressed,
        .board_ready=true,.reset_reviewed=true,.ipc_session_fresh=true,
        .old_commands_discarded=true,.feedback_revalidated=true};
    return true;
}
static bool stop(bool *stopped, void *unused) { (void)unused; *stopped=true; return true; }
static bool safe(void *unused) { (void)unused; ++membench_application[MB_SAFE_OUTPUT_CALLS]; return true; }
static bool wire(bool enable, void *unused) { (void)unused; (void)enable; return true; }
static bool prepare(void *unused) { (void)unused; return true; }
static StateStepResult retract(void *unused) { (void)unused; return STATE_STEP_COMPLETE; }
static bool read_first(uint32_t index, PvExecutionSample *out, void *unused)
{ (void)unused; if (!cached || index) return false; *out=first_sample; return true; }
static bool collision(const JointVector *q, void *unused) { (void)q; (void)unused; return true; }
static bool validation_idle(void *unused)
{ (void)unused; return validation_m4_idle((const ValidationShared *)(uintptr_t)SRAM4_VALIDATION_BASE); }

static bool storage_begin(void *unused)
{ (void)unused; cached=false; return physical_storage.begin(physical_storage.context); }
static bool storage_write(uint32_t index,const PvExecutionSample *sample,void *unused)
{
    (void)unused;
    const bool ok=physical_storage.write_sample(index,sample,physical_storage.context);
    if (ok) ++membench_application[MB_STORAGE_WRITES];
    return ok;
}
static bool storage_commit(const ValidatedTrajectory *meta,void *unused)
{
    (void)unused;
    const bool ok=physical_storage.commit(meta,physical_storage.context);
    if (ok) ++membench_application[MB_STORAGE_COMMITS];
    return ok;
}
static void storage_abort(void *unused) { (void)unused; physical_storage.abort(physical_storage.context); }
bool board_m4_supply_validation_backend(PathValidationStorage *out)
{
    *out=(PathValidationStorage){.begin=storage_begin,.write_sample=storage_write,
        .commit=storage_commit,.abort=storage_abort,.capacity_samples=STORAGE_SAMPLES};
    return true;
}
static bool metadata(ValidatedTrajectory *out,void *unused)
{
    (void)unused;
    if (!storage.committed || !storage.readback_verified) return false;
    *out=storage.metadata;
    return true;
}
static bool read_samples(uint32_t first,TrajectorySample *out,uint32_t count,void *unused)
{
    (void)unused;
#if defined(MEMBENCH_INJECT_READ_FAILURE)
    (void)first; (void)out; (void)count;
    membench_application[MB_STORAGE_READ_FAULT_INJECTED]=1;
    return false; /* negative image only: real reader failure path */
#endif
    const bool ok=qspi_nor_validated_storage_read_samples(first,(PvExecutionSample *)out,count,&storage);
    if (ok) membench_application[MB_STORAGE_READS]+=count;
    return ok;
}
bool board_m4_supply_trajectory_reader(CommittedTrajectoryReader *out)
{
    *out=(CommittedTrajectoryReader){.read_metadata=metadata,.read_samples=read_samples,.capacity_samples=STORAGE_SAMPLES};
    return true;
}
bool mb_cache_first_sample(void)
{
    cached=qspi_nor_validated_storage_read_sample(0,&first_sample,&storage);
    return cached;
}
bool board_supply_ft_serial_config(FtBoardSerialConfig *out)
{
    *out=(FtBoardSerialConfig){.decoder=mock_protocol(),.monotonic_us=now_us,
        .link_timeout_us=5000,.fragment_timeout_us=1500,.max_sample_age_us=3000};
    return true;
}
bool __wrap_ft_uart2_dma_start(FtSerialPort *port,FtUart2NowUs clock,void *context)
{ (void)clock; (void)context; serial=port; return true; }
bool __wrap_ft_uart2_dma_service(void)
{
    if (mb_ft_stopped) return false;
    uint8_t frame[PACKET_LEN];
    packet(frame,++ft_sequence,1500);
    if (!ft_serial_isr_receive(serial,frame,sizeof(frame),now_us(NULL))) return false;
    (void)ft_serial_poll(serial,now_us(NULL),128);
    membench_application[MB_FT_FRAMES]=serial->diagnostics.frames_accepted;
    return true;
}
void __wrap_ft_uart2_dma_stop(void) { mb_ft_stopped=true; }
void mb_ft_capacity(void)
{
    uint8_t bytes[64]={0};
    /* Fill the actual FT_RX ring at the ISR/provider boundary. Preserve its
     * one task-owned decoder/consumer; never parse from this scenario task. */
    taskENTER_CRITICAL();
    uint32_t depth=atomic_load(&serial->write_cursor)-atomic_load(&serial->read_cursor);
    while (depth<FT_SERIAL_RING_SIZE) {
        const uint32_t count=FT_SERIAL_RING_SIZE-depth < 64 ? FT_SERIAL_RING_SIZE-depth : 64;
        MB_ASSERT(ft_serial_isr_receive(serial,bytes,count,now_us(NULL)));
        depth+=count;
    }
    MB_ASSERT(!ft_serial_isr_receive(serial,bytes,1,now_us(NULL)));
    membench_application[MB_MAX_FT_RING]=atomic_load(&serial->write_cursor)-atomic_load(&serial->read_cursor);
    MB_ASSERT(membench_application[MB_MAX_FT_RING]==FT_SERIAL_RING_SIZE);
    MB_ASSERT(atomic_load(&serial->overflow_events)>=1);
    taskEXIT_CRITICAL();
}
void mb_set_manual_pose(unsigned endpoint)
{
    const double initial[6]={.2,-1.,1.2,-1.,1.,.5};
    taskENTER_CRITICAL();
    for (unsigned i=0;i<6;i++) {
        int32_t units;
        MB_ASSERT(avatar_m_joint_rad_to_position_units(&scales[i],initial[i]+(i==0 ? .02*endpoint : 0),&units));
        avatar_m_node_set_actual_position(&nodes[i],units);
        nodes[i].active_target_valid=false;
    }
    taskEXIT_CRITICAL();
}
void membench_application_switch(void)
{
    const ValidationShared *v=(const ValidationShared *)(uintptr_t)SRAM4_VALIDATION_BASE;
    const TrajectoryShared *t=(const TrajectoryShared *)(uintptr_t)SRAM4_TRAJECTORY_BASE;
    if (v->magic==VALIDATION_MAGIC) {
        const uint32_t depth=v->head-v->tail;
        if (depth>membench_application[MB_MAX_VALIDATION_QUEUE]) membench_application[MB_MAX_VALIDATION_QUEUE]=depth;
    }
    if (t->magic==TRAJECTORY_MAGIC) {
        const uint32_t depth=t->written-t->consumed;
        if (depth>membench_application[MB_MAX_TRAJECTORY_RING]) membench_application[MB_MAX_TRAJECTORY_RING]=depth;
    }
}

/* Observation-only wrappers retain the real implementation and return value. */
bool __real_generate_line_waypoints(Vec3,Vec3,size_t,Vec3 *);
bool __wrap_generate_line_waypoints(Vec3 start,Vec3 end,size_t count,Vec3 *out)
{
    const bool ok=__real_generate_line_waypoints(start,end,count,out);
    if (ok && count>membench_application[MB_MAX_RAW_GEOMETRY])
        membench_application[MB_MAX_RAW_GEOMETRY]=count;
    return ok;
}
TrajectoryReadResult __real_trajectory_m7_take(TrajectoryConsumer *,uint32_t,TrajectorySample *);
TrajectoryReadResult __wrap_trajectory_m7_take(TrajectoryConsumer *c,uint32_t index,TrajectorySample *out)
{
    const TrajectoryReadResult result=__real_trajectory_m7_take(c,index,out);
    if (result==TRAJECTORY_READ_OK) ++membench_application[MB_PLAYBACK_SAMPLES];
    return result;
}
TrajectoryReadResult __real_trajectory_m7_skip(TrajectoryConsumer *,uint32_t,uint32_t);
TrajectoryReadResult __wrap_trajectory_m7_skip(TrajectoryConsumer *c,uint32_t index,uint32_t count)
{
    const TrajectoryReadResult result=__real_trajectory_m7_skip(c,index,count);
    if (result==TRAJECTORY_READ_OK) membench_application[MB_SKIPPED_PLAYBACK_SAMPLES]+=count;
    return result;
}
void *__real__sbrk(ptrdiff_t);
void *__wrap__sbrk(ptrdiff_t increment)
{
    static ptrdiff_t current;
    void *result=__real__sbrk(increment);
    if (result!=(void *)-1) {
        current+=increment;
        if (current>(ptrdiff_t)membench_application[MB_C_HEAP_GROWTH])
            membench_application[MB_C_HEAP_GROWTH]=current;
    }
    return result;
}
void mb_providers_initialize(void)
{
    robot_config_init_ur5(&robot);
    robot.configuration.homeDefined=true;
    const double initial[6]={.2,-1.,1.2,-1.,1.,.5};
    for (unsigned i=0;i<6;i++) {
        robot.configuration.home[i]=initial[i];
        MB_ASSERT(avatar_m_position_scale_default(&scales[i],50));
        MB_ASSERT(avatar_m_node_init(&nodes[i],i+1));
    }
    mb_set_manual_pose(0);
    backend=(CanBackend){.send=send_frame,.receive=receive_frame,.close=close_bus};
    const CanopenMasterConfig mc={.backend=&backend,.node_ids={1,2,3,4,5,6},.node_count=6,
        .heartbeat_timeout_ms=300,.sdo_timeout_ms=100};
    MB_ASSERT(canopen_master_init(&master,&mc));
    w25_model_init(&nor,NOR_BYTES,2*SLOT_SIZE);
    nor.suppress_event_trace=true;
    const W25Q512JVBus bus={w25_model_command,w25_model_time,w25_model_idle,&nor};
    MB_ASSERT(qspi_nor_validated_storage_set_bus(&bus));
    MB_ASSERT(qspi_nor_validated_storage_init(&storage,0,STORAGE_SAMPLES));
    qspi_nor_validated_storage_bind(&storage,&physical_storage);
    homing=(HomingConfig){.duration=.02,.dt=.002,.positionTolerance=.001,.requiredStableCycles=2,.maxVerificationCycles=100};
    teaching=(TeachingConfig){.default_speed_mps=.010f,.minimum_speed_mps=.001f,.maximum_speed_mps=.1f,
        .speed_low_mps=.005f,.speed_medium_mps=.010f,.speed_high_mps=.025f,
        .duplicate_position_tolerance_m=.001f,.duplicate_orientation_tolerance_rad=.01f,
        .minimum_point_separation_m=.002f,.collinearity_epsilon_m2=1e-10f};
    validation=(PathValidationConfig){.default_tcp_speed_mps=.01,.max_tcp_speed_mps=.1,
        .max_tcp_acceleration_mps2=.25,.max_tcp_jerk_mps3=1.,.minimum_segment_length_m=.002,
        .maximum_fk_position_error_m=.002,.maximum_fk_orientation_error_rad=.02,
        .minimum_singularity_sigma=0,.maximum_joint_step_rad=.2,
        .maximum_position_quantization_error_rad=.001,.geometry_points_per_segment=512,.arc_length_spacing_m=.005};
    adls_default_parameters(&validation.ik_parameters);
    workspace=(PathValidationWorkspace){.geometry_capacity=512,.raw_geometry=raw,.arc_geometry=arc,
        .l_original=lengths,.l_arc=arc_lengths,.ik_scratch=&ik_scratch};
    validation_m7_init(&client,(ValidationShared *)(uintptr_t)SRAM4_VALIDATION_BASE);
    proxy=validation_m7_storage(&client,STORAGE_SAMPLES);
    approach=(ApproachConfig){.duration_safety_factor=1.2,.minimum_leg_duration_s=.02,
        .maximum_leg_duration_s=10,.final_position_tolerance_rad=.005,.following_error_limit_rad=.1,
        .required_stable_cycles=2,.maximum_verification_cycles=100,.validation_samples_per_step=64,
        .require_collision_check=true};
    approach_services=(ApproachServices){.read_validated_sample=read_first,.collision_free=collision,
        .controlled_stop=stop,.controlled_stop_timeout_ms=20};
    execution=(PathExecutionConfig){.controlled_stop_timeout_ms=20,.clearance_stable_cycles=2};
    physical_execution=(PathExecutionServices){.set_wire_feed_enabled=wire,.prepare_retraction=prepare,
        .step_retraction=retract,.clearance_verified=prepare,.controlled_stop=stop,.prefill_timeout_ms=100};
    MB_ASSERT(trajectory_execution_bind(&binding,(TrajectoryShared *)(uintptr_t)SRAM4_TRAJECTORY_BASE,
                                       &mb_artifact,&physical_execution));
    MB_ASSERT(trajectory_execution_attach_playback(&binding,(ValidationShared *)(uintptr_t)SRAM4_VALIDATION_BASE));
    paused=(PausedServices){.command_hold=stop,.wire_feed_off=safe};
    faults=(FaultServices){.force_safe_outputs=safe};
    emergency=(EmergencyStopServices){.force_safe_outputs=safe};
    HandGuidingDryRunConfig gc={.max_sensor_age_us=3000,.max_feedback_age_us=10000,
        .max_runtime_age_ms=20,.max_safety_age_ms=20,.require_new_feedback=true,
        .read_sample=application_ft_service_read};
    gc.pipeline_config.sensor.geometry_verified=true;
    gc.pipeline_config.sensor.flange_T_sensor.rotation_wxyz[0]=1;
    gc.pipeline_config.sensor.sensor_T_grip.rotation_wxyz[0]=1;
    gc.pipeline_config.bias.valid=true;
    gc.pipeline_config.require_new_sample=true;
    gc.pipeline_config.mapper.model_verified=true;
    gc.pipeline_config.mapper.damping_lambda=.02;
    gc.pipeline_config.mapper.minimum_sigma=1e-10;
    gc.pipeline_config.mapper.maximum_step_s=.004;
    gc.pipeline_config.admittance.maximum_step_s=.004;
    for (unsigned i=0;i<6;i++) {
        gc.pipeline_config.admittance.virtual_mass[i]=2;
        gc.pipeline_config.admittance.damping[i]=4;
        gc.pipeline_config.admittance.speed_limit[i]=.5;
        gc.pipeline_config.admittance.acceleration_limit[i]=10;
        gc.pipeline_config.mapper.joint_acceleration_limit[i]=100;
        gc.pipeline_config.mapper.tracking_error_limit[i]=.2;
    }
    MB_ASSERT(hand_guiding_dry_run_init(&mb_guidance,&gc));
    mb_config=(SupervisorTaskConfig){.canopen_master=&master,.avatar_position_scales=scales,.robot=&robot,
        .homing_config=&homing,.teaching_config=&teaching,.validation_config=&validation,
        .validation_workspace=&workspace,.validation_storage=&proxy,.validated_trajectory=&mb_artifact,
        .validation_run_in_worker=true,.validation_worker_priority=2,.validation_worker_program=&worker_program,
        .validation_sample_budget=64,.first_program_id=1,.period_ticks=2,
        .ipc=(IpcShared *)(uintptr_t)SRAM4_IPC_BASE,.validation_enter_ready=validation_idle,
        .approach_config=&approach,.approach_services=&approach_services,
        .path_execution_config=&execution,.path_execution_services=&binding.services,
        .paused_services=&paused,.fault_services=&faults,.emergency_stop_services=&emergency,
        .io={.read_inputs=inputs,.write_outputs=outputs},.drive_restart_board={.read=board},
        .reset_diagnostics={.captured=true,.cause=RESET_CAUSE_POWER_ON},
        .safety_evidence={.verify=verify,.max_age_ms=100},.ipc_session={.issue=issue},
        .guidance_dry_run=hand_guiding_dry_run_port(&mb_guidance)};
    previous_tick=xTaskGetTickCount();
}
