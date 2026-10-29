/* P05 host-only integration; never part of the physical STM32 images.
 * Existing W25Q512JV validated storage driver and existing host NOR protocol
 * model are used for the SAME validated and replayed trajectory; no second
 * flash emulator, duplicated CANopen stack or production authorization. */
#include "../../HandGuiding/ft_serial_transport.h"
#include "../../HandGuiding/hand_guiding_pipeline.h"
#include "../../HandGuiding/Tests/p05_mock_ft_protocol.h"
#include "../../Firmware/STM32H745/CM7/Application/CANComm/CANopen/canopen_master.h"
#include "p05_production_supervisor.h"
#include "../../Firmware/STM32H745/CM7/Application/StateMachine/States/state_path_execution.h"
#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"
#include "../../ServoDrive/AvatarM/avatar_m_position.h"
#include "../../Intercore/validation_storage_bridge.h"
#include "../../Intercore/trajectory_storage_service.h"
#include "../../ControlCore/Kinematics/control_fk.h"
#include "../../ControlCore/Math/math3d.h"
#include "../../StateMachine/States/state_path_validation.h"
#include "../Renode/Storage/qspi_nor_validated_storage.h"
#include "../Renode/Storage/w25_artifact_crc.h"
#include "../Tests/w25q_nor_model.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define AXES 6U
#define MAX_SAMPLES 10000U
#define RX_CAPACITY 64U
#define TEACH_CYCLES 140U
#define REQUIRE(test,why) do { if (!(test)) { \
    fprintf(stderr,"P05_ASSERT_FAIL %s (%s:%d)\n",why,__FILE__,__LINE__); \
    return 1; } } while(0)

/* One host-only simulated CAN transport, enforcing exactly six staged RPDOs
 * and a SYNC before any feedback update. No hardware interface or HAL handle. */
typedef struct {
    CanFrame rx[RX_CAPACITY];
    unsigned head,tail,tx_cycles,rx_cycles,target_frames;
    int32_t pending[AXES],actual[AXES];
    unsigned staged;
    bool fail_tx,drop_tpdo,order_fault,nmt_started;
} VirtualBus;
static bool enqueue(VirtualBus *b, const CanFrame *f)
{
    if (b->tail-b->head == RX_CAPACITY) return false;
    b->rx[b->tail++ % RX_CAPACITY] = *f;
    return true;
}
static CanBackendResult virtual_send(void *ctx, const CanFrame *f)
{
    VirtualBus *b=ctx;
    if (b->fail_tx) return CAN_BACKEND_ERROR;
    if (f->id==0U) {
        if (f->dlc!=2U || f->data[1]!=0U) return CAN_BACKEND_ERROR;
        if (f->data[0]==0x01U) { b->nmt_started=true;return CAN_BACKEND_OK; }
        b->nmt_started=false;return CAN_BACKEND_OK;
    }
    if (!b->nmt_started && f->id!=0x77FU) return CAN_BACKEND_ERROR;
    if (f->id >= 0x501U && f->id <= 0x506U) {
        const unsigned axis=(unsigned)(f->id-0x501U);
        int32_t target;
        if (axis != b->staged ||
            !avatar_m_parse_rpdo4((uint8_t)(axis+1U),f,&target)) {
            b->order_fault=true;return CAN_BACKEND_ERROR;
        }
        b->pending[axis]=target;
        ++b->staged;
        ++b->target_frames;
    } else if (f->id == 0x080U) {
        if (b->staged != AXES || f->dlc != 0U) {
            b->order_fault=true;return CAN_BACKEND_ERROR;
        }
        b->staged=0U;
        memcpy(b->actual,b->pending,sizeof(b->actual));
        ++b->tx_cycles;
        if (!b->drop_tpdo) {
            for (unsigned axis=0;axis<AXES;axis++) {
                CanFrame feedback;
                if (!avatar_m_build_tpdo4((uint8_t)(axis+1U),b->actual[axis],
                                           0x0027U,&feedback) ||
                    !enqueue(b,&feedback)) return CAN_BACKEND_ERROR;
            }
            ++b->rx_cycles;
        }
    }
    return CAN_BACKEND_OK;
}
static CanBackendResult virtual_receive(void *ctx, CanFrame *f)
{
    VirtualBus *b=ctx;
    if(b->head==b->tail) return CAN_BACKEND_WOULD_BLOCK;
    *f=b->rx[b->head++ % RX_CAPACITY];
    return CAN_BACKEND_OK;
}
static void virtual_close(void *ctx) {(void)ctx;}
static bool host_guidance_owner(void *ctx) { (void)ctx; return true; }
static bool heartbeat(VirtualBus *b)
{
    for(unsigned axis=0;axis<AXES;axis++) {
        CanFrame f={.id=0x701U+axis,.dlc=1U,.data={0x05U}};
        if(!enqueue(b,&f))return false;
    }
    return true;
}
/* Simulation-only gate. Desktop CanopenMaster does not implement the STM32
 * C2B policy. Never expose these functions to a physical firmware build. */
typedef struct {bool authorised; bool latched; unsigned blocked;} SimGate;
static bool gated_send(SimGate *g,CanopenMaster *m,const int32_t pos[AXES])
{
    if(!g->authorised||g->latched){g->blocked++;return false;}
    if(!canopen_master_send_target_cycle(m,pos,AXES)){
        g->latched=true;g->authorised=false;return false;
    }
    return true;
}

/* Test-only W25Q bus is the repository's existing NOR protocol model.
 * The ACTUAL qspi_nor_validated_storage driver owns durable A/B slots,
 * verification and CRC. This wrapper only records the M7-submitted bytes
 * for independent comparisons after readback; it never supplies playback. */
#define NOR_SLOT_BYTES ((((4096U + MAX_SAMPLES * 24U) + 4095U) / 4096U) * 4096U)
#define NOR_IMAGE_BYTES (2U * NOR_SLOT_BYTES)
static Model nor_device;
static uint8_t nor_image[NOR_IMAGE_BYTES];
static QspiNorValidatedStorage nor_writer;
static QspiNorValidatedStorage nor_reloaded;
typedef struct {
    PathValidationStorage physical_backend;
    PvExecutionSample submitted[MAX_SAMPLES];
    uint32_t count;
    uint32_t commits;
} StorageRecorder;
static StorageRecorder recorder;
static bool model_storage_begin(void *ctx)
{
    StorageRecorder *s=ctx;
    s->count=0U;
    return s->physical_backend.begin(s->physical_backend.context);
}
static bool model_storage_write(uint32_t i,const PvExecutionSample *sample,void *ctx)
{
    StorageRecorder *s=ctx;
    if (!sample || i!=s->count || i>=MAX_SAMPLES ||
        !s->physical_backend.write_sample(i,sample,s->physical_backend.context))
        return false;
    s->submitted[i]=*sample; /* immutable M7 validation evidence */
    ++s->count;
    return true;
}
static bool model_storage_commit(const ValidatedTrajectory *meta,void *ctx)
{
    StorageRecorder *s=ctx;
    if (!meta || meta->sample_count!=s->count ||
        !s->physical_backend.commit(meta,s->physical_backend.context))
        return false;
    ++s->commits;
    return true;
}
static void model_storage_abort(void *ctx)
{
    StorageRecorder *s=ctx;
    s->physical_backend.abort(s->physical_backend.context);
}
static bool nor_read_meta(ValidatedTrajectory *m,void *ctx)
{
    QspiNorValidatedStorage *s=ctx;
    if (!s || !m || !s->committed || !s->readback_verified)return false;
    *m=s->metadata;
    return true;
}
static bool nor_read_samples(uint32_t first,TrajectorySample *dst,
                             uint32_t count,void *ctx)
{
    /* Actual W25Q read opcodes, not the recorder's RAM copy. */
    return qspi_nor_validated_storage_read_samples(first,
            (PvExecutionSample *)dst,count,ctx);
}
static bool initialize_existing_nor_model(void)
{
    w25_model_init(&nor_device,nor_image,sizeof(nor_image));
    nor_device.suppress_event_trace=true; /* unbounded trajectory ops, no 512-event trace */
    W25Q512JVBus bus={w25_model_command,w25_model_time,
                       w25_model_idle,&nor_device};
    return qspi_nor_validated_storage_set_bus(&bus) &&
        qspi_nor_validated_storage_init(&nor_writer,0U,MAX_SAMPLES);
}
static bool reload_existing_nor_model(void)
{
    /* Drop all MCU-owned volatile slot state; keep only simulated NOR bytes. */
    w25_model_power_cycle(&nor_device);
    W25Q512JVBus bus={w25_model_command,w25_model_time,
                       w25_model_idle,&nor_device};
    return qspi_nor_validated_storage_set_bus(&bus) &&
        qspi_nor_validated_storage_init(&nor_reloaded,0U,MAX_SAMPLES) &&
        qspi_nor_validated_storage_load_committed(&nor_reloaded) &&
        nor_reloaded.committed && nor_reloaded.readback_verified;
}
static bool taught_point(const RobotConfig *robot,const double q[AXES],
                         uint32_t now_ms,TaughtPoint *p)
{
    double T[4][4];control_fk(robot,q,T);
    Mat3 rot;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)rot.m[i][j]=T[i][j];
    Quat quat=rotm_to_quat(rot);
    memset(p,0,sizeof(*p));
    for(int i=0;i<3;i++)p->position_m[i]=(float)T[i][3];
    p->orientation_quat[0]=(float)quat.w;
    p->orientation_quat[1]=(float)quat.x;
    p->orientation_quat[2]=(float)quat.y;
    p->orientation_quat[3]=(float)quat.z;
    for(int i=0;i<6;i++)p->joint_position_rad[i]=(float)q[i];
    p->record_timestamp_ms=now_ms;
    p->calibration_version=1U;p->frame_id=1U;p->tool_id=1U;
    p->point_valid=true;
    return true;
}
static HandGuidingPipelineConfig guidance_config(void)
{
    HandGuidingPipelineConfig c={0};
    c.sensor.geometry_verified=true; /* synthetic model ONLY */
    c.sensor.flange_T_sensor.rotation_wxyz[0]=1;
    c.sensor.sensor_T_grip.rotation_wxyz[0]=1;
    c.bias.valid=true;
    c.mapper.model_verified=true;
    c.mapper.damping_lambda=0.02;
    c.mapper.minimum_sigma=1e-10;
    c.mapper.maximum_step_s=0.004;
    c.admittance.maximum_step_s=0.004;
    c.require_new_sample=true;
    for(int i=0;i<6;i++){
        c.admittance.virtual_mass[i]=2.0;
        c.admittance.damping[i]=4.0;
        c.admittance.speed_limit[i]=0.5;
        c.admittance.acceleration_limit[i]=10.0;
        c.mapper.joint_acceleration_limit[i]=100.0;
        c.mapper.tracking_error_limit[i]=0.2;
    }
    return c;
}
static PathValidationConfig validation_config(void)
{
    PathValidationConfig c={0};
    c.default_tcp_speed_mps=.010;c.max_tcp_speed_mps=.100;
    c.max_tcp_acceleration_mps2=.250;c.max_tcp_jerk_mps3=1.0;
    c.minimum_segment_length_m=.002;
    c.maximum_fk_position_error_m=.002;
    c.maximum_fk_orientation_error_rad=.02;
    c.minimum_singularity_sigma=0.;c.maximum_joint_step_rad=.20;
    c.maximum_position_quantization_error_rad=.001;
    c.geometry_points_per_segment=200;
    c.arc_length_spacing_m=.005;
    adls_default_parameters(&c.ik_parameters);
    return c;
}
static void write_joint(FILE *f,const char *phase,uint32_t i,
                        const double ref[AXES],const double fb[AXES])
{
    fprintf(f,"%s,%u",phase,i);
    for(int j=0;j<6;j++)fprintf(f,",%.10f",ref[j]);
    for(int j=0;j<6;j++)fprintf(f,",%.10f",fb[j]);
    fputc('\n',f);
}
static void write_pose(FILE *f,const char *phase,uint32_t i,
                       const RobotConfig *robot,const double q[AXES])
{
    double T[4][4];control_fk(robot,q,T);
    fprintf(f,"%s,%u,%.10f,%.10f,%.10f\n",phase,i,
            T[0][3],T[1][3],T[2][3]);
}
static void joint_from_units(const VirtualBus *bus,
                             const AvatarMPositionScale scales[AXES],
                             double q[AXES])
{
    for(unsigned j=0;j<AXES;j++)
        (void)avatar_m_position_units_to_joint_rad(&scales[j],bus->actual[j],&q[j]);
}
static uint32_t crc32_step(uint32_t crc,const void *data,size_t count)
{
    const uint8_t *s=data;
    for(size_t i=0;i<count;i++){
        crc^=s[i];
        for(unsigned j=0;j<8;j++)crc=(crc>>1U)^
             (UINT32_C(0xEDB88320)&(uint32_t)-(int32_t)(crc&1U));
    }
    return crc;
}
int main(int argc,char **argv)
{
    if(argc!=2){fprintf(stderr,"usage: %s OUTPUT_DIRECTORY\n",argv[0]);return 2;}
    char path[1024];
#define OPEN_CSV(ptr,name,header) do{ \
    if(snprintf(path,sizeof(path),"%s/%s",argv[1],name)>=(int)sizeof(path))return 2; \
    ptr=fopen(path,"w");if(!(ptr))return 2;fputs(header,ptr); \
    }while(0)
    FILE *joint,*pose,*compare;
    OPEN_CSV(joint,"joint_trace.csv", "phase,cycle,q1_ref,q2_ref,q3_ref,q4_ref,q5_ref,q6_ref,q1_fb,q2_fb,q3_fb,q4_fb,q5_fb,q6_fb\n");
    OPEN_CSV(pose,"cartesian_trace.csv", "phase,cycle,x_m,y_m,z_m\n");
    OPEN_CSV(compare,"trajectory_compare.csv", "cycle,axis1_committed,axis1_replayed,all_six_equal\n");
    RobotConfig robot;robot_config_init_ur5(&robot);
    AvatarMPositionScale scales[AXES];
    for(unsigned j=0;j<AXES;j++)REQUIRE(avatar_m_position_scale_default(&scales[j],50.0),"scale");
    VirtualBus bus={0};
    CanBackend back={.context=&bus,.send=virtual_send,.receive=virtual_receive,.close=virtual_close};
    CanopenMaster master;
    CanopenMasterConfig mc={.backend=&back,.node_ids={1,2,3,4,5,6},
                             .node_count=6,.heartbeat_timeout_ms=20,
                             .sdo_timeout_ms=10};
    REQUIRE(canopen_master_init(&master,&mc),"master init");
    REQUIRE(canopen_master_send_nmt_all(&master,CANOPEN_NMT_START) &&
            bus.nmt_started,"NMT operational start");
    double measured[AXES]={.20,-1.0,1.2,-1.0,1.0,.50};
    for(unsigned j=0;j<AXES;j++)REQUIRE(avatar_m_joint_rad_to_position_units(
        &scales[j],measured[j],&bus.actual[j]),"initial position");
    memcpy(bus.pending,bus.actual,sizeof(bus.actual));
    REQUIRE(heartbeat(&bus),"startup heartbeat");
    for(unsigned j=0;j<AXES;j++){
        CanFrame f;
        REQUIRE(avatar_m_build_tpdo4((uint8_t)(j+1),bus.actual[j],0x0027U,&f) &&
                enqueue(&bus,&f),"startup TPDO4");
    }
    REQUIRE(canopen_master_poll(&master,0U),"initial CAN poll");
    REQUIRE(canopen_master_ready_for_motion(&master,0U),"six drives feedback ready");
    REQUIRE(canopen_master_bind_runtime_owner(&master,host_guidance_owner,NULL),
            "isolated HOST guidance owner");
    canopen_master_set_drive_permission(&master,true); /* isolated HOST guidance */
    SimGate gate={.authorised=true}; /* host only; NEVER a production verifier */
    FtSerialPort serial;FtSerialProtocol protocol=mock_protocol();
    REQUIRE(ft_serial_init(&serial,&protocol,5000,1500,3000),"serial init");
    HandGuidingPipelineConfig gc=guidance_config();
    HandGuidingPipelineState gs={0};
    HandGuidingPipelineOutput gout={0};
    TaughtProgram program={0};
    program.program_id=5;program.draft_revision=1;
    program.segment_count=1;program.global_speed_scale=1.0f;
    program.status=TEACH_DRAFT_SUBMITTED;
    TaughtSegment *segment=&program.segments[0];
    segment->segment_id=1;segment->type=TEACH_SEGMENT_LINE;
    segment->point_count=2;segment->speed_mps=.010f;
    segment->orientation_mode=TEACH_ORIENTATION_INTERPOLATED;
    segment->segment_valid=true;
    REQUIRE(taught_point(&robot,measured,0,&segment->points[0]),"record first point");
    uint32_t now_us=1000;
    for(unsigned cycle=0;cycle<TEACH_CYCLES;cycle++){
        uint8_t wire[PACKET_LEN];FtSensorSample s;
        packet(wire,cycle+1,1500);
        REQUIRE(ft_serial_isr_receive(&serial,wire,PACKET_LEN,now_us),"serial receive");
        REQUIRE(ft_serial_poll(&serial,now_us+100,256)==PACKET_LEN,"serial drain");
        REQUIRE(ft_serial_latest(&serial,now_us+100,&s),"fresh serial sample");
        HandGuidingPipelineInput in={0};
        in.sample=&s;in.measured_joint_rad=measured;in.now_us=now_us+100;
        in.max_sensor_age_us=3000;in.period_s=.002;
        in.gates=(HandGuidingPipelineGates){.teaching_active=true,
            .motion_permitted=true,.robot_homed=true,.drives_ready=true,
            .feedback_fresh=true}; /* simulation claims only */
        if(cycle==0) REQUIRE(hand_guiding_pipeline_arm(&gs,&gc,&robot,&in),"arm guidance");
        else {
            REQUIRE(hand_guiding_pipeline_step(&gs,&robot,&in,&gout),"guidance step");
            REQUIRE(gout.valid,"valid mapped output");
            int32_t targets[AXES];
            for(unsigned j=0;j<AXES;j++){
                REQUIRE(isfinite(gout.joint_position_ref[j]) &&
                        isfinite(gout.joint_velocity_ref[j]),"finite mapped references");
                REQUIRE(avatar_m_joint_rad_to_position_units(&scales[j],
                     gout.joint_position_ref[j],&targets[j]),"joint conversion");
            }
            REQUIRE(gated_send(&gate,&master,targets),"authorised six target cycle");
            REQUIRE(heartbeat(&bus),"heartbeat six nodes");
            REQUIRE(canopen_master_poll(&master,now_us/1000U),"CAN feedback poll");
            REQUIRE(canopen_master_ready_for_motion(&master,now_us/1000U),"fresh six-drive state");
            REQUIRE(!bus.order_fault,"RPDO4 ordering/SYNC");
            joint_from_units(&bus,scales,measured);
            write_joint(joint,"teach",cycle,gout.joint_position_ref,measured);
            write_pose(pose,"teach",cycle,&robot,measured);
        }
        now_us+=2000;
    }
    REQUIRE(taught_point(&robot,measured,now_us/1000,
                         &segment->points[1]),"record measured endpoint");
    REQUIRE(segment->points[0].point_valid && segment->points[1].point_valid,
            "two real recorded points");
    program.draft_crc=state_path_validation_calculate_draft_crc(&program);
    PathValidationConfig vc=validation_config();
    static Vec3 raw[512],arc[512];
    static real_t lengths[512],arc_lengths[512];
    static ADLSInfo scratch;
    PathValidationWorkspace ws={.geometry_capacity=512,.raw_geometry=raw,
        .arc_geometry=arc,.l_original=lengths,.l_arc=arc_lengths,
        .ik_scratch=&scratch};
    ValidationShared shared;TrajectoryShared trajectory;
    validation_shared_initialize(&shared);
    trajectory_shared_initialize(&trajectory);
    ValidationM7Client m7;
    ValidationM4Worker m4;
    PlaybackM4Worker replay;
    TrajectoryConsumer consumer;
    validation_m7_init(&m7,&shared);
    trajectory_m7_init(&consumer,&trajectory);
    REQUIRE(initialize_existing_nor_model(),"existing W25Q model + QSPI driver");
    qspi_nor_validated_storage_bind(&nor_writer,&recorder.physical_backend);
    PathValidationStorage qspi={.begin=model_storage_begin,
        .write_sample=model_storage_write,.commit=model_storage_commit,
        .abort=model_storage_abort,.capacity_samples=MAX_SAMPLES,
        .context=&recorder};
    CommittedTrajectoryReader reader={.read_metadata=nor_read_meta,
        .read_samples=nor_read_samples,.capacity_samples=MAX_SAMPLES,
        .context=&nor_reloaded};
    REQUIRE(playback_m4_init(&replay,&shared,&trajectory,&reader),"CM4 replay owner");
    REQUIRE(validation_m4_init(&m4,&shared,&qspi),"CM4 W25Q storage owner");
    PathValidationStorage proxy=validation_m7_storage(&m7,MAX_SAMPLES);
    ValidatedTrajectory validated={0};
    PathValidationState validation={0};
    PathValidationServices services={0};
    state_path_validation_enter(&validation,&robot,scales,&vc,&services,&ws,
        &proxy,&validated,&program,program.draft_revision,program.draft_crc);
    REQUIRE(validation.initialized && validation.result==PV_RESULT_RUNNING,
            "start real path validation");
    PathValidationOutputs vout={0};StateStepResult state=STATE_STEP_RUNNING;
    for(unsigned spin=0;spin<30000 && state==STATE_STEP_RUNNING;spin++){
        state=state_path_validation_step(&validation,64,&vout);
        REQUIRE(validation_m4_step(&m4),"CM4 validation service");
    }
    REQUIRE(state==STATE_STEP_COMPLETE && vout.report.result==PV_RESULT_VALID &&
            vout.trajectory_ready && recorder.commits==1U &&
            nor_writer.committed && nor_writer.readback_verified,
            "validated W25Q QSPI committed trajectory");
    REQUIRE(validated.sample_count>0 &&
            validated.sample_count<=MAX_SAMPLES,"validated capacity");
    REQUIRE(reload_existing_nor_model(),"power-cycle and reload exact NOR artifact");
    REQUIRE(nor_reloaded.metadata.program_id==validated.program_id &&
            nor_reloaded.metadata.source_revision==validated.source_revision &&
            nor_reloaded.metadata.artifact_crc==validated.artifact_crc &&
            nor_reloaded.metadata.sample_count==validated.sample_count &&
            nor_reloaded.metadata.sample_data_crc==validated.sample_data_crc &&
            w25_artifact_crc(&nor_reloaded.metadata)==validated.artifact_crc,
            "exact durable metadata and identity after power-cycle");
    REQUIRE(recorder.count==validated.sample_count,"all validated samples submitted");
    uint32_t computed_crc=UINT32_MAX;
    for(uint32_t i=0;i<validated.sample_count;i++){
        PvExecutionSample from_flash={0};
        REQUIRE(qspi_nor_validated_storage_read_sample(i,&from_flash,&nor_reloaded),
                "read one actual QSPI NOR sample");
        REQUIRE(!memcmp(&from_flash,&recorder.submitted[i],sizeof(from_flash)),
                "QSPI readback matches validator-submitted bytes");
        computed_crc=crc32_step(computed_crc,&from_flash,sizeof(from_flash));
    }
    REQUIRE(validated.sample_data_crc==~computed_crc,
            "CRC of exact W25Q readback matches committed metadata");
    TrajectoryIdentity id={.program_id=validated.program_id,
        .source_revision=validated.source_revision,
        .artifact_crc=validated.artifact_crc,
        .sample_count=validated.sample_count,
        .sample_period_us=validated.sample_period_us};
    REQUIRE(playback_m7_request(&shared,&trajectory,&id),"request exact committed ID");
    for(unsigned spin=0;spin<100 && shared.playback_status!=PLAYBACK_STREAMING;spin++)
        REQUIRE(playback_m4_step(&replay),"CM4 verification/refill");
    REQUIRE(shared.playback_status==PLAYBACK_STREAMING,"CRC-validated stream");
    /* The producer publishes the epoch before its bounded 256-sample refills.
     * The consumer requires min(sample_count,512) staged samples. */
    REQUIRE(playback_m4_step(&replay),"prefill batch one");
    REQUIRE(playback_m4_step(&replay),"prefill batch two");
    REQUIRE(trajectory_m7_begin(&consumer,&id),"begin exact playback epoch");
    /* The live production task owns replay dispatch, using the same master,
     * the actual JointDrivePort and production PathExecution state. */
    REQUIRE(heartbeat(&bus) && canopen_master_poll(&master,now_us/1000U),
            "fresh network before Supervisor commissioning");
    master.owner_check=NULL; master.owner_check_context=NULL; /* host-only ownership handoff */
    REQUIRE(p05_production_begin(&master,&validated,now_us),"production task preflight");
    now_us+=4000U;
    unsigned replayed=0;
    for(uint32_t i=0;i<validated.sample_count;i++){
        (void)playback_m4_step(&replay);
        TrajectorySample s;
        REQUIRE(trajectory_m7_take(&consumer,i,&s)==TRAJECTORY_READ_OK,
                "read preverified committed sample");
        REQUIRE(!memcmp(&s,&recorder.submitted[i],sizeof(s)),
                "stream is exactly W25Q-committed validation sample");
        P05SupervisorObservation observation;
        REQUIRE(p05_production_cycle((const PvExecutionSample *)&s,i,now_us,
                P05_SUP_NORMAL,&observation),"production Supervisor replay dispatch");
        REQUIRE(observation.samples_read==1U && observation.state_samples==i+1U,
                "real production PathExecution consumed this flash sample");
        REQUIRE(heartbeat(&bus),"playback heartbeat");
        REQUIRE(canopen_master_poll(&master,now_us/1000U),"replay feedback");
        REQUIRE(!memcmp(bus.actual,s.target_position_units,sizeof(bus.actual)),
                "virtual motor followed committed target");
        joint_from_units(&bus,scales,measured);
        write_joint(joint,"replay",i,measured,measured);
        write_pose(pose,"replay",i,&robot,measured);
        fprintf(compare,"%u,%d,%d,1\n",i,
             recorder.submitted[i].target_position_units[0],s.target_position_units[0]);
        replayed++;
        now_us+=2000U;
    }
    REQUIRE(trajectory_m7_finish(&consumer),"finish trajectory");
    playback_m7_cancel(&shared);
    REQUIRE(playback_m4_step(&replay),"CM4 release stream");
    /* Negative storage case: corrupt the committed sample after completion.
     * The existing CM4 full-artifact CRC verification must refuse a new epoch. */
    /* Physically corrupt one byte in the EXISTING test-only NOR model, not
     * the immutable recorder. M4 full-artifact CRC must reject the new epoch. */
    const uint32_t flash_payload_start=nor_reloaded.base_address+
        (uint32_t)nor_reloaded.active_slot*nor_reloaded.slot_bytes+4096U;
    REQUIRE(flash_payload_start<sizeof(nor_image),"W25Q image payload bound");
    nor_image[flash_payload_start] ^= 1U;
    REQUIRE(playback_m7_request(&shared,&trajectory,&id),"CRC rejection request");
    for(unsigned spin=0;spin<100 && shared.playback_status!=PLAYBACK_FAILED;spin++)
        (void)playback_m4_step(&replay);
    REQUIRE(shared.playback_status==PLAYBACK_FAILED,
            "corrupted committed artifact rejected before playback");
    REQUIRE(!trajectory_m7_begin(&consumer,&id),"no corrupted stream published");
    playback_m7_cancel(&shared);
    (void)playback_m4_step(&replay);
    /* Each adversarial case starts a distinct explicit HOST commissioning
     * session. Recovery attempts within the faulted session NEVER call begin.
     * Both complete target frames and SYNCs must remain unchanged on denial. */
    static const char *names[]={"revoked_motor_authorization","missing_drive_feedback",
        "stale_drive_feedback","cm4_liveness_failure","missed_control_deadline",
        "latched_fault","can_tx_failure"};
    P05SupervisorObservation observations[7], recovery[7];
    unsigned denied_cases=0U, recovery_denied=0U;
    unsigned denied_frames[7],denied_syncs[7];
    for(unsigned test=0;test<7U;++test) {
        bus.drop_tpdo=false; bus.fail_tx=false;
        now_us+=10000U;
        REQUIRE(heartbeat(&bus),"fault case heartbeat");
        for(unsigned axis=0;axis<AXES;++axis) {
            CanFrame f;
            REQUIRE(avatar_m_build_tpdo4((uint8_t)(axis+1U),bus.actual[axis],
                    0x0027U,&f) && enqueue(&bus,&f),"fault case fresh TPDO");
        }
        REQUIRE(canopen_master_poll(&master,now_us/1000U),"fault case feedback");
        REQUIRE(p05_production_begin(&master,&validated,now_us),"new HOST test session");
        now_us+=4000U;
        unsigned positive_frames=bus.target_frames,positive_syncs=bus.tx_cycles;
        REQUIRE(p05_production_cycle(&recorder.submitted[0],0U,now_us,
                P05_SUP_NORMAL,NULL),"positive control before every injection");
        REQUIRE(bus.target_frames==positive_frames+6U &&
                bus.tx_cycles==positive_syncs+1U,"positive six RPDOs then SYNC");
        REQUIRE(heartbeat(&bus) && canopen_master_poll(&master,now_us/1000U),
                "positive control feedback");
        now_us+=2000U;
        P05SupervisorStimulus stimulus=P05_SUP_NORMAL;
        if(test==0U) stimulus=P05_SUP_REVOKED;
        if(test==1U) {
            /* Inject a missing observation in the actual coordinator cache. */
            master.drives[2].feedback_valid=false;
        }
        uint32_t next_index=1U;
        if(test==2U) {
            bus.drop_tpdo=true;
            /* Keep the task cadence and node heartbeats healthy while ONLY
             * TPDO feedback stops. Isolate the production 20ms watchdog. */
            for(unsigned soak=0U;soak<10U;++soak) {
                REQUIRE(heartbeat(&bus),"stale-feedback heartbeat");
                REQUIRE(p05_production_cycle(&recorder.submitted[next_index],
                    next_index,now_us,P05_SUP_NORMAL,NULL),"watchdog grace cycle");
                REQUIRE(canopen_master_poll(&master,now_us/1000U),"dropped TPDO poll");
                ++next_index; now_us+=2000U;
            }
        }
        if(test==3U) stimulus=P05_SUP_CM4_LOST;
        if(test==4U) stimulus=P05_SUP_DEADLINE;
        if(test==5U) stimulus=P05_SUP_LATCH_FAULT;
        if(test==6U) bus.fail_tx=true;
        unsigned before=bus.tx_cycles, frames=bus.target_frames;
        REQUIRE(!p05_production_cycle(&recorder.submitted[next_index],next_index,now_us,stimulus,
                &observations[test]),"production denies injected motion");
        REQUIRE(bus.tx_cycles==before && bus.target_frames==frames,
                "no RPDO or SYNC after unsafe stimulus");
        REQUIRE(!observations[test].permission && observations[test].inhibited &&
                observations[test].fault && observations[test].samples_read<=1U,
                "production fault latches and revokes actual master");
        if(test!=6U) REQUIRE(observations[test].samples_read==0U,
                "unsafe motion denied before reading the next sample");
        if(test==0U) REQUIRE((observations[test].fault_code&0xFFFFU)==0xE0B1U,
                "authorization revocation fault");
        if(test==2U) REQUIRE(observations[test].path_error==PATH_EXEC_ERR_CYCLIC_FEEDBACK,
                "TPDO age alone trips production watchdog");
        if(test==3U) REQUIRE((observations[test].fault_code&0xFFFFU)==0xE018U,
                "CM4 liveness alone trips production fault");
        if(test==4U) REQUIRE((observations[test].fault_code&0xFFFFU)==0xE004U,
                "dispatch-adjacent deadline fault");
        denied_frames[test]=bus.target_frames-frames;
        denied_syncs[test]=bus.tx_cycles-before;
        ++denied_cases;
        bus.drop_tpdo=false; bus.fail_tx=false;
        now_us+=2000U;
        REQUIRE(heartbeat(&bus),"recovered heartbeat");
        for(unsigned axis=0;axis<AXES;++axis) {
            CanFrame f;
            REQUIRE(avatar_m_build_tpdo4((uint8_t)(axis+1U),bus.actual[axis],
                    0x0027U,&f) && enqueue(&bus,&f),"recovered TPDO");
        }
        REQUIRE(canopen_master_poll(&master,now_us/1000U),"recovered CAN inputs");
        REQUIRE(!p05_production_cycle(&recorder.submitted[1],1U,now_us,
                P05_SUP_RESTART,&recovery[test]),"no automatic restart");
        REQUIRE(!recovery[test].permission && recovery[test].inhibited &&
                recovery[test].samples_read==0U && bus.tx_cycles==before &&
                bus.target_frames==frames,"restored inputs/HMI do not reauthorize");
        REQUIRE(!canopen_master_send_target_cycle(&master,
                recorder.submitted[1].target_position_units,AXES),
                "production coordinator independently rejects bypass attempt");
        ++recovery_denied;
    }
    FtSensorSample stale;
    REQUIRE(!ft_serial_latest(&serial,now_us+5000,&stale),"stale F/T denied");
    REQUIRE(nor_reloaded.committed && replayed==validated.sample_count,
            "completed exact W25Q-committed trajectory replay");
    REQUIRE(denied_cases==7U && recovery_denied==7U,"all in-chain negatives executed");
    fclose(joint);fclose(pose);fclose(compare);
    if(snprintf(path,sizeof(path),"%s/production_supervisor_trace.json",argv[1])>=(int)sizeof(path))return 2;
    FILE *sup=fopen(path,"w");REQUIRE(sup!=NULL,"production Supervisor evidence");
    fputs("{\"production_supervisor_executed\":true,\"same_live_canopen_process\":true,"
          "\"task_entry_executed\":true,\"time_domain\":\"deterministic injected host time\","
          "\"scenarios\":{",sup);
    for(unsigned test=0;test<7U;++test) {
        fprintf(sup,"%s\"%s\":{\"status\":\"PASS\",\"fault_code\":%u,"
            "\"motion_denied\":true,\"restart_denied\":true,"
            "\"permission\":%s,\"inhibited\":%s,\"path_error\":%u,"
            "\"denied_rpdo_frames\":%u,\"denied_syncs\":%u}",
            test?",":"",names[test],observations[test].fault_code,
            observations[test].permission?"true":"false",
            observations[test].inhibited?"true":"false",observations[test].path_error,
            denied_frames[test],denied_syncs[test]);
    }
    fputs("}}\n",sup);fclose(sup);
    if(snprintf(path,sizeof(path),"%s/qspi_readback.json",argv[1])>=(int)sizeof(path))return 2;
    FILE *nor_proof=fopen(path,"w");REQUIRE(nor_proof!=NULL,"W25 proof file");
    fprintf(nor_proof,"{\"backend\":\"existing_qspi_nor_validated_storage_and_w25_model\","
        "\"same_artifact\":true,\"same_samples\":true,\"crc_verified\":true,"
        "\"read_from_simulated_flash\":true,\"sample_count\":%u,"
        "\"artifact_crc\":%u,\"sample_data_crc\":%u}\n",
        validated.sample_count,validated.artifact_crc,validated.sample_data_crc);
    fclose(nor_proof);
    if(snprintf(path,sizeof(path),"%s/faults.csv",argv[1])>=(int)sizeof(path))return 2;
    FILE *faults=fopen(path,"w");REQUIRE(faults!=NULL,"fault output");
    fputs("fault,detected,contained,automatic_authorization\n"
          "committed_sample_crc_corruption,1,1,0\n"
          "stale_ft_sample,1,1,0\n",faults);
    for(unsigned test=0;test<7U;++test)
        fprintf(faults,"%s,1,1,0\n",names[test]);
    fclose(faults);
    if(snprintf(path,sizeof(path),"%s/integration.json",argv[1])>=(int)sizeof(path))return 2;
    FILE *result=fopen(path,"w");REQUIRE(result!=NULL,"result output");
    fprintf(result,"{\"result\":\"PASS\",\"simulation_only\":true,\"qspi_proven\":true,"
                   "\"supervisor_production_invoked\":true,\"teach_cycles\":%u,"
                   "\"can_target_cycles\":%u,\"can_feedback_cycles\":%u,"
                   "\"validated_samples\":%u,\"stream_replayed_samples\":%u,"
                   "\"blocked_recovery_attempts\":%u}\n",
            TEACH_CYCLES,bus.tx_cycles,bus.rx_cycles,validated.sample_count,
            replayed,recovery_denied);
    fclose(result);
    puts("P05 host virtual chain PASS (existing W25Q NOR model + actual QSPI storage; production Supervisor task controls live CANopen playback)");
    return 0;
}
