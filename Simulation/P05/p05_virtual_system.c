/* P05 HOST-ONLY executable. Not compiled in either STM32 image.
 * Real HandGuiding/ControlCore, PathValidation, CANopen, intercore storage
 * protocol and streaming code; physical CAN and serial are replaced ONLY
 * by simulated node electronics.  In-memory durable-store fixture is NOT
 * a QSPI emulator nor a claim of NOR verification. See acceptance report. */
#include "../../HandGuiding/ft_serial_transport.h"
#include "../../HandGuiding/hand_guiding_pipeline.h"
#include "../../HandGuiding/Tests/p05_mock_ft_protocol.h"
#include "../../CANComm/CANopen/canopen_master.h"
#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"
#include "../../ServoDrive/AvatarM/avatar_m_position.h"
#include "../../Intercore/validation_storage_bridge.h"
#include "../../Intercore/trajectory_storage_service.h"
#include "../../ControlCore/Kinematics/control_fk.h"
#include "../../ControlCore/Math/math3d.h"
#include "../../StateMachine/States/state_path_validation.h"
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
    unsigned head,tail,tx_cycles,rx_cycles;
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
static void revoke(SimGate *g) {g->latched=true;g->authorised=false;}

/* Host-only storage callbacks: reuse exactly the production validation/IPC/
 * streaming algorithms; the bytes here are a mock durable medium, NOT NOR.
 * Real Renode tests cover the independent W25Q512JV durable backend. */
typedef struct {
    PvExecutionSample samples[MAX_SAMPLES];
    ValidatedTrajectory metadata;
    uint32_t count;
    bool writing,committed,failed;
} HostArtifact;
static HostArtifact artifact_store;
static bool host_begin(void *ctx)
{
    HostArtifact *s=ctx;
    s->writing=true;s->committed=false;s->count=0;return true;
}
static bool host_write(uint32_t i,const PvExecutionSample *v,void *ctx)
{
    HostArtifact *s=ctx;
    if(!s->writing||s->failed||i!=s->count||i>=MAX_SAMPLES)return false;
    s->samples[i]=*v;++s->count;return true;
}
static bool host_commit(const ValidatedTrajectory *m,void *ctx)
{
    HostArtifact *s=ctx;
    if(!s->writing||s->failed||m->sample_count!=s->count)return false;
    s->metadata=*m;s->committed=true;s->writing=false;return true;
}
static void host_abort(void *ctx)
{
    HostArtifact *s=ctx;s->writing=false;s->committed=false;
}
static bool host_read_meta(ValidatedTrajectory *m,void *ctx)
{
    HostArtifact *s=ctx;if(!s->committed||s->failed)return false;
    *m=s->metadata;return true;
}
static bool host_read_samples(uint32_t first,TrajectorySample *dst,
                              uint32_t n,void *ctx)
{
    HostArtifact *s=ctx;
    if(!s->committed||s->failed||n==0||first>s->count||n>s->count-first)
        return false;
    memcpy(dst,&s->samples[first],n*sizeof(*dst));return true;
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
    PathValidationStorage host={.begin=host_begin,.write_sample=host_write,
        .commit=host_commit,.abort=host_abort,
        .capacity_samples=MAX_SAMPLES,.context=&artifact_store};
    CommittedTrajectoryReader reader={.read_metadata=host_read_meta,
        .read_samples=host_read_samples,.capacity_samples=MAX_SAMPLES,
        .context=&artifact_store};
    REQUIRE(playback_m4_init(&replay,&shared,&trajectory,&reader),"CM4 replay owner");
    REQUIRE(validation_m4_init(&m4,&shared,&host),"CM4 storage owner");
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
            vout.trajectory_ready && artifact_store.committed,
            "validated committed trajectory");
    REQUIRE(validated.sample_count>0 &&
            validated.sample_count<=MAX_SAMPLES,"validated capacity");
    uint32_t computed_crc=UINT32_MAX;
    for(uint32_t i=0;i<validated.sample_count;i++)
        computed_crc=crc32_step(computed_crc,&artifact_store.samples[i],
                                sizeof(artifact_store.samples[i]));
    REQUIRE(validated.sample_data_crc==~computed_crc,
            "validator committed CRC matches exact stored samples");
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
    unsigned replayed=0;
    for(uint32_t i=0;i<validated.sample_count;i++){
        (void)playback_m4_step(&replay);
        TrajectorySample s;
        REQUIRE(trajectory_m7_take(&consumer,i,&s)==TRAJECTORY_READ_OK,
                "read preverified committed sample");
        REQUIRE(!memcmp(&s,&artifact_store.samples[i],sizeof(s)),
                "same samples on readback");
        REQUIRE(gated_send(&gate,&master,s.target_position_units),"replay targets");
        REQUIRE(heartbeat(&bus),"playback heartbeat");
        REQUIRE(canopen_master_poll(&master,i*2U+now_us/1000U),"replay feedback");
        REQUIRE(!memcmp(bus.actual,s.target_position_units,sizeof(bus.actual)),
                "virtual motor followed committed target");
        joint_from_units(&bus,scales,measured);
        write_joint(joint,"replay",i,measured,measured);
        write_pose(pose,"replay",i,&robot,measured);
        fprintf(compare,"%u,%d,%d,1\n",i,
             artifact_store.samples[i].target_position_units[0],s.target_position_units[0]);
        replayed++;
    }
    REQUIRE(trajectory_m7_finish(&consumer),"finish trajectory");
    playback_m7_cancel(&shared);
    REQUIRE(playback_m4_step(&replay),"CM4 release stream");
    /* Negative storage case: corrupt the committed sample after completion.
     * The existing CM4 full-artifact CRC verification must refuse a new epoch. */
    artifact_store.samples[0].target_position_units[0] ^= 1;
    REQUIRE(playback_m7_request(&shared,&trajectory,&id),"CRC rejection request");
    for(unsigned spin=0;spin<100 && shared.playback_status!=PLAYBACK_FAILED;spin++)
        (void)playback_m4_step(&replay);
    REQUIRE(shared.playback_status==PLAYBACK_FAILED,
            "corrupted committed artifact rejected before playback");
    REQUIRE(!trajectory_m7_begin(&consumer,&id),"no corrupted stream published");
    playback_m7_cancel(&shared);
    (void)playback_m4_step(&replay);
    /* No TPDO4 freshness: simulated feedback counters cannot advance. The
     * host gate revokes independently; production Supervisor is not spoofed. */
    int32_t target[AXES]={0};
    uint32_t previous_tpdo=canopen_master_tpdo_rx_count(&master,0);
    bus.drop_tpdo=true;
    REQUIRE(gated_send(&gate,&master,target),"loss injection target transmitted");
    REQUIRE(heartbeat(&bus),"loss-injection heartbeat");
    REQUIRE(canopen_master_poll(&master,now_us/1000U+500),"poll missing feedback");
    REQUIRE(canopen_master_tpdo_rx_count(&master,0)==previous_tpdo,
            "TPDO4 loss observed");
    revoke(&gate);
    unsigned before=bus.tx_cycles;
    REQUIRE(!gated_send(&gate,&master,target) && bus.tx_cycles==before,
            "lost feedback contains further simulated commands");
    bus.drop_tpdo=false;
    /* A distinct explicit host test authority exercises transport TX veto.
     * This does NOT reauthorize the feedback-faulted gate above. */
    SimGate tx_gate={.authorised=true};
    bus.fail_tx=true;
    REQUIRE(!gated_send(&tx_gate,&master,target),"CAN TX failure detected");
    REQUIRE(tx_gate.latched && !tx_gate.authorised,"TX failure revokes simulation gate");
    bus.fail_tx=false;
    before=bus.tx_cycles;
    REQUIRE(!gated_send(&tx_gate,&master,target) && bus.tx_cycles==before,
            "no automatic reauthorization after recovery");
    FtSensorSample stale;
    REQUIRE(!ft_serial_latest(&serial,now_us+5000,&stale),"stale F/T denied");
    REQUIRE(artifact_store.committed && replayed==validated.sample_count,
            "completed validated durable-mock replay");
    fclose(joint);fclose(pose);fclose(compare);
    if(snprintf(path,sizeof(path),"%s/faults.csv",argv[1])>=(int)sizeof(path))return 2;
    FILE *faults=fopen(path,"w");REQUIRE(faults!=NULL,"fault output");
    fputs("fault,detected,contained,automatic_authorization\n"
          "committed_sample_crc_corruption,1,1,0\n"
          "tpdo_feedback_loss,1,1,0\n"
          "can_tx_failure,1,1,0\n"
          "stale_ft_sample,1,1,0\n",faults);
    fclose(faults);
    if(snprintf(path,sizeof(path),"%s/integration.json",argv[1])>=(int)sizeof(path))return 2;
    FILE *result=fopen(path,"w");REQUIRE(result!=NULL,"result output");
    fprintf(result,"{\"result\":\"PASS\",\"simulation_only\":true,\"qspi_proven\":false,"
                   "\"supervisor_production_invoked\":false,\"teach_cycles\":%u,"
                   "\"can_target_cycles\":%u,\"can_feedback_cycles\":%u,"
                   "\"validated_samples\":%u,\"stream_replayed_samples\":%u,"
                   "\"blocked_recovery_attempts\":%u}\n",
            TEACH_CYCLES,bus.tx_cycles,bus.rx_cycles,validated.sample_count,
            replayed,gate.blocked+tx_gate.blocked);
    fclose(result);
    puts("P05 host virtual chain PASS (QSPI and production Supervisor NOT exercised)");
    return 0;
}
