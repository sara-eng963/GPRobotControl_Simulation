/*
 * Host-only Approach regression: fake JointDrivePort, no CANopen/SIL Kit.
 * Tests startup feedback, readiness, validation identity and motion cycle.
 * Does not demonstrate physical safety or real-time timing.
 */
#include "../States/state_approach.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    bool configured, healthy, poll_ok, send_ok, auto_feedback;
    bool valid[JOINT_DRIVE_AXES];
    bool enabled[JOINT_DRIVE_AXES];
    int32_t actual[JOINT_DRIVE_AXES];
    int32_t target[JOINT_DRIVE_AXES];
    uint32_t sequence[JOINT_DRIVE_AXES];
    unsigned cycles;
    bool pending;
} FakeDrive;

static int failures;
#define CHECK(x,msg) do { if (x) printf("[PASS] %s\n",msg); \
  else { printf("[FAIL] %s\n",msg); ++failures; } } while (0)

static bool configured(void *ctx) { return ((FakeDrive *)ctx)->configured; }
static bool poll_drive(void *ctx, uint32_t now)
{
    (void)now;
    FakeDrive *f = ctx;
    if (f->pending && f->auto_feedback) {
        for (size_t i=0;i<JOINT_DRIVE_AXES;++i) {
            f->actual[i]=f->target[i];
            ++f->sequence[i];
        }
        f->pending=false;
    }
    return f->poll_ok;
}
static bool health(void *ctx,uint32_t now)
{ (void)now; return ((FakeDrive *)ctx)->healthy; }
static bool feedback_valid(void *ctx)
{
    FakeDrive *f=ctx;
    for (size_t i=0;i<JOINT_DRIVE_AXES;++i)
        if(!f->valid[i])return false;
    return true;
}
static bool read_axis(void *ctx,size_t axis,JointDriveAxisFeedback *out)
{
    FakeDrive *f=ctx;
    if(!out || axis>=JOINT_DRIVE_AXES)return false;
    out->feedback_valid=f->valid[axis];
    out->operation_enabled=f->enabled[axis];
    out->actual_position_units=f->actual[axis];
    return true;
}
static bool send_targets(void *ctx,const int32_t *targets,size_t count)
{
    FakeDrive *f=ctx;
    if(!f->send_ok || count!=JOINT_DRIVE_AXES || !targets)return false;
    memcpy(f->target,targets,sizeof(f->target));
    ++f->cycles;
    f->pending=true;
    return true;
}
static uint32_t feedback_count(void *ctx,size_t axis)
{ return ((FakeDrive *)ctx)->sequence[axis]; }
static JointDrivePort make_port(FakeDrive *f)
{
    JointDrivePort p={
        .context=f,.is_configured=configured,.poll=poll_drive,
        .ready_for_motion=health,.healthy=health,
        .all_feedback_valid=feedback_valid,.read_axis=read_axis,
        .send_targets=send_targets,.feedback_sequence=feedback_count
    };
    return p;
}
static void reset_fake(FakeDrive *f)
{
    memset(f,0,sizeof(*f));
    f->configured=true;f->healthy=true;f->poll_ok=true;
    f->send_ok=true;f->auto_feedback=true;
    for (size_t i=0;i<JOINT_DRIVE_AXES;++i) {
        f->valid[i]=true; f->enabled[i]=true; f->sequence[i]=1;
    }
}
static PvExecutionSample target_sample;
static bool read_sample(uint32_t index,PvExecutionSample *sample,void *ctx)
{
    (void)ctx;
    if(index!=0 || !sample)return false;
    *sample=target_sample;
    return true;
}
static RobotConfig robot;
static AvatarMPositionScale scales[JOINT_DRIVE_AXES];
static ValidatedTrajectory trajectory;
static ApproachRequest request;
static ApproachConfig config;
static ApproachServices services;
static ApproachControlInputs inputs;
static ApproachOutputs outputs;
static ApproachState state;

static void setup_case(void)
{
    robot_config_init_ur5(&robot);
    for(size_t i=0;i<JOINT_DRIVE_AXES;++i) {
        if(!avatar_m_position_scale_default(&scales[i],50.0)) ++failures;
        if(!avatar_m_joint_rad_to_position_units(&scales[i],0.025,
            &target_sample.target_position_units[i])) ++failures;
    }
    memset(&trajectory,0,sizeof(trajectory));
    trajectory.program_id=7;
    trajectory.source_revision=4;
    trajectory.artifact_crc=0xBEEFAAAA;
    trajectory.sample_count=1;
    trajectory.sample_period_us=PATH_VALIDATION_SAMPLE_PERIOD_US;
    request=(ApproachRequest){
        .operation=APPROACH_OPERATION_PREVIEW,
        .trajectory=&trajectory,.trajectory_ready=true,
        .expected_program_id=trajectory.program_id,
        .expected_source_revision=trajectory.source_revision,
        .expected_artifact_crc=trajectory.artifact_crc
    };
    config=(ApproachConfig){
        .duration_safety_factor=1.1,.minimum_leg_duration_s=0.05,
        .maximum_leg_duration_s=10.0,.final_position_tolerance_rad=0.001,
        .following_error_limit_rad=0.10,.required_stable_cycles=2,
        .maximum_verification_cycles=100,
        .validation_samples_per_step=64,.require_collision_check=false
    };
    services=(ApproachServices){.read_validated_sample=read_sample};
    inputs=(ApproachControlInputs){.motion_permission=true};
}
static void enter(FakeDrive *f,JointDrivePort *port)
{
    *port=make_port(f);
    state_approach_enter(&state,&robot,port,scales,
                         &request,&config,&services);
}
static StateStepResult step(uint32_t time_ms)
{ return state_approach_step(&state,&inputs,time_ms,&outputs); }
static void advance_to_read_start(void)
{
    (void)step(0); /* CHECK_REQUEST -> LOAD_TARGET */
    (void)step(1); /* LOAD_TARGET -> READ_START */
}
int main(void)
{
    setup_case();
    FakeDrive fake;
    JointDrivePort port;

    reset_fake(&fake);
    enter(&fake,&port);
    advance_to_read_start();
    CHECK(state.phase==APPROACH_PHASE_READ_START,
          "APPROACH validates trajectory identity through phase machine");
    fake.healthy=false;
    CHECK(step(2)==STATE_STEP_FAILED &&
          state.error==APPROACH_ERR_COMMUNICATION,
          "APPROACH rejects unhealthy drive communication");

    reset_fake(&fake);
    enter(&fake,&port);advance_to_read_start();
    fake.valid[2]=false;
    CHECK(step(2)==STATE_STEP_FAILED &&
          state.error==APPROACH_ERR_FEEDBACK &&
          state.failed_joint==3,
          "APPROACH reports missing measured feedback for axis 3");

    reset_fake(&fake);
    enter(&fake,&port);advance_to_read_start();
    fake.enabled[4]=false;
    CHECK(step(2)==STATE_STEP_FAILED &&
          state.error==APPROACH_ERR_DRIVE_NOT_READY &&
          state.failed_joint==5,
          "APPROACH reports non-enabled drive for axis 5");

    reset_fake(&fake);
    fake.poll_ok=false;
    enter(&fake,&port);
    CHECK(step(0)==STATE_STEP_FAILED &&
          state.error==APPROACH_ERR_COMMUNICATION,
          "APPROACH rejects failed drive polling");

    reset_fake(&fake);
    request.expected_artifact_crc++;
    enter(&fake,&port);
    (void)step(0);
    CHECK(step(1)==STATE_STEP_FAILED &&
          state.error==APPROACH_ERR_TRAJECTORY_MISMATCH,
          "APPROACH rejects stale validated trajectory identity");
    request.expected_artifact_crc--;

    reset_fake(&fake);
    enter(&fake,&port);
    StateStepResult result=STATE_STEP_RUNNING;
    for(uint32_t ms=0;ms<1500 && result==STATE_STEP_RUNNING;++ms)
        result=step(ms);
    CHECK(result==STATE_STEP_COMPLETE &&
          state.result==APPROACH_RESULT_COMPLETE,
          "APPROACH completes simulated route and final target verification");
    CHECK(fake.cycles>1U && state.samples_sent>0U,
          "APPROACH transmits multiple coordinated six-axis command cycles");
    bool exact=true;
    for(size_t i=0;i<JOINT_DRIVE_AXES;++i)
        if(fake.target[i]!=target_sample.target_position_units[i])
            exact=false;
    CHECK(exact,"Final command retains exact validated sample-0 units");

    printf("\nAPPROACH port tests failed: %d\n",failures);
    return failures!=0;
}
