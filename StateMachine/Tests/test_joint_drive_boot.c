/*
 * BOOT is a nonblocking consumer of two fake interfaces: commissioning and
 * coordinated six-axis motion. No CANopen or SIL Kit code is linked here.
 */
#include "../States/state_boot.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    bool initialized;
    bool node_layout_ok;
    bool poll_ok;
    bool allow_reset;
    bool bootup_seen;
    bool operational_seen;
    bool pending_feedback;
    bool fail_identity;
    bool fail_identity_transaction;
    bool faulted_statusword;
    bool feedback_valid;
    unsigned poll_count;
    unsigned transactions;
    unsigned commands;
    unsigned clears;
    uint32_t feedback_sequence[JOINT_DRIVE_AXES];
    DriveCommissionOperation operation;
    size_t operation_axis;
    uint32_t operation_arg;
    bool active_operation;
    uint32_t heartbeat_timeout_ms;
} FakeSystem;

static int failures;
#define CHECK(test,desc) do { \
    if (test) printf("[PASS] %s\n",desc); \
    else { printf("[FAIL] %s\n",desc); ++failures; } \
} while (0)

static bool motion_initialized(void *ctx)
{ return ((FakeSystem *)ctx)->initialized; }
static bool motion_configured(void *ctx)
{ return ((FakeSystem *)ctx)->initialized && ((FakeSystem *)ctx)->node_layout_ok; }
static bool motion_poll(void *ctx,uint32_t now_ms)
{
    (void)now_ms;
    FakeSystem *fake=ctx;
    ++fake->poll_count;
    if(fake->pending_feedback) {
        for(size_t axis=0;axis<JOINT_DRIVE_AXES;++axis)
            ++fake->feedback_sequence[axis];
        fake->pending_feedback=false;
    }
    return fake->poll_ok;
}
static bool motion_health(void *ctx,uint32_t now_ms)
{
    (void)now_ms;
    return ((FakeSystem *)ctx)->operational_seen;
}
static bool motion_feedback(void *ctx)
{ return ((FakeSystem *)ctx)->feedback_valid; }
static bool motion_axis(void *ctx,size_t axis,JointDriveAxisFeedback *out)
{
    FakeSystem *fake=ctx;
    if(axis>=JOINT_DRIVE_AXES || out==NULL)return false;
    *out=(JointDriveAxisFeedback){
        .feedback_valid=fake->feedback_valid,
        .operation_enabled=true,
        .actual_position_units=0
    };
    return true;
}
static bool motion_send(void *ctx,const int32_t *target,size_t count)
{
    FakeSystem *fake=ctx;
    if(!target || count!=JOINT_DRIVE_AXES)return false;
    ++fake->commands;
    fake->pending_feedback=true;
    return true;
}
static uint32_t motion_count(void *ctx,size_t axis)
{ return ((FakeSystem *)ctx)->feedback_sequence[axis]; }
static JointDrivePort make_motion(FakeSystem *f)
{
    JointDrivePort p={
        .context=f,
        .is_configured=motion_configured,.poll=motion_poll,
        .ready_for_motion=motion_health,.healthy=motion_health,
        .all_feedback_valid=motion_feedback,.read_axis=motion_axis,
        .send_targets=motion_send,.feedback_sequence=motion_count
    };
    return p;
}

static void reset_runtime(void *ctx,uint32_t timeout)
{
    FakeSystem *f=ctx;
    f->heartbeat_timeout_ms=timeout;
    memset(f->feedback_sequence,0,sizeof(f->feedback_sequence));
}
static bool network(void *ctx,DriveNetworkCommand command)
{
    FakeSystem *f=ctx;
    if(command==DRIVE_NETWORK_RESET_COMMUNICATION) {
        if(!f->allow_reset)return false;
        f->bootup_seen=true;
        f->operational_seen=false;
        return true;
    }
    if(command==DRIVE_NETWORK_START) {
        f->operational_seen=true;
        return true;
    }
    return false;
}
static bool node_state(void *ctx,DriveNodeHeartbeatState state)
{
    FakeSystem *f=ctx;
    if(state==DRIVE_NODE_BOOTUP)return f->bootup_seen;
    if(state==DRIVE_NODE_OPERATIONAL)return f->operational_seen;
    return false;
}
static bool begin(void *ctx,DriveCommissionOperation operation,
                  size_t axis,uint32_t arg,uint32_t now_ms)
{
    (void)now_ms;
    FakeSystem *f=ctx;
    if(axis>=JOINT_DRIVE_AXES || f->active_operation)return false;
    if(f->fail_identity && operation==DRIVE_COMMISSION_READ_IDENTITY)return false;
    f->operation=operation;
    f->operation_axis=axis;
    f->operation_arg=arg;
    f->active_operation=true;
    ++f->transactions;
    return true;
}
static DriveOperationResult result(void *ctx)
{
    FakeSystem *f=ctx;
    DriveOperationResult r={0};
    if(!f->active_operation) {
        r.status=DRIVE_OPERATION_IDLE;
        return r;
    }
    if(f->fail_identity_transaction &&
       f->operation==DRIVE_COMMISSION_READ_IDENTITY) {
        r.status=DRIVE_OPERATION_ABORT;
        r.abort_code=0x06020000U;
        return r;
    }
    r.status=DRIVE_OPERATION_COMPLETE;
    r.has_response=true;
    switch(f->operation) {
        case DRIVE_COMMISSION_READ_IDENTITY:
            r.is_read=true; r.data_size=4;
            r.value=f->operation_arg==1 ? 0x12345678U : 0x87654321U;
            break;
        case DRIVE_COMMISSION_READ_HEARTBEAT_CONSUMER:
            r.is_read=true; r.data_size=4; r.value=0x007F07D0U;
            break;
        case DRIVE_COMMISSION_READ_MODE_DISPLAY:
            r.is_read=true; r.data_size=1; r.value=7U;
            break;
        case DRIVE_COMMISSION_READ_STATUSWORD:
            r.is_read=true; r.data_size=2;
            r.value=f->faulted_statusword ? 0x0008U : 0x0027U;
            break;
        case DRIVE_COMMISSION_READ_ACTUAL_POSITION:
            r.is_read=true; r.data_size=4; r.value=0U;
            break;
        default:
            r.is_write_ok=true;
            break;
    }
    return r;
}
static void clear(void *ctx)
{
    FakeSystem *f=ctx;
    f->active_operation=false;
    ++f->clears;
}
static DriveCommissioningPort make_commissioning(FakeSystem *f)
{
    DriveCommissioningPort p={
        .context=f,
        .is_initialized=motion_initialized,
        .is_configured=motion_configured,
        .reset_runtime=reset_runtime,
        .send_network_command=network,
        .all_heartbeat_state=node_state,
        .begin_operation=begin,
        .operation_result=result,
        .clear_operation=clear,
        .expected_vendor_id=0x12345678U,
        .expected_product_code=0x87654321U,
        .expected_heartbeat_consumer=0x007F07D0U,
        .expected_mode_display=7U
    };
    return p;
}
static void fake_init(FakeSystem *f)
{
    memset(f,0,sizeof(*f));
    f->initialized=true;
    f->node_layout_ok=true;
    f->poll_ok=true;
    f->allow_reset=true;
    f->feedback_valid=true;
}
static StateStepResult simulate(BootState *boot,
     DriveCommissioningPort *commissioning,JointDrivePort *motion,
     uint32_t max_steps)
{
    StateStepResult status=STATE_STEP_RUNNING;
    for(uint32_t ms=0;ms<max_steps && status==STATE_STEP_RUNNING;++ms)
        status=state_boot_step(boot,commissioning,motion,ms);
    return status;
}
int main(void)
{
    FakeSystem fake;
    BootState boot;

    fake_init(&fake);
    JointDrivePort motion=make_motion(&fake);
    DriveCommissioningPort commissioning=make_commissioning(&fake);
    state_boot_enter(&boot);
    CHECK(simulate(&boot,&commissioning,&motion,1000U)==STATE_STEP_COMPLETE &&
          boot.phase==BOOT_PHASE_COMPLETE,
          "BOOT completes full nonblocking six-axis commissioning");
    CHECK(fake.commands>=20U && fake.clears>=6U,
          "BOOT verifies 20 consecutive cyclic feedback exchanges");
    CHECK(fake.heartbeat_timeout_ms==300U,
          "BOOT configures heartbeat timeout");
    CHECK(fake.transactions>=50U,
          "BOOT performs per-drive identities, heartbeat and mode transactions");

    fake_init(&fake);
    fake.node_layout_ok=false;
    state_boot_enter(&boot);
    CHECK(simulate(&boot,&commissioning,&motion,10U)==STATE_STEP_FAILED &&
          boot.error==BOOT_ERROR_NODE_COUNT,
          "BOOT reports incorrect six-axis node layout");

    fake_init(&fake);
    fake.initialized=false;
    state_boot_enter(&boot);
    CHECK(simulate(&boot,&commissioning,&motion,10U)==STATE_STEP_FAILED &&
          boot.error==BOOT_ERROR_MASTER_INIT,
          "BOOT distinguishes uninitialized network master");

    fake_init(&fake);
    fake.allow_reset=false;
    state_boot_enter(&boot);
    CHECK(simulate(&boot,&commissioning,&motion,100U)==STATE_STEP_FAILED &&
          boot.error==BOOT_ERROR_RESET_COMMUNICATION,
          "BOOT fails closed when network reset cannot be sent");

    fake_init(&fake);
    fake.fail_identity=true;
    state_boot_enter(&boot);
    CHECK(simulate(&boot,&commissioning,&motion,100U)==STATE_STEP_FAILED &&
          boot.error==BOOT_ERROR_NODE_IDENTITY && boot.failedAxis==1,
          "BOOT identifies which drive failed identity request");

    fake_init(&fake);
    fake.fail_identity_transaction=true;
    state_boot_enter(&boot);
    CHECK(simulate(&boot,&commissioning,&motion,100U)==STATE_STEP_FAILED &&
          boot.error==BOOT_ERROR_NODE_IDENTITY,
          "BOOT rejects aborted identity read");

    fake_init(&fake);
    fake.faulted_statusword=true;
    state_boot_enter(&boot);
    CHECK(simulate(&boot,&commissioning,&motion,1000U)==STATE_STEP_FAILED &&
          boot.error==BOOT_ERROR_DRIVE_FAULT && boot.failedAxis==1,
          "BOOT rejects CiA402 drive fault");

    printf("\nBOOT tests failed: %d\n",failures);
    return failures!=0;
}
