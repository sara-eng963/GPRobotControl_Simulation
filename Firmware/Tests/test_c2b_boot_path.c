/* Executes the PRODUCTION firmware state_boot.c with mocked ports.
 * No hardware is connected, and no physical stop claim is implied. */
#include "state_boot.h"
#include "cia402.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t reads;
    uint32_t writes;
    uint32_t targets;
    DriveCommissionOperation last_operation;
    uint16_t returned_status;
} Mock;
static bool yes(void *ctx) { return ctx != NULL; }
static void reset_runtime(void *ctx, uint32_t ms) {(void)ctx; (void)ms;}
static bool nmt(void *ctx, DriveNetworkCommand c) {(void)c; return ctx != NULL;}
static bool heartbeat(void *ctx, DriveNodeHeartbeatState s)
{ (void)s; return ctx != NULL; }
static bool start_sdo(void *ctx, DriveCommissionOperation op,
                      size_t axis, uint32_t arg, uint32_t now)
{
    Mock *m = (Mock*)ctx;
    (void)axis; (void)arg; (void)now;
    m->last_operation = op;
    if (op == DRIVE_COMMISSION_WRITE_CONTROLWORD) ++m->writes;
    if (op == DRIVE_COMMISSION_READ_STATUSWORD) ++m->reads;
    return true;
}
static DriveOperationResult sdo_result(void *ctx)
{
    Mock *m = (Mock*)ctx;
    DriveOperationResult r = {0};
    r.status = DRIVE_OPERATION_COMPLETE;
    r.has_response = true;
    r.is_read = m->last_operation == DRIVE_COMMISSION_READ_STATUSWORD;
    r.is_write_ok = !r.is_read;
    r.data_size = r.is_read ? 2U : 0U;
    r.value = m->returned_status;
    return r;
}
static void sdo_clear(void *ctx) {(void)ctx;}
static bool jp(void *ctx, uint32_t now) {(void)now; return ctx != NULL;}
static bool jr(void *ctx, size_t axis, JointDriveAxisFeedback *feedback)
{ (void)axis; (void)ctx; *feedback = (JointDriveAxisFeedback){0}; return true; }
static bool jt(void *ctx, const int32_t *target, size_t n)
{ (void)target; (void)n; ++((Mock*)ctx)->targets; return true; }
static uint32_t js(void *ctx, size_t axis)
{(void)ctx; (void)axis; return 0U;}
static DriveCommissioningPort make_commissioning(Mock *m)
{
    DriveCommissioningPort p = {0};
    p.context=m; p.is_initialized=yes; p.is_configured=yes;
    p.reset_runtime=reset_runtime; p.send_network_command=nmt;
    p.all_heartbeat_state=heartbeat; p.begin_operation=start_sdo;
    p.operation_result=sdo_result; p.clear_operation=sdo_clear;
    return p;
}
static JointDrivePort make_joint(Mock *m)
{
    JointDrivePort p={0}; p.context=m; p.is_configured=yes;
    p.poll=jp; p.ready_for_motion=jp; p.healthy=jp;
    p.all_feedback_valid=yes; p.read_axis=jr;
    p.send_targets=jt; p.feedback_sequence=js; return p;
}
int main(void)
{
    Mock m={0}; m.returned_status=CIA402_STATE_SWITCH_ON_DISABLED;
    DriveCommissioningPort c=make_commissioning(&m);
    JointDrivePort d=make_joint(&m);
    BootState b;
    state_boot_enter(&b);
    /* Read six pre-enable statuswords and stop before any 6040 write. */
    b.phase=BOOT_PHASE_PRE_ENABLE_STATUS;
    for (unsigned i=0;i<13U;i++)
        assert(state_boot_step(&b,&c,&d,i*2U)==STATE_STEP_RUNNING);
    assert(b.phase==BOOT_PHASE_WAIT_AUTHORIZATION);
    assert(b.preenable_status_checked && m.reads==6U && m.writes==0U);
    for (unsigned i=0;i<100U;i++)
        assert(state_boot_step(&b,&c,&d,50U+i*2U)==STATE_STEP_RUNNING);
    assert(b.phase==BOOT_PHASE_WAIT_AUTHORIZATION && m.writes==0U);
    /* Only a fresh external gate decision may advance BOOT. */
    b.drive_enable_authorized=true;
    assert(state_boot_step(&b,&c,&d,300U)==STATE_STEP_RUNNING);
    assert(b.phase==BOOT_PHASE_ENABLE_DRIVES);
    assert(state_boot_step(&b,&c,&d,302U)==STATE_STEP_RUNNING); /* read begin */
    assert(state_boot_step(&b,&c,&d,304U)==STATE_STEP_RUNNING); /* read complete */
    assert(state_boot_step(&b,&c,&d,306U)==STATE_STEP_RUNNING); /* write begin */
    assert(m.writes==1U);
    b.drive_enable_authorized=false;
    assert(state_boot_step(&b,&c,&d,308U)==STATE_STEP_FAILED);
    assert(m.writes==1U && m.targets==0U);
    /* Directly entering the ENABLE phase with no auth must fail closed. */
    state_boot_enter(&b); b.phase=BOOT_PHASE_ENABLE_DRIVES;
    assert(state_boot_step(&b,&c,&d,320U)==STATE_STEP_FAILED);
    assert(m.writes==1U);
    /* An already-operation-enabled drive on controller startup is rejected. */
    state_boot_enter(&b); b.phase=BOOT_PHASE_PRE_ENABLE_STATUS;
    m.returned_status=CIA402_STATE_OPERATION_ENABLED;
    assert(state_boot_step(&b,&c,&d,322U)==STATE_STEP_RUNNING);
    assert(state_boot_step(&b,&c,&d,324U)==STATE_STEP_FAILED);
    assert(m.writes==1U);
    puts("PASS: production BOOT path requires authorization before 6040 SDO writes");
    return 0;
}
