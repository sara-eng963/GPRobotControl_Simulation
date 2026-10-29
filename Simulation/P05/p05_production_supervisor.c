/* Compile the ACTUAL production Supervisor TU through P04's existing
 * production fixture. Only RTOS scheduling, board and peer observations are
 * injected. The CAN master and JointDrivePort below are the live six-drive
 * simulation objects, never FakeDrive. This TU is host-only. */
#define ALIGNMENT_SUPERVISOR_SOURCE "../../Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.c"
#define main p05_unused_alignment_regressions
#include "../../Firmware/Tests/test_production_supervisor.c"
#undef main
#include "p05_production_supervisor.h"

static CanopenMaster *live_master;
static const PvExecutionSample *pending_sample;
static uint32_t pending_index, sample_reads;
static uint64_t injected_clock;
static bool late_dispatch;
static unsigned releases;
static jmp_buf cycle_exit;

static uint64_t clock_read(void *unused)
{ (void)unused; return injected_clock; }
static bool timer_start(void *unused)
{ (void)unused; return true; }
static uint32_t wait_release(void *unused, TickType_t ticks)
{
    (void)unused;
    CHECK(ticks == pdMS_TO_TICKS(6U));
    if (++releases == 2U) longjmp(cycle_exit, 1);
    CHECK(releases == 1U);
    return 1U;
}
static bool observe_release(void *unused, uint32_t count, SupervisorTimingWake *out)
{
    (void)unused;
    CHECK(count == 1U);
    *out = (SupervisorTimingWake){.timestamp_valid=true,
        .isr_service_us=injected_clock-100U, .wake_sample_us=injected_clock,
        .isr_gap_valid=true, .isr_service_gap_us=2000U};
    return true;
}
static bool board_inputs(void *unused, SupervisorInputSnapshot *out)
{
    if (late_dispatch) injected_clock += 1900U; /* dispatch-adjacent 2ms boundary */
    return p03_good_board_inputs(unused,out);
}
static bool consume_live_sample(uint32_t index, PvExecutionSample *out, void *unused)
{
    (void)unused;
    if (!pending_sample || index != pending_index) return false;
    *out = *pending_sample;
    ++sample_reads;
    return true;
}
void vTaskDelayUntil(TickType_t *previous, TickType_t ticks)
{ (void)previous; (void)ticks; CHECK(false); }

static void task_cycle(uint32_t now_us)
{
    now_tick=now_us/1000U;
    injected_clock=now_us;
    releases=0U;
    /* Each bounded call re-enters the real infinite task from its entry.
     * Unbind only this fixture's prior task identity for that re-entry;
     * production bind_runtime_owner still executes and checks ownership. */
    CHECK(live_master->owner_check == NULL ||
          live_master->owner_check_context == &ctx);
    live_master->owner_check=NULL;
    live_master->owner_check_context=NULL;
    if (setjmp(cycle_exit)==0) supervisor_task_entry(&ctx);
    CHECK(releases==2U && critical_depth==0U);
}
bool p05_production_begin(CanopenMaster *live,
                           const ValidatedTrajectory *metadata, uint32_t now_us)
{
    CHECK(live && metadata && now_us>=4000U);
    setup(ROBOT_STATE_PATH_EXECUTION);
    stopped=true; /* injected host hold acknowledgement; no physical stopping claim */
    live_master=live;
    ctx.config.canopen_master=live;
    ctx.joint_drive=canopen_joint_drive_port_make(live);
    /* Real metadata from the durable, CRC-checked playback epoch. */
    trajectory=*metadata;
    ctx.config.validated_trajectory=&trajectory;
    ctx.config.ipc=(IpcShared *)&ctx;
    ctx.config.reset_diagnostics.captured=true;
    ctx.config.reset_diagnostics.cause=RESET_CAUSE_POWER_ON;
    ctx.config.drive_restart_board.read=p02_board_read;
    ctx.boot.preenable_status_checked=true;
    ctx.entered_state=ROBOT_STATE_BOOT; /* real enter_active_state initializes playback */
    p03_prepare_pre_dispatch();
    p03_ready_evidence(&ctx,now_us/1000U);
    ctx.config.io.read_inputs=board_inputs;
    execution_services.read_sample=consume_live_sample;
    ctx.config.timing=(SupervisorTimingPort){.monotonic_us=clock_read,
        .start=timer_start,.wait_next=wait_release,.observe_wake=observe_release};
    g_timing=ctx.config.timing;
    /* Isolated HOST commissioning: exercise the real release/press policy.
     * No physical board provider or firmware permission is supplied. */
    drive_restart_gate_init(&ctx.restart_gate);
    const DriveRestartSystemSample system=
        supervisor_restart_system_sample(&ctx,now_us/1000U);
    p02_authorization_press=false;
    CHECK(!drive_restart_gate_tick(&ctx.restart_gate,
        &ctx.config.reset_diagnostics,&ctx.config.drive_restart_board,&system));
    p02_authorization_press=true;
    CHECK(drive_restart_gate_tick(&ctx.restart_gate,
        &ctx.config.reset_diagnostics,&ctx.config.drive_restart_board,&system));
    if (live->owner_check==NULL)
        CHECK(canopen_master_bind_runtime_owner(live,supervisor_is_canopen_owner,&ctx));
    canopen_master_set_drive_permission(live,true);
    pending_sample=NULL; sample_reads=0U; late_dispatch=false;
    task_cycle(now_us);
    p03_ready_evidence(&ctx,(now_us+2000U)/1000U);
    task_cycle(now_us+2000U);
    CHECK(ctx.machine.activeState==ROBOT_STATE_PATH_EXECUTION);
    CHECK(ctx.path_execution.phase==PATH_EXEC_PHASE_FOLLOW_TRAJECTORY);
    CHECK(sample_reads==0U);
    return true;
}
bool p05_production_cycle(const PvExecutionSample *sample, uint32_t index,
                          uint32_t now_us, P05SupervisorStimulus stimulus,
                          P05SupervisorObservation *out)
{
    pending_sample=sample; pending_index=index;
    const uint32_t before=sample_reads;
    p03_ready_evidence(&ctx,now_us/1000U);
    late_dispatch=stimulus==P05_SUP_DEADLINE;
    if (stimulus==P05_SUP_REVOKED)
        canopen_master_set_drive_permission(live_master,false);
    if (stimulus==P05_SUP_CM4_LOST)
        ctx.peer_evidence.last_response_ms=now_us/1000U-51U;
    if (stimulus==P05_SUP_LATCH_FAULT) {
        const SupervisorEvent fault=make_result_event(SUPERVISOR_EVENT_FAULT_DETECTED,
            ctx.machine.activeState,ROBOT_FAULT_SEVERITY_RECOVERABLE,0x505U);
        p02_enqueue((SupervisorMessage){.type=SUPERVISOR_MESSAGE_EVENT,.data.event=fault});
    }
    if (stimulus==P05_SUP_RESTART) {
        /* Restored inputs + held approval + queued HMI requests cannot clear
         * the inhibited session. These are processed by real production code. */
        p02_enqueue((SupervisorMessage){.type=SUPERVISOR_MESSAGE_HMI_COMMAND,
            .data.hmi_command=SUP_HMI_RESET});
        p02_enqueue((SupervisorMessage){.type=SUPERVISOR_MESSAGE_HMI_COMMAND,
            .data.hmi_command=SUP_HMI_RESUME});
    }
    task_cycle(now_us);
    pending_sample=NULL;
    if (out) *out=(P05SupervisorObservation){
        .permission=live_master->drive_command_permission,
        .inhibited=ctx.restart_gate.inhibited,
        .fault=ctx.machine.activeState==ROBOT_STATE_FAULT,
        .fault_code=ctx.machine.activeFaultCode,
        .samples_read=sample_reads-before,
        .state_samples=ctx.path_execution.sample_index,
        .path_error=(uint32_t)ctx.path_execution.error};
    return sample_reads==before+1U &&
        ctx.machine.activeState==ROBOT_STATE_PATH_EXECUTION &&
        live_master->drive_command_permission && !ctx.restart_gate.inhibited;
}
