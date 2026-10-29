/* Host simulation providers. These symbols are linked ONLY into robot_simulation.
 * The production task, restart policy and CAN coordinator remain unchanged.
 * Linux scheduling is not STM32 deadline evidence; use the production task's
 * software-period fallback, never fabricate TIM3 observations. */
#include "../../Firmware/STM32H745/CM7/Application/Intercore/trajectory_storage_service.h"
static IpcShared robot_ipc;
static TrajectoryShared robot_trajectory;
static TrajectoryConsumer robot_consumer;
static PlaybackM4Worker robot_playback;
static uint32_t robot_session;
static bool robot_requested;
static bool robot_validation_ready(void *ctx);
static uint32_t sim_now_ms(void);
static void robot_host_initialize(void)
{
    FILE *random=fopen("/dev/urandom","rb");
    if (!random || fread(&robot_session,sizeof(robot_session),1,random)!=1 || !robot_session) {
        fprintf(stderr,"Host session creation failed\n"); exit(1);
    }
    fclose(random);
    ipc_initialize(&robot_ipc);
    trajectory_shared_initialize(&robot_trajectory);
    trajectory_m7_init(&robot_consumer,&robot_trajectory);
}
static bool robot_issue_session(void *ctx,uint32_t *session)
{ (void)ctx; *session=robot_session; return robot_session!=0; }
static bool robot_verify_safety(void *ctx,const RobotSafetySnapshot *s,
                                 uint32_t *session,uint32_t *sequence)
{
    (void)ctx;
    if (!s || !s->statusValid || s->timestampMs==0) return false;
    *session=robot_session; *sequence=s->timestampMs;
    return true; /* Synthetic host input producer, never a physical verifier. */
}
static void robot_publish_safety(SupervisorMessage *m)
{ (void)m; /* The host producer's same-clock timestamp is its sequence. */ }
static bool robot_restart_sample(void *ctx,DriveRestartBoardSample *out)
{
    (void)ctx;
    SupervisorDiagnostics s={0};
    supervisor_task_get_diagnostics(&s);
    /* Explicit simulated commissioning release/press, only after production
     * BOOT completes pre-enable checks. A fresh process is required after faults. */
    static unsigned ready;
    if (s.boot.preenable_status_checked && ready<10U) ++ready;
    *out=(DriveRestartBoardSample){.valid=true,.operator_pressed=ready>=10U,
        .board_ready=!estop&&!protective&&!injected_fault,
        .reset_reviewed=true,.ipc_session_fresh=ipc_valid(&robot_ipc),
        .old_commands_discarded=true,.feedback_revalidated=true};
    return true;
}
static void robot_host_configure(SupervisorTaskConfig *c)
{
    c->ipc=&robot_ipc;
    c->ipc_session=(RuntimeIpcSessionPort){robot_issue_session,NULL};
    c->safety_evidence=(RuntimeSafetyPort){robot_verify_safety,NULL,100U};
    c->drive_restart_board=(DriveRestartBoardPort){robot_restart_sample,NULL};
    c->reset_diagnostics.captured=true;
    c->reset_diagnostics.cause=RESET_CAUSE_POWER_ON;
}
static void robot_peer_step(void)
{
    static bool hello_sent;
    static uint32_t sequence=1U;
    if (!hello_sent) {
        IpcMessage hello={.type=IPC_CM4_HELLO,.sequence=1U,
            .data={0,RUNTIME_PEER_PROTOCOL_VERSION,1U}};
        hello.data[0]=robot_session;
        hello_sent=ipc_m4_send(&robot_ipc,&hello);
        if (!hello_sent) return;
    }
    IpcMessage status;
    while (ipc_m4_receive(&robot_ipc,&status)) {
        if (status.type!=IPC_CONTROLLER_STATUS || !status.data[9]) continue;
        IpcMessage progress={.type=IPC_CM4_PROGRESS,.sequence=++sequence,
            .timestamp_ms=sim_now_ms(),
            .data={0,RUNTIME_PEER_PROTOCOL_VERSION,3U,1U,1U,1U,0}};
        progress.data[0]=robot_session;
        progress.data[6]=status.data[9];
        (void)ipc_m4_send(&robot_ipc,&progress);
    }
}
static bool robot_read_metadata(ValidatedTrajectory *meta,void *ctx)
{
    QspiNorValidatedStorage *s=ctx;
    if (!s->committed || !s->readback_verified) return false;
    *meta=s->metadata; return true;
}
static bool robot_read_samples(uint32_t first,TrajectorySample *out,uint32_t count,void *ctx)
{ return qspi_nor_validated_storage_read_samples(first,(PvExecutionSample *)out,count,ctx); }
static void robot_playback_initialize(void)
{
    const CommittedTrajectoryReader reader={robot_read_metadata,robot_read_samples,
        SIM_PV_STORAGE_CAPACITY,&host_flash_storage};
    if (!playback_m4_init(&robot_playback,&flash_validation_shared,&robot_trajectory,&reader)) {
        fprintf(stderr,"CM4 playback init failed\n"); exit(1);
    }
}
static TrajectoryIdentity robot_identity(void)
{
    return (TrajectoryIdentity){.program_id=validated_trajectory.program_id,
        .source_revision=validated_trajectory.source_revision,
        .artifact_crc=validated_trajectory.artifact_crc,
        .sample_count=validated_trajectory.sample_count,
        .sample_period_us=validated_trajectory.sample_period_us};
}
static bool robot_stream_ready(uint32_t next,uint32_t count)
{
    if (count!=validated_trajectory.sample_count || next!=0U) return robot_consumer.active;
    const TrajectoryIdentity identity=robot_identity();
    if (!robot_requested) {
        if (!playback_m7_request(&flash_validation_shared,&robot_trajectory,&identity)) return false;
        robot_requested=true;
    }
    if (robot_consumer.active) return true;
    if (!trajectory_m7_begin(&robot_consumer,&identity)) return false;
    printf("[ROBOT_PLAYBACK] epoch=%u samples=%u CRC=%u\n",robot_consumer.epoch,
        identity.sample_count,identity.artifact_crc);
    return true;
}
static bool robot_take(uint32_t index,PvExecutionSample *sample)
{
    TrajectorySample raw;
    if (trajectory_m7_take(&robot_consumer,index,&raw)!=TRAJECTORY_READ_OK) return false;
    memcpy(sample,&raw,sizeof(raw)); return true;
}
static bool robot_skip(uint32_t first,uint32_t count)
{ return trajectory_m7_skip(&robot_consumer,first,count)==TRAJECTORY_READ_OK; }
static void robot_end_stream(void)
{
    playback_m7_cancel(&flash_validation_shared);
    if (robot_consumer.active) {
        printf("[ROBOT_PLAYBACK] released epoch=%u consumed=%u total=%u\n",
            robot_consumer.epoch,robot_consumer.next_index,robot_consumer.expected.sample_count);
        (void)trajectory_m7_finish(&robot_consumer);
    }
    else (void)trajectory_m7_discard(&robot_consumer);
    robot_requested=false;
}
static bool robot_validation_ready(void *ctx)
{
    (void)ctx;
    robot_end_stream();
    return __atomic_load_n(&robot_trajectory.ready,__ATOMIC_ACQUIRE)==0U &&
        validation_m4_idle(&flash_validation_shared);
}

/* Simulation-only teaching actuator, executed by the Supervisor owner task.
 * The physical Supervisor stays observation-only. No second CAN writer task. */
static QueueHandle_t robot_guided_targets;
static HandGuidingCommandGateState robot_guidance_gate;
static SupervisorGuidanceDryRunPort robot_observer;
static int32_t robot_requested_targets[6];
static bool robot_have_target;
static bool robot_guidance_tick(void *ctx,const SupervisorGuidanceDryRunCycle *cycle)
{
    (void)ctx;
    int32_t targets[6];
    bool ok=cycle->motion_permitted && cycle->robot_homed &&
        cycle->drives_ready && cycle->feedback_healthy && cycle->safety_valid &&
        !cycle->estop_active && !cycle->protective_stop_active && !cycle->global_fault_active;
    if (virtual_ft_mode) {
        SupervisorGuidanceCandidate candidate={0};
        ok=ok && robot_observer.tick(robot_observer.context,cycle);
        const bool present=ok && robot_observer.take_candidate(robot_observer.context,&candidate);
        /* Shared pipeline arms from measured joints on its first successful
         * tick. That tick intentionally has no motor candidate. */
        if (ok && !present && !robot_guidance_gate.have_previous) return true;
        ok=ok && present && hand_guiding_command_gate_validate(&robot_guidance_gate,
                &diagnostic_gate_limits,cycle,&candidate,targets);
    } else {
        if (xQueueReceive(robot_guided_targets,robot_requested_targets,0)==pdPASS) robot_have_target=true;
        for (unsigned j=0;j<6;j++) {
            const int32_t actual=cycle->feedback[j].actual_position_units;
            int64_t step=robot_have_target ? (int64_t)robot_requested_targets[j]-actual : 0;
            if (step>2000) step=2000;
            if (step< -2000) step= -2000;
            targets[j]=actual+(int32_t)step;
        }
    }
    if (!ok || !canopen_master_send_target_cycle(&can_boot_master,targets,6U)) {
        SupervisorMessage fault={.type=SUPERVISOR_MESSAGE_EVENT};
        fault.data.event.type=SUPERVISOR_EVENT_FAULT_DETECTED;
        fault.data.event.faultSeverity=ROBOT_FAULT_SEVERITY_RECOVERABLE;
        fault.data.event.faultCode=0xE061U;
        (void)supervisor_task_post(&fault,0);
        return false;
    }
    return true;
}
static void robot_guidance_stop(void *ctx)
{
    (void)ctx;
    StateMachine machine={0};
    if (supervisor_task_get_state(&machine) && machine.activeState!=ROBOT_STATE_TEACHING) {
        robot_have_target=false;
        if (robot_guided_targets) xQueueReset(robot_guided_targets);
    }
    hand_guiding_command_gate_reset(&robot_guidance_gate);
    if (robot_observer.stop) robot_observer.stop(robot_observer.context);
}
static void robot_configure_guidance(SupervisorTaskConfig *c)
{
    robot_guided_targets=xQueueCreate(1U,sizeof(int32_t)*6U);
    if (!robot_guided_targets) { fprintf(stderr,"Host guidance mailbox failed\n"); exit(1); }
    if (host_guidance_enabled) robot_observer=host_guidance_port(&host_guidance_demo);
    diagnostic_gate_limits.maximum_candidate_age_us=10000U;
    for (unsigned j=0;j<6;j++) {
        diagnostic_gate_limits.max_step_rad[j]=0.008;
        diagnostic_gate_limits.max_target_feedback_error_rad[j]=0.1;
        diagnostic_gate_limits.max_quantization_error_rad[j]=0.00001;
    }
    c->guidance_dry_run=(SupervisorGuidanceDryRunPort){.tick=robot_guidance_tick,
        .stop=robot_guidance_stop};
}
