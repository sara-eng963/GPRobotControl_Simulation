/* Execute the exact production wrapper, including private worker ownership.
 * Including this TU exposes private state without adding a firmware test API.
 * Only scheduling/IPC are mocked. --wrap supplies a numerical batch seam for
 * precise preemption at RUNNING/COMPLETE/FAILED publication boundaries; the
 * separate production validation fixture executes the real numerical engine.
 * These tests do not establish physical stopping or real-time deadlines.
 */
#ifndef ALIGNMENT_SUPERVISOR_SOURCE
#define ALIGNMENT_SUPERVISOR_SOURCE "supervisor_task.c"
#endif
#include ALIGNMENT_SUPERVISOR_SOURCE
#include "fake_joint_drive.h"
#include <setjmp.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); \
} } while (0)

static SupervisorTaskContext ctx;
static CanopenMaster master;
static FakeDrive drive;
static RobotConfig robot;
static AvatarMPositionScale scales[ROBOT_DOF];
static TeachingConfig teaching_config;
static TaughtProgram frozen_program;
static ValidatedTrajectory trajectory;
static ApproachConfig approach_config;
static ApproachServices approach_services;
static PathExecutionConfig execution_config;
static PathExecutionServices execution_services;
static HomingConfig homing_config;
static PausedServices paused_services;
static FaultServices fault_services;
static EmergencyStopServices emergency_services;
static unsigned critical_depth, stop_calls, off_calls, abort_calls, batches;
static bool stopped, stop_ok;
static TickType_t now_tick;
static jmp_buf worker_exit;
static bool in_worker, cancel_in_batch, cancel_at_delay;
static StateStepResult batch_result;

void alignment_enter_critical(void) { ++critical_depth; }
void alignment_exit_critical(void) { CHECK(critical_depth > 0); --critical_depth; }
TickType_t xTaskGetTickCount(void) { return now_tick; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return &ctx; }

/* All unused RTOS functions are discarded with --gc-sections. */
bool ipc_valid(const IpcShared *ipc) { return ipc != NULL; }
bool ipc_m7_send(IpcShared *ipc, const IpcMessage *msg)
{ (void)ipc; (void)msg; return false; }
bool ipc_m7_receive(IpcShared *ipc, IpcMessage *msg)
{ (void)ipc; (void)msg; return false; }

static bool owner(void *unused) { (void)unused; return true; }
static bool stop(bool *out, void *unused)
{ (void)unused; ++stop_calls; *out = stopped; return stop_ok; }
static bool relay(bool enabled, void *unused)
{ (void)unused; CHECK(!enabled); ++off_calls; return true; }
static bool safe(void *unused) { (void)unused; ++off_calls; return true; }
static bool read_sample(uint32_t index, PvExecutionSample *out, void *unused)
{ (void)unused; memset(out, 0, sizeof(*out)); (void)index; return true; }
static bool yes(void *unused) { (void)unused; return true; }
static StateStepResult complete(void *unused) { (void)unused; return STATE_STEP_COMPLETE; }

static void setup(RobotStateId state)
{
    memset(&ctx, 0, sizeof(ctx));
    memset(&master, 0, sizeof(master));
    memset(&trajectory, 0, sizeof(trajectory));
    memset(&frozen_program, 0, sizeof(frozen_program));
    memset(&g_timing, 0, sizeof(g_timing));
    state_machine_init(&ctx.machine);
    state_machine_enable_unified_execution(&ctx.machine);
    ctx.machine.activeState = state;
    ctx.entered_state = state;
    ctx.machine.safety = (RobotSafetySnapshot){.statusValid=true, .motionPermitted=true,
        .drivesReady=true, .communicationHealthy=true};
    fake_joint_drive_reset(&drive);
    ctx.joint_drive = fake_joint_drive_make_port(&drive);
    master.owner_check = owner;
    master.drive_command_permission = true;
#ifndef ALIGNMENT_DESKTOP
    ctx.restart_gate.authorized = true;
#endif
    teaching_config = (TeachingConfig){
        .default_speed_mps=0.010F, .minimum_speed_mps=0.001F,
        .maximum_speed_mps=0.1F, .speed_low_mps=0.005F,
        .speed_medium_mps=0.010F, .speed_high_mps=0.025F,
        .duplicate_position_tolerance_m=0.001F,
        .duplicate_orientation_tolerance_rad=0.01F,
        .minimum_point_separation_m=0.002F, .collinearity_epsilon_m2=1e-10F
    };
    execution_config = (PathExecutionConfig){.controlled_stop_timeout_ms=20U,
                                            .clearance_stable_cycles=1U};
    execution_services = (PathExecutionServices){.read_sample=read_sample,
        .set_wire_feed_enabled=relay, .controlled_stop=stop,
        .prepare_retraction=yes, .step_retraction=complete, .clearance_verified=yes};
    approach_services = (ApproachServices){.read_validated_sample=read_sample,
        .controlled_stop=stop, .controlled_stop_timeout_ms=20U};
    fault_services = (FaultServices){.force_safe_outputs=safe};
    emergency_services = (EmergencyStopServices){.force_safe_outputs=safe};
    paused_services = (PausedServices){.command_hold=stop, .wire_feed_off=safe};
    ctx.config = (SupervisorTaskConfig){.canopen_master=&master,
        .robot=&robot, .avatar_position_scales=scales,
        .teaching_config=&teaching_config, .validated_trajectory=&trajectory,
        .validation_worker_program=&frozen_program, .validation_sample_budget=1U,
        .path_execution_config=&execution_config, .path_execution_services=&execution_services,
        .approach_config=&approach_config, .approach_services=&approach_services,
        .homing_config=&homing_config, .paused_services=&paused_services,
        .fault_services=&fault_services, .emergency_stop_services=&emergency_services};
    state_teaching_enter(&ctx.teaching, &teaching_config, 42U);
    ctx.submitted_program.program_id = 42U;
    frozen_program.program_id = 42U;
    ctx.machine.recordedProgramAvailable = true;
    ctx.machine.validatedTrajectoryAvailable = true;
    ctx.machine.previewAccepted = true;
    ctx.machine.executionMode = ROBOT_EXECUTION_PREVIEW;
    ctx.approach_outputs.report.samples_sent = 9U;
    ctx.path_execution_outputs.sample_index = 9U;
    now_tick=0U; stop_calls=off_calls=abort_calls=batches=0U;
    stopped=false; stop_ok=true; cancel_in_batch=cancel_at_delay=in_worker=false;
    critical_depth=0U;
}

static void policy(SupervisorEventType type)
{
    SupervisorEvent event = make_result_event(type, ctx.machine.activeState,
        ROBOT_FAULT_SEVERITY_NONE, 0U);
    handle_policy_event(&ctx, &event);
}

static void check_revoked(void)
{
#ifndef ALIGNMENT_DESKTOP
    CHECK(!master.drive_command_permission && ctx.restart_gate.inhibited);
#endif
}

static void check_retained(void)
{
    CHECK(ctx.machine.recordedProgramAvailable && ctx.machine.validatedTrajectoryAvailable);
    CHECK(ctx.submitted_program.program_id == 42U);
    CHECK(ctx.path_execution_outputs.sample_index == 9U ||
          ctx.machine.activeState == ROBOT_STATE_PATH_EXECUTION);
}

static void check_cleared(void)
{
    CHECK(ctx.machine.activeState == ROBOT_STATE_IDLE);
    CHECK(!ctx.machine.recordedProgramAvailable && !ctx.machine.validatedTrajectoryAvailable);
    CHECK(!ctx.machine.previewAccepted && ctx.machine.executionMode == ROBOT_EXECUTION_NONE);
    CHECK(ctx.submitted_program.program_id == 0U);
    CHECK(ctx.teaching.draft.segment_count == 0U);
    CHECK(ctx.approach_outputs.report.samples_sent == 0U);
    CHECK(ctx.path_execution_outputs.sample_index == 0U);
}

static void test_hmi_and_homing(void)
{
    setup(ROBOT_STATE_TEACHING);
    const SupervisorHmiCommand commands[] = {SUP_HMI_SPEED_HIGH, SUP_HMI_SPEED_LOW,
                                            SUP_HMI_SPEED_MEDIUM};
    const TeachingEvent events[] = {TEACH_EVENT_SPEED_HIGH, TEACH_EVENT_SPEED_LOW,
                                    TEACH_EVENT_SPEED_MEDIUM};
    for (unsigned i=0; i<3U; ++i) {
        handle_hmi_command(&ctx, commands[i]);
        CHECK(ctx.pending_teaching_event == events[i]);
    }
    handle_hmi_command(&ctx, SUP_HMI_RESET);
    check_cleared();
    setup(ROBOT_STATE_IDLE);
    for (unsigned i=0; i<3U; ++i) {
        handle_hmi_command(&ctx, (SupervisorHmiCommand)i);
        CHECK(ctx.machine.activeState == ROBOT_STATE_IDLE);
        CHECK(ctx.machine.validatedTrajectoryAvailable && ctx.machine.previewAccepted);
    }
    handle_hmi_command(&ctx, SUP_HMI_RESET);
    check_cleared();
    setup(ROBOT_STATE_HOMING);
    ctx.homing.phase=HOMING_PHASE_EXECUTE_TRAJECTORY;
    ctx.homing.samplesSent=17U;
    ctx.homing.lastCommandMs=100U;
    const HomingState saved=ctx.homing;
    handle_hmi_command(&ctx, SUP_HMI_PAUSE);
    CHECK(ctx.machine.activeState == ROBOT_STATE_PAUSED);
    handle_hmi_command(&ctx, SUP_HMI_RESUME);
    CHECK(ctx.machine.activeState == ROBOT_STATE_PAUSED); /* Hold not confirmed. */
    ctx.paused.hold_established=true;
    handle_hmi_command(&ctx, SUP_HMI_RESUME);
    CHECK(ctx.machine.activeState == ROBOT_STATE_HOMING);
    CHECK(memcmp(&saved, &ctx.homing, sizeof(saved)) == 0);
    puts("PASS: production HMI presets, protected path, RESET, Homing hold/resume");
}

static void prepare_execution(void)
{
    trajectory.sample_count=1000U;
    trajectory.sample_period_us=2000U;
    ctx.path_execution.request.trajectory=&trajectory;
    ctx.path_execution.command_period_ms=2U;
    ctx.path_execution.initialized=true;
    ctx.path_execution.drive=ctx.joint_drive;
    ctx.path_execution.services=execution_services;
    ctx.path_execution.config=execution_config;
    ctx.path_execution.phase=PATH_EXEC_PHASE_FOLLOW_TRAJECTORY;
    ctx.path_execution.result=PATH_EXEC_RESULT_RUNNING;
    ctx.path_execution_inputs.motion_permission=true;
}

static void test_pending_reset_completion(void)
{
    for (unsigned reset=0U; reset<2U; ++reset) {
        setup(ROBOT_STATE_PATH_EXECUTION);
        prepare_execution();
        ctx.machine.previewAccepted=false;
        /* Real terminal latch: this step returns RUNNING even though the
         * module result becomes COMPLETE. Messages are drained next tick. */
        ctx.path_execution.phase=PATH_EXEC_PHASE_VERIFY_CLEARANCE;
        run_active_state(&ctx);
        CHECK(ctx.machine.activeState==ROBOT_STATE_PATH_EXECUTION);
        CHECK(ctx.path_execution.result==PATH_EXEC_RESULT_COMPLETE);
        const SupervisorMessage completion={.type=SUPERVISOR_MESSAGE_EVENT,
            .data.event={.type=SUPERVISOR_EVENT_STATE_COMPLETE,
                         .sourceState=ROBOT_STATE_PATH_EXECUTION}};
        if (!reset) {
            handle_message(&ctx, &completion);
            CHECK(ctx.machine.activeState==ROBOT_STATE_IDLE && ctx.machine.previewAccepted);
            continue;
        }
        const SupervisorMessage request={.type=SUPERVISOR_MESSAGE_HMI_COMMAND,
                                         .data.hmi_command=SUP_HMI_RESET};
        handle_message(&ctx, &request);
        const uint32_t rejections=ctx.machine.rejectedEventCount;
        handle_message(&ctx, &completion);
        CHECK(ctx.machine.activeState==ROBOT_STATE_PATH_EXECUTION);
        CHECK(ctx.path_execution_inputs.reset_requested && !ctx.machine.previewAccepted);
        CHECK(ctx.machine.rejectedEventCount==rejections+1U);
        CHECK(ctx.submitted_program.program_id==42U);
        run_active_state(&ctx);
        CHECK(ctx.path_execution.phase==PATH_EXEC_PHASE_CONTROLLED_STOP);
        CHECK(ctx.path_execution.result==PATH_EXEC_RESULT_RUNNING);
        CHECK(!ctx.path_execution_inputs.reset_requested);
        /* The request flag is now consumed; its old terminal result must
         * not authorize another queued completion while stopping. */
        handle_message(&ctx, &completion);
        CHECK(ctx.machine.activeState==ROBOT_STATE_PATH_EXECUTION);
        CHECK(ctx.machine.rejectedEventCount==rejections+2U);
        now_tick=1U; run_active_state(&ctx);
        CHECK(ctx.machine.activeState==ROBOT_STATE_PATH_EXECUTION && stop_calls==1U);
        CHECK(ctx.submitted_program.program_id==42U && !ctx.machine.previewAccepted);
        stopped=true; run_active_state(&ctx); run_active_state(&ctx);
        check_cleared();
    }
    puts("PASS: actual message routing preserves RESET before/after stop entry; ordinary completion works");
}

static void test_homing_long_pause(void)
{
    /* Resume 250 ms after the last command, beyond Homing's unchanged
     * 100-ms cyclic-feedback timeout. Exercise real steps and trajectory. */
    for (unsigned mode=0U; mode<6U; ++mode) {
        setup(ROBOT_STATE_HOMING);
        robot_config_init_ur5(&robot);
        robot.configuration.homeDefined=true;
        for (unsigned i=0; i<ROBOT_DOF; ++i) robot.configuration.home[i]=0.015;
        homing_config=(HomingConfig){.duration=0.05, .dt=0.002,
            .positionTolerance=0.001, .requiredStableCycles=2U, .maxVerificationCycles=10U};
        state_homing_enter(&ctx.homing);
        for (now_tick=0U; now_tick<4U; ++now_tick) run_active_state(&ctx);
        CHECK(ctx.machine.activeState==ROBOT_STATE_HOMING && ctx.homing.awaitingFeedback);
        CHECK(drive.commands==1U && ctx.homing.samplesSent==1U);
        const HomingState saved=ctx.homing;
        JointTrajectory next=ctx.homing.trajectory;
        JointTrajectorySample expected;
        CHECK(joint_trajectory_next(&next, &expected));
        handle_hmi_command(&ctx, SUP_HMI_PAUSE);
        run_active_state(&ctx); /* wire off */
        stopped=true; ++now_tick; run_active_state(&ctx); /* confirmed hold */
        CHECK(ctx.paused.hold_established);
        now_tick=253U; run_active_state(&ctx);
        CHECK(drive.commands==1U && memcmp(&saved, &ctx.homing, sizeof(saved))==0);
        if (mode!=0U) {
            for (unsigned i=0; i<ROBOT_DOF; ++i) {
                ++drive.sequence[i];
                drive.axis[i].timestamp_us=UINT64_C(253000);
                if (mode==1U || (mode==4U && i==4U))
                    drive.axis[i].timestamp_us=UINT64_C(5000); /* feedback from early in pause */
                if (mode==5U && i==4U)
                    drive.axis[i].timestamp_us=UINT64_C(254000); /* future timestamp */
            }
            if (mode==3U) drive.axis[2].feedback_valid=false;
        }
        handle_hmi_command(&ctx, SUP_HMI_RESUME);
        CHECK(ctx.machine.activeState==ROBOT_STATE_HOMING);
        CHECK(memcmp(&saved, &ctx.homing, sizeof(saved))==0);
        run_active_state(&ctx);
        if (mode==2U) {
            CHECK(ctx.machine.activeState==ROBOT_STATE_HOMING && drive.commands==2U);
            CHECK(ctx.homing.samplesSent==saved.samplesSent+1U);
            CHECK(joint_trajectory_samples_generated(&ctx.homing.trajectory)==
                  joint_trajectory_samples_generated(&saved.trajectory)+1U);
            for (unsigned i=0; i<ROBOT_DOF; ++i) {
                int32_t units;
                CHECK(avatar_m_joint_rad_to_position_units(&scales[i], expected.q.q[i], &units));
                CHECK(drive.last_targets[i]==units);
            }
            run_active_state(&ctx); /* same time: no catch-up command */
            CHECK(drive.commands==2U && ctx.homing.samplesSent==saved.samplesSent+1U);
        } else {
            CHECK(ctx.machine.activeState==ROBOT_STATE_FAULT && drive.commands==1U);
            CHECK(ctx.homing.samplesSent==saved.samplesSent);
            CHECK(joint_trajectory_samples_generated(&ctx.homing.trajectory)==
                  joint_trajectory_samples_generated(&saved.trajectory));
            CHECK(ctx.homing.error==(mode==3U ? HOMING_ERROR_POSITION_FEEDBACK
                                             : HOMING_ERROR_CYCLIC_FEEDBACK));
            check_revoked();
        }
    }
    puts("PASS: long Homing pause retains cursor; resume needs valid current feedback on every axis");
}

static void test_homing_pause_without_pending_feedback(void)
{
    for (unsigned verify=0U; verify<2U; ++verify)
    for (unsigned fresh=0U; fresh<2U; ++fresh) {
        setup(ROBOT_STATE_HOMING);
        robot_config_init_ur5(&robot);
        robot.configuration.homeDefined=true;
        for (unsigned i=0; i<ROBOT_DOF; ++i) robot.configuration.home[i]=0.015;
        homing_config=(HomingConfig){.duration=0.05, .dt=0.002,
            .positionTolerance=0.001, .requiredStableCycles=2U, .maxVerificationCycles=10U};
        state_homing_enter(&ctx.homing);
        for (now_tick=0U; now_tick<3U; ++now_tick) run_active_state(&ctx);
        if (verify) {
            drive.auto_feedback=true;
            while (now_tick<500U && ctx.homing.phase!=HOMING_PHASE_VERIFY_HOME) {
                run_active_state(&ctx); ++now_tick;
            }
            CHECK(ctx.homing.phase==HOMING_PHASE_VERIFY_HOME);
            drive.auto_feedback=false;
        }
        CHECK(!ctx.homing.awaitingFeedback);
        const HomingState saved=ctx.homing;
        const unsigned commands=drive.commands;
        JointTrajectory next=ctx.homing.trajectory;
        JointTrajectorySample expected;
        if (!verify) CHECK(joint_trajectory_next(&next, &expected));
        handle_hmi_command(&ctx, SUP_HMI_PAUSE);
        run_active_state(&ctx);
        stopped=true; ++now_tick; run_active_state(&ctx);
        CHECK(ctx.paused.hold_established);
        now_tick+=250U;
        if (fresh) for (unsigned i=0; i<ROBOT_DOF; ++i) {
            ++drive.sequence[i];
            drive.axis[i].timestamp_us=(uint64_t)now_tick*1000U;
        }
        handle_hmi_command(&ctx, SUP_HMI_RESUME);
        CHECK(memcmp(&saved, &ctx.homing, sizeof(saved))==0);
        run_active_state(&ctx);
        if (fresh) {
            CHECK(ctx.machine.activeState==ROBOT_STATE_HOMING && drive.commands==commands+1U);
            CHECK(ctx.homing.samplesSent==saved.samplesSent+(verify ? 0U : 1U));
            for (unsigned i=0; i<ROBOT_DOF; ++i) {
                int32_t units;
                CHECK(avatar_m_joint_rad_to_position_units(&scales[i],
                    verify ? saved.qHome.q[i] : expected.q.q[i], &units));
                CHECK(drive.last_targets[i]==units);
            }
        } else {
            CHECK(ctx.machine.activeState==ROBOT_STATE_FAULT && drive.commands==commands);
            CHECK(ctx.homing.error==HOMING_ERROR_CYCLIC_FEEDBACK);
            CHECK(ctx.homing.samplesSent==saved.samplesSent);
            check_revoked();
        }
    }
    puts("PASS: long pause before first Homing sample or verification hold also requires fresh feedback");
}

static void test_path_reset(void)
{
    setup(ROBOT_STATE_PATH_EXECUTION);
    prepare_execution();
    handle_hmi_command(&ctx, SUP_HMI_RESET);
    check_retained();
    /* Later input snapshots and HOME must not erase the accepted RESET. */
    SupervisorMessage input={.type=SUPERVISOR_MESSAGE_PATH_EXECUTION_INPUTS};
    handle_message(&ctx, &input);
    handle_hmi_command(&ctx, SUP_HMI_HOME);
    handle_hmi_command(&ctx, SUP_HMI_PAUSE);
    CHECK(ctx.machine.activeState == ROBOT_STATE_PATH_EXECUTION);
    CHECK(ctx.path_execution_inputs.reset_requested);
    run_active_state(&ctx);
    CHECK(ctx.path_execution.phase == PATH_EXEC_PHASE_CONTROLLED_STOP);
    check_retained();
    policy(SUPERVISOR_EVENT_STATE_ABORTED_RESET);
    policy(SUPERVISOR_EVENT_STATE_COMPLETE);
    CHECK(ctx.machine.activeState == ROBOT_STATE_PATH_EXECUTION);
    check_retained();
    for (now_tick=1U; now_tick<5U; ++now_tick) {
        handle_hmi_command(&ctx, SUP_HMI_PAUSE);
        run_active_state(&ctx);
        CHECK(ctx.machine.activeState == ROBOT_STATE_PATH_EXECUTION);
        check_retained();
    }
    CHECK(stop_calls == 4U && off_calls > 0U && drive.commands == 0U);
    stopped=true;
    run_active_state(&ctx);
    run_active_state(&ctx); /* Terminal result consumption if needed. */
    check_cleared();

    setup(ROBOT_STATE_PATH_EXECUTION);
    prepare_execution();
    handle_hmi_command(&ctx, SUP_HMI_HOME);
    run_active_state(&ctx);
    const uint32_t deadline_start=ctx.path_execution.phase_started_ms;
    now_tick=3U;
    handle_hmi_command(&ctx, SUP_HMI_RESET);
    run_active_state(&ctx);
    CHECK(ctx.path_execution.abort_reason == PATH_EXEC_ABORT_RESET);
    CHECK(ctx.path_execution.phase_started_ms == deadline_start);
    stopped=true;
    run_active_state(&ctx); run_active_state(&ctx);
    check_cleared();

    setup(ROBOT_STATE_PATH_EXECUTION);
    prepare_execution();
    handle_hmi_command(&ctx, SUP_HMI_RESET); run_active_state(&ctx);
    now_tick=20U;
    run_active_state(&ctx);
    CHECK(ctx.machine.activeState == ROBOT_STATE_FAULT);
    check_revoked();
    CHECK(ctx.submitted_program.program_id == 42U); /* Failure isn't RESET completion. */
    handle_hmi_command(&ctx, SUP_HMI_RESET);
#ifndef ALIGNMENT_DESKTOP
    CHECK(ctx.machine.activeState == ROBOT_STATE_FAULT);
    CHECK(!master.drive_command_permission);
#endif
    puts("PASS: production RESET waits for stop; HOME upgrade, timeout and revocation");
}

static void test_approach_reset(void)
{
    setup(ROBOT_STATE_APPROACH);
    ctx.approach.initialized=true;
    ctx.approach.drive=ctx.joint_drive;
    ctx.approach.position_scales=scales;
    ctx.approach.services=approach_services;
    ctx.approach.phase=APPROACH_PHASE_EXECUTE_LEG;
    ctx.approach.result=APPROACH_RESULT_RUNNING;
    handle_hmi_command(&ctx, SUP_HMI_RESET);
    SupervisorMessage input={.type=SUPERVISOR_MESSAGE_APPROACH_CONTROL};
    handle_message(&ctx, &input);
    CHECK(ctx.approach_control.reset_requested);
    run_active_state(&ctx);
    CHECK(ctx.approach.phase == APPROACH_PHASE_CONTROLLED_STOP);
    check_retained();
    now_tick=2U; run_active_state(&ctx);
    CHECK(ctx.machine.activeState == ROBOT_STATE_APPROACH && stop_calls == 1U);
    check_retained();
    stopped=true; run_active_state(&ctx);
    check_cleared();

    setup(ROBOT_STATE_APPROACH);
    ctx.approach.initialized=true;
    ctx.approach.drive=ctx.joint_drive;
    ctx.approach.position_scales=scales;
    ctx.approach.services=approach_services;
    ctx.approach.phase=APPROACH_PHASE_EXECUTE_LEG;
    ctx.approach.result=APPROACH_RESULT_RUNNING;
    handle_hmi_command(&ctx, SUP_HMI_HOME); run_active_state(&ctx);
    now_tick=3U;
    handle_hmi_command(&ctx, SUP_HMI_RESET); run_active_state(&ctx);
    CHECK(ctx.approach.error == APPROACH_ERR_RESET_REQUESTED);
    CHECK(ctx.approach.stop_started_ms == 0U);
    now_tick=20U; run_active_state(&ctx);
    CHECK(ctx.machine.activeState == ROBOT_STATE_FAULT);
    CHECK(ctx.approach.error == APPROACH_ERR_STOP_TIMEOUT);
    CHECK(ctx.submitted_program.program_id == 42U);
    check_revoked();
    puts("PASS: production Approach does not acknowledge an incomplete stop");
}

static void worker_abort(void *unused)
{ (void)unused; CHECK(in_worker); ++abort_calls; }

StateStepResult __wrap_state_path_validation_step(PathValidationState *state,
    uint16_t budget, PathValidationOutputs *out)
{
    CHECK(in_worker && budget == 1U && state == &ctx.validation);
    ++batches;
    state->result = batch_result == STATE_STEP_COMPLETE ? PV_RESULT_VALID :
                    (batch_result == STATE_STEP_RUNNING ? PV_RESULT_RUNNING : PV_RESULT_INVALID);
    state->storage_open=batch_result == STATE_STEP_RUNNING;
    state->storage.abort=worker_abort;
    state->sample_index=123U;
    out->report.result=batch_result == STATE_STEP_COMPLETE ? PV_RESULT_VALID : PV_RESULT_RUNNING;
    out->trajectory_ready=batch_result == STATE_STEP_COMPLETE;
    if (cancel_in_batch) {
        const PathValidationState saved=*state;
        const TaughtProgram frozen=frozen_program;
        handle_hmi_command(&ctx, SUP_HMI_RESET); /* Higher-priority preemption. */
        CHECK(memcmp(state, &saved, sizeof(saved)) == 0);
        CHECK(memcmp(&frozen, &frozen_program, sizeof(frozen)) == 0);
        CHECK(ctx.validation_worker_state == VALIDATION_WORKER_RUNNING);
    }
    return batch_result;
}

void vTaskDelay(TickType_t ticks)
{
    CHECK(in_worker && ticks == 1U && critical_depth == 0U);
    if (cancel_at_delay && batches > 0U) {
        cancel_at_delay=false;
        handle_hmi_command(&ctx, SUP_HMI_RESET);
    }
    if (ctx.validation_worker_state == VALIDATION_WORKER_IDLE ||
        ctx.validation_worker_state == VALIDATION_WORKER_DONE)
        longjmp(worker_exit, 1);
    CHECK(++now_tick < 10U);
}

static void run_worker(void)
{
    in_worker=true;
    if (setjmp(worker_exit) == 0) validation_worker_entry(&ctx);
    in_worker=false;
    CHECK(critical_depth == 0U);
}

static void test_cancellation(void)
{
    const StateStepResult results[]={STATE_STEP_RUNNING, STATE_STEP_COMPLETE, STATE_STEP_FAILED};
    for (unsigned i=0; i<3U; ++i) {
        setup(ROBOT_STATE_PATH_VALIDATION);
        ctx.config.validation_run_in_worker=true;
        ctx.validation_worker_state=VALIDATION_WORKER_QUEUED;
        cancel_in_batch=true; batch_result=results[i];
        run_worker();
        CHECK(ctx.validation_worker_state == VALIDATION_WORKER_IDLE);
        CHECK(!ctx.validation_outputs.trajectory_ready);
        CHECK(ctx.validation_outputs.report.result == 0);
        CHECK(abort_calls == (results[i] == STATE_STEP_RUNNING ? 1U : 0U));
        CHECK(batches == 1U);
        check_cleared();
    }
    setup(ROBOT_STATE_PATH_VALIDATION);
    ctx.config.validation_run_in_worker=true;
    ctx.validation_worker_state=VALIDATION_WORKER_QUEUED;
    cancel_at_delay=true; batch_result=STATE_STEP_RUNNING;
    run_worker();
    CHECK(ctx.validation_worker_state == VALIDATION_WORKER_IDLE && abort_calls == 1U);
    CHECK(!ctx.validation_outputs.trajectory_ready);
    check_cleared();
    setup(ROBOT_STATE_PATH_VALIDATION);
    ctx.config.validation_run_in_worker=true;
    ctx.validation_worker_state=VALIDATION_WORKER_QUEUED;
    handle_hmi_command(&ctx, SUP_HMI_RESET);
    run_worker();
    CHECK(batches == 0U && abort_calls == 0U);
    CHECK(ctx.validation_worker_state == VALIDATION_WORKER_IDLE);

    setup(ROBOT_STATE_PATH_VALIDATION);
    ctx.config.validation_run_in_worker=true;
    ctx.validation_worker_state=VALIDATION_WORKER_QUEUED;
    ctx.machine.validatedTrajectoryAvailable=false;
    batch_result=STATE_STEP_COMPLETE;
    run_worker();
    CHECK(ctx.validation_worker_state == VALIDATION_WORKER_DONE);
    CHECK(ctx.validation_outputs.trajectory_ready && abort_calls == 0U);
    run_active_state(&ctx);
    CHECK(ctx.machine.activeState == ROBOT_STATE_IDLE);
    CHECK(ctx.machine.validatedTrajectoryAvailable && !ctx.machine.previewAccepted);
    puts("PASS: production worker cancellation before/between/during terminal and active batches");
}

static void test_completion_and_feedback(void)
{
    setup(ROBOT_STATE_PATH_EXECUTION);
    prepare_execution();
    ctx.path_execution.phase=PATH_EXEC_PHASE_COMPLETE;
    ctx.path_execution.result=PATH_EXEC_RESULT_COMPLETE;
    run_active_state(&ctx);
    CHECK(ctx.machine.activeState == ROBOT_STATE_IDLE && ctx.machine.previewAccepted);
    CHECK(ctx.machine.validatedTrajectoryAvailable);
    setup(ROBOT_STATE_PATH_EXECUTION);
    prepare_execution();
    ctx.path_execution.feedback_watch_started=true;
    ctx.path_execution.feedback_watch_started_us=0U;
    for (unsigned i=0; i<JOINT_DRIVE_AXES; ++i)
        ctx.path_execution.initial_tpdo_count[i]=drive.axis[i].sequence;
    stopped=true;
    now_tick=21U;
    run_active_state(&ctx);
    CHECK(ctx.path_execution.error == PATH_EXEC_ERR_CYCLIC_FEEDBACK);
    CHECK(ctx.machine.activeState == ROBOT_STATE_FAULT && drive.commands == 0U);
    check_revoked();
    puts("PASS: production path completion returns Idle; 21-ms stale feedback faults");
}

int main(void)
{
    robot_config_init_ur5(&robot);
    for (unsigned i=0; i<ROBOT_DOF; ++i)
        CHECK(avatar_m_position_scale_default(&scales[i], 50.0));
    CHECK(SUP_HMI_SPEED_LOW == 9 && SUP_HMI_SPEED_MEDIUM == 10 && SUP_HMI_SPEED_HIGH == 11);
    test_hmi_and_homing();
    test_pending_reset_completion();
    test_homing_long_pause();
    test_homing_pause_without_pending_feedback();
    test_path_reset();
    test_approach_reset();
    test_cancellation();
    test_completion_and_feedback();
    puts("PASS: exact production Supervisor behavioral regressions (mock scheduling)");
    return 0;
}
