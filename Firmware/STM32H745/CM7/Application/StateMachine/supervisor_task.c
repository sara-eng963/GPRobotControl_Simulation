#include "supervisor_task.h"
#include "../ServoDrive/JointDrive/canopen_joint_drive_port.h"
#include "../ServoDrive/JointDrive/canopen_drive_commissioning_port.h"

#include <stdio.h>
#include <string.h>

#if defined(HOST_TIMING)
#include "../Simulation/Timing/host_timing_trace.h"
#define TRACE_BEGIN(source) timing_begin(source)
#define TRACE_END(source, success) timing_end(source, success)
#else
#define TRACE_BEGIN(source) ((void)0)
#define TRACE_END(source, success) ((void)0)
#endif

#ifndef SUPERVISOR_DEBUG
#define SUPERVISOR_DEBUG 0
#endif

#if SUPERVISOR_DEBUG
#define SUPERVISOR_LOG(...) printf(__VA_ARGS__)
#else
#define SUPERVISOR_LOG(...) ((void)0)
#endif

#define SUPERVISOR_QUEUE_LENGTH       32U
#define SUPERVISOR_TASK_STACK_WORDS   4096U
#define VALIDATION_TASK_STACK_WORDS   4096U
#define SUPERVISOR_MAX_MESSAGES_CYCLE 16U

typedef enum {
    VALIDATION_WORKER_IDLE = 0,
    VALIDATION_WORKER_QUEUED,
    VALIDATION_WORKER_RUNNING,
    VALIDATION_WORKER_DONE
} ValidationWorkerState;

/* Real M7 installs a timer port; PC tests keep their RTOS-tick clock. */
static SupervisorTimingPort g_timing;

static uint64_t supervisor_now_us(void)
{
    if (g_timing.monotonic_us != NULL)
        return g_timing.monotonic_us(g_timing.context);
    return (uint64_t)xTaskGetTickCount() * UINT64_C(1000000) /
           (uint64_t)configTICK_RATE_HZ;
}

static uint32_t supervisor_now_ms(void)
{
    return (uint32_t)(supervisor_now_us() / UINT64_C(1000));
}

typedef struct
{
    StateMachine machine;
    SupervisorInputSnapshot gpio_inputs;
    SupervisorDriveSnapshot drive_snapshot;
    uint32_t last_m4_ipc_sequence;
    uint32_t m7_status_ipc_sequence;
    uint32_t last_m7_ipc_status_ms;
    bool m7_ipc_status_sent;
    SupervisorTaskConfig config;

    BootState boot;
    HomingState homing;
    IdleState idle;
    TeachingState teaching;
    PathValidationState validation;
    ApproachState approach;
    PathExecutionState path_execution;
    JointDrivePort joint_drive;
    DriveCommissioningPort commissioning;
    PausedState paused;
    FaultState fault;
    EmergencyStopState emergency_stop;

    TeachingRuntimeInputs teaching_runtime;
    TeachingEvent pending_teaching_event;
    ApproachControlInputs approach_control;
    PathExecutionInputs path_execution_inputs;

    TeachingOutputs teaching_outputs;
    PathValidationOutputs validation_outputs;
    ApproachOutputs approach_outputs;
    PathExecutionOutputs path_execution_outputs;

    TaughtProgram submitted_program;

    RobotStateId entered_state;
    uint32_t next_program_id;
    bool initialized;
    bool execution_stream_retained;
    uint32_t timer_missed_periods;
    uint32_t timer_timeouts;
    uint32_t timer_overruns;
    uint32_t timer_max_cycle_us;
    uint32_t timer_service_gap_warnings;
    uint32_t timer_service_gap_severe;
    uint32_t timer_wake_gap_warnings;
    uint32_t timer_wake_gap_severe;
    uint32_t timer_late_wakeups;
    uint32_t timer_deadline_misses;
    uint32_t timer_invalid_timestamps;
    uint32_t timer_max_isr_to_wake_us;
    uint32_t timer_max_isr_to_completion_us;
    uint32_t timer_max_wake_gap_us;
    uint32_t guidance_dry_run_ok_cycles;
    uint32_t guidance_dry_run_rejected_cycles;
    SupervisorTimingWake last_timing_wake;
    uint64_t previous_timing_wake_us;
    bool last_timing_wake_available;
    bool previous_timing_wake_valid;
    bool fault_response_step_ok;
    SupervisionProgressSample progress;
    DriveRestartGate restart_gate;
    RuntimeSafetyEvidence safety_evidence;
    RuntimePeerEvidence peer_evidence;

    /* Only the validation worker mutates 'validation' or flash storage when
     * async mode is enabled. Supervisor reads exclusively a copied result.
     * The draft is frozen here until the worker acknowledges cancellation. */
    ValidationWorkerState validation_worker_state;
    bool validation_worker_cancel;
} SupervisorTaskContext;

static SupervisorTaskContext g_supervisor;

#if (configSUPPORT_STATIC_ALLOCATION == 1)
static StaticQueue_t g_queue_control;
static uint8_t g_queue_storage[
    SUPERVISOR_QUEUE_LENGTH * sizeof(SupervisorMessage)
];
#endif
static QueueHandle_t g_queue;

#if (configSUPPORT_STATIC_ALLOCATION == 1)
static StaticTask_t g_task_control;
static StackType_t g_task_stack[SUPERVISOR_TASK_STACK_WORDS];
#endif
static TaskHandle_t g_task;
static TaskHandle_t g_validation_task;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
static StaticTask_t g_validation_task_control;
static StackType_t g_validation_task_stack[VALIDATION_TASK_STACK_WORDS];
#endif

/* The lower-priority validation worker owns numerical validation state.
 * With a synchronous backend it also owns storage. With the optional async
 * M7/M4 bridge, callbacks only enqueue samples and poll durable status;
 * a separate M4-role task owns all flash erase/program/commit operations.
 * Cancellation is cooperative and never interrupts in-flight NOR I/O. */
static void validation_worker_entry(void *argument)
{
    SupervisorTaskContext *context = (SupervisorTaskContext *)argument;
    for (;;) {
        bool start = false;
        taskENTER_CRITICAL();
        if (context->validation_worker_state == VALIDATION_WORKER_QUEUED) {
            if (context->validation_worker_cancel) {
                context->validation_worker_state = VALIDATION_WORKER_IDLE;
            } else {
                context->validation_worker_state = VALIDATION_WORKER_RUNNING;
                start = true;
            }
        }
        taskEXIT_CRITICAL();
        if (!start) {
            vTaskDelay(1);
            continue;
        }

        state_path_validation_enter(
            &context->validation,
            context->config.robot,
            context->config.avatar_position_scales,
            context->config.validation_config,
            context->config.validation_services,
            context->config.validation_workspace,
            context->config.validation_storage,
            context->config.validated_trajectory,
            context->config.validation_worker_program,
            context->config.validation_worker_program->draft_revision,
            context->config.validation_worker_program->draft_crc);

        for (;;) {
            bool cancel;
            taskENTER_CRITICAL();
            cancel = context->validation_worker_cancel;
            taskEXIT_CRITICAL();
            if (cancel) {
                /* Never race the same flash device with another validation.
                 * Abort is called by this sole storage owner, not Supervisor. */
                state_path_validation_cancel(&context->validation);
                taskENTER_CRITICAL();
                context->validation_worker_state = VALIDATION_WORKER_IDLE;
                taskEXIT_CRITICAL();
                break;
            }

            PathValidationOutputs snapshot = {0};
            TRACE_BEGIN(TIMING_VALIDATION);
            StateStepResult result = state_path_validation_step(
                &context->validation,
                context->config.validation_sample_budget,
                &snapshot);
            TRACE_END(TIMING_VALIDATION, result != STATE_STEP_FAILED);
            taskENTER_CRITICAL();
            const bool cancelled = context->validation_worker_cancel;
            if (!cancelled) {
                context->validation_outputs = snapshot;
                if (result != STATE_STEP_RUNNING)
                    context->validation_worker_state = VALIDATION_WORKER_DONE;
            }
            taskEXIT_CRITICAL();
            if (cancelled) {
                state_path_validation_cancel(&context->validation);
                taskENTER_CRITICAL();
                context->validation_worker_state = VALIDATION_WORKER_IDLE;
                taskEXIT_CRITICAL();
                break;
            }
            if (result != STATE_STEP_RUNNING)
                break;
            /* Yield after one bounded numerical batch. Async storage may
             * report BUSY; M4 continues flash IO without blocking M7. */
            vTaskDelay(1);
        }
    }
}

static uint32_t state_error_code(RobotStateId state, uint32_t detail)
{
    return (((uint32_t)state & 0xFFU) << 16U) | (detail & 0xFFFFU);
}

static SupervisorEvent make_result_event(
    SupervisorEventType type,
    RobotStateId source,
    RobotFaultSeverity severity,
    uint32_t fault_code
)
{
    SupervisorEvent event;
    memset(&event, 0, sizeof(event));
    event.type = type;
    event.sourceState = source;
    event.faultSeverity = severity;
    event.faultCode = fault_code;
    return event;
}

static void enter_active_state(SupervisorTaskContext *context)
{
    StateMachine *machine = &context->machine;

    if (context->entered_state == machine->activeState)
    {
        return;
    }

    /* Dispatch and consumption share this task: on exit no take is in flight.
     * PAUSED retains the stream; sample count is never proof of quiescence. */
    if (context->execution_stream_retained &&
        machine->activeState != ROBOT_STATE_PATH_EXECUTION &&
        machine->activeState != ROBOT_STATE_PAUSED) {
        state_path_execution_release_stream(&context->path_execution);
        context->execution_stream_retained = false;
    }
    /* An exited validation must be allowed to finish/cancel on its owner.
     * Do not reset its storage from another task or overwrite its draft. */
    if (context->config.validation_run_in_worker &&
        context->entered_state == ROBOT_STATE_PATH_VALIDATION &&
        machine->activeState != ROBOT_STATE_PATH_VALIDATION) {
        taskENTER_CRITICAL();
        if (context->validation_worker_state == VALIDATION_WORKER_QUEUED ||
            context->validation_worker_state == VALIDATION_WORKER_RUNNING)
            context->validation_worker_cancel = true;
        taskEXIT_CRITICAL();
    }

    /* A subsequent validation is deferred until the previous worker has
     * reached a safe quiescent boundary, with no Supervisor-side waiting. */
    if (context->config.validation_run_in_worker &&
        machine->activeState == ROBOT_STATE_PATH_VALIDATION) {
        bool busy;
        taskENTER_CRITICAL();
        busy = context->validation_worker_state == VALIDATION_WORKER_QUEUED ||
               context->validation_worker_state == VALIDATION_WORKER_RUNNING;
        taskEXIT_CRITICAL();
        if (busy) return;
    }

    /* Stop acknowledgment may take an arbitrary flash read duration.
     * Never block this control task while waiting for it. */
    if (machine->activeState == ROBOT_STATE_PATH_VALIDATION &&
        context->config.validation_enter_ready != NULL &&
        !context->config.validation_enter_ready(
            context->config.validation_enter_context))
        return;

    context->entered_state = machine->activeState;
    SUPERVISOR_LOG("[SUPERVISOR] Enter %s\n",
                   state_machine_state_name(machine->activeState));

    switch (machine->activeState)
    {
        case ROBOT_STATE_BOOT:
            state_boot_enter(&context->boot);
            break;

        case ROBOT_STATE_HOMING:
            if (machine->previousState == ROBOT_STATE_PAUSED)
                break;
            state_homing_enter(&context->homing);
            break;

        case ROBOT_STATE_IDLE:
            state_idle_enter(&context->idle);
            break;

        case ROBOT_STATE_TEACHING:
            context->pending_teaching_event = TEACH_EVENT_NONE;
            state_teaching_enter(
                &context->teaching,
                context->config.teaching_config,
                context->next_program_id++
            );
            break;

        case ROBOT_STATE_PATH_VALIDATION:
            if (context->config.validation_run_in_worker) {
                /* State, workspace and QSPI transport have one owner: the
                 * validation worker. Publish request after draft copy. */
                *context->config.validation_worker_program = context->submitted_program;
                taskENTER_CRITICAL();
                memset(&context->validation_outputs, 0,
                       sizeof(context->validation_outputs));
                context->validation_outputs.report.result = PV_RESULT_RUNNING;
                context->validation_worker_cancel = false;
                context->validation_worker_state = VALIDATION_WORKER_QUEUED;
                taskEXIT_CRITICAL();
                break;
            }
            state_path_validation_enter(
                &context->validation,
                context->config.robot,
                context->config.avatar_position_scales,
                context->config.validation_config,
                context->config.validation_services,
                context->config.validation_workspace,
                context->config.validation_storage,
                context->config.validated_trajectory,
                &context->submitted_program,
                context->submitted_program.draft_revision,
                context->submitted_program.draft_crc
            );
            break;

        case ROBOT_STATE_APPROACH:
        {
            if (machine->previousState == ROBOT_STATE_PAUSED &&
                context->approach.initialized)
            {
                break;
            }
            ApproachRequest request;
            memset(&request, 0, sizeof(request));

            request.operation =
                (machine->executionMode == ROBOT_EXECUTION_PREVIEW)
                    ? APPROACH_OPERATION_PREVIEW
                    : APPROACH_OPERATION_WELD;
            request.trajectory = context->config.validated_trajectory;
            request.trajectory_ready =
                machine->validatedTrajectoryAvailable;
            request.expected_program_id =
                context->config.validated_trajectory->program_id;
            request.expected_source_revision =
                context->config.validated_trajectory->source_revision;
            request.expected_artifact_crc =
                context->config.validated_trajectory->artifact_crc;

            memset(&context->approach_control, 0,
                   sizeof(context->approach_control));
            state_approach_enter(
                &context->approach,
                context->config.robot,
                &context->joint_drive,
                context->config.avatar_position_scales,
                &request,
                context->config.approach_config,
                context->config.approach_services
            );
            break;
        }

        case ROBOT_STATE_PATH_EXECUTION:
        {
            context->execution_stream_retained = true;
            if (machine->previousState == ROBOT_STATE_PAUSED &&
                context->path_execution.initialized)
            {
                state_path_execution_resume(
                    &context->path_execution,
                    context->path_execution_inputs.now_ms
                );
                break;
            }
            PathExecutionRequest request;
            memset(&request, 0, sizeof(request));
            request.mode = machine->executionMode;
            request.trajectory = context->config.validated_trajectory;
            request.trajectory_ready = machine->validatedTrajectoryAvailable;
            request.expected_program_id =
                context->config.validated_trajectory->program_id;
            request.expected_source_revision =
                context->config.validated_trajectory->source_revision;
            request.expected_artifact_crc =
                context->config.validated_trajectory->artifact_crc;
            memset(&context->path_execution_inputs, 0,
                   sizeof(context->path_execution_inputs));
            state_path_execution_enter(
                &context->path_execution,
                &context->joint_drive,
                &request,
                context->config.path_execution_config,
                context->config.path_execution_services
            );
            break;
        }

        case ROBOT_STATE_PAUSED:
            state_paused_enter(
                &context->paused,
                machine->resumeState,
                context->config.paused_services
            );
            break;

        case ROBOT_STATE_FAULT:
            state_fault_enter(
                &context->fault,
                machine->previousState,
                machine->faultSeverity,
                machine->activeFaultCode,
                true,
                context->config.fault_services
            );
            break;

        case ROBOT_STATE_EMERGENCY_STOP:
            state_emergency_stop_enter(
                &context->emergency_stop,
                machine->previousState,
                machine->safety.timestampMs,
                context->config.emergency_stop_services
            );
            break;

        default:
            /* Later state modules are added here without changing policy. */
            break;
    }
}

static void clear_runtime_program_context(
    SupervisorTaskContext *context
)
{
    state_teaching_reset_draft(
        &context->teaching
    );

    state_teaching_get_outputs(
        &context->teaching,
        &context->teaching_outputs
    );

    memset(
        &context->submitted_program,
        0,
        sizeof(context->submitted_program)
    );

    /* Only the validation worker may mutate its state. */
    if (!context->config.validation_run_in_worker)
    {
        state_path_validation_cancel(&context->validation);
        memset(&context->validation, 0, sizeof(context->validation));
        memset(&context->validation_outputs, 0,
               sizeof(context->validation_outputs));
    }

    memset(
        &context->approach_outputs,
        0,
        sizeof(context->approach_outputs)
    );

    memset(
        &context->path_execution_outputs,
        0,
        sizeof(context->path_execution_outputs)
    );

    if (context->config.validation_run_in_worker)
    {
        taskENTER_CRITICAL();
        context->validation_worker_cancel = true;
        memset(&context->validation_outputs, 0,
               sizeof(context->validation_outputs));
        if (context->validation_worker_state !=
            VALIDATION_WORKER_RUNNING)
        {
            context->validation_worker_state = VALIDATION_WORKER_IDLE;
        }
        taskEXIT_CRITICAL();
    }
}

static void handle_policy_event(
    SupervisorTaskContext *context,
    const SupervisorEvent *event
)
{
    /* Result envelopes cannot substitute for the owning motion module's
     * acknowledgement. A queued completion must not overtake an accepted
     * RESET/HOME, even if COMPLETE was latched before this message batch. */
    if (event->type == SUPERVISOR_EVENT_STATE_ABORTED_RESET ||
        event->type == SUPERVISOR_EVENT_STATE_ABORTED_HOME ||
        event->type == SUPERVISOR_EVENT_STATE_COMPLETE) {
        const bool complete = event->type == SUPERVISOR_EVENT_STATE_COMPLETE;
        const bool reset = event->type == SUPERVISOR_EVENT_STATE_ABORTED_RESET;
        bool acknowledged = true;
        if (context->machine.activeState == ROBOT_STATE_APPROACH)
            acknowledged = complete
                ? !context->approach_control.reset_requested &&
                  !context->approach_control.home_requested &&
                  context->approach.result == APPROACH_RESULT_COMPLETE
                : (context->approach.result == APPROACH_RESULT_ABORTED &&
                   context->approach.error == (reset ? APPROACH_ERR_RESET_REQUESTED
                                                    : APPROACH_ERR_HOME_REQUESTED));
        if (context->machine.activeState == ROBOT_STATE_PATH_EXECUTION)
            acknowledged = complete
                ? !context->path_execution_inputs.reset_requested &&
                  !context->path_execution_inputs.home_requested &&
                  context->path_execution.result == PATH_EXEC_RESULT_COMPLETE
                : (context->path_execution.result == PATH_EXEC_RESULT_ABORTED &&
                   context->path_execution.abort_reason == (reset ? PATH_EXEC_ABORT_RESET
                                                                 : PATH_EXEC_ABORT_HOME));
        if (!acknowledged) {
            ++context->machine.rejectedEventCount;
            return;
        }
    }

    if (event->type == SUPERVISOR_EVENT_PAUSE_RESUME &&
        ((context->machine.activeState == ROBOT_STATE_PAUSED &&
          !context->paused.hold_established) ||
         (context->machine.activeState == ROBOT_STATE_APPROACH &&
          (context->approach_control.reset_requested ||
           context->approach_control.home_requested ||
           context->approach.phase == APPROACH_PHASE_CONTROLLED_STOP)) ||
         (context->machine.activeState == ROBOT_STATE_PATH_EXECUTION &&
          (context->path_execution_inputs.reset_requested ||
           context->path_execution_inputs.home_requested ||
           context->path_execution.phase == PATH_EXEC_PHASE_CONTROLLED_STOP)))) {
        ++context->machine.rejectedEventCount;
        return;
    }

    /*
     * HOME/RESET during motion are requests to the active module first.
     * The global transition is performed only after that module confirms a
     * controlled stop with STATE_ABORTED_HOME/RESET.
     */
    if (event->type == SUPERVISOR_EVENT_RESET ||
        event->type == SUPERVISOR_EVENT_HOME)
    {
        const bool reset = event->type == SUPERVISOR_EVENT_RESET;
        if (context->machine.activeState == ROBOT_STATE_APPROACH)
        {
            context->approach_control.reset_requested |= reset;
            context->approach_control.home_requested |= !reset;
            return;
        }
        if (context->machine.activeState == ROBOT_STATE_PATH_EXECUTION)
        {
            context->path_execution_inputs.reset_requested |= reset;
            context->path_execution_inputs.home_requested |= !reset;
            return;
        }
    }

    if (event->type == SUPERVISOR_EVENT_RESET &&
        (context->machine.activeState == ROBOT_STATE_FAULT ||
         context->machine.activeState == ROBOT_STATE_EMERGENCY_STOP) &&
        !context->restart_gate.authorized)
    {
        /* A state-machine RESET cannot clear a revoked boot authorization. */
        return;
    }

    /* Revoke synchronously, before a later RESET/HMI queue message can
     * restore a motion state in this same 500 Hz Supervisor cycle. */
    if (event->type == SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED ||
        event->type == SUPERVISOR_EVENT_ESTOP_ASSERTED ||
        event->type == SUPERVISOR_EVENT_FAULT_DETECTED ||
        event->type == SUPERVISOR_EVENT_STATE_FAILED)
    {
        drive_restart_gate_inhibit(&context->restart_gate);
        context->boot.drive_enable_authorized = false;
        canopen_master_set_drive_permission(context->config.canopen_master,
                                            false);
    }

    RobotStateId before = context->machine.activeState;
    SupervisorResult result =
        state_machine_handle_event(&context->machine, event);

    (void)result;

    SUPERVISOR_LOG("[SUPERVISOR] Event %s: result=%d, state=%s\n",
                   state_machine_event_name(event->type),
                   (int)result,
                   state_machine_state_name(context->machine.activeState));

    if (context->machine.activeState != before)
    {
        enter_active_state(context);
    }
    /*
     * A RESET request during active motion is not completion.
     * Preserve runtime context until the active state acknowledges
     * its controlled stop with STATE_ABORTED_RESET.
     */
    const bool reset_completed =
        (event->type == SUPERVISOR_EVENT_RESET &&
         (result == SUPERVISOR_RESULT_HANDLED ||
          result == SUPERVISOR_RESULT_TRANSITIONED)) ||
        (event->type == SUPERVISOR_EVENT_STATE_ABORTED_RESET &&
         result == SUPERVISOR_RESULT_TRANSITIONED);

    if (reset_completed)
    {
        clear_runtime_program_context(context);
    }

}

static void handle_hmi_command(SupervisorTaskContext *context,
                               SupervisorHmiCommand command)
{
    TeachingEvent teaching = TEACH_EVENT_NONE;
    SupervisorEventType policy = SUPERVISOR_EVENT_NONE;
    switch (command)
    {
        case SUP_HMI_LINE: teaching = TEACH_EVENT_SELECT_LINE; break;
        case SUP_HMI_ARC: teaching = TEACH_EVENT_SELECT_ARC; break;
        case SUP_HMI_CIRCLE: teaching = TEACH_EVENT_SELECT_CIRCLE; break;
        case SUP_HMI_RECORD: teaching = TEACH_EVENT_RECORD_POINT; break;
        case SUP_HMI_SPEED_LOW: teaching = TEACH_EVENT_SPEED_LOW; break;
        case SUP_HMI_SPEED_MEDIUM: teaching = TEACH_EVENT_SPEED_MEDIUM; break;
        case SUP_HMI_SPEED_HIGH: teaching = TEACH_EVENT_SPEED_HIGH; break;
        case SUP_HMI_VALIDATE_PREVIEW:
            if (context->machine.activeState == ROBOT_STATE_TEACHING)
                teaching = TEACH_EVENT_VALIDATE_PATH;
            else policy = SUPERVISOR_EVENT_VALIDATE_PREVIEW;
            break;
        case SUP_HMI_START_REPLAY: policy = SUPERVISOR_EVENT_START_REPLAY; break;
        case SUP_HMI_PAUSE:
            if (context->machine.activeState != ROBOT_STATE_PAUSED)
                policy = SUPERVISOR_EVENT_PAUSE_RESUME;
            break;
        case SUP_HMI_RESUME:
            if (context->machine.activeState == ROBOT_STATE_PAUSED)
                policy = SUPERVISOR_EVENT_PAUSE_RESUME;
            break;
        case SUP_HMI_PAUSE_RESUME: policy = SUPERVISOR_EVENT_PAUSE_RESUME; break;
        case SUP_HMI_HOME: policy = SUPERVISOR_EVENT_HOME; break;
        case SUP_HMI_RESET: policy = SUPERVISOR_EVENT_RESET; break;
        default: return;
    }
    if (command == SUP_HMI_LINE || command == SUP_HMI_ARC ||
        command == SUP_HMI_CIRCLE)
    {
        if (context->machine.activeState == ROBOT_STATE_IDLE)
        {
            SupervisorEvent event = make_result_event(SUPERVISOR_EVENT_TEACH,
                ROBOT_STATE_IDLE, ROBOT_FAULT_SEVERITY_NONE, 0U);
            handle_policy_event(context, &event);
        }
    }
    if (teaching != TEACH_EVENT_NONE &&
        context->machine.activeState == ROBOT_STATE_TEACHING)
        context->pending_teaching_event = teaching;
    if (policy != SUPERVISOR_EVENT_NONE)
    {
        SupervisorEvent event = make_result_event(policy,
            context->machine.activeState, ROBOT_FAULT_SEVERITY_NONE, 0U);
        handle_policy_event(context, &event);
    }
}

/* A single CANopen runtime owner. The check is injected into the RTOS-neutral
 * CANopen coordinator only from its owning FreeRTOS task. */
static bool supervisor_is_canopen_owner(void *context)
{
    return xTaskGetCurrentTaskHandle() == (TaskHandle_t)context;
}

/* Snapshot protects readers from direct access to mutable drive/heartbeat
 * data. For Phase 1 all readers share this scheduler and its critical region. */
static void publish_drive_snapshot(SupervisorTaskContext *context)
{
    const CanopenMaster *master = context->config.canopen_master;
    SupervisorDriveSnapshot snapshot = {0};
    snapshot.timestamp_ms = supervisor_now_ms();
    if (master != NULL && master->initialized) {
        snapshot.valid = true;
        snapshot.drives_ready = canopen_master_all_feedback_valid(master) &&
            canopen_master_all_drives_operation_enabled(master);
        snapshot.heartbeats_operational =
            canopen_master_all_heartbeats_operational(master);
        for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis) {
            const AvatarMDrive *drive = canopen_master_drive(master, axis);
            if (drive != NULL) {
                snapshot.feedback_valid[axis] = drive->feedback_valid;
                snapshot.actual_position_units[axis] =
                    drive->feedback.actual_position;
            }
        }
    }
    taskENTER_CRITICAL();
    context->drive_snapshot = snapshot;
    taskEXIT_CRITICAL();
}

/* P03: both explicit CM4 protocol rejection and timeout fault here. */
static void emit_step_failure(SupervisorTaskContext *context, uint32_t detail);
/* Only M7 consumes the M4->M7 ring. No shared FreeRTOS queue or mutex. */
static void consume_m4_ipc(SupervisorTaskContext *context)
{
    if (context->config.ipc == NULL) return;
    IpcMessage msg;
    for (uint32_t count = 0U; count < 8U; ++count) {
        /* The teaching state has ONE pending event slot. Do not consume
         * another M4 command until the previous teaching event is handled
         * by run_active_state() in the current/next Supervisor cycle. */
        if (context->pending_teaching_event != TEACH_EVENT_NONE) break;
        if (!ipc_m7_receive(context->config.ipc, &msg)) break;
        if (msg.type == IPC_CM4_HELLO) {
            if (!runtime_peer_on_hello(&context->peer_evidence, &msg) &&
                context->restart_gate.authorized &&
                context->machine.activeState != ROBOT_STATE_FAULT &&
                context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
                emit_step_failure(context, 0xE018U);
            continue;
        }
        if (msg.type == IPC_CM4_PROGRESS) {
            if (!runtime_peer_accept(&context->peer_evidence, &msg,
                                     supervisor_now_ms()) &&
                context->restart_gate.authorized &&
                context->machine.activeState != ROBOT_STATE_FAULT &&
                context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
                emit_step_failure(context, 0xE018U);
            continue;
        }
        /* Old CM7 sessions and prior challenge generations cannot replay HMI. */
        if (msg.type != IPC_HMI_COMMAND ||
            !runtime_peer_fresh(&context->peer_evidence, supervisor_now_ms()) ||
            msg.data[1] != context->peer_evidence.expected_session ||
            msg.data[2] == 0U ||
            msg.data[2] != context->peer_evidence.last_accepted_challenge)
            continue;
        if (msg.type != IPC_HMI_COMMAND || msg.sequence == 0U ||
            (int32_t)(msg.sequence - context->last_m4_ipc_sequence) <= 0 ||
            msg.data[0] >= (uint32_t)SUP_HMI_COMMAND_COUNT) continue;
        context->last_m4_ipc_sequence = msg.sequence;
        handle_hmi_command(context, (SupervisorHmiCommand)msg.data[0]);
    }
}

/* Status is best-effort. Never wait when a slow M4 fills the telemetry ring. */
static void publish_m7_ipc_status(SupervisorTaskContext *context)
{
    if (context->config.ipc == NULL) return;
    const uint32_t now = supervisor_now_ms();
    if (context->m7_ipc_status_sent &&
        (uint32_t)(now - context->last_m7_ipc_status_ms) < 10U) return;
    IpcMessage msg = {0};
    msg.type = IPC_CONTROLLER_STATUS;
    msg.sequence = ++context->m7_status_ipc_sequence;
    msg.timestamp_ms = now;
    msg.data[0] = (uint32_t)context->machine.activeState;
    msg.data[1] = (uint32_t)context->machine.safety.motionPermitted;
    msg.data[2] = context->machine.activeFaultCode;
    msg.data[3] = context->machine.transitionCount;
    msg.data[4] = (uint32_t)context->machine.safety.estopActive;
    msg.data[5] = context->last_m4_ipc_sequence; /* receipt, not acceptance */
    msg.data[6] = context->machine.rejectedEventCount;
    msg.data[7] = context->peer_evidence.expected_session;
    msg.data[8] = RUNTIME_PEER_PROTOCOL_VERSION;
    /* One outstanding CM7-local round-trip; retransmit same nonce. */
    uint32_t challenge = 0U;
    if (runtime_peer_challenge(&context->peer_evidence, now, &challenge))
        msg.data[9] = challenge;
    msg.data[10] = context->peer_evidence.last_accepted_challenge;
    (void)ipc_m7_send(context->config.ipc, &msg);
    context->last_m7_ipc_status_ms = now;
    context->m7_ipc_status_sent = true;
}

/* P02: monitor after bounded CAN reception, before any motion-state step.
 * PathExecution owns the exact 20-ms/20,000-us cyclic watchdog. PAUSED and
 * PATH_VALIDATION do not command cyclic motion: PDO age alone is not a fault
 * while SYNC traffic is absent, but network/drive validity is still checked.
 * Application-level containment does not prove that a drive has stopped. */
static bool supervisor_motion_containment_check(
    SupervisorTaskContext *context, uint32_t now_ms,
    bool guided_dispatch)
{
    const RobotStateId state = context->machine.activeState;
    if (state == ROBOT_STATE_BOOT || state == ROBOT_STATE_FAULT ||
        state == ROBOT_STATE_EMERGENCY_STOP)
        return true;
    if (!context->restart_gate.authorized &&
        !context->restart_gate.inhibited)
        return true;

    uint32_t detail = 0U;
    if (!context->restart_gate.authorized ||
        !context->config.canopen_master->drive_command_permission)
        detail = 0xE0B1U;
    else if (!state_machine_can_move(&context->machine))
        detail = 0xE110U;
    else if (!joint_drive_port_poll(&context->joint_drive, now_ms))
        detail = 0xE113U;
    else if (!joint_drive_port_healthy(&context->joint_drive, now_ms))
        detail = 0xE111U;
    else if (!joint_drive_port_ready(&context->joint_drive, now_ms))
        detail = 0xE112U;
    else {
        const bool require_fresh_tpdo =
            state == ROBOT_STATE_HOMING ||
            (state == ROBOT_STATE_TEACHING && guided_dispatch) ||
            state == ROBOT_STATE_APPROACH;
        for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis) {
            JointDriveAxisFeedback feedback = {0};
            if (!joint_drive_port_read_axis(&context->joint_drive,
                                            axis, &feedback) ||
                !feedback.feedback_valid || !feedback.operation_enabled ||
                (require_fresh_tpdo &&
                 (uint32_t)(now_ms -
                    (uint32_t)(feedback.timestamp_us / UINT64_C(1000))) >
                    100U)) {
                detail = 0xE200U + (uint32_t)axis;
                break;
            }
        }
    }
    if (detail == 0U) return true;
    const SupervisorEvent fault = make_result_event(
        SUPERVISOR_EVENT_FAULT_DETECTED, state,
        ROBOT_FAULT_SEVERITY_RECOVERABLE, state_error_code(state, detail));
    handle_policy_event(context, &fault);
    return false;
}

/* Forward declarations for helpers called by the bounded cycle path. */
static void handle_message(SupervisorTaskContext *context,
                           const SupervisorMessage *message);
static void emit_step_failure(SupervisorTaskContext *context, uint32_t detail);

/* Production cycle and host fixture share this actual acquisition path.
 * A missing/failed board callback during an authorized session is a fault;
 * this is still an observation channel, not a safety-rated E-stop input. */
static void supervisor_sample_inputs(SupervisorTaskContext *context)
{
    SupervisorInputSnapshot sampled = {0};
    if (context->config.io.read_inputs != NULL) {
        const bool acquired = context->config.io.read_inputs(
            context->config.io.context, &sampled);
        sampled.valid = acquired && sampled.valid;
    }
    taskENTER_CRITICAL();
    context->gpio_inputs = sampled;
    taskEXIT_CRITICAL();
    if (!sampled.valid && context->restart_gate.authorized &&
        context->machine.activeState != ROBOT_STATE_FAULT &&
        context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
        emit_step_failure(context, 0xE015U);
}

/* BOOT must not require drivesReady before enabling the drives: that is
 * a circular prerequisite. Enforce the independent safety and communication
 * fields before enable; require drivesReady after BOOT for any motion state.
 * The existing C2B board release/press and pre-enable checks remain intact. */
static DriveRestartSystemSample supervisor_restart_system_sample(
    const SupervisorTaskContext *context, uint32_t now_ms)
{
    const RobotSafetySnapshot *s = &context->machine.safety;
    const bool boot = context->machine.activeState == ROBOT_STATE_BOOT;
    const DriveRestartSystemSample sample = {
        .communication_ready =
            canopen_master_healthy(context->config.canopen_master, now_ms) &&
            s->communicationHealthy,
        .state_ready = context->machine.initialized &&
            context->machine.activeState != ROBOT_STATE_FAULT &&
            context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP &&
            context->machine.faultSeverity == ROBOT_FAULT_SEVERITY_NONE &&
            (boot || s->drivesReady),
        .safety_valid = s->statusValid && s->motionPermitted &&
            runtime_safety_fresh(&context->safety_evidence,
                                 &context->config.safety_evidence, now_ms),
        .estop_clear = !s->estopActive,
        .protective_stop_clear = !s->protectiveStopActive,
        .fault_clear = !s->globalFaultActive,
        .ipc_valid = context->config.ipc != NULL &&
                     ipc_valid(context->config.ipc) &&
                     runtime_peer_fresh(&context->peer_evidence, now_ms),
        .preenable_status_checked = context->boot.preenable_status_checked
    };
    return sample;
}

/* A complete drain is confirmed by a failed receive OR a zero-depth check
 * after the exact 16-message boundary. This check is a bounded snapshot, not
 * a concurrency barrier against future enqueues. Never dispatch queued motion
 * targets. A Teaching event intentionally stops the drain to protect the
 * existing single pending event slot; it is NOT queue overload. */
typedef enum {
    SUPERVISOR_DRAIN_COMPLETE = 0,
    SUPERVISOR_DRAIN_TEACHING_EVENT,
    SUPERVISOR_DRAIN_OVERLOAD
} SupervisorDrainResult;

static SupervisorDrainResult supervisor_drain_messages(
    SupervisorTaskContext *context)
{
    SupervisorMessage message;
    SupervisorMessage deferred_target = {0};
    bool have_target = false;
    for (uint32_t count = 0U; count < SUPERVISOR_MAX_MESSAGES_CYCLE;
         ++count) {
        if (xQueueReceive(g_queue, &message, 0U) != pdPASS) {
            if (have_target) handle_message(context, &deferred_target);
            return SUPERVISOR_DRAIN_COMPLETE;
        }
        if (message.type == SUPERVISOR_MESSAGE_GUIDED_TARGET) {
            deferred_target = message;
            have_target = true;
        } else {
            handle_message(context, &message);
        }
        if (context->pending_teaching_event != TEACH_EVENT_NONE) {
            /* Preserve P01: let TEACHING consume Record before dequeuing
             * another event. Any event in a motion state is not exempt. */
            return context->machine.activeState == ROBOT_STATE_TEACHING
                ? SUPERVISOR_DRAIN_TEACHING_EVENT
                : SUPERVISOR_DRAIN_OVERLOAD;
        }
    }
    /* Exactly 16 messages with no remainder is NOT an overload. Querying
     * queue depth is O(1) and does not consume the next SAFETY snapshot. */
    if (uxQueueMessagesWaiting(g_queue) == 0U) {
        if (have_target) handle_message(context, &deferred_target);
        return SUPERVISOR_DRAIN_COMPLETE;
    }
    return SUPERVISOR_DRAIN_OVERLOAD;
}

/* A saturated queue must not starve Approach/PathExecution stop services or
 * the 500-Hz fault watchdog. Do not step a motion state while safety updates
 * are unread: latch FAULT immediately and run its safe-output service in the
 * same cycle. Physical stopping remains unverified. */
static void supervisor_process_message_batch(SupervisorTaskContext *context)
{
    const SupervisorDrainResult result = supervisor_drain_messages(context);
    if (result == SUPERVISOR_DRAIN_OVERLOAD &&
        context->machine.activeState != ROBOT_STATE_FAULT &&
        context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
        emit_step_failure(context, 0xE016U);
}

static void handle_message(
    SupervisorTaskContext *context,
    const SupervisorMessage *message
)
{
    switch (message->type)
    {
        case SUPERVISOR_MESSAGE_HMI_COMMAND:
            handle_hmi_command(context, message->data.hmi_command);
            break;

        case SUPERVISOR_MESSAGE_GUIDED_TARGET:
            /* P02 default-deny: this envelope has only six position values.
             * No trusted producer identity, freshness, homing evidence or
             * teaching-runtime consent can be authenticated here. The board
             * explicitly commissions hand-guiding in observation-only mode.
             * Do not silently convert an HMI/IPC message into motor motion. */
            (void)message->data.guided_target_units;
            if (context->machine.rejectedEventCount != UINT32_MAX)
                ++context->machine.rejectedEventCount;
            break;
        case SUPERVISOR_MESSAGE_EVENT:
            handle_policy_event(context, &message->data.event);
            break;

        case SUPERVISOR_MESSAGE_SAFETY:
        {
            const RobotSafetySnapshot *incoming = &message->data.safety;
            /* An unverified unsafe observation may still ASSERT a stop;
             * it can never clear one. Preserve E-stop precedence. */
            if (incoming->estopActive &&
                context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP) {
                const SupervisorEvent stop = make_result_event(
                    SUPERVISOR_EVENT_ESTOP_ASSERTED,
                    context->machine.activeState,
                    ROBOT_FAULT_SEVERITY_EMERGENCY,
                    ROBOT_FAULT_CODE_EMERGENCY_STOP);
                handle_policy_event(context, &stop);
                return;
            }
            if (incoming->protectiveStopActive &&
                context->machine.activeState != ROBOT_STATE_FAULT &&
                context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP) {
                const SupervisorEvent stop = make_result_event(
                    SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED,
                    context->machine.activeState,
                    ROBOT_FAULT_SEVERITY_PROTECTIVE_STOP,
                    ROBOT_FAULT_CODE_PROTECTIVE_STOP);
                handle_policy_event(context, &stop);
                return;
            }
            if (!runtime_safety_accept(&context->safety_evidence,
                                       &context->config.safety_evidence,
                                       incoming, supervisor_now_ms())) {
                /* Do not replace the last trusted sample with replay. */
                if (context->restart_gate.authorized &&
                    context->machine.activeState != ROBOT_STATE_FAULT &&
                    context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
                    emit_step_failure(context, 0xE017U);
                return;
            }
            state_machine_update_safety(&context->machine, incoming);
            const RobotSafetySnapshot *s = &context->machine.safety;
            const RobotStateId state = context->machine.activeState;
            SupervisorEvent event = {0};
            /* E-stop has highest priority; do not downgrade an existing
             * emergency to an ordinary recoverable fault. Input snapshots
             * are not hardware safety functions. */
            if (s->estopActive && state != ROBOT_STATE_EMERGENCY_STOP) {
                event = make_result_event(SUPERVISOR_EVENT_ESTOP_ASSERTED,
                    state, ROBOT_FAULT_SEVERITY_EMERGENCY,
                    ROBOT_FAULT_CODE_EMERGENCY_STOP);
            } else if (state != ROBOT_STATE_EMERGENCY_STOP &&
                       state != ROBOT_STATE_FAULT &&
                       s->protectiveStopActive) {
                event = make_result_event(
                    SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED, state,
                    ROBOT_FAULT_SEVERITY_PROTECTIVE_STOP,
                    ROBOT_FAULT_CODE_PROTECTIVE_STOP);
            } else if (state != ROBOT_STATE_EMERGENCY_STOP &&
                       state != ROBOT_STATE_FAULT &&
                       (state != ROBOT_STATE_BOOT ||
                        context->restart_gate.authorized) &&
                       (!s->statusValid || !s->motionPermitted ||
                        (state != ROBOT_STATE_BOOT && !s->drivesReady) ||
                        !s->communicationHealthy ||
                        s->globalFaultActive)) {
                const uint32_t detail = !s->statusValid ? 0xE010U :
                    !s->motionPermitted ? 0xE011U :
                    (state != ROBOT_STATE_BOOT && !s->drivesReady) ? 0xE012U :
                    !s->communicationHealthy ? 0xE013U : 0xE014U;
                event = make_result_event(SUPERVISOR_EVENT_FAULT_DETECTED,
                    state, ROBOT_FAULT_SEVERITY_RECOVERABLE,
                    state_error_code(state, detail));
            }
            if (event.type != SUPERVISOR_EVENT_NONE)
                handle_policy_event(context, &event);
            break;
        }

        case SUPERVISOR_MESSAGE_TEACHING_EVENT:
            context->pending_teaching_event =
                message->data.teaching_event;
            break;

        case SUPERVISOR_MESSAGE_TEACHING_RUNTIME:
            context->teaching_runtime =
                message->data.teaching_runtime;
            break;

        case SUPERVISOR_MESSAGE_APPROACH_CONTROL:
        {
            const bool reset = context->approach_control.reset_requested;
            const bool home = context->approach_control.home_requested;
            context->approach_control = message->data.approach_control;
            context->approach_control.reset_requested |= reset;
            context->approach_control.home_requested |= home;
            break;
        }

        case SUPERVISOR_MESSAGE_PATH_EXECUTION_INPUTS:
        {
            const bool reset = context->path_execution_inputs.reset_requested;
            const bool home = context->path_execution_inputs.home_requested;
            context->path_execution_inputs = message->data.path_execution_inputs;
            context->path_execution_inputs.reset_requested |= reset;
            context->path_execution_inputs.home_requested |= home;
            break;
        }

        default:
            break;
    }
}

static void emit_step_failure(
    SupervisorTaskContext *context,
    uint32_t detail
)
{
    SupervisorEvent event = make_result_event(
        SUPERVISOR_EVENT_STATE_FAILED,
        context->machine.activeState,
        ROBOT_FAULT_SEVERITY_RECOVERABLE,
        state_error_code(context->machine.activeState, detail)
    );
    handle_policy_event(context, &event);
}

static void emit_step_complete(SupervisorTaskContext *context)
{
    SupervisorEvent event = make_result_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        context->machine.activeState,
        ROBOT_FAULT_SEVERITY_NONE,
        ROBOT_FAULT_CODE_NONE
    );
    handle_policy_event(context, &event);
}

static void run_active_state(SupervisorTaskContext *context)
{
    StateStepResult step = STATE_STEP_RUNNING;

    switch (context->machine.activeState)
    {
        case ROBOT_STATE_BOOT:
            step = state_boot_step(
                &context->boot,
                &context->commissioning,
                &context->joint_drive,
                supervisor_now_ms()
            );
            if (step == STATE_STEP_FAILED)
                emit_step_failure(context, (uint32_t)context->boot.error);
            else if (step == STATE_STEP_COMPLETE)
                emit_step_complete(context);
            break;

        case ROBOT_STATE_HOMING:
            step = state_homing_step(
                &context->homing,
                context->config.homing_config,
                context->config.robot,
                &context->joint_drive,
                context->config.avatar_position_scales,
                supervisor_now_ms()
            );
            if (step == STATE_STEP_FAILED)
                emit_step_failure(context, (uint32_t)context->homing.error);
            else if (step == STATE_STEP_COMPLETE)
                emit_step_complete(context);
            break;

        case ROBOT_STATE_IDLE:
            step = state_idle_step(
                &context->idle,
                IDLE_COMMAND_NONE,
                &context->joint_drive,
                supervisor_now_ms()
            );
            if (step == STATE_STEP_FAILED)
                emit_step_failure(context, (uint32_t)context->idle.error);
            break;

        case ROBOT_STATE_TEACHING:
            step = state_teaching_step(
                &context->teaching,
                context->config.robot,
                &context->teaching_runtime,
                &context->joint_drive,
                context->config.avatar_position_scales,
                supervisor_now_ms(),
                context->pending_teaching_event,
                &context->teaching_outputs
            );
            context->pending_teaching_event = TEACH_EVENT_NONE;

            if (step == STATE_STEP_FAILED)
            {
                emit_step_failure(context,
                                  (uint32_t)context->teaching.error);
            }
            else if (step == STATE_STEP_COMPLETE)
            {
                context->submitted_program = context->teaching.draft;
                emit_step_complete(context);
            }
            break;

        case ROBOT_STATE_PATH_VALIDATION:
            /* Entry waits for producer ownership release across ticks. */
            if (context->entered_state != ROBOT_STATE_PATH_VALIDATION)
                break;
            if (context->config.validation_run_in_worker) {
                ValidationWorkerState worker_status;
                PathValidationOutputs snapshot;
                taskENTER_CRITICAL();
                worker_status = context->validation_worker_state;
                snapshot = context->validation_outputs;
                taskEXIT_CRITICAL();
                if (worker_status != VALIDATION_WORKER_DONE)
                    break;
                /* DONE publishes a completed flash commit or a failure;
                 * no PREVIEW/WELD can run on a queued write. */
                SupervisorEventType type =
                    (snapshot.report.result == PV_RESULT_VALID)
                    ? SUPERVISOR_EVENT_STATE_COMPLETE
                    : SUPERVISOR_EVENT_VALIDATION_REJECTED;
                SupervisorEvent event = make_result_event(
                    type, ROBOT_STATE_PATH_VALIDATION,
                    ROBOT_FAULT_SEVERITY_NONE, ROBOT_FAULT_CODE_NONE);
                handle_policy_event(context, &event);
                break;
            }
            step = state_path_validation_step(
                &context->validation,
                context->config.validation_sample_budget,
                &context->validation_outputs
            );

            if (step == STATE_STEP_FAILED)
            {
                emit_step_failure(context,
                                  (uint32_t)context->validation.error);
            }
            else if (step == STATE_STEP_COMPLETE)
            {
                SupervisorEventType type =
                    (context->validation_outputs.report.result ==
                     PV_RESULT_VALID)
                        ? SUPERVISOR_EVENT_STATE_COMPLETE
                        : SUPERVISOR_EVENT_VALIDATION_REJECTED;
                SupervisorEvent event = make_result_event(
                    type,
                    ROBOT_STATE_PATH_VALIDATION,
                    ROBOT_FAULT_SEVERITY_NONE,
                    ROBOT_FAULT_CODE_NONE
                );
                handle_policy_event(context, &event);
            }
            break;

        case ROBOT_STATE_APPROACH:
            step = state_approach_step(
                &context->approach,
                &context->approach_control,
                supervisor_now_ms(),
                &context->approach_outputs
            );

            /* HOME and RESET are one-shot requests. */
            context->approach_control.home_requested = false;
            context->approach_control.reset_requested = false;

            if (step == STATE_STEP_FAILED)
            {
                emit_step_failure(context,
                                  (uint32_t)context->approach.error);
            }
            else if (step == STATE_STEP_COMPLETE)
            {
                SupervisorEventType type = SUPERVISOR_EVENT_STATE_COMPLETE;

                if (context->approach.result == APPROACH_RESULT_ABORTED)
                {
                    if (context->approach.error ==
                        APPROACH_ERR_HOME_REQUESTED)
                    {
                        type = SUPERVISOR_EVENT_STATE_ABORTED_HOME;
                    }
                    else if (context->approach.error ==
                             APPROACH_ERR_RESET_REQUESTED)
                    {
                        type = SUPERVISOR_EVENT_STATE_ABORTED_RESET;
                    }
                    else
                    {
                        emit_step_failure(
                            context,
                            (uint32_t)context->approach.error
                        );
                        break;
                    }
                }

                SupervisorEvent event = make_result_event(
                    type,
                    ROBOT_STATE_APPROACH,
                    ROBOT_FAULT_SEVERITY_NONE,
                    ROBOT_FAULT_CODE_NONE
                );
                handle_policy_event(context, &event);
            }
            break;

        case ROBOT_STATE_PATH_EXECUTION:
            /* Incoming operator messages never own trajectory clock time. */
            context->path_execution_inputs.now_ms = supervisor_now_ms();
            context->path_execution_inputs.now_us = supervisor_now_us();
            context->path_execution_inputs.motion_permission =
                context->machine.safety.motionPermitted;
            context->path_execution_inputs.estop_active =
                context->machine.safety.estopActive;
            context->path_execution_inputs.protective_stop_active =
                context->machine.safety.protectiveStopActive;
            context->path_execution_inputs.external_fault_active =
                context->machine.safety.globalFaultActive;
            step = state_path_execution_step(
                &context->path_execution,
                &context->path_execution_inputs,
                &context->path_execution_outputs
            );
            context->path_execution_inputs.reset_requested = false;
            context->path_execution_inputs.home_requested = false;
            context->path_execution_inputs.pause_requested = false;

            if (step == STATE_STEP_FAILED)
            {
                emit_step_failure(
                    context,
                    (uint32_t)context->path_execution.error
                );
            }
            else if (step == STATE_STEP_COMPLETE)
            {
                SupervisorEventType type = SUPERVISOR_EVENT_STATE_COMPLETE;
                if (context->path_execution.result ==
                    PATH_EXEC_RESULT_ABORTED)
                {
                    type = (context->path_execution.abort_reason ==
                            PATH_EXEC_ABORT_HOME)
                        ? SUPERVISOR_EVENT_STATE_ABORTED_HOME
                        : SUPERVISOR_EVENT_STATE_ABORTED_RESET;
                }
                SupervisorEvent event = make_result_event(
                    type,
                    ROBOT_STATE_PATH_EXECUTION,
                    ROBOT_FAULT_SEVERITY_NONE,
                    ROBOT_FAULT_CODE_NONE
                );
                handle_policy_event(context, &event);
            }
            break;

        case ROBOT_STATE_PAUSED:
        {
            PausedInputs inputs;
            memset(&inputs, 0, sizeof(inputs));
            inputs.estop_active = context->machine.safety.estopActive;
            inputs.external_fault_active =
                context->machine.safety.globalFaultActive;
            step = state_paused_step(&context->paused, &inputs);
            if (step == STATE_STEP_FAILED)
                emit_step_failure(context, 1U);
            break;
        }

        case ROBOT_STATE_FAULT:
        {
            FaultInputs inputs;
            inputs.fault_cause_active =
                context->machine.safety.globalFaultActive ||
                context->machine.safety.protectiveStopActive;
            inputs.safety_healthy =
                context->machine.safety.statusValid &&
                !inputs.fault_cause_active &&
                !context->machine.safety.estopActive;
            step = state_fault_step(&context->fault, &inputs);
            context->fault_response_step_ok = (step != STATE_STEP_FAILED);
            (void)step;
            break;
        }

        case ROBOT_STATE_EMERGENCY_STOP:
        {
            EmergencyStopInputs inputs;
            inputs.estop_active = context->machine.safety.estopActive;
            inputs.reset_acknowledged =
                context->machine.emergencyResetAcknowledged;
            inputs.safety_healthy =
                context->machine.safety.statusValid &&
                !context->machine.safety.estopActive &&
                !context->machine.safety.protectiveStopActive &&
                !context->machine.safety.globalFaultActive;
            step = state_emergency_stop_step(
                &context->emergency_stop, &inputs);
            context->fault_response_step_ok = (step != STATE_STEP_FAILED);
            (void)step;
            break;
        }

        default:
            /* Legacy execution states are intentionally not dispatched. */
            break;
    }
}

/* Observation-only Stage 4: runs within the M7 Supervisor owner task.
 * All feedback is copied, and this function has NO motor-target dispatch.
 * The read_sample callback is supplied explicitly by the commissioned board;
 * the firmware deliberately has no synthetic F/T provider. */
static void service_guidance_dry_run(SupervisorTaskContext *context)
{
    const SupervisorGuidanceDryRunPort *port =
        &context->config.guidance_dry_run;
    if (port->tick == NULL) return;
    if (context->machine.activeState != ROBOT_STATE_TEACHING ||
        !context->teaching_runtime.manual_guidance_active) {
        port->stop(port->context);
        return;
    }

    const RobotSafetySnapshot *safety = &context->machine.safety;
    SupervisorGuidanceDryRunCycle cycle = {0};
    cycle.now_us = supervisor_now_us();
    cycle.now_ms = (uint32_t)(cycle.now_us / UINT64_C(1000));
    cycle.feedback_healthy =
        joint_drive_port_healthy(&context->joint_drive, cycle.now_ms);
    for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis) {
        if (!joint_drive_port_read_axis(&context->joint_drive, axis,
                                        &cycle.feedback[axis]))
            cycle.feedback_healthy = false;
    }
    cycle.scales = context->config.avatar_position_scales;
    cycle.robot = context->config.robot;
    cycle.period_s = (double)context->config.period_ticks /
                     (double)configTICK_RATE_HZ;
    cycle.runtime_timestamp_ms = context->teaching_runtime.timestamp_ms;
    cycle.safety_timestamp_ms = safety->timestampMs;
    cycle.teaching_active = true;
    cycle.manual_guidance_requested = true;
    cycle.motion_permitted = safety->motionPermitted &&
        context->teaching_runtime.motion_permitted;
    cycle.robot_homed = context->teaching_runtime.robot_homed;
    cycle.drives_ready = safety->drivesReady &&
        canopen_master_ready_for_motion(context->config.canopen_master,
                                        cycle.now_ms);
    cycle.communication_healthy = safety->communicationHealthy;
    cycle.safety_valid = safety->statusValid;
    cycle.estop_active = safety->estopActive ||
        context->teaching_runtime.estop_active;
    cycle.protective_stop_active = safety->protectiveStopActive ||
        context->teaching_runtime.protective_stop_active;
    cycle.global_fault_active = safety->globalFaultActive ||
        context->teaching_runtime.global_fault_active;

    const bool observer_ok = port->tick(port->context, &cycle);
    if (observer_ok) {
        if (context->guidance_dry_run_ok_cycles < UINT32_MAX)
            ++context->guidance_dry_run_ok_cycles;
    } else {
        if (context->guidance_dry_run_rejected_cycles < UINT32_MAX)
            ++context->guidance_dry_run_rejected_cycles;
    }
    /* Candidate references remain exclusively in the observer state.
     * No candidate extraction, motor dispatch or safety authority. */
}

/* P03: called on EVERY 500 Hz cycle before any active-state dispatch.
 * Fault service is never conditional on additional peer traffic. */
static void supervisor_runtime_evidence_check(
    SupervisorTaskContext *context, uint32_t now_ms)
{
    if (!context->restart_gate.authorized ||
        context->machine.activeState == ROBOT_STATE_FAULT ||
        context->machine.activeState == ROBOT_STATE_EMERGENCY_STOP)
        return;
    if (!runtime_safety_fresh(&context->safety_evidence,
                              &context->config.safety_evidence, now_ms))
        emit_step_failure(context, 0xE017U);
    else if (!runtime_peer_fresh(&context->peer_evidence, now_ms))
        emit_step_failure(context, 0xE018U);
}

static void supervisor_pre_dispatch_cycle(SupervisorTaskContext *context)
{
    supervisor_sample_inputs(context);
    supervisor_process_message_batch(context);
    consume_m4_ipc(context);
    supervisor_runtime_evidence_check(context, supervisor_now_ms());
    (void)supervisor_motion_containment_check(context, supervisor_now_ms(), false);
}

/* P04: validate the source-owned TIM3 observation before entering a motion
 * state. Completion overruns remain post-execution (cannot be predicted);
 * no motion is executed after an already-observable late/invalid wake.
 * TIM3 service times and TIM2 wake sample share the CM7 clock domain. */
static bool supervisor_timing_pre_dispatch_check(
    SupervisorTaskContext *context, uint64_t task_now_us)
{
    const SupervisorTimingAssessment a = context->last_timing_wake_available
        ? supervisor_timing_assess(&context->last_timing_wake, task_now_us,
              context->previous_timing_wake_valid,
              context->previous_timing_wake_us)
        : (SupervisorTimingAssessment){0};
    uint32_t detail = 0U;
    if (!a.valid)
        detail = 0xE019U; /* invalid or missing CM7 timing evidence */
    else if (a.service_gap_severe || a.wake_gap_severe)
        detail = 0xE003U; /* serviced interrupt / wake cadence violated */
    else if (a.late_completion)
        detail = 0xE004U; /* task already >=2 ms from serviced IRQ */
    if (detail == 0U) return true;
    if (context->machine.activeState != ROBOT_STATE_FAULT &&
        context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
        emit_step_failure(context, detail);
    return false;
}

static void supervisor_task_entry(void *argument)
{
    SupervisorTaskContext *context = argument;
    TickType_t last_wake = xTaskGetTickCount();
    if (context->config.canopen_master->initialized) {
        /* Initialization is pre-scheduler; bind only from the true owner. */
        const bool owner_bound = canopen_master_bind_runtime_owner(
            context->config.canopen_master,
            supervisor_is_canopen_owner,
            xTaskGetCurrentTaskHandle());
        configASSERT(owner_bound);
        if (!owner_bound) {
            /* Never run an unowned CANopen control loop in release builds. */
            for (;;) { vTaskDelay(1); }
        }
    }

    const bool hardware_timed = context->config.timing.wait_next != NULL;
    if (hardware_timed &&
        !context->config.timing.start(context->config.timing.context)) {
        /* Refuse any operation when the physical timing source fails. */
        emit_step_failure(context, 0xE002U);
        for (;;) { vTaskDelay(1U); }
    }

    for (;;)
    {
        bool progress_cycle_ok = true;
        context->fault_response_step_ok = false;
        if (hardware_timed) {
            /* TIM3 increments a counting task notification every 2 ms.
             * A queued count >1 or missing count is a failed deadline. */
            const uint32_t periods = context->config.timing.wait_next(
                context->config.timing.context, pdMS_TO_TICKS(6U));
            if (periods != 1U) {
                progress_cycle_ok = false;
                if (periods == 0U) {
                    if (context->timer_timeouts < UINT32_MAX)
                        ++context->timer_timeouts;
                } else {
                    const uint32_t missed = periods - 1U;
                    const uint32_t available =
                        UINT32_MAX - context->timer_missed_periods;
                    context->timer_missed_periods +=
                        missed > available ? available : missed;
                }
                if (context->machine.activeState != ROBOT_STATE_FAULT &&
                    context->machine.activeState !=
                        ROBOT_STATE_EMERGENCY_STOP)
                    emit_step_failure(context, 0xE003U);
            }
            context->last_timing_wake_available = false;
            if (context->config.timing.observe_wake != NULL)
                context->last_timing_wake_available =
                    context->config.timing.observe_wake(
                        context->config.timing.context, periods,
                        &context->last_timing_wake);
        }
        const uint64_t cycle_start_us =
            hardware_timed ? supervisor_now_us() : UINT64_C(0);
        /* Reject known missed/invalid TIM3 timing before motor-state
         * dispatch; completion overruns are evaluated after work. */
        if (hardware_timed &&
            !supervisor_timing_pre_dispatch_check(context, cycle_start_us))
            progress_cycle_ok = false;
        TRACE_BEGIN(TIMING_SUPERVISOR);
        /* P03: actual production pre-dispatch cycle; tests call the same function. */
        supervisor_pre_dispatch_cycle(context);

        /*
         * Keep producing Node 127's heartbeat even while PAUSED, FAULT,
         * or EMERGENCY_STOP. A dead/stalled supervisor sends nothing, so
         * the AVATAR drives' 0x1016 consumer can detect controller loss.
         */
        bool controller_hb_ok = true;
        if (context->config.canopen_master != NULL &&
            context->config.canopen_master->initialized)
        {
            controller_hb_ok = canopen_master_service_heartbeat(
                context->config.canopen_master,
                supervisor_now_ms()
            );
        }
        if (!controller_hb_ok &&
            context->machine.activeState != ROBOT_STATE_FAULT &&
            context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
        {
            emit_step_failure(context, 0xE001U);
        }
        /* Retries a deferred validation entry without waiting or sleeping.
         * No-op in an already entered state. */
        enter_active_state(context);
        const bool was_authorized = context->restart_gate.authorized;
        /* A CANopen NMT reset can revoke permission inside the coordinator.
         * Never silently regrant the old session on the next 2 ms cycle. */
        if (was_authorized &&
            !context->config.canopen_master->drive_command_permission)
            drive_restart_gate_inhibit(&context->restart_gate);
        bool drive_permission = false;
        if (context->machine.activeState == ROBOT_STATE_FAULT ||
            context->machine.activeState == ROBOT_STATE_EMERGENCY_STOP)
        {
            drive_restart_gate_inhibit(&context->restart_gate);
        }
        else if (context->restart_gate.authorized ||
            (context->machine.activeState == ROBOT_STATE_BOOT &&
             context->boot.phase == BOOT_PHASE_WAIT_AUTHORIZATION))
        {
            /* Safety snapshot must be independently produced and current.
             * Board provider is responsible for live input freshness. */
            const DriveRestartSystemSample input =
                supervisor_restart_system_sample(context, supervisor_now_ms());
            drive_permission = drive_restart_gate_tick(
                &context->restart_gate, &context->config.reset_diagnostics,
                &context->config.drive_restart_board, &input);
        }
        context->boot.drive_enable_authorized = drive_permission;
        canopen_master_set_drive_permission(context->config.canopen_master,
                                           drive_permission);
        if (was_authorized && !drive_permission &&
            context->machine.activeState != ROBOT_STATE_FAULT &&
            context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
            emit_step_failure(context, 0xE0B1U);
        /* Work after the initial wake check (queue, IPC, heartbeat and
         * authorization) can consume the remaining deadline. Recheck the
         * CM7-local TIM2 clock immediately before state dispatch. This
         * cannot predict duration of the subsequent state step. FAULT and
         * EMERGENCY_STOP service must still run after a timing fault. */
        if (hardware_timed &&
            !supervisor_timing_pre_dispatch_check(context, supervisor_now_us()))
            progress_cycle_ok = false;
        /* Complete, intentional Teaching break, or overload->FAULT: never
         * suspend state/fault stepping merely because messages arrived. */
        if (controller_hb_ok ||
            context->machine.activeState == ROBOT_STATE_FAULT ||
            context->machine.activeState == ROBOT_STATE_EMERGENCY_STOP)
        {
            run_active_state(context);
        }
        service_guidance_dry_run(context);
        publish_drive_snapshot(context);
        publish_m7_ipc_status(context);
        if (context->config.io.write_outputs != NULL)
        {
            SupervisorOutputSnapshot outputs;
            supervisor_io_make_status(&context->machine, &outputs);
            /* The adapter owns output diagnostics; this result does not
             * silently acknowledge any fault or change motion permission. */
            (void)context->config.io.write_outputs(
                context->config.io.context, &outputs);
        }
        TRACE_END(TIMING_SUPERVISOR, true);
        if (hardware_timed) {
            const uint64_t completion_us = supervisor_now_us();
            const uint64_t elapsed = completion_us - cycle_start_us;
            const uint32_t sampled = elapsed > UINT32_MAX
                ? UINT32_MAX : (uint32_t)elapsed;
            if (sampled > context->timer_max_cycle_us)
                context->timer_max_cycle_us = sampled;
            if (elapsed >= UINT64_C(2000)) {
                progress_cycle_ok = false;
                if (context->timer_overruns < UINT32_MAX)
                    ++context->timer_overruns;
                if (context->machine.activeState != ROBOT_STATE_FAULT &&
                    context->machine.activeState !=
                        ROBOT_STATE_EMERGENCY_STOP)
                    emit_step_failure(context, 0xE004U);
            }
            if (!context->last_timing_wake_available)
                progress_cycle_ok = false;
            if (context->last_timing_wake_available) {
                const SupervisorTimingAssessment a = supervisor_timing_assess(
                    &context->last_timing_wake, completion_us,
                    context->previous_timing_wake_valid,
                    context->previous_timing_wake_us);
                if (!a.valid) {
                    progress_cycle_ok = false;
                    if (context->timer_invalid_timestamps < UINT32_MAX)
                        ++context->timer_invalid_timestamps;
                } else {
                    /* Every count saturates; never let long deployments wrap. */
#define TIMING_INC_SAT(name, condition) do { \
    if ((condition) && context->name < UINT32_MAX) ++context->name; \
} while (0)
                    TIMING_INC_SAT(timer_service_gap_warnings, a.service_gap_warning);
                    TIMING_INC_SAT(timer_service_gap_severe, a.service_gap_severe);
                    TIMING_INC_SAT(timer_wake_gap_warnings, a.wake_gap_warning);
                    TIMING_INC_SAT(timer_wake_gap_severe, a.wake_gap_severe);
                    TIMING_INC_SAT(timer_late_wakeups, a.late_wakeup);
                    TIMING_INC_SAT(timer_deadline_misses, a.late_completion);
#undef TIMING_INC_SAT
                    if (a.isr_to_wake_us > context->timer_max_isr_to_wake_us)
                        context->timer_max_isr_to_wake_us = a.isr_to_wake_us;
                    if (a.isr_to_completion_us > context->timer_max_isr_to_completion_us)
                        context->timer_max_isr_to_completion_us = a.isr_to_completion_us;
                    if (a.wake_gap_us > context->timer_max_wake_gap_us)
                        context->timer_max_wake_gap_us = a.wake_gap_us;
                    context->previous_timing_wake_us =
                        context->last_timing_wake.wake_sample_us;
                    context->previous_timing_wake_valid = true;
                    /* No new fault codes or state transitions. At >=4 ms
                     * without serviced IRQ / task wake the periodic release
                     * requirement has been violated (E003). An observed
                     * ISR-to-completion >=2 ms violates even the post-ISR
                     * deadline bound (E004). Smaller service gaps are only
                     * diagnostic warnings, not inferred missed updates. */
                    if (a.service_gap_severe || a.wake_gap_severe ||
                        a.late_completion)
                        progress_cycle_ok = false;
                    if ((a.service_gap_severe || a.wake_gap_severe) &&
                        context->machine.activeState != ROBOT_STATE_FAULT &&
                        context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
                        emit_step_failure(context, 0xE003U);
                    if (a.late_completion &&
                        context->machine.activeState != ROBOT_STATE_FAULT &&
                        context->machine.activeState != ROBOT_STATE_EMERGENCY_STOP)
                        emit_step_failure(context, 0xE004U);
                }
            }
        } else {
            vTaskDelayUntil(&last_wake, context->config.period_ticks);
        }
        /* Only the actual task's finished cycle can publish. No timer ISR,
         * defaultTask or validation worker may synthesize this checkpoint. */
        const bool in_fault =
            context->machine.activeState == ROBOT_STATE_FAULT ||
            context->machine.activeState == ROBOT_STATE_EMERGENCY_STOP;
        const SupervisionProgressMode mode =
            context->machine.activeState == ROBOT_STATE_BOOT ?
            SUP_PROGRESS_STARTUP : (in_fault ? SUP_PROGRESS_FAULT : SUP_PROGRESS_ACTIVE);
        taskENTER_CRITICAL();
        supervision_record_completion(&context->progress, 1U,
            (uint32_t)xTaskGetTickCount(), mode, progress_cycle_ok,
            !in_fault || context->fault_response_step_ok);
        taskEXIT_CRITICAL();
    }
}

bool supervisor_task_init(const SupervisorTaskConfig *config)
{
    if ((config == NULL) ||
        (config->canopen_master == NULL) ||
        (config->avatar_position_scales == NULL) ||
        (config->robot == NULL) ||
        (config->homing_config == NULL) ||
        (config->teaching_config == NULL) ||
        (config->validation_config == NULL) ||
        (config->validation_workspace == NULL) ||
        (config->validation_storage == NULL) ||
        (config->validated_trajectory == NULL) ||
        (config->approach_config == NULL) ||
        (config->approach_services == NULL) ||
        (config->path_execution_config == NULL) ||
        (config->path_execution_services == NULL) ||
        (config->paused_services == NULL) ||
        (config->fault_services == NULL) ||
        (config->emergency_stop_services == NULL) ||
        ((config->guidance_dry_run.tick == NULL) !=
         (config->guidance_dry_run.stop == NULL)) ||
        (config->guidance_dry_run.take_candidate != NULL) ||
        (config->validation_sample_budget == 0U) ||
        (config->period_ticks == 0U) ||
        (((config->timing.monotonic_us != NULL) ||
          (config->timing.start != NULL) ||
          (config->timing.wait_next != NULL)) &&
         ((config->timing.monotonic_us == NULL) ||
          (config->timing.start == NULL) ||
          (config->timing.wait_next == NULL) ||
          (config->period_ticks != 2U))) ||
        (config->ipc != NULL && !ipc_valid(config->ipc)))
    {
        return false;
    }

    memset(&g_supervisor, 0, sizeof(g_supervisor));
    drive_restart_gate_init(&g_supervisor.restart_gate);
    g_supervisor.config = *config;
    /* A unique CM7 boot session must be issued by a real provider.
     * Zero remains default-denied; no synthetic tick-derived token. */
    uint32_t boot_session = 0U;
    if (config->ipc_session.issue != NULL &&
        !runtime_session_issue(&config->ipc_session, &boot_session))
        return false;
    g_supervisor.peer_evidence.expected_session = boot_session;
    g_timing = config->timing;
    g_supervisor.next_program_id = config->first_program_id;
    g_supervisor.entered_state = ROBOT_STATE_COUNT;

    /* Prefer static storage when enabled; otherwise use the configured heap. */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    g_queue = xQueueCreateStatic(
        SUPERVISOR_QUEUE_LENGTH,
        sizeof(SupervisorMessage),
        g_queue_storage,
        &g_queue_control
    );
#else
    g_queue = xQueueCreate(SUPERVISOR_QUEUE_LENGTH, sizeof(SupervisorMessage));
#endif
    if (g_queue == NULL)
    {
        return false;
    }

    g_supervisor.joint_drive =
        canopen_joint_drive_port_make(config->canopen_master);
    g_supervisor.commissioning =
        canopen_drive_commissioning_port_make(config->canopen_master);
    state_machine_init(&g_supervisor.machine);
    state_machine_enable_unified_execution(&g_supervisor.machine);
    g_supervisor.initialized = true;
    enter_active_state(&g_supervisor);
    return true;
}

bool supervisor_task_start(UBaseType_t priority)
{
    if (!g_supervisor.initialized || (g_task != NULL))
    {
        return false;
    }

    if (g_supervisor.config.validation_run_in_worker) {
        if (g_supervisor.config.validation_worker_program == NULL)
            return false;
        const UBaseType_t worker_priority =
            g_supervisor.config.validation_worker_priority;
        if (worker_priority >= priority) return false;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
        g_validation_task = xTaskCreateStatic(
            validation_worker_entry, "StorageValidate",
            VALIDATION_TASK_STACK_WORDS, &g_supervisor, worker_priority,
            g_validation_task_stack, &g_validation_task_control);
#else
        if (xTaskCreate(validation_worker_entry, "StorageValidate",
                        VALIDATION_TASK_STACK_WORDS, &g_supervisor,
                        worker_priority, &g_validation_task) != pdPASS)
            g_validation_task = NULL;
#endif
        if (g_validation_task == NULL) return false;
    }

#if (configSUPPORT_STATIC_ALLOCATION == 1)
    g_task = xTaskCreateStatic(
        supervisor_task_entry,
        "Supervisor",
        SUPERVISOR_TASK_STACK_WORDS,
        &g_supervisor,
        priority,
        g_task_stack,
        &g_task_control
    );
#else
    if (xTaskCreate(
            supervisor_task_entry,
            "Supervisor",
            SUPERVISOR_TASK_STACK_WORDS,
            &g_supervisor,
            priority,
            &g_task) != pdPASS)
    {
        g_task = NULL;
        return false;
    }
#endif
    return g_task != NULL;
}

bool supervisor_task_post(
    const SupervisorMessage *message,
    TickType_t wait_ticks
)
{
    return (message != NULL) && (g_queue != NULL) &&
           (xQueueSend(g_queue, message, wait_ticks) == pdPASS);
}

bool supervisor_task_post_from_isr(
    const SupervisorMessage *message,
    BaseType_t *higher_priority_task_woken
)
{
    return (message != NULL) && (g_queue != NULL) &&
           (xQueueSendFromISR(g_queue, message,
                              higher_priority_task_woken) == pdPASS);
}

bool supervisor_task_get_state(StateMachine *state)
{
    if ((state == NULL) || !g_supervisor.initialized)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *state = g_supervisor.machine;
    taskEXIT_CRITICAL();

    return true;
}

QueueHandle_t supervisor_task_queue(void)
{
    return g_queue;
}

bool supervisor_task_get_inputs(SupervisorInputSnapshot *inputs)
{
    if (inputs == NULL || !g_supervisor.initialized) return false;
    taskENTER_CRITICAL();
    *inputs = g_supervisor.gpio_inputs;
    taskEXIT_CRITICAL();
    return true;
}

bool supervisor_task_get_drive_snapshot(SupervisorDriveSnapshot *out)
{
    if (out == NULL || !g_supervisor.initialized) return false;
    taskENTER_CRITICAL();
    *out = g_supervisor.drive_snapshot;
    taskEXIT_CRITICAL();
    return true;
}

bool supervisor_task_post_hmi(SupervisorHmiCommand command, TickType_t wait_ticks)
{
    if ((unsigned int)command >= SUP_HMI_COMMAND_COUNT) return false;
    SupervisorMessage message = {0};
    message.type = SUPERVISOR_MESSAGE_HMI_COMMAND;
    message.data.hmi_command = command;
    return supervisor_task_post(&message, wait_ticks);
}

TaskHandle_t supervisor_task_handle(void)
{
    return g_task;
}

/* A separate low-priority CM7 task reads a snapshot, never live fields. */
bool supervisor_task_get_progress(SupervisionProgressSample *out)
{
    if (out == NULL || !g_supervisor.initialized) return false;
    taskENTER_CRITICAL();
    *out = g_supervisor.progress;
    taskEXIT_CRITICAL();
    return out->valid;
}

/* Only safe for same-scheduler readers. M4 must use shared IPC. */
bool supervisor_task_get_diagnostics(SupervisorDiagnostics *out)
{
    if (out == NULL || !g_supervisor.initialized) return false;
    taskENTER_CRITICAL();
    out->machine = g_supervisor.machine;
    out->boot = g_supervisor.boot;
    out->homing_phase = g_supervisor.homing.phase;
    out->homing_error = g_supervisor.homing.error;
    out->teaching = g_supervisor.teaching_outputs;
    out->validation = g_supervisor.validation_outputs;
    out->approach = g_supervisor.approach_outputs;
    out->execution = g_supervisor.path_execution_outputs;
    out->selected_geometry = g_supervisor.teaching.selected_segment_type;
    out->working_segment = g_supervisor.teaching.working_segment;
    out->captured_points = g_supervisor.teaching.captured_point_count;
    out->segment_count = g_supervisor.teaching.draft.segment_count;
    out->timer_missed_periods = g_supervisor.timer_missed_periods;
    out->timer_timeouts = g_supervisor.timer_timeouts;
    out->timer_overruns = g_supervisor.timer_overruns;
    out->timer_max_cycle_us = g_supervisor.timer_max_cycle_us;
    out->timer_service_gap_warnings = g_supervisor.timer_service_gap_warnings;
    out->timer_service_gap_severe = g_supervisor.timer_service_gap_severe;
    out->timer_wake_gap_warnings = g_supervisor.timer_wake_gap_warnings;
    out->timer_wake_gap_severe = g_supervisor.timer_wake_gap_severe;
    out->timer_late_wakeups = g_supervisor.timer_late_wakeups;
    out->timer_deadline_misses = g_supervisor.timer_deadline_misses;
    out->timer_invalid_timestamps = g_supervisor.timer_invalid_timestamps;
    out->timer_max_isr_to_wake_us = g_supervisor.timer_max_isr_to_wake_us;
    out->timer_max_isr_to_completion_us =
        g_supervisor.timer_max_isr_to_completion_us;
    out->timer_max_wake_gap_us = g_supervisor.timer_max_wake_gap_us;
    out->guidance_dry_run_ok_cycles =
        g_supervisor.guidance_dry_run_ok_cycles;
    out->guidance_dry_run_rejected_cycles =
        g_supervisor.guidance_dry_run_rejected_cycles;
    taskEXIT_CRITICAL();
    return true;
}
