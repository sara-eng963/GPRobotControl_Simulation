#include "supervisor_task.h"

#include <stdio.h>
#include <string.h>

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
#define SUPERVISOR_MAX_MESSAGES_CYCLE 16U

typedef struct
{
    StateMachine machine;
    SupervisorInputSnapshot gpio_inputs;
    SupervisorTaskConfig config;

    BootState boot;
    HomingState homing;
    IdleState idle;
    TeachingState teaching;
    PathValidationState validation;
    ApproachState approach;
    PathExecutionState path_execution;
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

    context->entered_state = machine->activeState;
    SUPERVISOR_LOG("[SUPERVISOR] Enter %s\n",
                   state_machine_state_name(machine->activeState));

    switch (machine->activeState)
    {
        case ROBOT_STATE_BOOT:
            state_boot_enter(&context->boot);
            break;

        case ROBOT_STATE_HOMING:
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
            state_path_validation_enter(
                &context->validation,
                context->config.robot,
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
                &request,
                context->config.approach_config,
                context->config.approach_services
            );
            break;
        }

        case ROBOT_STATE_PATH_EXECUTION:
        {
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

static void handle_policy_event(
    SupervisorTaskContext *context,
    const SupervisorEvent *event
)
{
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
            context->approach_control.reset_requested = reset;
            context->approach_control.home_requested = !reset;
            return;
        }
        if (context->machine.activeState == ROBOT_STATE_PATH_EXECUTION)
        {
            context->path_execution_inputs.reset_requested = reset;
            context->path_execution_inputs.home_requested = !reset;
            return;
        }
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
        case SUP_HMI_SPEED_UP: teaching = TEACH_EVENT_SPEED_INCREASE; break;
        case SUP_HMI_SPEED_DOWN: teaching = TEACH_EVENT_SPEED_DECREASE; break;
        case SUP_HMI_SPEED_DEFAULT: teaching = TEACH_EVENT_SPEED_DEFAULT; break;
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
        case SUP_HMI_RESET:
            if (context->machine.activeState == ROBOT_STATE_TEACHING)
                teaching = TEACH_EVENT_RESET;
            else policy = SUPERVISOR_EVENT_RESET;
            break;
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
        case SUPERVISOR_MESSAGE_EVENT:
            handle_policy_event(context, &message->data.event);
            break;

        case SUPERVISOR_MESSAGE_SAFETY:
            state_machine_update_safety(
                &context->machine,
                &message->data.safety
            );
            break;

        case SUPERVISOR_MESSAGE_TEACHING_EVENT:
            context->pending_teaching_event =
                message->data.teaching_event;
            break;

        case SUPERVISOR_MESSAGE_TEACHING_RUNTIME:
            context->teaching_runtime =
                message->data.teaching_runtime;
            break;

        case SUPERVISOR_MESSAGE_APPROACH_CONTROL:
            context->approach_control =
                message->data.approach_control;
            break;

        case SUPERVISOR_MESSAGE_PATH_EXECUTION_INPUTS:
            context->path_execution_inputs =
                message->data.path_execution_inputs;
            break;

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
                context->config.ethercat_config
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
                context->config.robot
            );
            if (step == STATE_STEP_FAILED)
                emit_step_failure(context, (uint32_t)context->homing.error);
            else if (step == STATE_STEP_COMPLETE)
                emit_step_complete(context);
            break;

        case ROBOT_STATE_IDLE:
            step = state_idle_step(&context->idle, IDLE_COMMAND_NONE);
            if (step == STATE_STEP_FAILED)
                emit_step_failure(context, (uint32_t)context->idle.error);
            break;

        case ROBOT_STATE_TEACHING:
            step = state_teaching_step(
                &context->teaching,
                context->config.robot,
                &context->teaching_runtime,
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
            context->path_execution_inputs.motion_permission =
                context->machine.safety.motionPermitted;
            context->path_execution_inputs.drives_ready =
                context->machine.safety.drivesReady;
            context->path_execution_inputs.ethercat_healthy =
                context->machine.safety.ethercatHealthy;
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
            (void)step;
            break;
        }

        default:
            /* Legacy execution states are intentionally not dispatched. */
            break;
    }
}

static void supervisor_task_entry(void *argument)
{
    SupervisorTaskContext *context = argument;
    TickType_t last_wake = xTaskGetTickCount();
    SupervisorMessage message;

    for (;;)
    {
        /* Acquisition only: raw inputs do not authorize motion or create HMI
         * events until a separately tested input adapter is connected. */
        SupervisorInputSnapshot sampled = {0};
        if (context->config.io.read_inputs != NULL)
        {
            sampled.valid = context->config.io.read_inputs(
                context->config.io.context, &sampled);
        }
        taskENTER_CRITICAL();
        context->gpio_inputs = sampled;
        taskEXIT_CRITICAL();
        for (uint32_t count = 0U;
             count < SUPERVISOR_MAX_MESSAGES_CYCLE;
             ++count)
        {
            if (xQueueReceive(g_queue, &message, 0U) != pdPASS)
            {
                break;
            }
            handle_message(context, &message);
            /* Consume Teaching commands before another can overwrite the
             * single pending event slot (Record must never be coalesced). */
            if (context->pending_teaching_event != TEACH_EVENT_NONE) break;
        }

        run_active_state(context);
        if (context->config.io.write_outputs != NULL)
        {
            SupervisorOutputSnapshot outputs;
            supervisor_io_make_status(&context->machine, &outputs);
            /* The adapter owns output diagnostics; this result does not
             * silently acknowledge any fault or change motion permission. */
            (void)context->config.io.write_outputs(
                context->config.io.context, &outputs);
        }
        vTaskDelayUntil(&last_wake, context->config.period_ticks);
    }
}

bool supervisor_task_init(const SupervisorTaskConfig *config)
{
    if ((config == NULL) ||
        (config->ethercat_config == NULL) ||
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
        (config->validation_sample_budget == 0U) ||
        (config->period_ticks == 0U))
    {
        return false;
    }

    memset(&g_supervisor, 0, sizeof(g_supervisor));
    g_supervisor.config = *config;
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
    taskEXIT_CRITICAL();
    return true;
}
