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
    SupervisorTaskConfig config;

    BootState boot;
    HomingState homing;
    IdleState idle;
    TeachingState teaching;
    PathValidationState validation;
    ApproachState approach;

    TeachingRuntimeInputs teaching_runtime;
    TeachingEvent pending_teaching_event;
    ApproachControlInputs approach_control;

    TeachingOutputs teaching_outputs;
    PathValidationOutputs validation_outputs;
    ApproachOutputs approach_outputs;

    TaughtProgram submitted_program;

    RobotStateId entered_state;
    uint32_t next_program_id;
    bool initialized;
} SupervisorTaskContext;

static SupervisorTaskContext g_supervisor;

static StaticQueue_t g_queue_control;
static uint8_t g_queue_storage[
    SUPERVISOR_QUEUE_LENGTH * sizeof(SupervisorMessage)
];
static QueueHandle_t g_queue;

static StaticTask_t g_task_control;
static StackType_t g_task_stack[SUPERVISOR_TASK_STACK_WORDS];
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

static void handle_message(
    SupervisorTaskContext *context,
    const SupervisorMessage *message
)
{
    switch (message->type)
    {
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

        default:
            /* ARC_STABILIZING/WELDING/RETRACTING modules come next. */
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
        for (uint32_t count = 0U;
             count < SUPERVISOR_MAX_MESSAGES_CYCLE;
             ++count)
        {
            if (xQueueReceive(g_queue, &message, 0U) != pdPASS)
            {
                break;
            }
            handle_message(context, &message);
        }

        run_active_state(context);
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
        (config->validation_sample_budget == 0U) ||
        (config->period_ticks == 0U))
    {
        return false;
    }

    memset(&g_supervisor, 0, sizeof(g_supervisor));
    g_supervisor.config = *config;
    g_supervisor.next_program_id = config->first_program_id;
    g_supervisor.entered_state = ROBOT_STATE_COUNT;

    g_queue = xQueueCreateStatic(
        SUPERVISOR_QUEUE_LENGTH,
        sizeof(SupervisorMessage),
        g_queue_storage,
        &g_queue_control
    );
    if (g_queue == NULL)
    {
        return false;
    }

    state_machine_init(&g_supervisor.machine);
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

    g_task = xTaskCreateStatic(
        supervisor_task_entry,
        "Supervisor",
        SUPERVISOR_TASK_STACK_WORDS,
        &g_supervisor,
        priority,
        g_task_stack,
        &g_task_control
    );
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

TaskHandle_t supervisor_task_handle(void)
{
    return g_task;
}
