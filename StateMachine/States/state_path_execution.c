#include "state_path_execution.h"

#include <stddef.h>
#include <string.h>

#define PATH_EXEC_CYCLIC_FEEDBACK_TIMEOUT_MS 100U

static bool elapsed(uint32_t now, uint32_t start, uint32_t duration)
{
    return (uint32_t)(now - start) >= duration;
}

static void enter_phase(
    PathExecutionState *state,
    PathExecutionPhase phase,
    uint32_t now_ms
)
{
    state->phase = phase;
    state->phase_started_ms = now_ms;
}

static StateStepResult fail(
    PathExecutionState *state,
    PathExecutionError error
)
{
    if (state != NULL)
    {
        if (state->services.set_wire_feed_enabled != NULL)
        {
            (void)state->services.set_wire_feed_enabled(
                false,
                state->services.context
            );
        }

        state->wire_feed_commanded = false;
        state->error = error;
        state->result = PATH_EXEC_RESULT_FAILED;
        state->phase = PATH_EXEC_PHASE_FAILED;
    }

    return STATE_STEP_FAILED;
}

static bool request_valid(const PathExecutionRequest *request)
{
    return request != NULL &&
           request->trajectory != NULL &&
           request->trajectory_ready &&
           (request->mode == ROBOT_EXECUTION_PREVIEW ||
            request->mode == ROBOT_EXECUTION_PRODUCTION) &&
           request->trajectory->sample_count > 0U &&
           request->trajectory->sample_period_us ==
               PATH_VALIDATION_SAMPLE_PERIOD_US &&
           request->trajectory->program_id == request->expected_program_id &&
           request->trajectory->source_revision ==
               request->expected_source_revision &&
           request->trajectory->artifact_crc ==
               request->expected_artifact_crc;
}

static bool config_valid(const PathExecutionConfig *config)
{
    return config != NULL &&
           config->controlled_stop_timeout_ms > 0U &&
           config->clearance_stable_cycles > 0U;
}

static bool services_valid(const PathExecutionServices *services)
{
    return services != NULL &&
           services->read_sample != NULL &&
           services->set_wire_feed_enabled != NULL &&
           services->prepare_retraction != NULL &&
           services->step_retraction != NULL &&
           services->clearance_verified != NULL &&
           services->controlled_stop != NULL;
}

static bool master_valid(const CanopenMaster *master)
{
    return master != NULL &&
           master->initialized &&
           master->node_count == CANOPEN_MASTER_MAX_NODES;
}

static bool master_motion_ready(
    const PathExecutionState *state,
    uint32_t now_ms
)
{
    return
        state != NULL &&
        master_valid(state->master) &&
        canopen_master_ready_for_motion(
            state->master,
            now_ms
        );
}

static void capture_tpdo_counts(PathExecutionState *state)
{
    for (size_t axis = 0U;
         axis < CANOPEN_MASTER_MAX_NODES;
         ++axis)
    {
        state->command_tpdo_count[axis] =
            canopen_master_tpdo_rx_count(
                state->master,
                axis
            );
    }
}

static bool fresh_feedback_arrived(
    const PathExecutionState *state
)
{
    for (size_t axis = 0U;
         axis < CANOPEN_MASTER_MAX_NODES;
         ++axis)
    {
        if (
            canopen_master_tpdo_rx_count(
                state->master,
                axis
            )
            <=
            state->command_tpdo_count[axis]
        )
        {
            return false;
        }
    }

    return true;
}

static bool command_due(
    const PathExecutionState *state,
    uint32_t now_ms
)
{
    return
        !state->command_clock_started ||
        elapsed(
            now_ms,
            state->last_command_ms,
            state->command_period_ms
        );
}

static bool send_execution_sample(
    PathExecutionState *state,
    const PvExecutionSample *sample,
    uint32_t now_ms
)
{
    if (
        state == NULL ||
        sample == NULL ||
        !master_valid(state->master)
    )
    {
        return false;
    }

    capture_tpdo_counts(state);

    if (!canopen_master_send_target_cycle(
            state->master,
            sample->target_position_units,
            CANOPEN_MASTER_MAX_NODES))
    {
        return false;
    }

    state->last_command_ms = now_ms;
    state->command_clock_started = true;
    state->awaiting_feedback = true;

    return true;
}

void state_path_execution_enter(
    PathExecutionState *state,
    CanopenMaster *master,
    const PathExecutionRequest *request,
    const PathExecutionConfig *config,
    const PathExecutionServices *services
)
{
    if (state == NULL)
    {
        return;
    }

    memset(state, 0, sizeof(*state));

    state->phase = PATH_EXEC_PHASE_CHECK_PREREQUISITES;
    state->result = PATH_EXEC_RESULT_RUNNING;
    state->initialized = true;
    state->master = master;

    if (
        request != NULL &&
        request->trajectory != NULL &&
        request->trajectory->sample_period_us >= 1000U
    )
    {
        state->command_period_ms =
            request->trajectory->sample_period_us /
            1000U;
    }

    if (request != NULL)
    {
        state->request = *request;
    }

    if (config != NULL)
    {
        state->config = *config;
    }

    if (services != NULL)
    {
        state->services = *services;
    }
}

static StateStepResult begin_abort(
    PathExecutionState *state,
    PathExecutionAbortReason reason,
    PathExecutionError error,
    uint32_t now_ms
)
{
    (void)state->services.set_wire_feed_enabled(
        false,
        state->services.context
    );

    state->wire_feed_commanded = false;
    state->abort_reason = reason;
    state->error = error;

    enter_phase(
        state,
        PATH_EXEC_PHASE_CONTROLLED_STOP,
        now_ms
    );

    return STATE_STEP_RUNNING;
}

StateStepResult state_path_execution_step(
    PathExecutionState *state,
    const PathExecutionInputs *inputs,
    PathExecutionOutputs *outputs
)
{
    if (state == NULL || inputs == NULL)
    {
        return fail(state, PATH_EXEC_ERR_NULL_ARGUMENT);
    }

    if (!state->initialized)
    {
        return fail(state, PATH_EXEC_ERR_INVALID_REQUEST);
    }

    if (
        !master_valid(state->master) ||
        !canopen_master_poll(
            state->master,
            inputs->now_ms
        )
    )
    {
        return fail(
            state,
            PATH_EXEC_ERR_COMMUNICATION
        );
    }

    if (inputs->estop_active)
    {
        return fail(state, PATH_EXEC_ERR_ESTOP);
    }

    if (inputs->protective_stop_active || inputs->external_fault_active)
    {
        return fail(state, PATH_EXEC_ERR_EXTERNAL_FAULT);
    }

    if (
        inputs->reset_requested &&
        state->phase != PATH_EXEC_PHASE_CONTROLLED_STOP
    )
    {
        return begin_abort(
            state,
            PATH_EXEC_ABORT_RESET,
            PATH_EXEC_ERR_RESET_REQUESTED,
            inputs->now_ms
        );
    }

    if (
        inputs->home_requested &&
        state->phase != PATH_EXEC_PHASE_CONTROLLED_STOP
    )
    {
        return begin_abort(
            state,
            PATH_EXEC_ABORT_HOME,
            PATH_EXEC_ERR_HOME_REQUESTED,
            inputs->now_ms
        );
    }

    /*
     * Every trajectory command is six RPDO4 frames followed by one SYNC.
     * The next sample is forbidden until TPDO4 has advanced on all six axes.
     */
    if (state->awaiting_feedback)
    {
        if (fresh_feedback_arrived(state))
        {
            state->awaiting_feedback = false;
        }
        else if (
            elapsed(
                inputs->now_ms,
                state->last_command_ms,
                PATH_EXEC_CYCLIC_FEEDBACK_TIMEOUT_MS
            )
        )
        {
            return fail(
                state,
                PATH_EXEC_ERR_CYCLIC_FEEDBACK
            );
        }
        else
        {
            state_path_execution_get_outputs(
                state,
                outputs
            );

            return STATE_STEP_RUNNING;
        }
    }

    switch (state->phase)
    {
        case PATH_EXEC_PHASE_CHECK_PREREQUISITES:
            if (
                !config_valid(&state->config) ||
                state->command_period_ms == 0U
            )
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_INVALID_CONFIG
                );
            }

            if (
                !services_valid(&state->services) ||
                !request_valid(&state->request)
            )
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_INVALID_REQUEST
                );
            }

            if (
                !inputs->motion_permission ||
                !master_motion_ready(
                    state,
                    inputs->now_ms
                )
            )
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_MOTION_PERMISSION
                );
            }

            enter_phase(
                state,
                PATH_EXEC_PHASE_CONFIGURE_WIRE_FEED,
                inputs->now_ms
            );
            break;

        case PATH_EXEC_PHASE_CONFIGURE_WIRE_FEED:
            if (state->request.mode == ROBOT_EXECUTION_PREVIEW)
            {
                if (!state->services.set_wire_feed_enabled(
                        false,
                        state->services.context))
                {
                    return fail(
                        state,
                        PATH_EXEC_ERR_WIRE_FEED_RELAY
                    );
                }

                state->wire_feed_commanded = false;
            }
            else
            {
                if (!state->services.set_wire_feed_enabled(
                        true,
                        state->services.context))
                {
                    return fail(
                        state,
                        PATH_EXEC_ERR_WIRE_FEED_RELAY
                    );
                }

                state->wire_feed_commanded = true;
            }

            enter_phase(
                state,
                PATH_EXEC_PHASE_FOLLOW_TRAJECTORY,
                inputs->now_ms
            );
            break;

        case PATH_EXEC_PHASE_FOLLOW_TRAJECTORY:
        {
            if (
                !inputs->motion_permission ||
                !master_motion_ready(
                    state,
                    inputs->now_ms
                )
            )
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_MOTION_PERMISSION
                );
            }

            if (inputs->following_error)
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_FOLLOWING_ERROR
                );
            }

            /*
             * Reaching sample_count here means the final command has already
             * received fresh TPDO4 feedback, because awaiting_feedback is
             * processed before the phase switch above.
             */
            if (
                state->sample_index >=
                state->request.trajectory->sample_count
            )
            {
                enter_phase(
                    state,
                    PATH_EXEC_PHASE_DISABLE_WIRE_FEED,
                    inputs->now_ms
                );
                break;
            }

            if (!command_due(
                    state,
                    inputs->now_ms))
            {
                break;
            }

            PvExecutionSample sample;

            if (!state->services.read_sample(
                    state->sample_index,
                    &sample,
                    state->services.context))
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_STORAGE_READ
                );
            }

            if (!send_execution_sample(
                    state,
                    &sample,
                    inputs->now_ms))
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_COMMUNICATION
                );
            }

            state->sample_index++;
            break;
        }

        case PATH_EXEC_PHASE_DISABLE_WIRE_FEED:
            if (!state->services.set_wire_feed_enabled(
                    false,
                    state->services.context))
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_WIRE_FEED_RELAY
                );
            }

            state->wire_feed_commanded = false;

            enter_phase(
                state,
                PATH_EXEC_PHASE_PREPARE_RETRACTION,
                inputs->now_ms
            );
            break;

        case PATH_EXEC_PHASE_PREPARE_RETRACTION:
            if (!state->services.prepare_retraction(
                    state->services.context))
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_RETRACTION
                );
            }

            enter_phase(
                state,
                PATH_EXEC_PHASE_EXECUTE_RETRACTION,
                inputs->now_ms
            );
            break;

        case PATH_EXEC_PHASE_EXECUTE_RETRACTION:
        {
            const StateStepResult retract =
                state->services.step_retraction(
                    state->services.context
                );

            if (retract == STATE_STEP_FAILED)
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_RETRACTION
                );
            }

            if (retract == STATE_STEP_COMPLETE)
            {
                enter_phase(
                    state,
                    PATH_EXEC_PHASE_VERIFY_CLEARANCE,
                    inputs->now_ms
                );
            }

            break;
        }

        case PATH_EXEC_PHASE_VERIFY_CLEARANCE:
            if (state->services.clearance_verified(
                    state->services.context))
            {
                state->clearance_stable_cycles++;

                if (
                    state->clearance_stable_cycles >=
                    state->config.clearance_stable_cycles
                )
                {
                    state->phase =
                        PATH_EXEC_PHASE_COMPLETE;

                    state->result =
                        PATH_EXEC_RESULT_COMPLETE;
                }
            }
            else
            {
                state->clearance_stable_cycles = 0U;
            }
            break;

        case PATH_EXEC_PHASE_CONTROLLED_STOP:
        {
            bool stopped = false;

            if (!state->services.controlled_stop(
                    &stopped,
                    state->services.context))
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_COMMUNICATION
                );
            }

            if (stopped)
            {
                state->phase =
                    PATH_EXEC_PHASE_ABORTED;

                state->result =
                    PATH_EXEC_RESULT_ABORTED;
            }
            else if (
                elapsed(
                    inputs->now_ms,
                    state->phase_started_ms,
                    state->config.controlled_stop_timeout_ms
                )
            )
            {
                return fail(
                    state,
                    PATH_EXEC_ERR_STOP_TIMEOUT
                );
            }

            break;
        }

        case PATH_EXEC_PHASE_COMPLETE:
        case PATH_EXEC_PHASE_ABORTED:
            state_path_execution_get_outputs(
                state,
                outputs
            );
            return STATE_STEP_COMPLETE;

        case PATH_EXEC_PHASE_FAILED:
            state_path_execution_get_outputs(
                state,
                outputs
            );
            return STATE_STEP_FAILED;

        default:
            return fail(
                state,
                PATH_EXEC_ERR_INVALID_REQUEST
            );
    }

    state_path_execution_get_outputs(
        state,
        outputs
    );

    return STATE_STEP_RUNNING;
}

void state_path_execution_get_outputs(
    const PathExecutionState *state,
    PathExecutionOutputs *outputs
)
{
    if (state == NULL || outputs == NULL)
    {
        return;
    }

    outputs->phase = state->phase;
    outputs->result = state->result;
    outputs->error = state->error;
    outputs->abort_reason = state->abort_reason;
    outputs->sample_index = state->sample_index;
    outputs->sample_count =
        state->request.trajectory != NULL
        ? state->request.trajectory->sample_count
        : 0U;
    outputs->phase_started_ms = state->phase_started_ms;
    outputs->clearance_stable_cycles =
        state->clearance_stable_cycles;
    outputs->command_period_ms =
        state->command_period_ms;
    outputs->wire_feed_commanded =
        state->wire_feed_commanded;
}

void state_path_execution_resume(
    PathExecutionState *state,
    uint32_t now_ms
)
{
    if (
        state == NULL ||
        !state->initialized ||
        state->result != PATH_EXEC_RESULT_RUNNING
    )
    {
        return;
    }

    state->command_clock_started = false;
    state->awaiting_feedback = false;

    if (
        state->request.mode == ROBOT_EXECUTION_PRODUCTION &&
        state->phase == PATH_EXEC_PHASE_FOLLOW_TRAJECTORY
    )
    {
        state->wire_feed_commanded = false;

        enter_phase(
            state,
            PATH_EXEC_PHASE_CONFIGURE_WIRE_FEED,
            now_ms
        );
    }
}

const char *state_path_execution_phase_name(PathExecutionPhase phase)
{
    static const char *names[] = {
        "IDLE", "CHECK_PREREQUISITES", "CONFIGURE_WIRE_FEED",
        "FOLLOW_TRAJECTORY", "DISABLE_WIRE_FEED",
        "PREPARE_RETRACTION", "EXECUTE_RETRACTION", "VERIFY_CLEARANCE",
        "CONTROLLED_STOP", "COMPLETE", "ABORTED", "FAILED"
    };

    return
        (phase <= PATH_EXEC_PHASE_FAILED)
        ? names[phase]
        : "UNKNOWN";
}

const char *state_path_execution_error_name(PathExecutionError error)
{
    static const char *names[] = {
        "NONE",
        "NULL_ARGUMENT",
        "INVALID_CONFIG",
        "INVALID_REQUEST",
        "TRAJECTORY_NOT_READY",
        "TRAJECTORY_MISMATCH",
        "STORAGE_READ",
        "COMMUNICATION",
        "WIRE_FEED_RELAY",
        "MOTION_PERMISSION",
        "FOLLOWING_ERROR",
        "RETRACTION",
        "CLEARANCE",
        "STOP_TIMEOUT",
        "EXTERNAL_FAULT",
        "ESTOP",
        "RESET_REQUESTED",
        "HOME_REQUESTED",
        "CYCLIC_FEEDBACK"
    };

    return
        (error <= PATH_EXEC_ERR_CYCLIC_FEEDBACK)
        ? names[error]
        : "UNKNOWN";
}
