#include "state_path_execution.h"

#include <stddef.h>
#include <string.h>

#if defined(GP_HOST_TIMING)
#include "../../Simulation/Timing/host_timing_trace.h"
#define PATH_TIMING_EVENT(source) do { \
    gp_timing_begin(source); \
    gp_timing_end(source, true); \
} while (0)
#define PATH_TIMING_BEGIN(source) gp_timing_begin(source)
#define PATH_TIMING_END(source, ok) gp_timing_end(source, ok)
#define PATH_TIMING_SKIP(n) gp_timing_value(GP_TIMING_PATH_SAMPLES_SKIPPED, (n))
#define PATH_TIMING_FEEDBACK_AGE(axis, age) gp_timing_value( \
    (GpTimingSource)(GP_TIMING_PATH_FEEDBACK_AGE_AXIS_1 + (axis)), (age))
#else
#define PATH_TIMING_EVENT(source) ((void)0)
#define PATH_TIMING_BEGIN(source) ((void)0)
#define PATH_TIMING_END(source, ok) ((void)0)
#define PATH_TIMING_SKIP(n) ((void)0)
#define PATH_TIMING_FEEDBACK_AGE(axis, age) ((void)0)
#endif

/* Ten missed 2-ms updates; provisional until physical PDO latency is measured.
 * Host receive timestamps are quantized to the injected FreeRTOS tick. */
#define PATH_EXEC_FEEDBACK_MAX_AGE_US UINT64_C(20000)

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
        /* A failure already latched must not resend motor commands. */
        if (state->phase == PATH_EXEC_PHASE_FAILED)
            return STATE_STEP_FAILED;

        /* Welding relay OFF precedes all stop/hold requests. */
        if (state->services.set_wire_feed_enabled != NULL)
        {
            (void)state->services.set_wire_feed_enabled(
                false,
                state->services.context
            );
        }

        state->wire_feed_commanded = false;

        /* Software hold for runtime storage/communication/deadline faults.
         * Preflight rejection issued no motion: no unsolicited target. 
         * A successful callback acknowledges issuing the command, NOT
         * verified deceleration, and is never a certified safety stop. */
        if ((error == PATH_EXEC_ERR_STORAGE_READ ||
             error == PATH_EXEC_ERR_TRAJECTORY_NOT_READY ||
             error == PATH_EXEC_ERR_CYCLIC_FEEDBACK ||
             error == PATH_EXEC_ERR_MISSED_DEADLINE) &&
            state->phase == PATH_EXEC_PHASE_FOLLOW_TRAJECTORY)
        {
            bool acknowledged = false;
            if (state->services.controlled_stop == NULL ||
                !state->services.controlled_stop(
                    &acknowledged, state->services.context) ||
                !acknowledged)
            {
                /* Remain fail-closed; report the unacknowledged hold. */
                error = PATH_EXEC_ERR_COMMUNICATION;
            }
        }
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

static bool stream_ready(const PathExecutionState *state)
{
    return state->services.stream_ready == NULL ||
        state->services.stream_ready(state->sample_index,
            state->request.trajectory->sample_count, state->services.context);
}

void state_path_execution_release_stream(PathExecutionState *state)
{
    if (state != NULL && state->initialized && state->services.end_stream != NULL)
        state->services.end_stream(state->services.context);
}

static bool drive_valid(const PathExecutionState *state)
{
    return state != NULL && joint_drive_port_valid(&state->drive);
}

static bool master_motion_ready(
    const PathExecutionState *state,
    uint32_t now_ms
)
{
    return
        state != NULL &&
        drive_valid(state) &&
        joint_drive_port_ready(&state->drive, now_ms);
}

static bool feedback_watchdog(PathExecutionState *s, uint64_t now_us)
{
    if (!s->feedback_watch_started) return true;
    for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis) {
        JointDriveAxisFeedback feedback = {0};
        if (!joint_drive_port_read_axis(&s->drive, axis, &feedback) ||
            !feedback.feedback_valid || !feedback.operation_enabled)
            return false;
        s->latest_feedback[axis] = feedback;
        if (feedback.sequence != s->initial_tpdo_count[axis])
            s->fresh_since_start[axis] = true;
        /* The first feedback must arrive within the watchdog interval;
         * afterwards use the newest independent TPDO4 per-axis timestamp. */
        const uint64_t reference = s->fresh_since_start[axis]
            ? feedback.timestamp_us : s->feedback_watch_started_us;
        if (now_us < reference) return false;
        s->feedback_age_us[axis] = now_us - reference;
        PATH_TIMING_FEEDBACK_AGE(axis, s->feedback_age_us[axis]);
        const uint64_t max_age = s->config.feedback_max_age_us != 0U
            ? s->config.feedback_max_age_us : PATH_EXEC_FEEDBACK_MAX_AGE_US;
        if (s->feedback_age_us[axis] > max_age)
            return false;
    }
    return true;
}

static bool final_feedback_arrived(const PathExecutionState *s)
{
    for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis)
        if (!s->fresh_since_start[axis] ||
            s->latest_feedback[axis].sequence ==
                s->final_command_tpdo_count[axis]) return false;
    return true;
}

static bool send_execution_sample(PathExecutionState *s,
                                  const PvExecutionSample *sample)
{
    if (s == NULL || sample == NULL || !drive_valid(s)) return false;
    for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis)
        if (!joint_drive_port_feedback_sequence(&s->drive, axis,
                                                &s->final_command_tpdo_count[axis]))
            return false;
    return joint_drive_port_send_targets(&s->drive, sample->target_position_units);
}

void state_path_execution_enter(
    PathExecutionState *state,
    const JointDrivePort *drive,
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
    if (drive != NULL) state->drive = *drive;

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
        !drive_valid(state) ||
        !joint_drive_port_poll(&state->drive, inputs->now_ms)
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

    /* All feedback is monitored but none gates the next time-due command. */
    if (state->phase == PATH_EXEC_PHASE_FOLLOW_TRAJECTORY &&
        !feedback_watchdog(state, inputs->now_us))
        return fail(state, PATH_EXEC_ERR_CYCLIC_FEEDBACK);

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

            if (!stream_ready(state))
                return fail(state, PATH_EXEC_ERR_TRAJECTORY_NOT_READY);

            enter_phase(
                state,
                PATH_EXEC_PHASE_CONFIGURE_WIRE_FEED,
                inputs->now_ms
            );
            break;

        case PATH_EXEC_PHASE_CONFIGURE_WIRE_FEED:
            /* A refill can fail between phases. Resume must also check SRAM
             * before re-enabling production wire feed. */
            if (!stream_ready(state))
                return fail(state, PATH_EXEC_ERR_TRAJECTORY_NOT_READY);
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

            /* Final TPDO4 acknowledgement remains mandatory; it is not
             * used to pace intermediate trajectory commands. */
            if (
                state->sample_index >=
                state->request.trajectory->sample_count
            )
            {
                if (!final_feedback_arrived(state)) break;
                enter_phase(
                    state,
                    PATH_EXEC_PHASE_DISABLE_WIRE_FEED,
                    inputs->now_ms
                );
                break;
            }

            /* Sample n belongs to t0+n*period, NOT to last actual send.
             * Pause/resume starts a new segment at its unconsumed index. */
            if (!state->command_clock_started) {
                state->trajectory_t0_us = inputs->now_us;
                state->segment_start_index = state->sample_index;
                state->command_clock_started = true;
            }
            if (inputs->now_us < state->trajectory_t0_us)
                return fail(state, PATH_EXEC_ERR_COMMUNICATION);
            const uint32_t period_us =
                state->request.trajectory->sample_period_us;
            const uint64_t due_relative =
                (inputs->now_us - state->trajectory_t0_us) / period_us;
            const uint64_t raw_due =
                (uint64_t)state->segment_start_index + due_relative;
            const uint32_t count = state->request.trajectory->sample_count;
            /* The final sample is not exempt from expiration. If even
             * its deadline is >1 period old, never transmit that stale
             * position as if the trajectory completed correctly. */
            const uint64_t last_deadline = state->trajectory_t0_us +
                (uint64_t)(count - 1U - state->segment_start_index) * period_us;
            if (inputs->now_us > last_deadline + period_us) {
                const uint32_t skip = count - state->sample_index;
                if (state->services.skip_samples == NULL ||
                    !state->services.skip_samples(state->sample_index,
                                                  skip, state->services.context))
                    return fail(state, PATH_EXEC_ERR_STORAGE_READ);
                state->skipped_samples += skip;
                state->sample_index += skip;
                PATH_TIMING_SKIP(skip);
                return fail(state, PATH_EXEC_ERR_MISSED_DEADLINE);
            }
            const uint32_t due_index =
                raw_due >= count ? count - 1U : (uint32_t)raw_due;
            if (due_index < state->sample_index) {
                PATH_TIMING_EVENT(GP_TIMING_PATH_PERIOD_WAIT);
                break;
            }

            /* Drop only samples whose deadlines are more than one
             * period old. At most two due indices can remain. */
            const uint64_t first_deadline = state->trajectory_t0_us +
                (uint64_t)(state->sample_index - state->segment_start_index) *
                period_us;
            if (inputs->now_us > first_deadline + period_us) {
                uint64_t oldest_kept = (inputs->now_us -
                    state->trajectory_t0_us - period_us - 1U) / period_us + 1U;
                oldest_kept += state->segment_start_index;
                if (oldest_kept > due_index) oldest_kept = due_index;
                if (oldest_kept > state->sample_index) {
                    const uint32_t skip =
                        (uint32_t)(oldest_kept - state->sample_index);
                    if (state->services.skip_samples == NULL ||
                        !state->services.skip_samples(state->sample_index,
                                                      skip, state->services.context))
                        return fail(state, PATH_EXEC_ERR_STORAGE_READ);
                    state->skipped_samples += skip;
                    state->sample_index += skip;
                    PATH_TIMING_SKIP(skip);
                }
            }

            /* Catch up only nonexpired due samples. A heavy supervisor
             * overrun is represented as skips, never silent clock drift. */
            while (state->sample_index <= due_index) {
                PvExecutionSample sample;
                if (!state->services.read_sample(
                        state->sample_index, &sample, state->services.context))
                    return fail(state, PATH_EXEC_ERR_STORAGE_READ);
                PATH_TIMING_BEGIN(GP_TIMING_PATH_COMMAND);
                const bool command_sent = send_execution_sample(state, &sample);
                PATH_TIMING_END(GP_TIMING_PATH_COMMAND, command_sent);
                if (!command_sent)
                    return fail(state, PATH_EXEC_ERR_COMMUNICATION);
                if (!state->feedback_watch_started) {
                    state->feedback_watch_started = true;
                    state->feedback_watch_started_us = inputs->now_us;
                    for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis)
                        state->initial_tpdo_count[axis] =
                            state->final_command_tpdo_count[axis];
                }
                state->sample_index++;
            }
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
    outputs->skipped_samples = state->skipped_samples;
    memcpy(outputs->feedback_age_us, state->feedback_age_us,
           sizeof(outputs->feedback_age_us));
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
    state->feedback_watch_started = false;
    memset(state->fresh_since_start, 0, sizeof(state->fresh_since_start));

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
        "CYCLIC_FEEDBACK",
        "MISSED_DEADLINE"
    };

    return
        (error <= PATH_EXEC_ERR_MISSED_DEADLINE)
        ? names[error]
        : "UNKNOWN";
}
