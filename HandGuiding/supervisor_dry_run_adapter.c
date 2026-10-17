#include "supervisor_dry_run_adapter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool fail(HandGuidingDryRunState *state)
{
    if (state != NULL) {
        hand_guiding_pipeline_stop(&state->pipeline);
        state->pipeline.fault_latched = true;
        state->feedback_sequence_initialized = false;
        memset(&state->last_output, 0, sizeof(state->last_output));
        if (state->rejected_cycles < UINT32_MAX) ++state->rejected_cycles;
    }
    return false;
}

bool hand_guiding_dry_run_init(
    HandGuidingDryRunState *state, const HandGuidingDryRunConfig *config)
{
    if (state == NULL) return false;
    memset(state, 0, sizeof(*state));
    if (config == NULL || config->read_sample == NULL ||
        config->max_sensor_age_us == 0U ||
        config->max_sensor_age_us >= UINT32_C(0x80000000) ||
        config->max_feedback_age_us == 0U ||
        config->max_runtime_age_ms == 0U ||
        config->max_runtime_age_ms >= UINT32_C(0x80000000) ||
        config->max_safety_age_ms == 0U ||
        config->max_safety_age_ms >= UINT32_C(0x80000000))
        return false;
    state->config = *config;
    state->configured = true;
    return true;
}

void hand_guiding_dry_run_reset(HandGuidingDryRunState *state)
{
    if (state == NULL) return;
    hand_guiding_pipeline_reset(&state->pipeline);
    state->feedback_sequence_initialized = false;
    memset(state->last_feedback_sequence, 0,
           sizeof(state->last_feedback_sequence));
    memset(&state->last_output, 0, sizeof(state->last_output));
}

void hand_guiding_dry_run_stop(void *context)
{
    HandGuidingDryRunState *state = (HandGuidingDryRunState *)context;
    if (state == NULL) return;
    hand_guiding_pipeline_stop(&state->pipeline);
    state->feedback_sequence_initialized = false;
    memset(&state->last_output, 0, sizeof(state->last_output));
}

/* A callback from M7 Supervisor, never an independent RTOS task. */
bool hand_guiding_dry_run_tick(
    void *context, const SupervisorGuidanceDryRunCycle *cycle)
{
    HandGuidingDryRunState *state = (HandGuidingDryRunState *)context;
    if (state == NULL || !state->configured) return false;
    memset(&state->last_output, 0, sizeof(state->last_output));

    if (cycle == NULL || state->pipeline.fault_latched ||
        !cycle->feedback_healthy || cycle->scales == NULL ||
        cycle->robot == NULL || !cycle->teaching_active ||
        !cycle->manual_guidance_requested || !cycle->motion_permitted ||
        !cycle->robot_homed || !cycle->drives_ready ||
        !cycle->communication_healthy || !cycle->safety_valid ||
        cycle->estop_active || cycle->protective_stop_active ||
        cycle->global_fault_active || !isfinite(cycle->period_s) ||
        cycle->period_s <= 0.0 ||
        (uint32_t)(cycle->now_ms - cycle->runtime_timestamp_ms) >
            state->config.max_runtime_age_ms ||
        (uint32_t)(cycle->now_ms - cycle->safety_timestamp_ms) >
            state->config.max_safety_age_ms)
        return fail(state);

    /* All six samples are copied by the M7 Supervisor. The adapter
     * cannot obtain a send-target callback or perform CANopen operations. */
    double measured_q[ROBOT_DOF];
    uint32_t sequences[ROBOT_DOF];
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis) {
        const JointDriveAxisFeedback *fb = &cycle->feedback[axis];
        if (!fb->feedback_valid || !fb->operation_enabled ||
            fb->timestamp_us > cycle->now_us ||
            cycle->now_us - fb->timestamp_us >
                state->config.max_feedback_age_us ||
            !avatar_m_position_scale_valid(&cycle->scales[axis]) ||
            !avatar_m_position_units_to_joint_rad(
                &cycle->scales[axis], fb->actual_position_units,
                &measured_q[axis]))
            return fail(state);
        sequences[axis] = fb->sequence;
        if (state->config.require_new_feedback &&
            state->feedback_sequence_initialized &&
            (int32_t)(sequences[axis] -
                      state->last_feedback_sequence[axis]) <= 0)
            return fail(state);
    }

    FtSensorSample sample = {0};
    if (!state->config.read_sample(state->config.sample_context, &sample))
        return fail(state);
    HandGuidingPipelineInput input = {0};
    input.sample = &sample;
    input.measured_joint_rad = measured_q;
    input.now_us = (uint32_t)cycle->now_us;
    input.max_sensor_age_us = state->config.max_sensor_age_us;
    input.period_s = cycle->period_s;
    input.gates.teaching_active = true;
    input.gates.motion_permitted = true;
    input.gates.robot_homed = true;
    input.gates.drives_ready = true;
    input.gates.feedback_fresh = true;

    bool valid;
    if (!state->pipeline.active) {
        valid = hand_guiding_pipeline_arm(
            &state->pipeline, &state->config.pipeline_config,
            cycle->robot, &input);
        /* Arm only seeds joint references from feedback; never output. */
    } else {
        valid = hand_guiding_pipeline_step(
            &state->pipeline, cycle->robot, &input,
            &state->last_output);
    }
    if (!valid) return fail(state);
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis)
        state->last_feedback_sequence[axis] = sequences[axis];
    state->feedback_sequence_initialized = true;
    if (state->accepted_cycles < UINT32_MAX) ++state->accepted_cycles;
    return true;
}

SupervisorGuidanceDryRunPort hand_guiding_dry_run_port(
    HandGuidingDryRunState *state)
{
    SupervisorGuidanceDryRunPort port = {0};
    if (state == NULL || !state->configured) return port;
    port.context = state;
    port.tick = hand_guiding_dry_run_tick;
    port.stop = hand_guiding_dry_run_stop;
    return port;
}
