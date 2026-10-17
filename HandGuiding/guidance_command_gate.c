#include "guidance_command_gate.h"
#include "../ServoDrive/AvatarM/avatar_m_position.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

void hand_guiding_command_gate_reset(HandGuidingCommandGateState *state)
{
    if (state != NULL) memset(state, 0, sizeof(*state));
}

static bool reject(HandGuidingCommandGateState *state,
                   int32_t target_units[ROBOT_DOF])
{
    if (target_units != NULL) memset(target_units, 0,
                                     sizeof(int32_t) * ROBOT_DOF);
    if (state != NULL) state->fault_latched = true;
    return false;
}

bool hand_guiding_command_gate_validate(
    HandGuidingCommandGateState *state,
    const HandGuidingCommandGateConfig *cfg,
    const SupervisorGuidanceDryRunCycle *cycle,
    const SupervisorGuidanceCandidate *candidate,
    int32_t target_units[ROBOT_DOF])
{
    if (target_units != NULL) memset(target_units, 0,
                                     sizeof(int32_t) * ROBOT_DOF);
    if (state == NULL || state->fault_latched || cfg == NULL || cycle == NULL ||
        candidate == NULL || target_units == NULL ||
        cycle->robot == NULL || cycle->scales == NULL ||
        cycle->robot->identity.dof != ROBOT_DOF ||
        !candidate->valid || !cycle->teaching_active ||
        !cycle->manual_guidance_requested || !cycle->motion_permitted ||
        !cycle->robot_homed || !cycle->drives_ready ||
        !cycle->communication_healthy || !cycle->safety_valid ||
        !cycle->feedback_healthy || cycle->estop_active ||
        cycle->protective_stop_active || cycle->global_fault_active ||
        cfg->maximum_candidate_age_us == 0U ||
        candidate->produced_us > cycle->now_us ||
        cycle->now_us - candidate->produced_us >
            cfg->maximum_candidate_age_us ||
        /* Only output produced by THIS Supervisor cycle is dispatchable. */
        candidate->produced_us != cycle->now_us ||
        (state->have_previous &&
         candidate->produced_us <= state->last_produced_us) ||
        !isfinite(cycle->period_s) || cycle->period_s <= 0.0)
        return reject(state, target_units);

    int32_t converted[ROBOT_DOF];
    double positions[ROBOT_DOF];
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        const JointDriveAxisFeedback *fb = &cycle->feedback[i];
        const RobotConfig *robot = cycle->robot;
        const double q = candidate->position_rad[i];
        const double qd = candidate->velocity_rad_s[i];
        double actual_q = 0.0, rounded_q = 0.0;
        const double bound_step = robot->limits.qdMax[i] * cycle->period_s;
        if (!fb->feedback_valid || !fb->operation_enabled ||
            fb->timestamp_us > cycle->now_us ||
            !isfinite(q) || !isfinite(qd) ||
            !isfinite(robot->limits.qMin[i]) ||
            !isfinite(robot->limits.qMax[i]) ||
            !isfinite(robot->limits.qdMax[i]) ||
            robot->limits.qMin[i] >= robot->limits.qMax[i] ||
            robot->limits.qdMax[i] <= 0.0 ||
            !isfinite(cfg->max_target_feedback_error_rad[i]) ||
            cfg->max_target_feedback_error_rad[i] <= 0.0 ||
            !isfinite(cfg->max_step_rad[i]) || cfg->max_step_rad[i] <= 0.0 ||
            !isfinite(cfg->max_quantization_error_rad[i]) ||
            cfg->max_quantization_error_rad[i] <= 0.0 ||
            !isfinite(bound_step) || bound_step <= 0.0 ||
            q < robot->limits.qMin[i] || q > robot->limits.qMax[i] ||
            fabs(qd) > robot->limits.qdMax[i] ||
            !avatar_m_position_units_to_joint_rad(
                &cycle->scales[i], fb->actual_position_units, &actual_q) ||
            fabs(q - actual_q) > cfg->max_target_feedback_error_rad[i] ||
            (state->have_previous &&
             (fabs(q - state->last_command_rad[i]) >
                   cfg->max_step_rad[i] ||
              fabs(q - state->last_command_rad[i]) > bound_step)) ||
            !avatar_m_joint_rad_to_position_units(
                &cycle->scales[i], q, &converted[i]) ||
            !avatar_m_position_units_to_joint_rad(
                &cycle->scales[i], converted[i], &rounded_q) ||
            fabs(rounded_q - q) > cfg->max_quantization_error_rad[i] ||
            rounded_q < robot->limits.qMin[i] ||
            rounded_q > robot->limits.qMax[i])
            return reject(state, target_units);
        positions[i] = q;
    }
    /* Commit state and all six converted targets atomically on success. */
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        state->last_command_rad[i] = positions[i];
        target_units[i] = converted[i];
    }
    state->have_previous = true;
    state->last_produced_us = candidate->produced_us;
    return true;
}
