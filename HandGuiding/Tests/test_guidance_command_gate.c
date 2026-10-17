#include "../guidance_command_gate.h"
#include "../../ServoDrive/AvatarM/avatar_m_position.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    RobotConfig robot = {0};
    robot_config_init_ur5(&robot); /* SYNTHETIC host test ONLY */
    AvatarMPositionScale scale[ROBOT_DOF];
    SupervisorGuidanceDryRunCycle cy = {0};
    HandGuidingCommandGateConfig cfg = {0};
    HandGuidingCommandGateState state = {0};
    SupervisorGuidanceCandidate candidate = {0};
    int32_t targets[ROBOT_DOF] = {0};
    const double q[ROBOT_DOF] = {0.3, -1.0, 1.2, -0.5, 0.7, 0.1};
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        assert(avatar_m_position_scale_default(&scale[i], 50.0));
        assert(avatar_m_joint_rad_to_position_units(
            &scale[i], q[i], &cy.feedback[i].actual_position_units));
        cy.feedback[i].feedback_valid = true;
        cy.feedback[i].operation_enabled = true;
        cy.feedback[i].timestamp_us = 10000U;
        cfg.max_step_rad[i] = 0.02; /* synthetic constraints */
        cfg.max_target_feedback_error_rad[i] = 0.05;
        cfg.max_quantization_error_rad[i] = 0.00001;
        candidate.position_rad[i] = q[i];
    }
    cy.robot = &robot;
    cy.scales = scale;
    cy.now_us = 10000U;
    cy.period_s = 0.002;
    cy.teaching_active = true;
    cy.manual_guidance_requested = true;
    cy.motion_permitted = true;
    cy.robot_homed = true;
    cy.drives_ready = true;
    cy.communication_healthy = true;
    cy.safety_valid = true;
    cy.feedback_healthy = true;
    cfg.maximum_candidate_age_us = 1000U;
    candidate.valid = true;
    candidate.produced_us = cy.now_us;
    assert(hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    assert(state.have_previous && !state.fault_latched);
    /* Candidate cannot be replayed, even if the clock remains unchanged. */
    assert(!hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    assert(state.fault_latched && targets[0] == 0);
    hand_guiding_command_gate_reset(&state);
    candidate.produced_us = cy.now_us - 1U;
    assert(!hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    hand_guiding_command_gate_reset(&state);
    candidate.produced_us = cy.now_us;
    cy.estop_active = true;
    assert(!hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    hand_guiding_command_gate_reset(&state);
    cy.estop_active = false;
    cy.feedback[2].operation_enabled = false;
    assert(!hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    hand_guiding_command_gate_reset(&state);
    cy.feedback[2].operation_enabled = true;
    candidate.position_rad[3] = NAN;
    assert(!hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    hand_guiding_command_gate_reset(&state);
    candidate.position_rad[3] = q[3];
    candidate.position_rad[0] = q[0] + 0.06;
    assert(!hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets)); /* tracking */
    hand_guiding_command_gate_reset(&state);
    candidate.position_rad[0] = q[0];
    assert(hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    cy.now_us = 12000U;
    candidate.produced_us = 12000U;
    candidate.position_rad[0] = q[0] + 0.03;
    assert(!hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets)); /* target delta */
    hand_guiding_command_gate_reset(&state);
    candidate.position_rad[0] = q[0];
    assert(hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets));
    cy.now_us = 14000U;
    candidate.produced_us = 14000U;
    candidate.position_rad[0] = q[0] + 0.001;
    assert(hand_guiding_command_gate_validate(
        &state, &cfg, &cy, &candidate, targets)); /* good small step */
    puts("PASS: one-cycle candidate, conversion and fail-closed command gate");
    return 0;
}
