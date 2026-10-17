#ifndef HAND_GUIDING_COMMAND_GATE_H
#define HAND_GUIDING_COMMAND_GATE_H

#include "../StateMachine/supervisor_guidance_dry_run_port.h"
#include "../ControlCore/Config/robot_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 4E3A: pure C, no CANopen, no RTOS, no motor-write API.
 * Values are application-supplied, not physical safety thresholds. */
typedef struct {
    uint64_t maximum_candidate_age_us;
    double max_target_feedback_error_rad[ROBOT_DOF];
    double max_step_rad[ROBOT_DOF];
    double max_quantization_error_rad[ROBOT_DOF];
} HandGuidingCommandGateConfig;

typedef struct {
    bool have_previous;
    bool fault_latched;
    uint64_t last_produced_us;
    double last_command_rad[ROBOT_DOF];
} HandGuidingCommandGateState;

/* Neither conversion nor software rejection constitutes an E-stop/STO.
 * The Supervisor remains sole owner of joint_drive_port_send_targets().
 * A candidate belongs only to its producing control cycle. */
void hand_guiding_command_gate_reset(HandGuidingCommandGateState *state);
bool hand_guiding_command_gate_validate(
    HandGuidingCommandGateState *state,
    const HandGuidingCommandGateConfig *config,
    const SupervisorGuidanceDryRunCycle *cycle,
    const SupervisorGuidanceCandidate *candidate,
    int32_t target_units[ROBOT_DOF]);

#endif
