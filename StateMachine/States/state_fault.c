#include "state_fault.h"

#include <stddef.h>
#include <string.h>

void state_fault_enter(
    FaultState *state,
    RobotStateId source_state,
    RobotFaultSeverity severity,
    uint32_t fault_code,
    bool homing_required,
    const FaultServices *services
)
{
    if (state == NULL)
        return;
    memset(state, 0, sizeof(*state));
    state->phase = FAULT_PHASE_ENTER;
    state->source_state = source_state;
    state->severity = severity;
    state->fault_code = fault_code;
    state->homing_required = homing_required;
    state->initialized = services != NULL &&
        services->force_safe_outputs != NULL;
    if (services != NULL)
        state->services = *services;
}

StateStepResult state_fault_step(FaultState *state, const FaultInputs *inputs)
{
    if (state == NULL || inputs == NULL || !state->initialized)
        return STATE_STEP_FAILED;
    switch (state->phase)
    {
        case FAULT_PHASE_ENTER:
        case FAULT_PHASE_SAFE_OUTPUTS:
            if (!state->services.force_safe_outputs(
                    state->services.context))
                return STATE_STEP_FAILED;
            state->phase = FAULT_PHASE_WAIT_CAUSE_CLEAR;
            break;
        case FAULT_PHASE_WAIT_CAUSE_CLEAR:
            if (!inputs->fault_cause_active && inputs->safety_healthy)
                state->phase = FAULT_PHASE_WAIT_RESET;
            break;
        case FAULT_PHASE_WAIT_RESET:
        case FAULT_PHASE_WAIT_HOME:
            break;
        default:
            return STATE_STEP_FAILED;
    }
    return STATE_STEP_RUNNING;
}
