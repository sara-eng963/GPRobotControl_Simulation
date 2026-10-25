#include "state_emergency_stop.h"

#include <stddef.h>
#include <string.h>

void state_emergency_stop_enter(
    EmergencyStopState *state,
    RobotStateId interrupted_state,
    uint32_t now_ms,
    const EmergencyStopServices *services
)
{
    if (state == NULL)
        return;
    memset(state, 0, sizeof(*state));
    state->phase = ESTOP_PHASE_ACTIVE;
    state->interrupted_state = interrupted_state;
    state->entered_ms = now_ms;
    state->initialized = services != NULL &&
        services->force_safe_outputs != NULL;
    if (services != NULL)
        state->services = *services;
}

StateStepResult state_emergency_stop_step(
    EmergencyStopState *state,
    const EmergencyStopInputs *inputs
)
{
    if (state == NULL || inputs == NULL || !state->initialized)
        return STATE_STEP_FAILED;
    if (!state->services.force_safe_outputs(state->services.context))
        return STATE_STEP_FAILED;
    if (inputs->estop_active)
    {
        state->phase = ESTOP_PHASE_WAIT_RELEASE;
        return STATE_STEP_RUNNING;
    }
    if (state->phase == ESTOP_PHASE_ACTIVE ||
        state->phase == ESTOP_PHASE_WAIT_RELEASE)
        state->phase = ESTOP_PHASE_WAIT_RESET;
    if (state->phase == ESTOP_PHASE_WAIT_RESET &&
        inputs->reset_acknowledged && inputs->safety_healthy)
        state->phase = ESTOP_PHASE_WAIT_HOME;
    return STATE_STEP_RUNNING;
}
