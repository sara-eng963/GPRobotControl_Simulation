#include "state_paused.h"

#include <stddef.h>
#include <string.h>

void state_paused_enter(
    PausedState *state,
    RobotStateId interrupted_state,
    const PausedServices *services
)
{
    if (state == NULL)
        return;
    memset(state, 0, sizeof(*state));
    state->phase = PAUSED_PHASE_ENTER;
    state->interrupted_state = interrupted_state;
    state->initialized = services != NULL &&
        services->command_hold != NULL && services->wire_feed_off != NULL;
    if (services != NULL)
        state->services = *services;
}

StateStepResult state_paused_step(
    PausedState *state,
    const PausedInputs *inputs
)
{
    bool stopped = false;
    if (state == NULL || inputs == NULL || !state->initialized)
        return STATE_STEP_FAILED;
    if (inputs->estop_active || inputs->external_fault_active)
    {
        state->phase = PAUSED_PHASE_FAILED;
        return STATE_STEP_FAILED;
    }
    switch (state->phase)
    {
        case PAUSED_PHASE_ENTER:
            if (!state->services.wire_feed_off(state->services.context))
                goto failed;
            state->phase = PAUSED_PHASE_HOLD;
            break;
        case PAUSED_PHASE_HOLD:
            if (!state->services.command_hold(
                    &stopped, state->services.context))
                goto failed;
            if (stopped || (inputs->motion_stopped && inputs->hold_confirmed))
            {
                state->hold_established = true;
                state->phase = PAUSED_PHASE_READY;
            }
            break;
        case PAUSED_PHASE_READY:
            return STATE_STEP_RUNNING;
        case PAUSED_PHASE_FAILED:
        default:
            return STATE_STEP_FAILED;
    }
    return STATE_STEP_RUNNING;
failed:
    state->phase = PAUSED_PHASE_FAILED;
    return STATE_STEP_FAILED;
}
