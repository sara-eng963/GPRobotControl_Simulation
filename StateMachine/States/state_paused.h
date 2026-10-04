#ifndef STATE_PAUSED_H
#define STATE_PAUSED_H

#include "../state_machine_types.h"

#include <stdbool.h>

typedef enum
{
    PAUSED_PHASE_ENTER = 0,
    PAUSED_PHASE_HOLD,
    PAUSED_PHASE_READY,
    PAUSED_PHASE_FAILED
} PausedPhase;

typedef struct
{
    bool motion_stopped;
    bool hold_confirmed;
    bool estop_active;
    bool external_fault_active;
} PausedInputs;

typedef bool (*PausedHoldFn)(bool *stopped, void *context);
typedef bool (*PausedWireFeedOffFn)(void *context);

typedef struct
{
    PausedHoldFn command_hold;
    PausedWireFeedOffFn wire_feed_off;
    void *context;
} PausedServices;

typedef struct
{
    PausedPhase phase;
    RobotStateId interrupted_state;
    bool hold_established;
    bool initialized;
    PausedServices services;
} PausedState;

void state_paused_enter(
    PausedState *state,
    RobotStateId interrupted_state,
    const PausedServices *services
);

StateStepResult state_paused_step(
    PausedState *state,
    const PausedInputs *inputs
);

#endif
