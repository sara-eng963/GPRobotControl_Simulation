#ifndef STATE_EMERGENCY_STOP_H
#define STATE_EMERGENCY_STOP_H

#include "../state_machine_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ESTOP_PHASE_ACTIVE = 0,
    ESTOP_PHASE_WAIT_RELEASE,
    ESTOP_PHASE_WAIT_RESET,
    ESTOP_PHASE_WAIT_HOME
} EmergencyStopPhase;

typedef struct
{
    bool estop_active;
    bool reset_acknowledged;
    bool safety_healthy;
} EmergencyStopInputs;

typedef bool (*EmergencySafeOutputsFn)(void *context);

typedef struct
{
    /* Must de-energize the wire-feed relay as part of the safe outputs. */
    EmergencySafeOutputsFn force_safe_outputs;
    void *context;
} EmergencyStopServices;

typedef struct
{
    EmergencyStopPhase phase;
    RobotStateId interrupted_state;
    uint32_t entered_ms;
    bool initialized;
    EmergencyStopServices services;
} EmergencyStopState;

void state_emergency_stop_enter(
    EmergencyStopState *state,
    RobotStateId interrupted_state,
    uint32_t now_ms,
    const EmergencyStopServices *services
);

StateStepResult state_emergency_stop_step(
    EmergencyStopState *state,
    const EmergencyStopInputs *inputs
);

#endif
