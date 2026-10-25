#ifndef STATE_FAULT_H
#define STATE_FAULT_H

#include "../state_machine_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    FAULT_PHASE_ENTER = 0,
    FAULT_PHASE_SAFE_OUTPUTS,
    FAULT_PHASE_WAIT_CAUSE_CLEAR,
    FAULT_PHASE_WAIT_RESET,
    FAULT_PHASE_WAIT_HOME
} FaultPhase;

typedef struct
{
    bool fault_cause_active;
    bool safety_healthy;
} FaultInputs;

typedef bool (*FaultSafeOutputsFn)(void *context);

typedef struct
{
    /* Must de-energize the wire-feed relay as part of the safe outputs. */
    FaultSafeOutputsFn force_safe_outputs;
    void *context;
} FaultServices;

typedef struct
{
    FaultPhase phase;
    RobotStateId source_state;
    RobotFaultSeverity severity;
    uint32_t fault_code;
    bool homing_required;
    bool initialized;
    FaultServices services;
} FaultState;

void state_fault_enter(
    FaultState *state,
    RobotStateId source_state,
    RobotFaultSeverity severity,
    uint32_t fault_code,
    bool homing_required,
    const FaultServices *services
);

StateStepResult state_fault_step(
    FaultState *state,
    const FaultInputs *inputs
);

#endif
