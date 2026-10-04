#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "state_machine_types.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * Result of presenting one event to the Supervisor.
 *
 * REJECTED means the event is not permitted in the current state.
 * TRANSITIONED means the active global state changed.
 * HANDLED means the event was accepted without changing state.
 */
typedef enum
{
    SUPERVISOR_RESULT_REJECTED = 0,
    SUPERVISOR_RESULT_HANDLED,
    SUPERVISOR_RESULT_TRANSITIONED

} SupervisorResult;


/*
 * Pure global state-machine context.
 *
 * This structure contains no FreeRTOS handles and no hardware objects.
 * It can therefore be tested as ordinary C code.
 */
typedef struct
{
    RobotStateId activeState;
    RobotStateId previousState;

    /*
     * State to restore when PAUSE/RESUME is used.
     */
    RobotStateId resumeState;

    RobotExecutionMode executionMode;

    RobotFaultSeverity faultSeverity;

    uint32_t activeFaultCode;

    /*
     * Program lifecycle flags.
     */
    bool recordedProgramAvailable;
    bool validatedTrajectoryAvailable;
    bool previewAccepted;

    /* Emergency recovery latch: release -> Reset -> Home. */
    bool emergencyStopReleased;
    bool emergencyResetAcknowledged;
    bool homingRequired;

    /* Enabled by the RTOS wrapper after all new runtime modules are wired. */
    bool unifiedExecutionPolicy;

    /*
     * Safety conditions last observed by the Supervisor.
     */
    RobotSafetySnapshot safety;

    /*
     * Diagnostic information.
     */
    SupervisorEventType lastEvent;
    uint32_t transitionCount;
    uint32_t rejectedEventCount;

    bool initialized;

} StateMachine;



/*
 * Initialize the Supervisor in BOOT.
 */
void state_machine_init(
    StateMachine *machine
);


/*
 * Process one event and, when permitted, perform one global transition.
 *
 * This function does not block, access hardware, call FreeRTOS, or execute
 * a robot state. It only evaluates supervisory transition logic.
 */
SupervisorResult state_machine_handle_event(
    StateMachine *machine,
    const SupervisorEvent *event
);


/*
 * Update the Supervisor's latest software safety snapshot.
 *
 * Safety events are still delivered separately so transitions are explicit
 * and testable.
 */
void state_machine_update_safety(
    StateMachine *machine,
    const RobotSafetySnapshot *safety
);


/*
 * Clear the recorded-program and validation lifecycle flags.
 *
 * This is the logical effect of RESET. Physical motion behavior is handled
 * by the relevant motion state and safety layer.
 */
void state_machine_clear_program(
    StateMachine *machine
);


/*
 * Debug and test helpers.
 */
const char *state_machine_state_name(
    RobotStateId state
);


const char *state_machine_event_name(
    SupervisorEventType event
);


bool state_machine_can_move(
    const StateMachine *machine
);

/* Select the new PATH_EXECUTION and release->Reset->Home recovery policy. */
void state_machine_enable_unified_execution(StateMachine *machine);

#endif /* STATE_MACHINE_H */
