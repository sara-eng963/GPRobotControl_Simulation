#include "state_machine.h"

#include <stddef.h>
#include <string.h>


void state_machine_init(
    StateMachine *machine
)
{
    /*
     * A null pointer is ignored because there is no context that can store
     * an error. The unit tests will cover this separately later.
     */
    if (machine == NULL)
    {
        return;
    }

    /*
     * Start from a completely deterministic, fail-safe baseline.
     *
     * In particular, this leaves all safety-health flags false:
     *
     *     motionPermitted = false
     *     drivesReady = false
     *     communicationHealthy = false
     *
     * Motion cannot be authorized until real status has been received.
     */
    memset(machine, 0, sizeof(*machine));

    machine->activeState = ROBOT_STATE_BOOT;
    machine->previousState = ROBOT_STATE_BOOT;
    machine->resumeState = ROBOT_STATE_IDLE;

    machine->executionMode = ROBOT_EXECUTION_NONE;

    machine->faultSeverity = ROBOT_FAULT_SEVERITY_NONE;
    machine->activeFaultCode = 0U;

    machine->recordedProgramAvailable = false;
    machine->validatedTrajectoryAvailable = false;
    machine->previewAccepted = false;
    machine->emergencyStopReleased = false;
    machine->emergencyResetAcknowledged = false;
    machine->homingRequired = false;
    machine->unifiedExecutionPolicy = false;

    machine->lastEvent = SUPERVISOR_EVENT_NONE;
    machine->transitionCount = 0U;
    machine->rejectedEventCount = 0U;

    machine->initialized = true;
}

void state_machine_enable_unified_execution(StateMachine *machine)
{
    if (machine != NULL && machine->initialized)
    {
        machine->unifiedExecutionPolicy = true;
    }
}

void state_machine_clear_program(
    StateMachine *machine
)
{
    if (machine == NULL)
    {
        return;
    }

    /*
     * Clear the supervisor's knowledge of the taught program and every
     * artifact derived from it.
     *
     * The coordinator will later use this request to clear the actual
     * Teaching and trajectory buffers as well.
     */
    machine->recordedProgramAvailable = false;
    machine->validatedTrajectoryAvailable = false;
    machine->previewAccepted = false;
}

void state_machine_update_safety(
    StateMachine *machine,
    const RobotSafetySnapshot *safety
)
{
    /*
     * The safety-monitoring task prepares a complete local snapshot and then
     * passes it to the Supervisor. The pure state machine only stores it.
     *
     * RTOS synchronization will be handled by the Supervisor task/queue
     * wrapper, not inside this platform-independent function.
     */
    if (
        (machine == NULL) ||
        (safety == NULL) ||
        !machine->initialized
    )
    {
        return;
    }

    machine->safety = *safety;
}

/*
 * Determine whether the physical safety conditions permit controlled
 * fault recovery.
 *
 * This helper deliberately does not inspect activeState because it is
 * used while the Supervisor is still in FAULT.
 */
static bool safety_ready_for_recovery(
    const StateMachine *machine
)
{
    if ((machine == NULL) || !machine->initialized)
    {
        return false;
    }

    return
        machine->safety.statusValid &&
        machine->safety.motionPermitted &&
        machine->safety.drivesReady &&
        machine->safety.communicationHealthy &&
        !machine->safety.estopActive &&
        !machine->safety.protectiveStopActive &&
        !machine->safety.globalFaultActive;
}

bool state_machine_can_move(
    const StateMachine *machine
)
{
    if (
        (machine == NULL) ||
        !machine->initialized
    )
    {
        return false;
    }

    /*
     * Fault and emergency states are never motion-authorized, even if an old
     * safety snapshot contained healthy values.
     */
    if (
        (machine->activeState == ROBOT_STATE_FAULT) ||
        (machine->activeState == ROBOT_STATE_EMERGENCY_STOP)
    )
    {
        return false;
    }

    return
        machine->safety.statusValid &&
        machine->safety.motionPermitted &&
        machine->safety.drivesReady &&
        machine->safety.communicationHealthy &&
        !machine->safety.estopActive &&
        !machine->safety.protectiveStopActive &&
        !machine->safety.globalFaultActive;
}

/*
 * Perform one global state transition.
 *
 * Keeping state assignment in one helper ensures that previousState and
 * transitionCount are updated consistently.
 */
static SupervisorResult transition_to(
    StateMachine *machine,
    RobotStateId nextState
)
{
    if (machine->activeState == nextState)
    {
        return SUPERVISOR_RESULT_HANDLED;
    }

    machine->previousState = machine->activeState;
    machine->activeState = nextState;
    machine->transitionCount++;

    return SUPERVISOR_RESULT_TRANSITIONED;
}


/*
 * Record a rejected event without modifying the active state.
 */
static SupervisorResult reject_event(
    StateMachine *machine
)
{
    machine->rejectedEventCount++;

    return SUPERVISOR_RESULT_REJECTED;
}

/*
 * States in which an operator-controlled pause is permitted.
 *
 * PAUSED is excluded because pressing the same button while PAUSED
 * requests a resume instead.
 */
static bool is_operator_pausable_state(RobotStateId state)
{
    switch (state)
    {
        case ROBOT_STATE_APPROACH:
        case ROBOT_STATE_PATH_EXECUTION:
        case ROBOT_STATE_ARC_STABILIZING:
        case ROBOT_STATE_WELDING:
        case ROBOT_STATE_RETRACTING:
            return true;

        default:
            return false;
    }
}

SupervisorResult state_machine_handle_event(
    StateMachine *machine,
    const SupervisorEvent *event
)
{
    /*
     * There is no valid context in which to record the rejection when either
     * pointer is null.
     */
    if ((machine == NULL) || (event == NULL))
    {
        return SUPERVISOR_RESULT_REJECTED;
    }

    if (!machine->initialized)
    {
        return SUPERVISOR_RESULT_REJECTED;
    }

    machine->lastEvent = event->type;

    /*
    * Completion, failure and controlled-abort events originate from a
    * particular state. Reject delayed events or events from another state.
    */
    if (
    ((event->type == SUPERVISOR_EVENT_STATE_COMPLETE) ||
     (event->type == SUPERVISOR_EVENT_STATE_FAILED) ||
     (event->type == SUPERVISOR_EVENT_VALIDATION_REJECTED) ||
     (event->type == SUPERVISOR_EVENT_STATE_ABORTED_HOME) ||
     (event->type == SUPERVISOR_EVENT_STATE_ABORTED_RESET)) &&
    (event->sourceState != machine->activeState)
    )
{
    return reject_event(machine);
}

    /*
     * A protective stop is not an operator Pause.
     *
     * It aborts the active execution and enters FAULT. Clearing the
     * protective device does not automatically resume robot motion.
     */
    if (
        event->type ==
        SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED
    )
    {
        machine->safety.protectiveStopActive = true;

        machine->faultSeverity =
            ROBOT_FAULT_SEVERITY_RECOVERABLE;

        machine->activeFaultCode =
            ROBOT_FAULT_CODE_PROTECTIVE_STOP;

        machine->executionMode = ROBOT_EXECUTION_NONE;

        /*
         * Do not retain an automatic-resume destination after a
         * protective stop.
         */
        machine->resumeState = ROBOT_STATE_IDLE;

        return transition_to(
            machine,
            ROBOT_STATE_FAULT
        );
    }

    if (
    event->type ==
    SUPERVISOR_EVENT_PROTECTIVE_STOP_CLEARED
)
{
    machine->safety.protectiveStopActive = false;

    /*
     * A protective input may clear while a higher-priority E-stop is
     * still active. Accept the input change without leaving the current
     * fault or emergency state.
     */
    if (
        (machine->activeState != ROBOT_STATE_FAULT) &&
        (machine->activeState != ROBOT_STATE_EMERGENCY_STOP)
    )
    {
        return reject_event(machine);
    }

    return SUPERVISOR_RESULT_HANDLED;
}

        /*
     * E-stop assertion has global priority over the normal operating
     * sequence. The independent safety circuit must stop the drives;
     * this software transition records the condition and prevents any
     * further supervisory motion command.
     */
    if (event->type == SUPERVISOR_EVENT_ESTOP_ASSERTED)
    {
        machine->safety.estopActive = true;

        machine->faultSeverity =
            ROBOT_FAULT_SEVERITY_EMERGENCY;

        machine->activeFaultCode =
            ROBOT_FAULT_CODE_EMERGENCY_STOP;

        machine->executionMode = ROBOT_EXECUTION_NONE;
        machine->resumeState = ROBOT_STATE_IDLE;
        machine->emergencyStopReleased = false;
        machine->emergencyResetAcknowledged = false;
        machine->homingRequired = true;

        return transition_to(
            machine,
            ROBOT_STATE_EMERGENCY_STOP
        );
    }

    /*
     * Physical E-stop release removes the active input but does not
     * acknowledge the emergency or authorize movement.
     */
    if (event->type == SUPERVISOR_EVENT_ESTOP_RELEASED)
    {
        machine->safety.estopActive = false;

        if (
            machine->activeState !=
            ROBOT_STATE_EMERGENCY_STOP
        )
        {
            return reject_event(machine);
        }

        machine->emergencyStopReleased = true;
        return SUPERVISOR_RESULT_HANDLED;
    }
        /*
     * Handle asynchronous faults reported by monitoring, communication,
     * drive, welding, or other system tasks.
     */
    if (event->type == SUPERVISOR_EVENT_FAULT_DETECTED)
    {
        machine->safety.globalFaultActive = true;

        machine->faultSeverity = event->faultSeverity;

        /*
         * A fault without a supplied classification is conservatively
         * treated as recoverable.
         */
        if (
            machine->faultSeverity ==
            ROBOT_FAULT_SEVERITY_NONE
        )
        {
            machine->faultSeverity =
                ROBOT_FAULT_SEVERITY_RECOVERABLE;
        }

        machine->activeFaultCode = event->faultCode;

        machine->executionMode = ROBOT_EXECUTION_NONE;
        machine->resumeState = ROBOT_STATE_IDLE;

        if (
            machine->faultSeverity ==
            ROBOT_FAULT_SEVERITY_EMERGENCY
        )
        {
            return transition_to(
                machine,
                ROBOT_STATE_EMERGENCY_STOP
            );
        }

        return transition_to(
            machine,
            ROBOT_STATE_FAULT
        );
    }
    /*
 * PAUSE/RESUME is context-dependent:
 *
 * - In a moving state, it pauses and remembers that state.
 * - In PAUSED, it resumes only when motion is permitted.
 * - In all other states, it is rejected.
 *
 * The motion controller will later be responsible for holding the
 * current joint command and preserving the trajectory sample index.
 * The Supervisor only manages the state transition policy.
 */
if (event->type == SUPERVISOR_EVENT_PAUSE_RESUME)
{
    if (machine->activeState == ROBOT_STATE_PAUSED)
    {
        /*
         * Do not resume automatically while a safety condition,
         * communication problem, or drive problem prevents motion.
         */
        if (!state_machine_can_move(machine))
        {
            return reject_event(machine);
        }

        /*
         * resumeState must contain a valid motion state. This also
         * prevents corrupted state data from initiating motion.
         */
        if (!is_operator_pausable_state(machine->resumeState))
        {
            return reject_event(machine);
        }

        RobotStateId stateToResume = machine->resumeState;

        /*
         * Clear resumeState before transitioning so that an old state
         * cannot accidentally be reused by a later pause operation.
         */
        machine->resumeState = ROBOT_STATE_IDLE;
        transition_to(machine, stateToResume);

        return SUPERVISOR_RESULT_TRANSITIONED;
    }

    if (is_operator_pausable_state(machine->activeState))
    {
        machine->resumeState = machine->activeState;
        transition_to(machine, ROBOT_STATE_PAUSED);

        return SUPERVISOR_RESULT_TRANSITIONED;
    }

    return reject_event(machine);
}

         /*
     * A failure reported by the currently active state transfers control
     * to fault handling. Store its classification and diagnostic code
     * before changing state.
     */
    if (event->type == SUPERVISOR_EVENT_STATE_FAILED)
    {
        machine->faultSeverity = event->faultSeverity;

        /*
         * If a state does not provide a classification, conservatively
         * treat the failure as recoverable.
         */
        if (
            machine->faultSeverity ==
            ROBOT_FAULT_SEVERITY_NONE
        )
        {
            machine->faultSeverity =
                ROBOT_FAULT_SEVERITY_RECOVERABLE;
        }

        machine->activeFaultCode = event->faultCode;

        /*
         * A failed state cannot retain an active Preview or Production
         * execution mode.
         */
        machine->executionMode = ROBOT_EXECUTION_NONE;

        if (
            machine->faultSeverity ==
            ROBOT_FAULT_SEVERITY_EMERGENCY
        )
        {
            return transition_to(
                machine,
                ROBOT_STATE_EMERGENCY_STOP
            );
        }

        return transition_to(
            machine,
            ROBOT_STATE_FAULT
        );
    }
    /*
 * Controlled Approach abort outcomes.
 *
 * An operator HOME or RESET request must not make the Supervisor leave a
 * moving state immediately. The Approach module first performs its safe
 * stop procedure. It then sends one of these result events.
 */
if (event->type == SUPERVISOR_EVENT_STATE_ABORTED_HOME)
{
    if ((machine->activeState != ROBOT_STATE_APPROACH) &&
        (machine->activeState != ROBOT_STATE_PATH_EXECUTION))
    {
        return reject_event(machine);
    }

    /*
     * HOME preserves the taught and validated program. Only the active
     * execution is cancelled.
     */
    machine->executionMode = ROBOT_EXECUTION_NONE;
    machine->resumeState = ROBOT_STATE_IDLE;

    return transition_to(
        machine,
        ROBOT_STATE_HOMING
    );
}

if (event->type == SUPERVISOR_EVENT_STATE_ABORTED_RESET)
{
    if ((machine->activeState != ROBOT_STATE_APPROACH) &&
        (machine->activeState != ROBOT_STATE_PATH_EXECUTION))
    {
        return reject_event(machine);
    }

    /*
     * RESET cancels execution and deletes the current program.
     */
    state_machine_clear_program(machine);

    machine->executionMode = ROBOT_EXECUTION_NONE;
    machine->resumeState = ROBOT_STATE_IDLE;

    return transition_to(
        machine,
        ROBOT_STATE_IDLE
    );
}
        /*
     * RESET directly clears the program in nonmoving operational states.
     *
     * Reset during active robot motion is rejected for now because the
     * controller must first perform a controlled stop.
     */
    if (event->type == SUPERVISOR_EVENT_RESET)
    {
        switch (machine->activeState)
        {
            case ROBOT_STATE_IDLE:
                state_machine_clear_program(machine);

                machine->executionMode = ROBOT_EXECUTION_NONE;
                machine->resumeState = ROBOT_STATE_IDLE;

                return SUPERVISOR_RESULT_HANDLED;

            case ROBOT_STATE_TEACHING:
            case ROBOT_STATE_PATH_VALIDATION:
                state_machine_clear_program(machine);

                machine->executionMode = ROBOT_EXECUTION_NONE;
                machine->resumeState = ROBOT_STATE_IDLE;

                return transition_to(
                    machine,
                    ROBOT_STATE_IDLE
                );
            case ROBOT_STATE_FAULT:
                /*
                 * Reset cannot recover the machine until every monitored
                 * safety condition is healthy.
                 */
                if (!safety_ready_for_recovery(machine))
                {
                    return reject_event(machine);
                }

                /*
                 * A fault-interrupted trajectory is no longer safe to
                 * replay without teaching and validation again.
                 */
                state_machine_clear_program(machine);

                machine->executionMode = ROBOT_EXECUTION_NONE;
                machine->resumeState = ROBOT_STATE_IDLE;

                machine->faultSeverity =
                    ROBOT_FAULT_SEVERITY_NONE;

                machine->activeFaultCode =
                    ROBOT_FAULT_CODE_NONE;

                /*
                 * Begin controlled recovery through HOMING rather than
                 * claiming that the stopped robot is already IDLE.
                 */
                return transition_to(
                    machine,
                    ROBOT_STATE_HOMING
                );
            case ROBOT_STATE_EMERGENCY_STOP:
                if (!safety_ready_for_recovery(machine))
                {
                    return reject_event(machine);
                }

                state_machine_clear_program(machine);

                machine->executionMode = ROBOT_EXECUTION_NONE;
                machine->resumeState = ROBOT_STATE_IDLE;

                if (!machine->unifiedExecutionPolicy)
                {
                    machine->faultSeverity = ROBOT_FAULT_SEVERITY_NONE;
                    machine->activeFaultCode = ROBOT_FAULT_CODE_NONE;
                    return transition_to(machine, ROBOT_STATE_HOMING);
                }

                if (!machine->emergencyStopReleased)
                {
                    return reject_event(machine);
                }

                /* Reset acknowledges recovery but never starts motion. */
                machine->emergencyResetAcknowledged = true;
                machine->homingRequired = true;
                return SUPERVISOR_RESULT_HANDLED;

                default:
                /*
                * Do not immediately abandon an active motion state. Its module must
                * first stop safely and report STATE_ABORTED_RESET.
                */
                return reject_event(machine);

        }
    }

    /*
     * HOME commands physical motion and therefore requires a healthy
     * motion-safety snapshot.
     *
     * Unlike RESET, HOME preserves the recorded and validated program.
     */
    if (event->type == SUPERVISOR_EVENT_HOME)
    {
        if (machine->activeState == ROBOT_STATE_EMERGENCY_STOP &&
            machine->unifiedExecutionPolicy)
        {
            if (!machine->emergencyStopReleased ||
                !machine->emergencyResetAcknowledged ||
                !safety_ready_for_recovery(machine))
            {
                return reject_event(machine);
            }

            machine->faultSeverity = ROBOT_FAULT_SEVERITY_NONE;
            machine->activeFaultCode = ROBOT_FAULT_CODE_NONE;
            machine->emergencyResetAcknowledged = false;
            machine->homingRequired = false;
            return transition_to(machine, ROBOT_STATE_HOMING);
        }

        if (machine->activeState == ROBOT_STATE_HOMING)
        {
            /*
             * HOME is idempotent while homing is already in progress.
             */
            return SUPERVISOR_RESULT_HANDLED;
        }

        if (machine->activeState != ROBOT_STATE_IDLE)
        {
            return reject_event(machine);
        }

        if (!state_machine_can_move(machine))
        {
            return reject_event(machine);
        }

        machine->executionMode = ROBOT_EXECUTION_NONE;
        machine->resumeState = ROBOT_STATE_IDLE;

        return transition_to(
            machine,
            ROBOT_STATE_HOMING
        );
    }


    switch (machine->activeState)
    {
        case ROBOT_STATE_BOOT:
        {
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE)
            {
                return transition_to(
                    machine,
                    ROBOT_STATE_HOMING
                );
            }

            break;
        }

        case ROBOT_STATE_HOMING:
        {
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE)
            {
                return transition_to(
                    machine,
                    ROBOT_STATE_IDLE
                );
            }

            break;
        }

                case ROBOT_STATE_IDLE:
        {
            /*
             * SUPERVISOR_EVENT_TEACH is emitted by the HMI adapter only
             * after the operator selects LINE, ARC or CIRCLE.
             *
             * The selected geometry itself remains owned by TeachingState.
             */
            if (event->type == SUPERVISOR_EVENT_TEACH)
            {
                machine->executionMode =
                    ROBOT_EXECUTION_NONE;

                /*
                 * Entering Teaching means the draft may be modified.
                 * Any earlier validation or accepted Preview can therefore
                 * no longer authorize production execution.
                 */
                machine->validatedTrajectoryAvailable =
                    false;

                machine->previewAccepted =
                    false;

                return transition_to(
                    machine,
                    ROBOT_STATE_TEACHING
                );
            }

                        /*
             * VALIDATE/PREVIEW has context-dependent behavior:
             *
             *   - With no recorded draft, it is rejected.
             *   - With an unvalidated draft, validation must run first.
             *   - With a validated trajectory, it starts Preview Approach.
             *
             * This branch handles only the third condition.
             */
            if (
                event->type ==
                SUPERVISOR_EVENT_VALIDATE_PREVIEW
            )
            {
                if (
                    !machine->recordedProgramAvailable ||
                    !machine->validatedTrajectoryAvailable ||
                    !state_machine_can_move(machine)
                )
                {
                    return reject_event(machine);
                }

                machine->executionMode =
                    ROBOT_EXECUTION_PREVIEW;

                /*
                 * Starting a new Preview invalidates any earlier Preview
                 * acceptance until the complete new Preview finishes safely.
                 */
                machine->previewAccepted =
                    false;

                return transition_to(
                    machine,
                    ROBOT_STATE_APPROACH
                );
            }
                        /*
             * Production execution requires:
             *
             *   - a recorded program,
             *   - a validated trajectory,
             *   - a successfully completed Preview,
             *   - current motion permission.
             */
            if (
                event->type ==
                SUPERVISOR_EVENT_START_REPLAY
            )
            {
                if (
                    !machine->recordedProgramAvailable ||
                    !machine->validatedTrajectoryAvailable ||
                    !machine->previewAccepted ||
                    !state_machine_can_move(machine)
                )
                {
                    return reject_event(machine);
                }

                machine->executionMode =
                    ROBOT_EXECUTION_PRODUCTION;

                return transition_to(
                    machine,
                    ROBOT_STATE_APPROACH
                );
            }

            break;
        }
            case ROBOT_STATE_TEACHING:
        {
            /*
             * Teaching reports COMPLETE only after its internal logic has:
             *
             *   - captured a structurally complete draft,
             *   - accepted Validate/Preview,
             *   - frozen the submitted Teaching program.
             *
             * Full geometric and motion feasibility is still owned by
             * PATH_VALIDATION.
             */
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE)
            {
                machine->recordedProgramAvailable =
                    true;

                machine->validatedTrajectoryAvailable =
                    false;

                machine->previewAccepted =
                    false;

                machine->executionMode =
                    ROBOT_EXECUTION_NONE;

                return transition_to(
                    machine,
                    ROBOT_STATE_PATH_VALIDATION
                );
            }

            break;
        }
                case ROBOT_STATE_PATH_VALIDATION:
        {
            /*
             * The trajectory generation and validation pipeline completed
             * successfully. Return to IDLE and wait for the operator to
             * explicitly request Preview.
             */
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE)
            {
                machine->recordedProgramAvailable =
                    true;

                machine->validatedTrajectoryAvailable =
                    true;

                machine->previewAccepted =
                    false;

                machine->executionMode =
                    ROBOT_EXECUTION_NONE;

                return transition_to(
                    machine,
                    ROBOT_STATE_IDLE
                );
            }

            /*
             * The validation pipeline ran correctly but rejected the path.
             *
             * This is an operator-correctable outcome, not a system fault.
             * Retain the draft and return to IDLE so the HMI can report the
             * validation reason and offer retry or Reset.
             */
            if (
                event->type ==
                SUPERVISOR_EVENT_VALIDATION_REJECTED
            )
            {
                machine->recordedProgramAvailable =
                    true;

                machine->validatedTrajectoryAvailable =
                    false;

                machine->previewAccepted =
                    false;

                machine->executionMode =
                    ROBOT_EXECUTION_NONE;

                return transition_to(
                    machine,
                    ROBOT_STATE_IDLE
                );
            }

            break;
        }
        case ROBOT_STATE_APPROACH:
        {
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE)
            {
                if (machine->unifiedExecutionPolicy &&
                    ((machine->executionMode == ROBOT_EXECUTION_PREVIEW) ||
                     (machine->executionMode == ROBOT_EXECUTION_PRODUCTION)))
                {
                    return transition_to(
                        machine,
                        ROBOT_STATE_PATH_EXECUTION
                    );
                }

                if (machine->executionMode == ROBOT_EXECUTION_PREVIEW)
                    return transition_to(machine, ROBOT_STATE_WELDING);
                if (machine->executionMode == ROBOT_EXECUTION_PRODUCTION)
                    return transition_to(machine, ROBOT_STATE_ARC_STABILIZING);

                /*
                 * Reaching Approach without a valid execution mode indicates
                 * a supervisory sequence error.
                 */
                return reject_event(machine);
            }

            break;
        }

        case ROBOT_STATE_PATH_EXECUTION:
        {
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE)
            {
                /*
                 * In PREVIEW mode, this state executes the same trajectory
                 * while the coordinator guarantees welding output is disabled.
                 *
                 * In PRODUCTION mode, it performs actual welding execution.
                 */
                if (
                    (machine->executionMode !=
                     ROBOT_EXECUTION_PREVIEW) &&
                    (machine->executionMode !=
                     ROBOT_EXECUTION_PRODUCTION)
                )
                {
                    return reject_event(machine);
                }

                RobotExecutionMode completedMode =
                    machine->executionMode;

                if (
                    (completedMode !=
                     ROBOT_EXECUTION_PREVIEW) &&
                    (completedMode !=
                     ROBOT_EXECUTION_PRODUCTION)
                )
                {
                    return reject_event(machine);
                }

                /*
                 * Preview is accepted only after:
                 *
                 *   - Approach completed,
                 *   - the complete path executed,
                 *   - retraction completed,
                 *   - no failure or safety event interrupted the sequence.
                 */
                if (
                    completedMode ==
                    ROBOT_EXECUTION_PREVIEW
                )
                {
                    machine->previewAccepted =
                        machine->validatedTrajectoryAvailable;
                }

                machine->executionMode =
                    ROBOT_EXECUTION_NONE;

                return transition_to(
                    machine,
                    ROBOT_STATE_HOMING
                );
            }

            break;
        }

        case ROBOT_STATE_ARC_STABILIZING:
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE &&
                machine->executionMode == ROBOT_EXECUTION_PRODUCTION)
                return transition_to(machine, ROBOT_STATE_WELDING);
            break;

        case ROBOT_STATE_WELDING:
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE &&
                (machine->executionMode == ROBOT_EXECUTION_PREVIEW ||
                 machine->executionMode == ROBOT_EXECUTION_PRODUCTION))
                return transition_to(machine, ROBOT_STATE_RETRACTING);
            break;

        case ROBOT_STATE_RETRACTING:
            if (event->type == SUPERVISOR_EVENT_STATE_COMPLETE &&
                (machine->executionMode == ROBOT_EXECUTION_PREVIEW ||
                 machine->executionMode == ROBOT_EXECUTION_PRODUCTION))
            {
                if (machine->executionMode == ROBOT_EXECUTION_PREVIEW)
                    machine->previewAccepted =
                        machine->validatedTrajectoryAvailable;
                machine->executionMode = ROBOT_EXECUTION_NONE;
                return transition_to(machine, ROBOT_STATE_IDLE);
            }
            break;

        default:
        {
            break;
        }
    }

    return reject_event(machine);
}

const char *state_machine_state_name(RobotStateId state)
{
    static const char *names[] = {
        "BOOT", "HOMING", "IDLE", "TEACHING", "PATH_VALIDATION",
        "APPROACH", "PATH_EXECUTION", "ARC_STABILIZING", "WELDING",
        "PAUSED", "RETRACTING", "FAULT", "EMERGENCY_STOP"
    };
    return ((unsigned)state < (unsigned)ROBOT_STATE_COUNT)
        ? names[state] : "UNKNOWN_STATE";
}

const char *state_machine_event_name(SupervisorEventType event)
{
    static const char *names[] = {
        "NONE", "TEACH", "VALIDATE_PREVIEW", "START_REPLAY",
        "PAUSE_RESUME", "RESET", "HOME", "STATE_COMPLETE",
        "STATE_FAILED", "VALIDATION_REJECTED", "STATE_ABORTED_HOME",
        "STATE_ABORTED_RESET", "PROTECTIVE_STOP_ASSERTED",
        "PROTECTIVE_STOP_CLEARED", "ESTOP_ASSERTED", "ESTOP_RELEASED",
        "FAULT_DETECTED", "FAULT_ACKNOWLEDGED"
    };
    return ((unsigned)event < (sizeof(names) / sizeof(names[0])))
        ? names[event] : "UNKNOWN_EVENT";
}
