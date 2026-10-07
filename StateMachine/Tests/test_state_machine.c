#include "../state_machine.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>


static unsigned int testsPassed = 0U;
static unsigned int testsFailed = 0U;


static void check_true(
    bool condition,
    const char *testName
)
{
    if (condition)
    {
        printf("[PASS] %s\n", testName);
        testsPassed++;
    }
    else
    {
        printf("[FAIL] %s\n", testName);
        testsFailed++;
    }
}


static void test_initialization(void)
{
    StateMachine machine;

    /*
     * Fill with nonzero data first. This catches an initialization function
     * that accidentally leaves fields uninitialized.
     */
    memset(&machine, 0xA5, sizeof(machine));

    state_machine_init(&machine);

    check_true(
        machine.initialized,
        "Supervisor reports initialized"
    );

    check_true(
        machine.activeState == ROBOT_STATE_BOOT,
        "Initial active state is BOOT"
    );

    check_true(
        machine.previousState == ROBOT_STATE_BOOT,
        "Initial previous state is BOOT"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_IDLE,
        "Initial resume state is IDLE"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Initial execution mode is NONE"
    );

    check_true(
        machine.faultSeverity == ROBOT_FAULT_SEVERITY_NONE,
        "Initial fault severity is NONE"
    );

    check_true(
        machine.activeFaultCode == 0U,
        "Initial fault code is zero"
    );

    check_true(
        !machine.recordedProgramAvailable,
        "No recorded program exists initially"
    );

    check_true(
        !machine.validatedTrajectoryAvailable,
        "No validated trajectory exists initially"
    );

    check_true(
        !machine.previewAccepted,
        "Preview is not accepted initially"
    );

    check_true(
        !machine.safety.motionPermitted,
        "Motion is not permitted before safety status is received"
    );

    check_true(
        !machine.safety.drivesReady,
        "Drives are not assumed ready initially"
    );

    check_true(
        !machine.safety.communicationHealthy,
        "Communication is not assumed healthy initially"
    );

    check_true(
        machine.lastEvent == SUPERVISOR_EVENT_NONE,
        "Initial last event is NONE"
    );

    check_true(
        machine.transitionCount == 0U,
        "Initial transition count is zero"
    );

    check_true(
        machine.rejectedEventCount == 0U,
        "Initial rejected-event count is zero"
    );
}
static SupervisorEvent make_event(
    SupervisorEventType type,
    RobotStateId sourceState
)
{
    SupervisorEvent event;

    memset(&event, 0, sizeof(event));

    event.type = type;
    event.sourceState = sourceState;
    event.faultSeverity = ROBOT_FAULT_SEVERITY_NONE;

    return event;
}


static void test_normal_startup(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    state_machine_init(&machine);

    /*
     * BOOT reports successful completion.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "BOOT completion is accepted as a transition"
    );

    check_true(
        machine.previousState == ROBOT_STATE_BOOT,
        "BOOT is stored as the previous state"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "BOOT completion enters HOMING"
    );

    check_true(
        machine.lastEvent == SUPERVISOR_EVENT_STATE_COMPLETE,
        "BOOT completion is stored as the last event"
    );

    check_true(
        machine.transitionCount == 1U,
        "BOOT to HOMING increments transition count"
    );

    /*
     * HOMING reports successful completion.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "HOMING completion is accepted as a transition"
    );

    check_true(
        machine.previousState == ROBOT_STATE_HOMING,
        "HOMING is stored as the previous state"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "HOMING completion enters IDLE"
    );

    check_true(
        machine.lastEvent == SUPERVISOR_EVENT_STATE_COMPLETE,
        "HOMING completion is stored as the last event"
    );

    check_true(
        machine.transitionCount == 2U,
        "Normal startup performs exactly two transitions"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Normal startup does not select an execution mode"
    );

    check_true(
        machine.rejectedEventCount == 0U,
        "Normal startup rejects no events"
    );
}
static void test_startup_rejects_invalid_events(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    state_machine_init(&machine);

    /*
     * START/REPLAY must not bypass BOOT.
     */
    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "START/REPLAY is rejected during BOOT"
    );

    check_true(
        machine.activeState == ROBOT_STATE_BOOT,
        "Rejected START/REPLAY leaves state in BOOT"
    );

    check_true(
        machine.transitionCount == 0U,
        "Rejected START/REPLAY causes no transition"
    );

    check_true(
        machine.rejectedEventCount == 1U,
        "Rejected START/REPLAY increments rejection count"
    );

    /*
     * Simulate a stale or incorrectly sourced HOMING completion message
     * arriving while BOOT is still active.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Stale HOMING completion is rejected during BOOT"
    );

    check_true(
        machine.activeState == ROBOT_STATE_BOOT,
        "Stale completion cannot bypass BOOT"
    );

    check_true(
        machine.rejectedEventCount == 2U,
        "Stale BOOT event increments rejection count"
    );

    /*
     * Complete BOOT correctly so the next group can test HOMING.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        (result == SUPERVISOR_RESULT_TRANSITIONED) &&
        (machine.activeState == ROBOT_STATE_HOMING),
        "Valid BOOT completion still enters HOMING"
    );

    /*
     * TEACH must not bypass HOMING.
     */
    event = make_event(
        SUPERVISOR_EVENT_TEACH,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "TEACH is rejected during HOMING"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "Rejected TEACH leaves state in HOMING"
    );

    check_true(
        machine.transitionCount == 1U,
        "Rejected TEACH does not increment transition count"
    );

    check_true(
        machine.rejectedEventCount == 3U,
        "Rejected TEACH increments rejection count"
    );

    /*
     * Simulate a delayed BOOT completion message arriving after the
     * Supervisor has already entered HOMING.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Delayed BOOT completion is rejected during HOMING"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "Delayed BOOT completion cannot bypass HOMING"
    );

    check_true(
        machine.rejectedEventCount == 4U,
        "Delayed completion increments rejection count"
    );

    /*
     * Complete HOMING correctly and prove that prior rejected events did not
     * corrupt the normal startup sequence.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Valid HOMING completion remains accepted"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Valid HOMING completion still enters IDLE"
    );

    check_true(
        machine.transitionCount == 2U,
        "Only the two valid startup transitions are counted"
    );

    check_true(
        machine.rejectedEventCount == 4U,
        "Four invalid startup events were recorded"
    );
}

static void test_startup_failures_enter_fault(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    /*
     * ------------------------------------------------------------
     * BOOT failure
     * ------------------------------------------------------------
     */
    state_machine_init(&machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_FAILED,
        ROBOT_STATE_BOOT
    );

    event.faultSeverity =
        ROBOT_FAULT_SEVERITY_RECOVERABLE;

    event.faultCode =
        1001U;

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "BOOT failure causes a state transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "BOOT failure enters FAULT"
    );

    check_true(
        machine.previousState == ROBOT_STATE_BOOT,
        "BOOT is retained as the failed previous state"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_RECOVERABLE,
        "BOOT failure severity is retained"
    );

    check_true(
        machine.activeFaultCode == 1001U,
        "BOOT failure code is retained"
    );

    check_true(
        machine.transitionCount == 1U,
        "BOOT failure performs one transition"
    );

    /*
     * ------------------------------------------------------------
     * HOMING failure
     * ------------------------------------------------------------
     */
    state_machine_init(&machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    /*
     * The existing tests already verify this transition. Here it is
     * performed only to reach HOMING normally.
     */
    if (
        (result != SUPERVISOR_RESULT_TRANSITIONED) ||
        (machine.activeState != ROBOT_STATE_HOMING)
    )
    {
        check_true(
            false,
            "Test setup reaches HOMING"
        );

        return;
    }

    event = make_event(
        SUPERVISOR_EVENT_STATE_FAILED,
        ROBOT_STATE_HOMING
    );

    event.faultSeverity =
        ROBOT_FAULT_SEVERITY_RECOVERABLE;

    event.faultCode =
        2001U;

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "HOMING failure causes a state transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "HOMING failure enters FAULT"
    );

    check_true(
        machine.previousState == ROBOT_STATE_HOMING,
        "HOMING is retained as the failed previous state"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_RECOVERABLE,
        "HOMING failure severity is retained"
    );

    check_true(
        machine.activeFaultCode == 2001U,
        "HOMING failure code is retained"
    );

    check_true(
        machine.transitionCount == 2U,
        "BOOT completion and HOMING failure make two transitions"
    );
}
static void test_idle_command_gating_and_teaching_entry(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    state_machine_init(&machine);

    /*
     * Reach IDLE through the valid startup sequence.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    (void)state_machine_handle_event(
        &machine,
        &event
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    (void)state_machine_handle_event(
        &machine,
        &event
    );

    /*
     * START/REPLAY cannot run because no program has been recorded,
     * validated or previewed.
     */
    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "START/REPLAY is rejected when no program exists"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Rejected START/REPLAY leaves state in IDLE"
    );

    /*
     * VALIDATE/PREVIEW cannot start because there is no recorded draft.
     */
    event = make_event(
        SUPERVISOR_EVENT_VALIDATE_PREVIEW,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "VALIDATE/PREVIEW is rejected when no draft exists"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Rejected VALIDATE/PREVIEW leaves state in IDLE"
    );

    check_true(
        machine.rejectedEventCount == 2U,
        "Two invalid IDLE commands are recorded"
    );

    /*
     * The HMI adapter emits TEACH only after LINE, ARC or CIRCLE was selected.
     */
    event = make_event(
        SUPERVISOR_EVENT_TEACH,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Geometry selection starts TEACHING"
    );

    check_true(
        machine.activeState == ROBOT_STATE_TEACHING,
        "TEACH command enters TEACHING"
    );

    check_true(
        machine.previousState == ROBOT_STATE_IDLE,
        "IDLE is retained as the previous state"
    );

    check_true(
        machine.transitionCount == 3U,
        "Teaching entry is the third valid transition"
    );

    check_true(
        machine.rejectedEventCount == 2U,
        "Teaching entry does not change rejection count"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Teaching does not select Preview or Production mode"
    );

    check_true(
        !machine.recordedProgramAvailable,
        "Entering Teaching alone does not create a program"
    );

    check_true(
        !machine.validatedTrajectoryAvailable,
        "Entering Teaching alone does not validate a trajectory"
    );

    check_true(
        !machine.previewAccepted,
        "Entering Teaching alone does not accept Preview"
    );
}
static void test_teaching_completion_enters_path_validation(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    state_machine_init(&machine);

    /*
     * Reach IDLE.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    (void)state_machine_handle_event(
        &machine,
        &event
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    (void)state_machine_handle_event(
        &machine,
        &event
    );

    /*
     * Simulate LINE/ARC/CIRCLE selection and enter Teaching.
     */
    event = make_event(
        SUPERVISOR_EVENT_TEACH,
        ROBOT_STATE_IDLE
    );

    (void)state_machine_handle_event(
        &machine,
        &event
    );

    /*
     * Teaching reports completion only after Validate/Preview submits a
     * structurally complete draft.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_TEACHING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Teaching completion causes a transition"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_PATH_VALIDATION,
        "Teaching completion enters PATH_VALIDATION"
    );

    check_true(
        machine.previousState ==
        ROBOT_STATE_TEACHING,
        "TEACHING is retained as the previous state"
    );

    check_true(
        machine.recordedProgramAvailable,
        "Teaching completion marks a recorded program available"
    );

    check_true(
        !machine.validatedTrajectoryAvailable,
        "Teaching completion does not claim validation success"
    );

    check_true(
        !machine.previewAccepted,
        "Teaching completion does not claim Preview acceptance"
    );

    check_true(
        machine.executionMode ==
        ROBOT_EXECUTION_NONE,
        "Path Validation starts with no execution mode"
    );

    check_true(
        machine.transitionCount == 4U,
        "Teaching submission is the fourth valid transition"
    );

    check_true(
        machine.lastEvent ==
        SUPERVISOR_EVENT_STATE_COMPLETE,
        "Teaching completion is stored as the last event"
    );

    check_true(
        machine.rejectedEventCount == 0U,
        "Valid Teaching submission rejects no events"
    );
}
static void reach_path_validation(
    StateMachine *machine
)
{
    SupervisorEvent event;

    state_machine_init(machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );

    event = make_event(
        SUPERVISOR_EVENT_TEACH,
        ROBOT_STATE_IDLE
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_TEACHING
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );
}

static void test_path_validation_outcomes(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    /*
     * ------------------------------------------------------------
     * Successful validation
     * ------------------------------------------------------------
     */
    reach_path_validation(&machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_PATH_VALIDATION
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Valid path causes a state transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Valid path returns to IDLE"
    );

    check_true(
        machine.previousState ==
        ROBOT_STATE_PATH_VALIDATION,
        "PATH_VALIDATION is retained as previous state"
    );

    check_true(
        machine.recordedProgramAvailable,
        "Valid path retains the recorded program"
    );

    check_true(
        machine.validatedTrajectoryAvailable,
        "Valid path marks a validated trajectory available"
    );

    check_true(
        !machine.previewAccepted,
        "Validation alone does not accept Preview"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Validation alone does not select execution mode"
    );

    check_true(
        machine.transitionCount == 5U,
        "Successful validation is the fifth transition"
    );

    /*
     * ------------------------------------------------------------
     * Normal validation rejection
     * ------------------------------------------------------------
     */
    reach_path_validation(&machine);

    event = make_event(
        SUPERVISOR_EVENT_VALIDATION_REJECTED,
        ROBOT_STATE_PATH_VALIDATION
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Rejected path causes a controlled transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Rejected path returns to IDLE"
    );

    check_true(
        machine.previousState ==
        ROBOT_STATE_PATH_VALIDATION,
        "Rejected validation retains its source state"
    );

    check_true(
        machine.recordedProgramAvailable,
        "Rejected validation retains the recorded draft"
    );

    check_true(
        !machine.validatedTrajectoryAvailable,
        "Rejected path is not marked validated"
    );

    check_true(
        !machine.previewAccepted,
        "Rejected path cannot satisfy Preview"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Rejected path selects no execution mode"
    );

    check_true(
        machine.transitionCount == 5U,
        "Validation rejection performs one controlled transition"
    );

    check_true(
        machine.rejectedEventCount == 0U,
        "Validation rejection is handled, not rejected as an event"
    );
}
static void reach_validated_idle(
    StateMachine *machine
)
{
    SupervisorEvent event;

    reach_path_validation(machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_PATH_VALIDATION
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );
}
static void test_preview_safety_gating_and_approach_entry(void)
{
    StateMachine machine;
    RobotSafetySnapshot safety;
    SupervisorEvent event;
    SupervisorResult result;

    reach_validated_idle(&machine);

    /*
     * A validated path exists, but startup safety values are deliberately
     * conservative. Motion must remain unavailable.
     */
    check_true(
        !state_machine_can_move(&machine),
        "Motion is blocked before healthy safety status arrives"
    );

    event = make_event(
        SUPERVISOR_EVENT_VALIDATE_PREVIEW,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Preview is rejected while motion is not permitted"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Unsafe Preview request leaves state in IDLE"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Unsafe Preview request selects no execution mode"
    );

    check_true(
        machine.rejectedEventCount == 1U,
        "Unsafe Preview request is counted"
    );

    /*
     * Supply a healthy software safety snapshot.
     */
    memset(&safety, 0, sizeof(safety));

    safety.estopActive = false;
    safety.protectiveStopActive = false;
    safety.globalFaultActive = false;
    safety.statusValid = true;

    safety.motionPermitted = true;
    safety.drivesReady = true;
    safety.communicationHealthy = true;

    safety.timestampMs = 5000U;

    state_machine_update_safety(
        &machine,
        &safety
    );

    check_true(
        machine.safety.timestampMs == 5000U,
        "Safety snapshot timestamp is stored"
    );

    check_true(
        state_machine_can_move(&machine),
        "Healthy safety status permits motion"
    );

    /*
     * Request Preview again. The trajectory is validated and motion is now
     * permitted, so Approach may start in Preview mode.
     */
    event = make_event(
        SUPERVISOR_EVENT_VALIDATE_PREVIEW,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Safe Preview request causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_APPROACH,
        "Safe Preview request enters APPROACH"
    );

    check_true(
        machine.previousState == ROBOT_STATE_IDLE,
        "Preview Approach retains IDLE as previous state"
    );

    check_true(
        machine.executionMode ==
        ROBOT_EXECUTION_PREVIEW,
        "Preview Approach selects PREVIEW execution mode"
    );

    check_true(
        machine.recordedProgramAvailable,
        "Preview retains the recorded program"
    );

    check_true(
        machine.validatedTrajectoryAvailable,
        "Preview retains the validated trajectory"
    );

    check_true(
        !machine.previewAccepted,
        "Preview is not accepted before motion completes"
    );

    check_true(
        machine.transitionCount == 6U,
        "Preview Approach is the sixth valid transition"
    );

    check_true(
        machine.rejectedEventCount == 1U,
        "Valid Preview does not add another rejection"
    );
}
static void reach_preview_approach(
    StateMachine *machine
)
{
    RobotSafetySnapshot safety;
    SupervisorEvent event;

    reach_validated_idle(machine);

    memset(&safety, 0, sizeof(safety));

    safety.statusValid = true;
    safety.motionPermitted = true;
    safety.drivesReady = true;
    safety.communicationHealthy = true;
    safety.timestampMs = 6000U;

    state_machine_update_safety(
        machine,
        &safety
    );

    event = make_event(
        SUPERVISOR_EVENT_VALIDATE_PREVIEW,
        ROBOT_STATE_IDLE
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );
}


static void test_preview_execution_and_acceptance(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    reach_preview_approach(&machine);

    /*
     * Preview Approach reaches the first trajectory point.
     * Arc Stabilizing is skipped in Preview mode.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Preview Approach completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_WELDING,
        "Preview Approach skips ARC_STABILIZING"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_PREVIEW,
        "Trajectory execution remains in PREVIEW mode"
    );

    check_true(
        !machine.previewAccepted,
        "Approach completion alone does not accept Preview"
    );

    /*
     * The full Preview trajectory finishes with welding output disabled.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Preview trajectory completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_RETRACTING,
        "Preview trajectory completion enters RETRACTING"
    );

    check_true(
        !machine.previewAccepted,
        "Preview is not accepted before safe retraction completes"
    );

    /*
     * Retraction completes successfully.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_RETRACTING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Preview retraction completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Completed Preview returns to IDLE"
    );

    check_true(
        machine.previewAccepted,
        "Complete fault-free Preview is accepted"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Completed Preview clears execution mode"
    );

    check_true(
        machine.validatedTrajectoryAvailable,
        "Completed Preview retains validated trajectory"
    );

    check_true(
        machine.transitionCount == 9U,
        "Complete Preview sequence performs nine transitions"
    );
}
static void reach_preview_accepted_idle(
    StateMachine *machine
)
{
    SupervisorEvent event;

    reach_preview_approach(machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_WELDING
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_RETRACTING
    );

    (void)state_machine_handle_event(
        machine,
        &event
    );
}
static void test_production_start_and_execution_sequence(void)
{
    StateMachine machine;
    RobotSafetySnapshot safety;
    SupervisorEvent event;
    SupervisorResult result;

    /*
     * ------------------------------------------------------------
     * Production is forbidden before Preview acceptance.
     * ------------------------------------------------------------
     */
    reach_validated_idle(&machine);

    memset(&safety, 0, sizeof(safety));

    safety.statusValid = true;
    safety.motionPermitted = true;
    safety.drivesReady = true;
    safety.communicationHealthy = true;
    safety.timestampMs = 7000U;

    state_machine_update_safety(
        &machine,
        &safety
    );

    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Production Start is rejected before Preview acceptance"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Rejected Production Start leaves state in IDLE"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Rejected Production Start selects no execution mode"
    );

    check_true(
        machine.rejectedEventCount == 1U,
        "Premature Production Start is counted"
    );

    /*
     * ------------------------------------------------------------
     * Reach IDLE with a successfully accepted Preview.
     * ------------------------------------------------------------
     */
    reach_preview_accepted_idle(&machine);

    check_true(
        machine.previewAccepted,
        "Production fixture begins with accepted Preview"
    );

    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Accepted Production Start causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_APPROACH,
        "Production Start enters APPROACH"
    );

    check_true(
        machine.executionMode ==
        ROBOT_EXECUTION_PRODUCTION,
        "Production Start selects PRODUCTION mode"
    );

    check_true(
        machine.previewAccepted,
        "Production Start retains Preview acceptance"
    );

    /*
     * Production Approach must enter Arc Stabilizing.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Production Approach completion causes a transition"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_ARC_STABILIZING,
        "Production Approach enters ARC_STABILIZING"
    );

    check_true(
        machine.executionMode ==
        ROBOT_EXECUTION_PRODUCTION,
        "Arc Stabilizing retains PRODUCTION mode"
    );

    /*
     * Stable arc permits welding trajectory execution.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_ARC_STABILIZING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Arc Stabilizing completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_WELDING,
        "Stable arc enters WELDING"
    );

    /*
     * Welding completion initiates retraction.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Production Welding completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_RETRACTING,
        "Production Welding completion enters RETRACTING"
    );

    /*
     * Retraction finishes the production cycle.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_RETRACTING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Production retraction completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Completed production cycle returns to IDLE"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Completed production cycle clears execution mode"
    );

    check_true(
        machine.previewAccepted,
        "Production completion retains Preview acceptance"
    );

    check_true(
        machine.validatedTrajectoryAvailable,
        "Production completion retains validated trajectory"
    );

    check_true(
        machine.transitionCount == 14U,
        "Preview and Production sequence performs fourteen transitions"
    );

    check_true(
        machine.rejectedEventCount == 0U,
        "Valid production sequence rejects no events"
    );
}

static void test_pause_resume_behavior(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    RobotSafetySnapshot unsafeSnapshot;

    printf("\n--- Test 11: Pause/Resume behavior ---\n");

    /*
     * Prepare an IDLE machine with a recorded, validated, and
     * preview-accepted trajectory.
     */
    reach_preview_accepted_idle(&machine);

    /*
     * Start production execution.
     */
    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "START/REPLAY begins production before pause test"
    );

    check_true(
        machine.activeState == ROBOT_STATE_APPROACH,
        "Production execution enters APPROACH"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_PRODUCTION,
        "Production mode is active before pause"
    );

    /*
     * Pause during Approach.
     */
    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "PAUSE during APPROACH causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_PAUSED,
        "PAUSE enters PAUSED state"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_APPROACH,
        "PAUSE remembers APPROACH as the interrupted state"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_PRODUCTION,
        "PAUSE preserves production execution mode"
    );

    /*
     * Simulate motion becoming unsafe while paused.
     */
    unsafeSnapshot = machine.safety;
    unsafeSnapshot.motionPermitted = false;

    state_machine_update_safety(&machine, &unsafeSnapshot);

    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_PAUSED
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "RESUME is rejected when motion is not permitted"
    );

    check_true(
        machine.activeState == ROBOT_STATE_PAUSED,
        "Rejected RESUME leaves the machine PAUSED"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_APPROACH,
        "Rejected RESUME preserves the interrupted state"
    );

    /*
     * Restore motion permission and resume.
     */
    unsafeSnapshot.motionPermitted = true;
    state_machine_update_safety(&machine, &unsafeSnapshot);

    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_PAUSED
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "RESUME succeeds after motion permission is restored"
    );

    check_true(
        machine.activeState == ROBOT_STATE_APPROACH,
        "RESUME returns to the interrupted APPROACH state"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_IDLE,
        "Successful RESUME clears the stored resume state"
    );

    /*
     * Complete Approach and Arc Stabilizing to reach Welding.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_ARC_STABILIZING,
        "Production APPROACH completion enters ARC_STABILIZING"
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_ARC_STABILIZING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_WELDING,
        "ARC_STABILIZING completion enters WELDING"
    );

    /*
     * Verify that the same button can pause and resume Welding.
     */
    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_PAUSED,
        "PAUSE during WELDING enters PAUSED"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_WELDING,
        "WELDING is retained as the interrupted state"
    );

    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_PAUSED
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_WELDING,
        "RESUME returns to WELDING"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_PRODUCTION,
        "WELDING resume retains production mode"
    );

    /*
     * Finish the production sequence.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_WELDING
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_RETRACTING
    );
    state_machine_handle_event(&machine, &event);

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Production returns to IDLE after retraction"
    );

    /*
     * PAUSE/RESUME has no valid meaning in IDLE.
     */
    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "PAUSE/RESUME is rejected in IDLE"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Rejected IDLE pause leaves the state unchanged"
    );
}
static void test_reset_and_home_behavior(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    uint32_t transitionsBefore;

    printf("\n--- Test 12: Reset and Home behavior ---\n");

    /*
     * RESET test: begin with a complete, preview-accepted program.
     */
    reach_preview_accepted_idle(&machine);

    check_true(
        machine.recordedProgramAvailable,
        "Reset fixture has a recorded program"
    );

    check_true(
        machine.validatedTrajectoryAvailable,
        "Reset fixture has a validated trajectory"
    );

    check_true(
        machine.previewAccepted,
        "Reset fixture has an accepted Preview"
    );

    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_HANDLED,
        "RESET is handled in IDLE"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "RESET in IDLE leaves the supervisor in IDLE"
    );

    check_true(
        !machine.recordedProgramAvailable,
        "RESET clears recorded-program availability"
    );

    check_true(
        !machine.validatedTrajectoryAvailable,
        "RESET clears validated-trajectory availability"
    );

    check_true(
        !machine.previewAccepted,
        "RESET clears Preview acceptance"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "RESET leaves no execution mode selected"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "RESET in IDLE creates no false state transition"
    );

    /*
     * HOME test: prepare another valid program and verify that HOME
     * preserves it.
     */
    reach_preview_accepted_idle(&machine);
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_HOME,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "HOME from IDLE causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "HOME from IDLE enters HOMING"
    );

    check_true(
        machine.previousState == ROBOT_STATE_IDLE,
        "HOME retains IDLE as the previous state"
    );

    check_true(
        machine.transitionCount == transitionsBefore + 1U,
        "HOME causes exactly one transition"
    );

    check_true(
        machine.recordedProgramAvailable &&
        machine.validatedTrajectoryAvailable &&
        machine.previewAccepted,
        "HOME preserves the complete program"
    );

    /*
     * Complete Homing and ensure that the program remains available.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "HOMING completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "HOMING completion returns to IDLE"
    );

    check_true(
        machine.recordedProgramAvailable &&
        machine.validatedTrajectoryAvailable &&
        machine.previewAccepted,
        "Completed HOMING preserves the program"
    );

    /*
     * Start production and verify that RESET cannot abruptly abandon
     * active robot motion.
     */
    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_APPROACH,
        "Active-motion test reaches APPROACH"
    );

    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "RESET is rejected during active APPROACH"
    );

    check_true(
        machine.activeState == ROBOT_STATE_APPROACH,
        "Rejected RESET does not abandon APPROACH"
    );

    check_true(
        machine.recordedProgramAvailable &&
        machine.validatedTrajectoryAvailable &&
        machine.previewAccepted,
        "Rejected RESET does not erase the program"
    );

    /*
     * HOME is also rejected during active motion.
     */
    event = make_event(
        SUPERVISOR_EVENT_HOME,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "HOME is rejected during active APPROACH"
    );

    check_true(
        machine.activeState == ROBOT_STATE_APPROACH,
        "Rejected HOME does not abandon APPROACH"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "Rejected active-motion commands create no transitions"
    );
}
static void test_protective_stop_behavior(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    uint32_t transitionsBefore;

    printf("\n--- Test 13: Protective-stop behavior ---\n");

    /*
     * Prepare a valid production run and reach WELDING.
     */
    reach_preview_accepted_idle(&machine);

    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_ARC_STABILIZING
    );
    state_machine_handle_event(&machine, &event);

    check_true(
        machine.activeState == ROBOT_STATE_WELDING &&
        machine.executionMode == ROBOT_EXECUTION_PRODUCTION,
        "Protective-stop fixture reaches production WELDING"
    );

    transitionsBefore = machine.transitionCount;

    /*
     * Simulate zone intrusion.
     */
    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Zone intrusion causes a state transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Zone intrusion enters FAULT"
    );

    check_true(
        machine.previousState == ROBOT_STATE_WELDING,
        "Protective stop retains WELDING as previous state"
    );

    check_true(
        machine.safety.protectiveStopActive,
        "Protective-stop input is retained as active"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_RECOVERABLE,
        "Protective stop is classified as recoverable"
    );

    check_true(
        machine.activeFaultCode ==
        ROBOT_FAULT_CODE_PROTECTIVE_STOP,
        "Protective-stop fault code is retained"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Protective stop cancels production execution mode"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_IDLE,
        "Protective stop removes automatic resume destination"
    );

    check_true(
        machine.recordedProgramAvailable &&
        machine.validatedTrajectoryAvailable &&
        machine.previewAccepted,
        "Protective stop preserves the stored program"
    );

    check_true(
        !state_machine_can_move(&machine),
        "Motion is prohibited while protective stop is active"
    );

    check_true(
        machine.transitionCount == transitionsBefore + 1U,
        "Protective stop performs exactly one transition"
    );

    /*
     * Simulate the operator leaving the protected zone.
     */
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_CLEARED,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_HANDLED,
        "Zone-clear event is handled"
    );

    check_true(
        !machine.safety.protectiveStopActive,
        "Zone-clear event removes the active protective input"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Zone clearance does not automatically exit FAULT"
    );

    check_true(
        !state_machine_can_move(&machine),
        "Motion remains prohibited after zone clearance"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "Zone clearance causes no automatic transition"
    );

    /*
     * The operator Pause/Resume button must not recover a protective
     * stop.
     */
    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Pause/Resume cannot recover a protective stop"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Rejected resume leaves the Supervisor in FAULT"
    );
}

static void test_protective_stop_recovery(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    uint32_t transitionsBefore;

    printf("\n--- Test 14: Protective-stop recovery ---\n");

    /*
     * Prepare a production run and reach WELDING.
     */
    reach_preview_accepted_idle(&machine);

    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_ARC_STABILIZING
    );
    state_machine_handle_event(&machine, &event);

    check_true(
        machine.activeState == ROBOT_STATE_WELDING,
        "Recovery fixture reaches WELDING"
    );

    /*
     * Assert the zone protective stop.
     */
    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_FAULT,
        "Zone intrusion enters FAULT before recovery"
    );

    transitionsBefore = machine.transitionCount;

    /*
     * Reset must not work while the protected zone remains occupied.
     */
    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "RESET is rejected while protective stop remains active"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Rejected RESET leaves the Supervisor in FAULT"
    );

    check_true(
        machine.activeFaultCode ==
        ROBOT_FAULT_CODE_PROTECTIVE_STOP,
        "Rejected RESET preserves the protective-stop fault"
    );

    check_true(
        machine.recordedProgramAvailable &&
        machine.validatedTrajectoryAvailable &&
        machine.previewAccepted,
        "Rejected RESET does not erase the stored program"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "Rejected RESET causes no transition"
    );

    /*
     * Clear the protected zone. This alone must not recover the robot.
     */
    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_CLEARED,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_HANDLED,
        "Protective-stop clearance is handled"
    );

    check_true(
        !machine.safety.protectiveStopActive,
        "Protective-stop input is inactive after clearance"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Zone clearance alone leaves the Supervisor in FAULT"
    );

    /*
     * A deliberate RESET now starts controlled recovery.
     */
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "RESET starts recovery after zone clearance"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "Protective-stop recovery enters HOMING"
    );

    check_true(
        machine.previousState == ROBOT_STATE_FAULT,
        "Recovery retains FAULT as the previous state"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_NONE,
        "Recovery clears fault severity"
    );

    check_true(
        machine.activeFaultCode ==
        ROBOT_FAULT_CODE_NONE,
        "Recovery clears the active fault code"
    );

    check_true(
        !machine.recordedProgramAvailable &&
        !machine.validatedTrajectoryAvailable &&
        !machine.previewAccepted,
        "Recovery RESET clears the interrupted program"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Recovery has no active execution mode"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_IDLE,
        "Recovery clears any resume destination"
    );

    check_true(
        machine.transitionCount == transitionsBefore + 1U,
        "Recovery RESET performs exactly one transition"
    );

    check_true(
        state_machine_can_move(&machine),
        "Healthy safety status permits controlled HOMING"
    );

    /*
     * Complete the recovery.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Recovery HOMING completion causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Completed protective-stop recovery enters IDLE"
    );
}
static void test_emergency_stop_behavior(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    uint32_t transitionsBefore;

    printf("\n--- Test 15: Emergency-stop behavior ---\n");

    /*
     * Prepare a production cycle and reach WELDING.
     */
    reach_preview_accepted_idle(&machine);

    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_ARC_STABILIZING
    );
    state_machine_handle_event(&machine, &event);

    check_true(
        machine.activeState == ROBOT_STATE_WELDING,
        "E-stop fixture reaches WELDING"
    );

    transitionsBefore = machine.transitionCount;

    /*
     * Assert the emergency stop during production welding.
     */
    event = make_event(
        SUPERVISOR_EVENT_ESTOP_ASSERTED,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "E-stop assertion causes a transition"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "E-stop assertion enters EMERGENCY_STOP"
    );

    check_true(
        machine.previousState == ROBOT_STATE_WELDING,
        "E-stop retains WELDING as the previous state"
    );

    check_true(
        machine.safety.estopActive,
        "E-stop input is retained as active"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_EMERGENCY,
        "E-stop is classified as an emergency"
    );

    check_true(
        machine.activeFaultCode ==
        ROBOT_FAULT_CODE_EMERGENCY_STOP,
        "E-stop fault code is retained"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "E-stop cancels the active execution mode"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_IDLE,
        "E-stop removes any automatic resume destination"
    );

    check_true(
        !state_machine_can_move(&machine),
        "Motion is prohibited while E-stop is active"
    );

    check_true(
        machine.recordedProgramAvailable &&
        machine.validatedTrajectoryAvailable &&
        machine.previewAccepted,
        "E-stop assertion initially preserves stored program data"
    );

    check_true(
        machine.transitionCount == transitionsBefore + 1U,
        "E-stop assertion performs exactly one transition"
    );

    /*
     * RESET must fail while the physical E-stop remains asserted.
     */
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "RESET is rejected while E-stop remains active"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "Rejected RESET leaves EMERGENCY_STOP active"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "Rejected emergency RESET causes no transition"
    );

    /*
     * Release the physical E-stop.
     */
    event = make_event(
        SUPERVISOR_EVENT_ESTOP_RELEASED,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_HANDLED,
        "E-stop release is handled"
    );

    check_true(
        !machine.safety.estopActive,
        "E-stop release clears the active input"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "E-stop release does not automatically exit emergency state"
    );

    check_true(
        !state_machine_can_move(&machine),
        "Motion remains prohibited after E-stop release"
    );

    /*
     * Pause/Resume cannot recover an E-stop.
     */
    event = make_event(
        SUPERVISOR_EVENT_PAUSE_RESUME,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Pause/Resume cannot recover an E-stop"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "Rejected Resume leaves EMERGENCY_STOP active"
    );

    /*
     * Explicit RESET after release starts controlled recovery.
     */
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "RESET starts recovery after E-stop release"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "E-stop recovery enters HOMING"
    );

    check_true(
        machine.previousState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "Recovery retains EMERGENCY_STOP as previous state"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_NONE &&
        machine.activeFaultCode == ROBOT_FAULT_CODE_NONE,
        "E-stop recovery clears fault information"
    );

    check_true(
        !machine.recordedProgramAvailable &&
        !machine.validatedTrajectoryAvailable &&
        !machine.previewAccepted,
        "E-stop recovery clears the interrupted program"
    );

    check_true(
        machine.transitionCount == transitionsBefore + 1U,
        "E-stop recovery performs exactly one transition"
    );

    /*
     * Complete recovery Homing.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_IDLE,
        "Completed E-stop recovery enters IDLE"
    );
}
static void test_asynchronous_fault_handling(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    RobotSafetySnapshot safety;
    uint32_t transitionsBefore;
    const uint32_t testFaultCode = 0x3456U;

    printf("\n--- Test 16: Asynchronous system-fault handling ---\n");

    /*
     * Prepare a validated production program and enter APPROACH.
     */
    reach_preview_accepted_idle(&machine);

    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_APPROACH,
        "System-fault fixture reaches production APPROACH"
    );

    transitionsBefore = machine.transitionCount;

    /*
     * Simulate an asynchronous recoverable system fault.
     */
    event = make_event(
        SUPERVISOR_EVENT_FAULT_DETECTED,
        ROBOT_STATE_APPROACH
    );

    event.faultSeverity =
        ROBOT_FAULT_SEVERITY_RECOVERABLE;

    event.faultCode = testFaultCode;

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Detected system fault causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Recoverable system fault enters FAULT"
    );

    check_true(
        machine.previousState == ROBOT_STATE_APPROACH,
        "System fault retains APPROACH as previous state"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_RECOVERABLE,
        "System-fault severity is retained"
    );

    check_true(
        machine.activeFaultCode == testFaultCode,
        "System-fault diagnostic code is retained"
    );

    check_true(
        machine.safety.globalFaultActive,
        "Detected system fault sets global-fault status"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "System fault cancels production execution mode"
    );

    check_true(
        machine.resumeState == ROBOT_STATE_IDLE,
        "System fault removes automatic resume destination"
    );

    check_true(
        machine.recordedProgramAvailable &&
        machine.validatedTrajectoryAvailable &&
        machine.previewAccepted,
        "Fault assertion initially preserves stored program data"
    );

    check_true(
        !state_machine_can_move(&machine),
        "Motion is prohibited while system fault is active"
    );

    check_true(
        machine.transitionCount == transitionsBefore + 1U,
        "System fault performs exactly one transition"
    );

    /*
     * RESET must fail while monitoring still reports an active fault.
     */
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "RESET is rejected while system fault remains active"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Rejected RESET leaves the Supervisor in FAULT"
    );

    check_true(
        machine.activeFaultCode == testFaultCode,
        "Rejected RESET preserves the diagnostic fault code"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "Rejected fault RESET causes no transition"
    );

    /*
     * Simulate the diagnostic task reporting that the fault condition
     * has been removed.
     */
    safety = machine.safety;
    safety.globalFaultActive = false;
    safety.timestampMs++;

    state_machine_update_safety(
        &machine,
        &safety
    );

    check_true(
        !machine.safety.globalFaultActive,
        "Cleared diagnostic condition updates safety status"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Fault clearance alone does not exit FAULT"
    );

    /*
     * Explicit RESET now begins controlled recovery.
     */
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "RESET starts recovery after system fault clears"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "System-fault recovery enters HOMING"
    );

    check_true(
        machine.previousState == ROBOT_STATE_FAULT,
        "Recovery retains FAULT as previous state"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_NONE &&
        machine.activeFaultCode == ROBOT_FAULT_CODE_NONE,
        "Recovery clears system-fault information"
    );

    check_true(
        !machine.recordedProgramAvailable &&
        !machine.validatedTrajectoryAvailable &&
        !machine.previewAccepted,
        "Recovery clears the interrupted program"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "Recovery retains no execution mode"
    );

    check_true(
        machine.transitionCount == transitionsBefore + 1U,
        "Recovery RESET performs exactly one transition"
    );

    check_true(
        state_machine_can_move(&machine),
        "Healthy conditions permit controlled recovery Homing"
    );

    /*
     * Complete recovery.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_IDLE,
        "Completed system-fault recovery enters IDLE"
    );
}
static void test_safety_event_priority(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    uint32_t transitionsBefore;

    printf("\n--- Test 17: Safety-event priority and repetition ---\n");

    /*
     * Begin in production WELDING.
     */
    reach_preview_accepted_idle(&machine);

    event = make_event(
        SUPERVISOR_EVENT_START_REPLAY,
        ROBOT_STATE_IDLE
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_APPROACH
    );
    state_machine_handle_event(&machine, &event);

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_ARC_STABILIZING
    );
    state_machine_handle_event(&machine, &event);

    check_true(
        machine.activeState == ROBOT_STATE_WELDING,
        "Safety-priority fixture reaches WELDING"
    );

    /*
     * First protective-stop assertion enters FAULT.
     */
    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_FAULT,
        "Protective-stop assertion enters FAULT"
    );

    transitionsBefore = machine.transitionCount;

    /*
     * A repeated assertion represents the same active input. It must not
     * create another transition or overwrite previousState.
     */
    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_ASSERTED,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_HANDLED,
        "Repeated protective-stop assertion is idempotent"
    );

    check_true(
        machine.activeState == ROBOT_STATE_FAULT,
        "Repeated protective stop remains in FAULT"
    );

    check_true(
        machine.previousState == ROBOT_STATE_WELDING,
        "Repeated protective stop preserves original previous state"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "Repeated protective stop creates no duplicate transition"
    );

    /*
     * E-stop must override the existing protective-stop fault.
     */
    event = make_event(
        SUPERVISOR_EVENT_ESTOP_ASSERTED,
        ROBOT_STATE_FAULT
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "E-stop overrides an existing protective-stop fault"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "E-stop escalation enters EMERGENCY_STOP"
    );

    check_true(
        machine.previousState == ROBOT_STATE_FAULT,
        "Emergency escalation retains FAULT as previous state"
    );

    check_true(
        machine.faultSeverity ==
        ROBOT_FAULT_SEVERITY_EMERGENCY,
        "Escalation stores emergency severity"
    );

    check_true(
        machine.activeFaultCode ==
        ROBOT_FAULT_CODE_EMERGENCY_STOP,
        "Escalation replaces the protective-stop fault code"
    );

    check_true(
        machine.safety.estopActive &&
        machine.safety.protectiveStopActive,
        "Both active safety inputs are retained"
    );

    /*
     * Clearing the protective stop while E-stop remains active must not
     * leave the emergency state.
     */
    transitionsBefore = machine.transitionCount;

    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_CLEARED,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_HANDLED,
        "Protective stop can clear during EMERGENCY_STOP"
    );

    check_true(
        !machine.safety.protectiveStopActive &&
        machine.safety.estopActive,
        "Protective input clears while E-stop remains active"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "Lower-priority clearance does not exit EMERGENCY_STOP"
    );

    check_true(
        machine.transitionCount == transitionsBefore,
        "Protective clearance causes no emergency transition"
    );

    /*
     * RESET is still blocked because E-stop remains active.
     */
    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "RESET remains blocked while E-stop is active"
    );

    /*
     * Release E-stop. Release alone must not cause recovery.
     */
    event = make_event(
        SUPERVISOR_EVENT_ESTOP_RELEASED,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_HANDLED,
        "E-stop release is handled after escalation"
    );

    check_true(
        !machine.safety.estopActive &&
        !machine.safety.protectiveStopActive,
        "Both safety inputs are clear before recovery"
    );

    check_true(
        machine.activeState ==
        ROBOT_STATE_EMERGENCY_STOP,
        "Clearing both inputs does not automatically recover"
    );

    /*
     * Explicit Reset now starts controlled recovery.
     */
    event = make_event(
        SUPERVISOR_EVENT_RESET,
        ROBOT_STATE_EMERGENCY_STOP
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_HOMING,
        "RESET starts recovery after every safety input clears"
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_HOMING
    );

    result = state_machine_handle_event(&machine, &event);

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED &&
        machine.activeState == ROBOT_STATE_IDLE,
        "Safety-priority recovery completes in IDLE"
    );
}
static void test_invalid_input_robustness(void)
{
    StateMachine machine;
    StateMachine uninitializedMachine;
    SupervisorEvent event;
    SupervisorResult result;
    uint32_t rejectionsBefore;

    printf("\n--- Test 18: Invalid-input robustness ---\n");

    /*
     * Null StateMachine pointer.
     */
    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        NULL,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Null StateMachine pointer is rejected"
    );

    /*
     * Null event pointer.
     */
    state_machine_init(&machine);

    result = state_machine_handle_event(
        &machine,
        NULL
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Null event pointer is rejected"
    );

    check_true(
        machine.activeState == ROBOT_STATE_BOOT,
        "Null event does not modify active state"
    );

    check_true(
        machine.rejectedEventCount == 0U,
        "Null event cannot modify diagnostic counters"
    );

    /*
     * Context memory exists but initialization was never called.
     */
    memset(
        &uninitializedMachine,
        0,
        sizeof(uninitializedMachine)
    );

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        &uninitializedMachine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Uninitialized StateMachine context is rejected"
    );

    check_true(
        !uninitializedMachine.initialized,
        "Rejected uninitialized context remains uninitialized"
    );

    check_true(
        uninitializedMachine.transitionCount == 0U,
        "Uninitialized context performs no transition"
    );

    /*
     * Unknown event value.
     */
    state_machine_init(&machine);
    rejectionsBefore = machine.rejectedEventCount;

    event = make_event(
        SUPERVISOR_EVENT_NONE,
        ROBOT_STATE_BOOT
    );

    event.type = (SupervisorEventType)999;

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Unknown event value is rejected"
    );

    check_true(
        machine.activeState == ROBOT_STATE_BOOT,
        "Unknown event cannot change active state"
    );

    check_true(
        machine.transitionCount == 0U,
        "Unknown event cannot create a transition"
    );

    check_true(
        machine.rejectedEventCount ==
        rejectionsBefore + 1U,
        "Unknown event increments rejection counter"
    );

    /*
     * Completion from an invalid source state.
     */
    rejectionsBefore = machine.rejectedEventCount;

    event = make_event(
        SUPERVISOR_EVENT_STATE_COMPLETE,
        ROBOT_STATE_COUNT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Completion from invalid source state is rejected"
    );

    check_true(
        machine.activeState == ROBOT_STATE_BOOT,
        "Invalid source cannot bypass BOOT"
    );

    check_true(
        machine.rejectedEventCount ==
        rejectionsBefore + 1U,
        "Invalid source increments rejection counter"
    );

    /*
     * Safety-clear events are invalid when their corresponding stop
     * state was never active.
     */
    reach_validated_idle(&machine);

    rejectionsBefore = machine.rejectedEventCount;

    event = make_event(
        SUPERVISOR_EVENT_PROTECTIVE_STOP_CLEARED,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Unexpected protective-stop clearance is rejected"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Unexpected protective clearance leaves IDLE unchanged"
    );

    event = make_event(
        SUPERVISOR_EVENT_ESTOP_RELEASED,
        ROBOT_STATE_IDLE
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Unexpected E-stop release is rejected"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Unexpected E-stop release leaves IDLE unchanged"
    );

    check_true(
        machine.rejectedEventCount ==
        rejectionsBefore + 2U,
        "Unexpected safety clearances are both counted"
    );

    /*
     * Simulate corrupted state memory.
     */
    machine.activeState = ROBOT_STATE_COUNT;
    rejectionsBefore = machine.rejectedEventCount;

    event = make_event(
        SUPERVISOR_EVENT_TEACH,
        ROBOT_STATE_COUNT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Event is rejected when active state is invalid"
    );

    check_true(
        machine.activeState == ROBOT_STATE_COUNT,
        "Invalid active state cannot cause uncontrolled transition"
    );

    check_true(
        machine.rejectedEventCount ==
        rejectionsBefore + 1U,
        "Invalid active-state event is counted"
    );
}
static void test_controlled_approach_abort_outcomes(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;
    uint32_t rejectionsBefore;

    printf("\n--- Controlled Approach abort outcomes ---\n");

    /*
     * Test 1: controlled HOME during Approach.
     *
     * The Approach state has already stopped safely before sending this
     * event. HOME must preserve the recorded and validated trajectory.
     */
    reach_preview_approach(&machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_ABORTED_HOME,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Controlled HOME abort causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_HOMING,
        "Controlled HOME abort enters HOMING"
    );

    check_true(
        machine.previousState == ROBOT_STATE_APPROACH,
        "HOME abort retains APPROACH as previous state"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "HOME abort clears execution mode"
    );

    check_true(
        machine.recordedProgramAvailable,
        "HOME abort preserves the recorded program"
    );

    check_true(
        machine.validatedTrajectoryAvailable,
        "HOME abort preserves the validated trajectory"
    );

    check_true(
        !machine.previewAccepted,
        "Aborted Preview is not accepted"
    );

    /*
     * Test 2: controlled RESET during Approach.
     *
     * RESET must clear the complete program lifecycle and return to IDLE.
     */
    reach_preview_approach(&machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_ABORTED_RESET,
        ROBOT_STATE_APPROACH
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_TRANSITIONED,
        "Controlled RESET abort causes a transition"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Controlled RESET abort returns to IDLE"
    );

    check_true(
        machine.previousState == ROBOT_STATE_APPROACH,
        "RESET abort retains APPROACH as previous state"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_NONE,
        "RESET abort clears execution mode"
    );

    check_true(
        !machine.recordedProgramAvailable,
        "RESET abort clears the recorded program"
    );

    check_true(
        !machine.validatedTrajectoryAvailable,
        "RESET abort clears the validated trajectory"
    );

    check_true(
        !machine.previewAccepted,
        "RESET abort clears Preview acceptance"
    );

    /*
     * Test 3: delayed duplicate result.
     *
     * The same abort event arriving after the transition must be rejected.
     */
    rejectionsBefore = machine.rejectedEventCount;

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Repeated RESET abort result is rejected as stale"
    );

    check_true(
        machine.activeState == ROBOT_STATE_IDLE,
        "Stale abort result leaves IDLE unchanged"
    );

    check_true(
        machine.rejectedEventCount == rejectionsBefore + 1U,
        "Stale abort result increments rejection count"
    );

    /*
     * Test 4: abort result outside a motion state.
     */
    state_machine_init(&machine);

    event = make_event(
        SUPERVISOR_EVENT_STATE_ABORTED_HOME,
        ROBOT_STATE_BOOT
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Controlled abort result is rejected outside APPROACH"
    );

    check_true(
        machine.activeState == ROBOT_STATE_BOOT,
        "Invalid abort outcome cannot bypass BOOT"
    );

    /*
     * Test 5: event claims to come from the wrong state.
     */
    reach_preview_approach(&machine);
    rejectionsBefore = machine.rejectedEventCount;

    event = make_event(
        SUPERVISOR_EVENT_STATE_ABORTED_HOME,
        ROBOT_STATE_WELDING
    );

    result = state_machine_handle_event(
        &machine,
        &event
    );

    check_true(
        result == SUPERVISOR_RESULT_REJECTED,
        "Wrong-source controlled abort is rejected"
    );

    check_true(
        machine.activeState == ROBOT_STATE_APPROACH,
        "Wrong-source abort leaves APPROACH active"
    );

    check_true(
        machine.executionMode == ROBOT_EXECUTION_PREVIEW,
        "Wrong-source abort preserves active Preview mode"
    );

    check_true(
        machine.rejectedEventCount == rejectionsBefore + 1U,
        "Wrong-source abort increments rejection count"
    );
}
static void test_unified_execution_and_emergency_recovery(void)
{
    StateMachine machine;
    SupervisorEvent event;
    SupervisorResult result;

    reach_preview_approach(&machine);
    state_machine_enable_unified_execution(&machine);

    event = make_event(SUPERVISOR_EVENT_STATE_COMPLETE,
                       ROBOT_STATE_APPROACH);
    result = state_machine_handle_event(&machine, &event);
    check_true(result == SUPERVISOR_RESULT_TRANSITIONED,
               "Unified policy accepts Approach completion");
    check_true(machine.activeState == ROBOT_STATE_PATH_EXECUTION,
               "Unified policy enters PATH_EXECUTION");

    event = make_event(SUPERVISOR_EVENT_STATE_COMPLETE,
                       ROBOT_STATE_PATH_EXECUTION);
    result = state_machine_handle_event(&machine, &event);
    check_true(result == SUPERVISOR_RESULT_TRANSITIONED,
               "Unified execution completion transitions");
    check_true(machine.activeState == ROBOT_STATE_HOMING,
               "Unified execution returns through HOMING");
    check_true(machine.previewAccepted,
               "Preview is accepted after execution and retraction");

    state_machine_init(&machine);
    event = make_event(SUPERVISOR_EVENT_STATE_COMPLETE, ROBOT_STATE_BOOT);
    (void)state_machine_handle_event(&machine, &event);
    event = make_event(SUPERVISOR_EVENT_STATE_COMPLETE, ROBOT_STATE_HOMING);
    (void)state_machine_handle_event(&machine, &event);
    RobotSafetySnapshot healthy = {
        .motionPermitted = true,
        .drivesReady = true,
        .communicationHealthy = true,
        .statusValid = true
    };
    state_machine_update_safety(&machine, &healthy);
    state_machine_enable_unified_execution(&machine);
    event = make_event(SUPERVISOR_EVENT_ESTOP_ASSERTED,
                       ROBOT_STATE_IDLE);
    (void)state_machine_handle_event(&machine, &event);
    check_true(machine.activeState == ROBOT_STATE_EMERGENCY_STOP,
               "E-stop enters EMERGENCY_STOP");

    event = make_event(SUPERVISOR_EVENT_ESTOP_RELEASED,
                       ROBOT_STATE_EMERGENCY_STOP);
    (void)state_machine_handle_event(&machine, &event);
    check_true(machine.activeState == ROBOT_STATE_EMERGENCY_STOP,
               "E-stop release alone remains in emergency state");

    event = make_event(SUPERVISOR_EVENT_RESET,
                       ROBOT_STATE_EMERGENCY_STOP);
    result = state_machine_handle_event(&machine, &event);
    check_true(result == SUPERVISOR_RESULT_HANDLED &&
               machine.activeState == ROBOT_STATE_EMERGENCY_STOP,
               "Emergency Reset acknowledges without starting motion");
    check_true(machine.emergencyResetAcknowledged,
               "Emergency Reset acknowledgement is latched");

    event = make_event(SUPERVISOR_EVENT_HOME,
                       ROBOT_STATE_EMERGENCY_STOP);
    result = state_machine_handle_event(&machine, &event);
    check_true(result == SUPERVISOR_RESULT_TRANSITIONED &&
               machine.activeState == ROBOT_STATE_HOMING,
               "Explicit Home begins emergency recovery homing");
}

int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " SUPERVISOR STATE-MACHINE UNIT TESTS\n"
        "============================================================\n"
    );

    test_initialization();
    test_normal_startup();
    test_startup_rejects_invalid_events();
    test_startup_failures_enter_fault();
    test_idle_command_gating_and_teaching_entry();
    test_teaching_completion_enters_path_validation();
    test_path_validation_outcomes();
    test_preview_safety_gating_and_approach_entry();
    test_preview_execution_and_acceptance();
    test_production_start_and_execution_sequence();
    test_pause_resume_behavior();
    test_reset_and_home_behavior();
    test_protective_stop_behavior();
    test_protective_stop_recovery();
    test_emergency_stop_behavior();
    test_asynchronous_fault_handling();
    test_safety_event_priority();
    test_invalid_input_robustness();
    test_controlled_approach_abort_outcomes();
    test_unified_execution_and_emergency_recovery();






    printf(
        "\n"
        "============================================================\n"
        " Tests passed: %u\n"
        " Tests failed: %u\n"
        "============================================================\n",
        testsPassed,
        testsFailed
    );

    return (testsFailed == 0U) ? 0 : 1;
}
