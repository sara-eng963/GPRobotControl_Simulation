#include "../state_machine.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/* ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * Unit-test the GLOBAL StateMachine coordinator only.
 *
 * This test deliberately does NOT link the real state implementations:
 *
 *      state_boot.c
 *      state_homing.c
 *      state_idle.c
 *      state_teaching.c
 *      state_path_validation.c
 *      state_approach.c
 *
 * Instead, small stubs below simulate each state's result.
 *
 * This lets us verify:
 *
 *      BOOT -> HOMING
 *      HOMING -> IDLE
 *      IDLE -> TEACHING
 *      TEACHING -> PATH_VALIDATION
 *      PATH_VALIDATION RUNNING / COMPLETE behavior
 *      PATH_VALIDATION -> APPROACH
 *      APPROACH RUNNING / COMPLETE behavior
 *
 * without requiring:
 *
 *      EtherCAT
 *      SOEM
 *      KickCAT
 *      A6-EC feedback
 *      real trajectory generation
 *
 * IMPORTANT:
 *
 * Build this test with:
 *
 *      StateMachine/state_machine.c
 *      StateMachine/Tests/test_state_machine.c
 *
 * and DO NOT add the individual state .c files to this test target, because
 * the stubs below intentionally replace them.
 * ============================================================================
 */


/* ============================================================================
 * TEST CONTROL
 * ============================================================================ */

static StateStepResult g_boot_result;
static StateStepResult g_homing_result;
static StateStepResult g_idle_result;
static StateStepResult g_teaching_result;
static StateStepResult g_path_validation_result;
static StateStepResult g_approach_result;

static bool g_teaching_validation_request;

static uint32_t g_teaching_submitted_revision;
static uint32_t g_teaching_submitted_crc;

static PathValidationResult g_path_validation_report_result;
static ApproachResult g_approach_report_result;


/* ============================================================================
 * CALL COUNTERS
 * ============================================================================ */

static unsigned g_boot_enter_calls;
static unsigned g_boot_step_calls;

static unsigned g_homing_enter_calls;
static unsigned g_homing_step_calls;

static unsigned g_idle_enter_calls;
static unsigned g_idle_step_calls;

static unsigned g_teaching_enter_calls;
static unsigned g_teaching_step_calls;

static unsigned g_path_validation_enter_calls;
static unsigned g_path_validation_step_calls;
static unsigned g_path_validation_get_outputs_calls;

static unsigned g_approach_enter_calls;
static unsigned g_approach_step_calls;
static unsigned g_approach_get_outputs_calls;


/* ============================================================================
 * PATH-VALIDATION HANDOFF CAPTURE
 * ============================================================================ */

static const TaughtProgram *g_path_validation_received_program;
static uint32_t g_path_validation_received_revision;
static uint32_t g_path_validation_received_crc;


/* ============================================================================
 * APPROACH HANDOFF CAPTURE
 * ============================================================================ */

static ApproachOperation g_approach_received_operation;
static const ValidatedTrajectory *g_approach_received_trajectory;
static bool g_approach_received_trajectory_ready;
static uint32_t g_approach_received_program_id;
static uint32_t g_approach_received_revision;
static uint32_t g_approach_received_artifact_crc;
static const JointVector *g_approach_received_clearance_poses;
static uint8_t g_approach_received_clearance_pose_count;


/* ============================================================================
 * RESET TEST STUBS
 * ============================================================================ */

static void reset_stub_state(void)
{
    g_boot_result =
        STATE_STEP_RUNNING;

    g_homing_result =
        STATE_STEP_RUNNING;

    g_idle_result =
        STATE_STEP_RUNNING;

    g_teaching_result =
        STATE_STEP_RUNNING;

    g_path_validation_result =
        STATE_STEP_RUNNING;

    g_approach_result =
        STATE_STEP_RUNNING;


    g_teaching_validation_request =
        false;

    g_teaching_submitted_revision =
        0U;

    g_teaching_submitted_crc =
        0U;


    g_path_validation_report_result =
        PV_RESULT_RUNNING;

    g_approach_report_result =
        APPROACH_RESULT_RUNNING;


    g_boot_enter_calls =
        0U;

    g_boot_step_calls =
        0U;

    g_homing_enter_calls =
        0U;

    g_homing_step_calls =
        0U;

    g_idle_enter_calls =
        0U;

    g_idle_step_calls =
        0U;

    g_teaching_enter_calls =
        0U;

    g_teaching_step_calls =
        0U;

    g_path_validation_enter_calls =
        0U;

    g_path_validation_step_calls =
        0U;

    g_path_validation_get_outputs_calls =
        0U;

    g_approach_enter_calls =
        0U;

    g_approach_step_calls =
        0U;

    g_approach_get_outputs_calls =
        0U;


    g_path_validation_received_program =
        NULL;

    g_path_validation_received_revision =
        0U;

    g_path_validation_received_crc =
        0U;


    g_approach_received_operation =
        APPROACH_OPERATION_NONE;

    g_approach_received_trajectory =
        NULL;

    g_approach_received_trajectory_ready =
        false;

    g_approach_received_program_id =
        0U;

    g_approach_received_revision =
        0U;

    g_approach_received_artifact_crc =
        0U;

    g_approach_received_clearance_poses =
        NULL;

    g_approach_received_clearance_pose_count =
        0U;
}


/* ============================================================================
 * STATE IMPLEMENTATION STUBS
 * ============================================================================
 *
 * These functions have the same public signatures as the real state modules.
 * state_machine.c links against these during this UNIT test only.
 * ============================================================================ */


/* ----------------------------------------------------------------------------
 * BOOT
 * ------------------------------------------------------------------------- */

void state_boot_enter(
    BootState *boot
)
{
    ++g_boot_enter_calls;

    if (boot != NULL)
    {
        memset(
            boot,
            0,
            sizeof(*boot)
        );

        boot->phase =
            BOOT_PHASE_INIT;

        boot->error =
            BOOT_ERROR_NONE;
    }
}


StateStepResult state_boot_step(
    BootState *boot,
    const EtherCATMasterConfig *ethercatConfig
)
{
    (void)boot;
    (void)ethercatConfig;

    ++g_boot_step_calls;

    return
        g_boot_result;
}


/* ----------------------------------------------------------------------------
 * HOMING
 * ------------------------------------------------------------------------- */

void state_homing_enter(
    HomingState *homing
)
{
    ++g_homing_enter_calls;

    if (homing != NULL)
    {
        memset(
            homing,
            0,
            sizeof(*homing)
        );

        homing->phase =
            HOMING_PHASE_INIT;

        homing->error =
            HOMING_ERROR_NONE;
    }
}


StateStepResult state_homing_step(
    HomingState *homing,
    const HomingConfig *config,
    const RobotConfig *robot
)
{
    (void)homing;
    (void)config;
    (void)robot;

    ++g_homing_step_calls;

    return
        g_homing_result;
}


/* ----------------------------------------------------------------------------
 * IDLE
 * ------------------------------------------------------------------------- */

void state_idle_enter(
    IdleState *idle
)
{
    ++g_idle_enter_calls;

    if (idle != NULL)
    {
        memset(
            idle,
            0,
            sizeof(*idle)
        );

        idle->phase =
            IDLE_PHASE_INIT;

        idle->error =
            IDLE_ERROR_NONE;

        idle->exitCommand =
            IDLE_COMMAND_NONE;
    }
}


StateStepResult state_idle_step(
    IdleState *idle,
    IdleCommand command
)
{
    ++g_idle_step_calls;

    if (
        idle != NULL &&
        g_idle_result == STATE_STEP_COMPLETE
    )
    {
        idle->exitCommand =
            command;
    }

    return
        g_idle_result;
}


/* ----------------------------------------------------------------------------
 * TEACHING
 * ------------------------------------------------------------------------- */

void state_teaching_enter(
    TeachingState *state,
    const TeachingConfig *config,
    uint32_t programId
)
{
    (void)config;

    ++g_teaching_enter_calls;

    if (state != NULL)
    {
        memset(
            state,
            0,
            sizeof(*state)
        );

        state->initialized =
            true;

        state->draft.program_id =
            programId;
    }
}


StateStepResult state_teaching_step(
    TeachingState *state,
    const RobotConfig *robot,
    const TeachingRuntimeInputs *runtime,
    TeachingEvent event,
    TeachingOutputs *outputs
)
{
    (void)state;
    (void)robot;
    (void)runtime;
    (void)event;

    ++g_teaching_step_calls;

    if (outputs != NULL)
    {
        memset(
            outputs,
            0,
            sizeof(*outputs)
        );

        outputs->validation_request =
            g_teaching_validation_request;

        outputs->submitted_revision =
            g_teaching_submitted_revision;

        outputs->submitted_crc =
            g_teaching_submitted_crc;
    }

    return
        g_teaching_result;
}


/* ----------------------------------------------------------------------------
 * PATH VALIDATION
 * ------------------------------------------------------------------------- */

void state_path_validation_enter(
    PathValidationState *state,
    const RobotConfig *robot,
    const PathValidationConfig *config,
    const PathValidationServices *services,
    PathValidationWorkspace *workspace,
    const PathValidationStorage *storage,
    ValidatedTrajectory *artifact_storage,
    const TaughtProgram *frozen_program,
    uint32_t submitted_revision,
    uint32_t submitted_crc
)
{
    (void)robot;
    (void)config;
    (void)services;
    (void)workspace;
    (void)storage;
    (void)artifact_storage;

    ++g_path_validation_enter_calls;


    g_path_validation_received_program =
        frozen_program;

    g_path_validation_received_revision =
        submitted_revision;

    g_path_validation_received_crc =
        submitted_crc;


    if (state != NULL)
    {
        memset(
            state,
            0,
            sizeof(*state)
        );

        state->initialized =
            true;

        state->result =
            PV_RESULT_RUNNING;

        state->phase =
            PV_PHASE_SNAPSHOT_CHECK;
    }
}


StateStepResult state_path_validation_step(
    PathValidationState *state,
    uint16_t sample_budget,
    PathValidationOutputs *outputs
)
{
    (void)state;
    (void)sample_budget;

    ++g_path_validation_step_calls;


    if (outputs != NULL)
    {
        memset(
            outputs,
            0,
            sizeof(*outputs)
        );

        outputs->report.result =
            g_path_validation_report_result;

        outputs->trajectory_ready =
            g_path_validation_report_result ==
            PV_RESULT_VALID;
    }


    return
        g_path_validation_result;
}


void state_path_validation_get_outputs(
    const PathValidationState *state,
    PathValidationOutputs *outputs
)
{
    (void)state;

    ++g_path_validation_get_outputs_calls;

    if (outputs != NULL)
    {
        memset(
            outputs,
            0,
            sizeof(*outputs)
        );

        outputs->report.result =
            g_path_validation_report_result;

        outputs->trajectory_ready =
            g_path_validation_report_result ==
            PV_RESULT_VALID;
    }
}


/* ----------------------------------------------------------------------------
 * APPROACH
 * ------------------------------------------------------------------------- */

void state_approach_enter(
    ApproachState *state,
    const RobotConfig *robot,
    const ApproachRequest *request,
    const ApproachConfig *config,
    const ApproachServices *services
)
{
    (void)robot;
    (void)config;
    (void)services;

    ++g_approach_enter_calls;


    if (request != NULL)
    {
        g_approach_received_operation =
            request->operation;

        g_approach_received_trajectory =
            request->trajectory;

        g_approach_received_trajectory_ready =
            request->trajectory_ready;

        g_approach_received_program_id =
            request->expected_program_id;

        g_approach_received_revision =
            request->expected_source_revision;

        g_approach_received_artifact_crc =
            request->expected_artifact_crc;

        g_approach_received_clearance_poses =
            request->clearance_poses;

        g_approach_received_clearance_pose_count =
            request->clearance_pose_count;
    }


    if (state != NULL)
    {
        memset(
            state,
            0,
            sizeof(*state)
        );

        state->initialized =
            true;

        state->result =
            APPROACH_RESULT_RUNNING;

        state->phase =
            APPROACH_PHASE_CHECK_REQUEST;
    }
}


StateStepResult state_approach_step(
    ApproachState *state,
    const ApproachControlInputs *inputs,
    ApproachOutputs *outputs
)
{
    (void)state;
    (void)inputs;

    ++g_approach_step_calls;


    if (outputs != NULL)
    {
        memset(
            outputs,
            0,
            sizeof(*outputs)
        );

        outputs->report.result =
            g_approach_report_result;
    }


    return
        g_approach_result;
}


void state_approach_get_outputs(
    const ApproachState *state,
    ApproachOutputs *outputs
)
{
    (void)state;

    ++g_approach_get_outputs_calls;


    if (outputs != NULL)
    {
        memset(
            outputs,
            0,
            sizeof(*outputs)
        );

        outputs->report.result =
            g_approach_report_result;
    }
}


/* ============================================================================
 * TEST FIXTURE
 * ============================================================================ */

typedef struct
{
    StateMachine machine;

    StateMachineDependencies dependencies;
    StateMachineInputs inputs;

    EtherCATMasterConfig ethercat_config;
    RobotConfig robot;
    HomingConfig homing_config;
    TeachingConfig teaching_config;

    PathValidationConfig path_validation_config;
    PathValidationServices path_validation_services;

    PathValidationWorkspace path_validation_workspace;
    PathValidationStorage path_validation_storage;

    ValidatedTrajectory validated_trajectory;

    ApproachConfig approach_config;
    ApproachServices approach_services;

    JointVector clearance_poses[APPROACH_MAX_CLEARANCE_POSES];

} TestFixture;


static void fixture_init(
    TestFixture *fixture
)
{
    assert(
        fixture != NULL
    );


    memset(
        fixture,
        0,
        sizeof(*fixture)
    );


    fixture->dependencies.ethercat_config =
        &fixture->ethercat_config;

    fixture->dependencies.robot =
        &fixture->robot;

    fixture->dependencies.homing_config =
        &fixture->homing_config;

    fixture->dependencies.teaching_config =
        &fixture->teaching_config;

    fixture->dependencies.path_validation_config =
        &fixture->path_validation_config;

    fixture->dependencies.path_validation_services =
        &fixture->path_validation_services;

    fixture->dependencies.path_validation_workspace =
        &fixture->path_validation_workspace;

    fixture->dependencies.path_validation_storage =
        &fixture->path_validation_storage;

    fixture->dependencies.validated_trajectory =
        &fixture->validated_trajectory;

    fixture->dependencies.approach_config =
        &fixture->approach_config;

    fixture->dependencies.approach_services =
        &fixture->approach_services;


    fixture->inputs.idle_command =
        IDLE_COMMAND_NONE;

    fixture->inputs.teaching_event =
        TEACH_EVENT_NONE;

    fixture->inputs.path_validation_sample_budget =
        8U;

    fixture->inputs.approach_operation =
        APPROACH_OPERATION_NONE;
}


/* ============================================================================
 * TEST 1
 *
 * INITIALIZATION ENTERS BOOT
 * ============================================================================ */

static void test_initial_state_is_boot(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    const bool ok =
        state_machine_init(
            &fixture.machine,
            1234U
        );


    assert(ok);

    assert(
        fixture.machine.initialized
    );

    assert(
        state_machine_current_state(
            &fixture.machine
        ) ==
        ROBOT_STATE_BOOT
    );

    assert(
        fixture.machine.previous_state ==
        ROBOT_STATE_BOOT
    );

    assert(
        fixture.machine.teaching_program_id ==
        1234U
    );

    assert(
        g_boot_enter_calls ==
        1U
    );


    printf(
        "[PASS] Initial state is BOOT\n"
    );
}


/* ============================================================================
 * TEST 2
 *
 * BOOT COMPLETE -> HOMING
 * ============================================================================ */

static void test_boot_to_homing(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    g_boot_result =
        STATE_STEP_COMPLETE;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_HOMING
    );

    assert(
        fixture.machine.previous_state ==
        ROBOT_STATE_BOOT
    );

    assert(
        g_boot_step_calls ==
        1U
    );

    assert(
        g_homing_enter_calls ==
        1U
    );


    printf(
        "[PASS] BOOT -> HOMING\n"
    );
}


/* ============================================================================
 * TEST 3
 *
 * HOMING COMPLETE -> IDLE
 * ============================================================================ */

static void test_homing_to_idle(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_HOMING;

    fixture.machine.previous_state =
        ROBOT_STATE_BOOT;


    g_homing_result =
        STATE_STEP_COMPLETE;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_IDLE
    );

    assert(
        fixture.machine.previous_state ==
        ROBOT_STATE_HOMING
    );

    assert(
        g_homing_step_calls ==
        1U
    );

    assert(
        g_idle_enter_calls ==
        1U
    );


    printf(
        "[PASS] HOMING -> IDLE\n"
    );
}


/* ============================================================================
 * TEST 4
 *
 * IDLE + TEACH COMMAND -> TEACHING
 * ============================================================================ */

static void test_idle_to_teaching(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            77U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_IDLE;

    fixture.machine.previous_state =
        ROBOT_STATE_HOMING;


    fixture.inputs.idle_command =
        IDLE_COMMAND_TEACH;


    g_idle_result =
        STATE_STEP_COMPLETE;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_TEACHING
    );

    assert(
        fixture.machine.previous_state ==
        ROBOT_STATE_IDLE
    );

    assert(
        g_idle_step_calls ==
        1U
    );

    assert(
        g_teaching_enter_calls ==
        1U
    );

    assert(
        fixture.machine.teaching.draft.program_id ==
        77U
    );


    printf(
        "[PASS] IDLE -> TEACHING\n"
    );
}


/* ============================================================================
 * TEST 5
 *
 * TEACHING RUNNING STAYS IN TEACHING
 * ============================================================================ */

static void test_teaching_running_stays_teaching(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            9U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_TEACHING;

    fixture.machine.previous_state =
        ROBOT_STATE_IDLE;


    g_teaching_result =
        STATE_STEP_RUNNING;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_TEACHING
    );

    assert(
        g_teaching_step_calls ==
        1U
    );

    assert(
        g_path_validation_enter_calls ==
        0U
    );


    printf(
        "[PASS] TEACHING remains active while RUNNING\n"
    );
}


/* ============================================================================
 * TEST 6
 *
 * TEACHING COMPLETE -> PATH VALIDATION
 *
 * This is the main integration test.
 * ============================================================================ */

static void test_teaching_to_path_validation(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            42U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_TEACHING;

    fixture.machine.previous_state =
        ROBOT_STATE_IDLE;


    /*
     * Put recognizable data in the frozen draft so we can verify that the FSM
     * passes the EXACT Teaching draft object into Path Validation.
     */
    fixture.machine.teaching.draft.program_id =
        42U;

    fixture.machine.teaching.draft.draft_revision =
        17U;

    fixture.machine.teaching.draft.draft_crc =
        0x1234ABCDU;


    g_teaching_result =
        STATE_STEP_COMPLETE;

    g_teaching_validation_request =
        true;

    g_teaching_submitted_revision =
        17U;

    g_teaching_submitted_crc =
        0x1234ABCDU;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );


    /*
     * Global transition happened.
     */
    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_PATH_VALIDATION
    );

    assert(
        fixture.machine.previous_state ==
        ROBOT_STATE_TEACHING
    );


    /*
     * Path Validation was entered exactly once.
     */
    assert(
        g_path_validation_enter_calls ==
        1U
    );


    /*
     * Most important handoff assertion:
     *
     * The FSM must pass Teaching's real frozen draft directly.
     */
    assert(
        g_path_validation_received_program ==
        &fixture.machine.teaching.draft
    );


    /*
     * The exact submitted snapshot identifiers must also be forwarded.
     */
    assert(
        g_path_validation_received_revision ==
        17U
    );

    assert(
        g_path_validation_received_crc ==
        0x1234ABCDU
    );


    /*
     * enter() stub sets PV_RESULT_RUNNING, which is required before the global
     * FSM accepts the transition.
     */
    assert(
        fixture.machine.path_validation.result ==
        PV_RESULT_RUNNING
    );


    printf(
        "[PASS] TEACHING -> PATH_VALIDATION handoff\n"
    );
}


/* ============================================================================
 * TEST 7
 *
 * TEACHING COMPLETE WITHOUT VALIDATION REQUEST IS REJECTED
 * ============================================================================ */

static void test_teaching_complete_without_request_fails(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            5U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_TEACHING;


    g_teaching_result =
        STATE_STEP_COMPLETE;

    g_teaching_validation_request =
        false;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_FAILED
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_TEACHING
    );

    assert(
        g_path_validation_enter_calls ==
        0U
    );


    printf(
        "[PASS] TEACHING COMPLETE without validation request is rejected\n"
    );
}


/* ============================================================================
 * TEST 8
 *
 * PATH VALIDATION RUNNING STAYS ACTIVE
 * ============================================================================ */

static void test_path_validation_running(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_PATH_VALIDATION;

    fixture.machine.previous_state =
        ROBOT_STATE_TEACHING;


    g_path_validation_result =
        STATE_STEP_RUNNING;

    g_path_validation_report_result =
        PV_RESULT_RUNNING;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_PATH_VALIDATION
    );

    assert(
        g_path_validation_step_calls ==
        1U
    );


    printf(
        "[PASS] PATH_VALIDATION remains active while RUNNING\n"
    );
}


/* ============================================================================
 * TEST 9
 *
 * PATH VALIDATION VALID COMPLETES BUT DOES NOT INVENT NEXT STATE
 * ============================================================================ */

static void test_path_validation_valid_complete(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_PATH_VALIDATION;

    fixture.machine.previous_state =
        ROBOT_STATE_TEACHING;


    g_path_validation_result =
        STATE_STEP_COMPLETE;

    g_path_validation_report_result =
        PV_RESULT_VALID;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_COMPLETE
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_PATH_VALIDATION
    );

    assert(
        fixture.machine.path_validation_outputs.report.result ==
        PV_RESULT_VALID
    );

    assert(
        fixture.machine.path_validation_outputs.trajectory_ready
    );


    printf(
        "[PASS] PATH_VALIDATION VALID completes and remains in state\n"
    );
}


/* ============================================================================
 * TEST 10
 *
 * PATH VALIDATION VALID + OPERATION REQUEST -> APPROACH
 * ============================================================================ */

static void test_path_validation_to_approach(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_PATH_VALIDATION;

    fixture.machine.previous_state =
        ROBOT_STATE_TEACHING;


    fixture.validated_trajectory.program_id =
        42U;

    fixture.validated_trajectory.source_revision =
        7U;

    fixture.validated_trajectory.artifact_crc =
        0xAABBCCDDU;


    fixture.inputs.approach_operation =
        APPROACH_OPERATION_PREVIEW;

    fixture.inputs.approach_clearance_poses =
        fixture.clearance_poses;

    fixture.inputs.approach_clearance_pose_count =
        1U;


    g_path_validation_result =
        STATE_STEP_COMPLETE;

    g_path_validation_report_result =
        PV_RESULT_VALID;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_APPROACH
    );

    assert(
        fixture.machine.previous_state ==
        ROBOT_STATE_PATH_VALIDATION
    );

    assert(
        g_approach_enter_calls ==
        1U
    );


    /*
     * Verify the global FSM passes the exact validated artifact identity and
     * requested Approach operation into state_approach_enter().
     */
    assert(
        g_approach_received_operation ==
        APPROACH_OPERATION_PREVIEW
    );

    assert(
        g_approach_received_trajectory ==
        &fixture.validated_trajectory
    );

    assert(
        g_approach_received_trajectory_ready
    );

    assert(
        g_approach_received_program_id ==
        42U
    );

    assert(
        g_approach_received_revision ==
        7U
    );

    assert(
        g_approach_received_artifact_crc ==
        0xAABBCCDDU
    );

    assert(
        g_approach_received_clearance_poses ==
        fixture.clearance_poses
    );

    assert(
        g_approach_received_clearance_pose_count ==
        1U
    );


    printf(
        "[PASS] PATH_VALIDATION -> APPROACH handoff\n"
    );
}


/* ============================================================================
 * TEST 11
 *
 * APPROACH RUNNING STAYS ACTIVE
 * ============================================================================ */

static void test_approach_running(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_APPROACH;

    fixture.machine.previous_state =
        ROBOT_STATE_PATH_VALIDATION;


    g_approach_result =
        STATE_STEP_RUNNING;

    g_approach_report_result =
        APPROACH_RESULT_RUNNING;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_RUNNING
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_APPROACH
    );

    assert(
        g_approach_step_calls ==
        1U
    );


    printf(
        "[PASS] APPROACH remains active while RUNNING\n"
    );
}


/* ============================================================================
 * TEST 12
 *
 * APPROACH COMPLETE IS EXPOSED WITHOUT INVENTING NEXT STATE
 * ============================================================================ */

static void test_approach_complete(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_APPROACH;

    fixture.machine.previous_state =
        ROBOT_STATE_PATH_VALIDATION;


    g_approach_result =
        STATE_STEP_COMPLETE;

    g_approach_report_result =
        APPROACH_RESULT_COMPLETE;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    assert(
        result ==
        STATE_STEP_COMPLETE
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_APPROACH
    );

    assert(
        fixture.machine.approach_outputs.report.result ==
        APPROACH_RESULT_COMPLETE
    );


    printf(
        "[PASS] APPROACH COMPLETE is exposed to the supervisor\n"
    );
}


/* ============================================================================
 * TEST 13
 *
 * PATH VALIDATION INVALID COMPLETES BUT IS NOT A GLOBAL HARDWARE FAILURE
 * ============================================================================ */

static void test_path_validation_invalid_complete(void)
{
    reset_stub_state();


    TestFixture fixture;
    fixture_init(
        &fixture
    );


    assert(
        state_machine_init(
            &fixture.machine,
            1U
        )
    );


    fixture.machine.current_state =
        ROBOT_STATE_PATH_VALIDATION;

    fixture.machine.previous_state =
        ROBOT_STATE_TEACHING;


    g_path_validation_result =
        STATE_STEP_COMPLETE;

    g_path_validation_report_result =
        PV_RESULT_INVALID;


    const StateStepResult result =
        state_machine_step(
            &fixture.machine,
            &fixture.dependencies,
            &fixture.inputs
        );


    /*
     * INVALID path is a completed validation result, not automatically a
     * controller crash/fault.
     */
    assert(
        result ==
        STATE_STEP_COMPLETE
    );

    assert(
        fixture.machine.current_state ==
        ROBOT_STATE_PATH_VALIDATION
    );

    assert(
        fixture.machine.path_validation_outputs.report.result ==
        PV_RESULT_INVALID
    );

    assert(
        !fixture.machine.path_validation_outputs.trajectory_ready
    );


    printf(
        "[PASS] PATH_VALIDATION INVALID is a normal completed result\n"
    );
}


/* ============================================================================
 * TEST 14
 *
 * NAME HELPER
 * ============================================================================ */

static void test_state_name_helper(void)
{
    assert(
        strcmp(
            state_machine_state_name(
                ROBOT_STATE_BOOT
            ),
            "BOOT"
        ) == 0
    );

    assert(
        strcmp(
            state_machine_state_name(
                ROBOT_STATE_HOMING
            ),
            "HOMING"
        ) == 0
    );

    assert(
        strcmp(
            state_machine_state_name(
                ROBOT_STATE_IDLE
            ),
            "IDLE"
        ) == 0
    );

    assert(
        strcmp(
            state_machine_state_name(
                ROBOT_STATE_TEACHING
            ),
            "TEACHING"
        ) == 0
    );

    assert(
        strcmp(
            state_machine_state_name(
                ROBOT_STATE_PATH_VALIDATION
            ),
            "PATH_VALIDATION"
        ) == 0
    );

    assert(
        strcmp(
            state_machine_state_name(
                ROBOT_STATE_APPROACH
            ),
            "APPROACH"
        ) == 0
    );


    printf(
        "[PASS] State-name helper\n"
    );
}


/* ============================================================================
 * MAIN
 * ============================================================================ */

int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " GLOBAL STATE MACHINE UNIT TEST\n"
        "============================================================\n"
    );


    test_initial_state_is_boot();

    test_boot_to_homing();

    test_homing_to_idle();

    test_idle_to_teaching();

    test_teaching_running_stays_teaching();

    test_teaching_to_path_validation();

    test_teaching_complete_without_request_fails();

    test_path_validation_running();

    test_path_validation_valid_complete();

    test_path_validation_to_approach();

    test_approach_running();

    test_approach_complete();

    test_path_validation_invalid_complete();

    test_state_name_helper();


    printf(
        "============================================================\n"
        " ALL GLOBAL STATE MACHINE TESTS PASSED\n"
        "============================================================\n\n"
    );


    return 0;
}