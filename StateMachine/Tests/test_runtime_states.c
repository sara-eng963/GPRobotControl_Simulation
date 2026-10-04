#include "../States/state_emergency_stop.h"
#include "../States/state_fault.h"
#include "../States/state_path_execution.h"
#include "../States/state_paused.h"

#include <stdio.h>
#include <string.h>

static unsigned passed;
static unsigned failed;

#define CHECK(condition, name) do {                                      \
    if (condition) { ++passed; printf("[PASS] %s\n", name); }          \
    else { ++failed; printf("[FAIL] %s\n", name); }                    \
} while (0)

typedef struct
{
    PvExecutionSample samples[3];
    unsigned writes;
    unsigned retract_steps;
    bool wire_feed_enabled;
    bool stopped;
} Mock;

static bool read_sample(uint32_t index, PvExecutionSample *sample, void *ctx)
{
    Mock *mock = ctx;
    if (index >= 3U) return false;
    *sample = mock->samples[index];
    return true;
}

static bool write_targets(const PvExecutionSample *sample, void *ctx)
{
    Mock *mock = ctx;
    (void)sample;
    mock->writes++;
    return true;
}

static bool wire_feed_enable(bool enable, void *ctx)
{
    ((Mock *)ctx)->wire_feed_enabled = enable;
    return true;
}

static bool prepare_retraction(void *ctx)
{
    ((Mock *)ctx)->retract_steps = 0U;
    return true;
}

static StateStepResult retract_step(void *ctx)
{
    Mock *mock = ctx;
    return (++mock->retract_steps >= 2U)
        ? STATE_STEP_COMPLETE : STATE_STEP_RUNNING;
}

static bool clearance(void *ctx)
{
    (void)ctx;
    return true;
}

static bool controlled_stop(bool *stopped, void *ctx)
{
    Mock *mock = ctx;
    mock->stopped = true;
    *stopped = true;
    return true;
}

static bool safe_outputs(void *ctx)
{
    ((Mock *)ctx)->wire_feed_enabled = false;
    return true;
}

static bool hold(bool *stopped, void *ctx)
{
    ((Mock *)ctx)->stopped = true;
    *stopped = true;
    return true;
}

static bool wire_feed_off(void *ctx)
{
    ((Mock *)ctx)->wire_feed_enabled = false;
    return true;
}

static void run_preview_test(void)
{
    Mock mock = {0};
    ValidatedTrajectory trajectory = {0};
    trajectory.program_id = 5U;
    trajectory.source_revision = 7U;
    trajectory.artifact_crc = 11U;
    trajectory.sample_count = 3U;
    trajectory.sample_period_us = PATH_VALIDATION_SAMPLE_PERIOD_US;

    PathExecutionRequest request = {
        ROBOT_EXECUTION_PREVIEW, &trajectory, true, 5U, 7U, 11U
    };
    PathExecutionConfig config = {1000U, 2U};
    PathExecutionServices services = {
        read_sample, write_targets, wire_feed_enable,
        prepare_retraction, retract_step, clearance,
        controlled_stop, &mock
    };
    PathExecutionInputs inputs = {
        .motion_permission = true,
        .drives_ready = true,
        .ethercat_healthy = true
    };
    PathExecutionState state;
    PathExecutionOutputs outputs;
    state_path_execution_enter(&state, &request, &config, &services);

    StateStepResult result = STATE_STEP_RUNNING;
    for (unsigned i = 0U; i < 30U && result == STATE_STEP_RUNNING; ++i)
    {
        inputs.now_ms = i;
        result = state_path_execution_step(&state, &inputs, &outputs);
    }

    CHECK(result == STATE_STEP_COMPLETE, "Preview execution completes");
    CHECK(mock.writes == 3U, "Every 1 ms sample is commanded once");
    CHECK(!mock.wire_feed_enabled,
          "Preview keeps the wire-feed relay OFF");
    CHECK(outputs.phase == PATH_EXEC_PHASE_COMPLETE,
          "Preview completes after retraction and clearance");
}

static void run_production_test(void)
{
    Mock mock = {0};
    ValidatedTrajectory trajectory = {0};
    trajectory.program_id = 1U;
    trajectory.source_revision = 2U;
    trajectory.artifact_crc = 3U;
    trajectory.sample_count = 3U;
    trajectory.sample_period_us = PATH_VALIDATION_SAMPLE_PERIOD_US;
    PathExecutionRequest request = {
        ROBOT_EXECUTION_PRODUCTION, &trajectory, true, 1U, 2U, 3U
    };
    PathExecutionConfig config = {100U, 1U};
    PathExecutionServices services = {
        read_sample, write_targets, wire_feed_enable,
        prepare_retraction, retract_step, clearance,
        controlled_stop, &mock
    };
    PathExecutionInputs inputs = {
        .motion_permission = true, .drives_ready = true,
        .ethercat_healthy = true
    };
    PathExecutionState state;
    state_path_execution_enter(&state, &request, &config, &services);
    for (unsigned i = 0U; i < 5U; ++i)
    {
        inputs.now_ms = i;
        (void)state_path_execution_step(&state, &inputs, NULL);
    }
    CHECK(mock.wire_feed_enabled,
          "Production switches the wire-feed relay ON");
    StateStepResult result = STATE_STEP_RUNNING;
    for (unsigned i = 5U; i < 30U && result == STATE_STEP_RUNNING; ++i)
    {
        inputs.now_ms = i;
        result = state_path_execution_step(&state, &inputs, NULL);
    }
    CHECK(result == STATE_STEP_COMPLETE, "Production execution completes");
    CHECK(!mock.wire_feed_enabled,
          "Production switches wire feed OFF before retraction");
}

static void run_abort_test(void)
{
    Mock mock = {0};
    ValidatedTrajectory trajectory = {1U, 2U, 0U, 3U, 0U, 3U, 0U,
        PATH_VALIDATION_SAMPLE_PERIOD_US};
    PathExecutionRequest request = {
        ROBOT_EXECUTION_PREVIEW, &trajectory, true, 1U, 2U, 3U
    };
    PathExecutionConfig config = {100U, 1U};
    PathExecutionServices services = {
        read_sample, write_targets, wire_feed_enable,
        prepare_retraction, retract_step, clearance,
        controlled_stop, &mock
    };
    PathExecutionInputs inputs = {
        .motion_permission = true, .drives_ready = true,
        .ethercat_healthy = true
    };
    PathExecutionState state;
    state_path_execution_enter(&state, &request, &config, &services);
    (void)state_path_execution_step(&state, &inputs, NULL);
    inputs.reset_requested = true;
    (void)state_path_execution_step(&state, &inputs, NULL);
    inputs.reset_requested = false;
    StateStepResult result = state_path_execution_step(&state, &inputs, NULL);
    CHECK(result == STATE_STEP_RUNNING || result == STATE_STEP_COMPLETE,
          "Reset uses controlled stop before reporting abort");
    result = state_path_execution_step(&state, &inputs, NULL);
    CHECK(result == STATE_STEP_COMPLETE, "Controlled Reset abort completes");
    CHECK(state.abort_reason == PATH_EXEC_ABORT_RESET,
          "Reset abort reason is retained");
}

static void run_safety_state_tests(void)
{
    Mock mock = {.wire_feed_enabled = true};
    EmergencyStopServices es = {safe_outputs, &mock};
    EmergencyStopState estop;
    EmergencyStopInputs ei = {.estop_active = true};
    state_emergency_stop_enter(&estop, ROBOT_STATE_PATH_EXECUTION, 10U, &es);
    (void)state_emergency_stop_step(&estop, &ei);
    CHECK(!mock.wire_feed_enabled,
          "E-stop forces the wire-feed relay OFF");
    CHECK(estop.phase == ESTOP_PHASE_WAIT_RELEASE,
          "E-stop waits for physical release");
    ei.estop_active = false;
    (void)state_emergency_stop_step(&estop, &ei);
    CHECK(estop.phase == ESTOP_PHASE_WAIT_RESET,
          "Release alone does not complete E-stop recovery");
    ei.reset_acknowledged = true;
    ei.safety_healthy = true;
    (void)state_emergency_stop_step(&estop, &ei);
    CHECK(estop.phase == ESTOP_PHASE_WAIT_HOME,
          "Reset acknowledgement advances to explicit Home wait");

    FaultServices fs = {safe_outputs, &mock};
    FaultState fault;
    FaultInputs fi = {.fault_cause_active = true, .safety_healthy = false};
    state_fault_enter(&fault, ROBOT_STATE_PATH_EXECUTION,
                      ROBOT_FAULT_SEVERITY_RECOVERABLE, 99U, true, &fs);
    (void)state_fault_step(&fault, &fi);
    (void)state_fault_step(&fault, &fi);
    CHECK(fault.phase == FAULT_PHASE_WAIT_CAUSE_CLEAR,
          "Fault remains latched while its cause exists");
    fi.fault_cause_active = false;
    fi.safety_healthy = true;
    (void)state_fault_step(&fault, &fi);
    CHECK(fault.phase == FAULT_PHASE_WAIT_RESET,
          "Cleared fault waits for explicit Reset");

    PausedServices ps = {hold, wire_feed_off, &mock};
    PausedState paused;
    PausedInputs pi = {0};
    state_paused_enter(&paused, ROBOT_STATE_PATH_EXECUTION, &ps);
    (void)state_paused_step(&paused, &pi);
    (void)state_paused_step(&paused, &pi);
    CHECK(paused.phase == PAUSED_PHASE_READY && paused.hold_established,
          "Paused state establishes a confirmed hold");
}

int main(void)
{
    printf("\nRUNTIME STATE MODULE TESTS\n");
    run_preview_test();
    run_production_test();
    run_abort_test();
    run_safety_state_tests();
    printf("Tests passed: %u\nTests failed: %u\n", passed, failed);
    return failed == 0U ? 0 : 1;
}
