#include "../States/state_emergency_stop.h"
#include "../States/state_fault.h"
#include "../States/state_paused.h"

#include <stdio.h>

static unsigned passed;
static unsigned failed;

#define CHECK(condition, name) do {                                      \
    if (condition) { ++passed; printf("[PASS] %s\n", name); }          \
    else { ++failed; printf("[FAIL] %s\n", name); }                    \
} while (0)

typedef struct
{
    bool wire_feed_enabled;
    bool stopped;
} Mock;

static bool safe_outputs(void *ctx)
{
    ((Mock *)ctx)->wire_feed_enabled = false;
    return true;
}

static bool hold(bool *stopped, void *ctx)
{
    Mock *mock = (Mock *)ctx;
    mock->stopped = true;
    *stopped = true;
    return true;
}

static bool wire_feed_off(void *ctx)
{
    ((Mock *)ctx)->wire_feed_enabled = false;
    return true;
}

static void run_safety_state_tests(void)
{
    Mock mock = {.wire_feed_enabled = true};

    EmergencyStopServices es = {
        safe_outputs,
        &mock
    };

    EmergencyStopState estop;
    EmergencyStopInputs ei = {
        .estop_active = true
    };

    state_emergency_stop_enter(
        &estop,
        ROBOT_STATE_PATH_EXECUTION,
        10U,
        &es
    );

    (void)state_emergency_stop_step(
        &estop,
        &ei
    );

    CHECK(
        !mock.wire_feed_enabled,
        "E-stop forces the wire-feed relay OFF"
    );

    CHECK(
        estop.phase == ESTOP_PHASE_WAIT_RELEASE,
        "E-stop waits for physical release"
    );

    ei.estop_active = false;

    (void)state_emergency_stop_step(
        &estop,
        &ei
    );

    CHECK(
        estop.phase == ESTOP_PHASE_WAIT_RESET,
        "Release alone does not complete E-stop recovery"
    );

    ei.reset_acknowledged = true;
    ei.safety_healthy = true;

    (void)state_emergency_stop_step(
        &estop,
        &ei
    );

    CHECK(
        estop.phase == ESTOP_PHASE_WAIT_HOME,
        "Reset acknowledgement advances to explicit Home wait"
    );

    FaultServices fs = {
        safe_outputs,
        &mock
    };

    FaultState fault;
    FaultInputs fi = {
        .fault_cause_active = true,
        .safety_healthy = false
    };

    state_fault_enter(
        &fault,
        ROBOT_STATE_PATH_EXECUTION,
        ROBOT_FAULT_SEVERITY_RECOVERABLE,
        99U,
        true,
        &fs
    );

    (void)state_fault_step(&fault, &fi);
    (void)state_fault_step(&fault, &fi);

    CHECK(
        fault.phase == FAULT_PHASE_WAIT_CAUSE_CLEAR,
        "Fault remains latched while its cause exists"
    );

    fi.fault_cause_active = false;
    fi.safety_healthy = true;

    (void)state_fault_step(
        &fault,
        &fi
    );

    CHECK(
        fault.phase == FAULT_PHASE_WAIT_RESET,
        "Cleared fault waits for explicit Reset"
    );

    PausedServices ps = {
        hold,
        wire_feed_off,
        &mock
    };

    PausedState paused;
    PausedInputs pi = {0};

    state_paused_enter(
        &paused,
        ROBOT_STATE_PATH_EXECUTION,
        &ps
    );

    (void)state_paused_step(
        &paused,
        &pi
    );

    (void)state_paused_step(
        &paused,
        &pi
    );

    CHECK(
        paused.phase == PAUSED_PHASE_READY &&
        paused.hold_established,
        "Paused state establishes a confirmed hold"
    );
}

int main(void)
{
    printf("\nRUNTIME SAFETY STATE MODULE TESTS\n");
    run_safety_state_tests();
    printf(
        "Tests passed: %u\nTests failed: %u\n",
        passed,
        failed
    );

    return failed == 0U ? 0 : 1;
}
