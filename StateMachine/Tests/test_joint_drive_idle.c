/* Host-only application-layer regression: IDLE must not know CANopen. */
#include "../States/state_idle.h"
#include "fake_joint_drive.h"

#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(cond, name) do { \
    if (cond) { printf("[PASS] %s\n", name); } \
    else { printf("[FAIL] %s\n", name); ++failures; } \
} while (0)

static void reset_fake(FakeDrive *f)
{
    fake_joint_drive_reset(f);
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i)
        f->axis[i].actual_position_units = (int32_t)(1000 + i * 100);
}

int main(void)
{
    FakeDrive fake;
    IdleState idle;

    reset_fake(&fake);
    JointDrivePort port = fake_joint_drive_make_port(&fake);
    CHECK(joint_drive_port_all_enabled(&port) &&
          fake.configuration_checks == 1U,
          "Composite enable check validates port configuration exactly once");
    fake.configured = false;
    CHECK(!joint_drive_port_all_enabled(&port) &&
          fake.configuration_checks == 2U,
          "Composite enable check rejects an unconfigured drive");
    fake.configured = true;
    state_idle_enter(&idle);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0) == STATE_STEP_RUNNING &&
          idle.phase == IDLE_PHASE_HOLDING,
          "IDLE enters holding after valid six-axis feedback");
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 1) == STATE_STEP_RUNNING &&
          fake.commands == 1U,
          "IDLE issues first six-axis hold cycle");
    bool held_exact = true;
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i)
        held_exact = held_exact &&
            fake.last_targets[i] == fake.axis[i].actual_position_units;
    CHECK(held_exact, "Hold commands equal raw feedback positions without conversion");

    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 2) == STATE_STEP_RUNNING &&
          fake.commands == 1U && idle.awaitingFeedback,
          "IDLE waits for fresh TPDO4 data before another hold cycle");
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) ++fake.sequence[i];
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 3) == STATE_STEP_RUNNING &&
          fake.commands == 2U && idle.cyclesHeld == 1U,
          "Fresh feedback authorizes the next hold cycle");

    fake.network_ok = false;
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 4) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_COMMUNICATION,
          "Network failure produces communication fault");

    reset_fake(&fake);
    fake.feedback_ok = false;
    state_idle_enter(&idle);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_POSITION_FEEDBACK,
          "Missing feedback retains its distinct fault");

    reset_fake(&fake);
    fake.axis[1].operation_enabled = false;
    state_idle_enter(&idle);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_DRIVE_NOT_ENABLED && idle.failedAxis == 2,
          "Disabled axis reports the correct joint number");

    reset_fake(&fake);
    state_idle_enter(&idle);
    (void)state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0);
    (void)state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 1);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 102) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_CYCLIC_FEEDBACK,
          "Missing post-command feedback trips watchdog");

    reset_fake(&fake);
    state_idle_enter(&idle);
    (void)state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_TEACH, &port, 1) == STATE_STEP_RUNNING &&
          idle.phase == IDLE_PHASE_COMPLETE &&
          idle.exitCommand == IDLE_COMMAND_TEACH,
          "Teach command exits holding through state transition");
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 2) == STATE_STEP_COMPLETE,
          "Completed IDLE state reports completion");

    printf("\nIDLE tests failed: %d\n", failures);
    return failures != 0;
}
