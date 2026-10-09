/* Host-only HOMING state test: no CANopen, SIL Kit or motor hardware. */
#include "../States/state_homing.h"
#include "fake_joint_drive.h"

#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(condition, message) do { \
    if (condition) printf("[PASS] %s\n", message); \
    else { printf("[FAIL] %s\n", message); ++failures; } \
} while (0)

static void reset_drive(FakeDrive *f)
{
    fake_joint_drive_reset(f);
}

static void initialize_robot(RobotConfig *robot, AvatarMPositionScale *scale)
{
    robot_config_init_ur5(robot);
    robot->configuration.homeDefined = true;
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
        robot->configuration.home[i] = 0.015;
        if (!avatar_m_position_scale_default(&scale[i], 51.0))
            ++failures;
    }
}
static HomingConfig make_config(void)
{
    HomingConfig c = {
        .duration = 0.05,
        .dt = 0.002,
        .positionTolerance = 0.001,
        .requiredStableCycles = 2,
        .maxVerificationCycles = 10
    };
    return c;
}
static void to_position_read(HomingState *h, const HomingConfig *c,
                             const RobotConfig *robot,
                             const JointDrivePort *port,
                             const AvatarMPositionScale *scale)
{
    state_homing_enter(h);
    (void)state_homing_step(h, c, robot, port, scale, 0U);
}
static void to_execution(HomingState *h, const HomingConfig *c,
                         const RobotConfig *robot, const JointDrivePort *port,
                         const AvatarMPositionScale *scale)
{
    to_position_read(h, c, robot, port, scale);
    (void)state_homing_step(h, c, robot, port, scale, 1U);
    (void)state_homing_step(h, c, robot, port, scale, 2U);
}
int main(void)
{
    RobotConfig robot;
    AvatarMPositionScale scale[JOINT_DRIVE_AXES];
    initialize_robot(&robot, scale);
    const HomingConfig c = make_config();
    FakeDrive fake;
    HomingState h;

    reset_drive(&fake);
    JointDrivePort port = fake_joint_drive_make_port(&fake);
    to_execution(&h, &c, &robot, &port, scale);
    CHECK(h.phase == HOMING_PHASE_EXECUTE_TRAJECTORY,
          "HOMING enters trajectory execution through the port");
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 3U) ==
              STATE_STEP_RUNNING && fake.commands == 1U && h.awaitingFeedback,
          "HOMING sends first six-joint sample and awaits feedback");
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 4U) ==
              STATE_STEP_RUNNING && fake.commands == 1U,
          "HOMING does not send another command before fresh feedback");
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 103U) ==
              STATE_STEP_FAILED && h.error == HOMING_ERROR_CYCLIC_FEEDBACK,
          "HOMING preserves missing-feedback watchdog");

    reset_drive(&fake);
    fake.network_ok = false;
    to_position_read(&h, &c, &robot, &port, scale);
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 1U) ==
              STATE_STEP_FAILED && h.error == HOMING_ERROR_COMMUNICATION,
          "HOMING distinguishes communication failures");

    reset_drive(&fake);
    fake.feedback_ok = false;
    to_position_read(&h, &c, &robot, &port, scale);
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 1U) ==
              STATE_STEP_FAILED && h.error == HOMING_ERROR_POSITION_FEEDBACK,
          "HOMING detects missing feedback");

    reset_drive(&fake);
    fake.axis[4].operation_enabled = false;
    to_position_read(&h, &c, &robot, &port, scale);
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 1U) ==
              STATE_STEP_FAILED && h.error == HOMING_ERROR_DRIVE_NOT_ENABLED &&
              h.failedAxis == 5,
          "HOMING preserves disabled-axis fault reporting");

    reset_drive(&fake);
    fake.fail_axis_read = true;
    to_position_read(&h, &c, &robot, &port, scale);
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 1U) ==
              STATE_STEP_FAILED && h.error == HOMING_ERROR_DRIVE_NOT_ENABLED &&
              h.failedAxis == 3,
          "HOMING detects unreadable axis status");

    reset_drive(&fake);
    fake.send_ok = false;
    to_execution(&h, &c, &robot, &port, scale);
    CHECK(state_homing_step(&h, &c, &robot, &port, scale, 3U) ==
              STATE_STEP_FAILED && h.error == HOMING_ERROR_COMMUNICATION,
          "HOMING detects failed six-axis command transactions");

    reset_drive(&fake);
    fake.auto_feedback = true;
    state_homing_enter(&h);
    StateStepResult result = STATE_STEP_RUNNING;
    for (uint32_t ms = 0; ms < 500U && result == STATE_STEP_RUNNING; ++ms)
        result = state_homing_step(&h, &c, &robot, &port, scale, ms);
    CHECK(result == STATE_STEP_COMPLETE && h.phase == HOMING_PHASE_COMPLETE,
          "HOMING completes full trajectory and home verification");
    CHECK(fake.commands > 1U && h.samplesSent > 1U &&
          h.stableCycles >= c.requiredStableCycles,
          "HOMING verifies fresh feedback after multiple target cycles");

    printf("\nHOMING port tests failed: %d\n", failures);
    return failures != 0;
}
