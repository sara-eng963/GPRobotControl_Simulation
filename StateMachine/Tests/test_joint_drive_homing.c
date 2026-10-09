/* Host-only HOMING state test: no CANopen, SIL Kit or motor hardware. */
#include "../States/state_homing.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    bool configured, network_ok, feedback_ok, send_ok, auto_feedback;
    bool pending_feedback;
    bool fail_axis_read;
    uint32_t sequence[JOINT_DRIVE_AXES];
    JointDriveAxisFeedback axis[JOINT_DRIVE_AXES];
    int32_t last_targets[JOINT_DRIVE_AXES];
    unsigned commands;
} FakeDrive;

static int failures;

#define CHECK(condition, message) do { \
    if (condition) printf("[PASS] %s\n", message); \
    else { printf("[FAIL] %s\n", message); ++failures; } \
} while (0)

static bool configured(void *context)
{ return ((FakeDrive *)context)->configured; }

static bool poll(void *context, uint32_t now_ms)
{
    (void)now_ms;
    FakeDrive *f = (FakeDrive *)context;
    if (f->pending_feedback && f->auto_feedback) {
        for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
            f->axis[i].actual_position_units = f->last_targets[i];
            ++f->sequence[i];
        }
        f->pending_feedback = false;
    }
    return true;
}
static bool healthy(void *context, uint32_t now_ms)
{
    (void)now_ms;
    return ((FakeDrive *)context)->network_ok;
}
static bool all_feedback_valid(void *context)
{ return ((FakeDrive *)context)->feedback_ok; }

static bool read_axis(void *context, size_t axis, JointDriveAxisFeedback *out)
{
    FakeDrive *f = (FakeDrive *)context;
    if (axis >= JOINT_DRIVE_AXES || out == NULL ||
        (f->fail_axis_read && axis == 2U)) return false;
    *out = f->axis[axis];
    return true;
}
static bool send_targets(void *context, const int32_t *targets, size_t count)
{
    FakeDrive *f = (FakeDrive *)context;
    if (!f->send_ok || count != JOINT_DRIVE_AXES || targets == NULL)
        return false;
    memcpy(f->last_targets, targets, sizeof(f->last_targets));
    ++f->commands;
    f->pending_feedback = true;
    return true;
}
static uint32_t feedback_sequence(void *context, size_t axis)
{ return ((FakeDrive *)context)->sequence[axis]; }

static JointDrivePort make_port(FakeDrive *f)
{
    const JointDrivePort port = {
        .context = f,
        .is_configured = configured,
        .poll = poll,
        .ready_for_motion = healthy,
        .healthy = healthy,
        .all_feedback_valid = all_feedback_valid,
        .read_axis = read_axis,
        .send_targets = send_targets,
        .feedback_sequence = feedback_sequence
    };
    return port;
}
static void reset_drive(FakeDrive *f)
{
    memset(f, 0, sizeof(*f));
    f->configured = true;
    f->network_ok = true;
    f->feedback_ok = true;
    f->send_ok = true;
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
        f->axis[i].feedback_valid = true;
        f->axis[i].operation_enabled = true;
        f->sequence[i] = 1U;
    }
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
    JointDrivePort port = make_port(&fake);
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
