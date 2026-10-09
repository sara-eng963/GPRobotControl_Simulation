/*
 * Host-only TEACHING regression: fake JointDrivePort, no CANopen or SIL Kit.
 * Verifies measured position conversion and recording eligibility.
 * Does not test hardware safety or manual-guidance control.
 */
#include "../States/state_teaching.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    bool configured, poll_ok, network_ok, feedback_ok;
    bool drive_enabled[JOINT_DRIVE_AXES];
    bool axis_feedback_valid[JOINT_DRIVE_AXES];
    int32_t actual_units[JOINT_DRIVE_AXES];
    unsigned polls, reads, sends;
} FakeDrive;

static int failures;
#define CHECK(condition, message) do { \
    if (condition) printf("[PASS] %s\n", message); \
    else { printf("[FAIL] %s\n", message); ++failures; } \
} while (0)

static bool configured(void *ctx) { return ((FakeDrive *)ctx)->configured; }
static bool poll_drive(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    FakeDrive *fake = (FakeDrive *)ctx;
    ++fake->polls;
    return fake->poll_ok;
}
static bool healthy(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    return ((FakeDrive *)ctx)->network_ok;
}
static bool all_feedback(void *ctx) { return ((FakeDrive *)ctx)->feedback_ok; }
static bool read_axis(void *ctx, size_t axis, JointDriveAxisFeedback *out)
{
    FakeDrive *fake = (FakeDrive *)ctx;
    if (axis >= JOINT_DRIVE_AXES || out == NULL) return false;
    ++fake->reads;
    out->feedback_valid = fake->axis_feedback_valid[axis];
    out->operation_enabled = fake->drive_enabled[axis];
    out->actual_position_units = fake->actual_units[axis];
    return true;
}
static bool send_targets(void *ctx, const int32_t *targets, size_t count)
{
    (void)targets;
    FakeDrive *fake = (FakeDrive *)ctx;
    ++fake->sends;
    return count == JOINT_DRIVE_AXES;
}
static uint32_t feedback_sequence(void *ctx, size_t axis)
{
    (void)ctx; (void)axis;
    return 1U;
}
static JointDrivePort make_port(FakeDrive *fake)
{
    JointDrivePort port = {
        .context = fake,
        .is_configured = configured,
        .poll = poll_drive,
        .ready_for_motion = healthy,
        .healthy = healthy,
        .all_feedback_valid = all_feedback,
        .read_axis = read_axis,
        .send_targets = send_targets,
        .feedback_sequence = feedback_sequence
    };
    return port;
}
static void reset_fake(FakeDrive *fake)
{
    memset(fake, 0, sizeof(*fake));
    fake->configured = true;
    fake->poll_ok = true;
    fake->network_ok = true;
    fake->feedback_ok = true;
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
        fake->drive_enabled[i] = true;
        fake->axis_feedback_valid[i] = true;
        fake->actual_units[i] = (int32_t)(100 + 10 * i);
    }
}
static TeachingConfig teaching_config(void)
{
    TeachingConfig c = {
        .default_speed_mps = 0.01F,
        .minimum_speed_mps = 0.001F,
        .maximum_speed_mps = 0.1F,
        .speed_step_mps = 0.001F,
        .minimum_point_separation_m = 0.002F,
        .collinearity_epsilon_m2 = 1.0e-10F
    };
    return c;
}
static TeachingRuntimeInputs teaching_runtime(void)
{
    TeachingRuntimeInputs r = {
        .timestamp_ms = 100U, .calibration_version = 1U,
        .robot_homed = true, .robot_motion_settled = true,
        .manual_guidance_active = true, .motion_permitted = true
    };
    return r;
}
int main(void)
{
    RobotConfig robot;
    AvatarMPositionScale scales[JOINT_DRIVE_AXES];
    robot_config_init_ur5(&robot);
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i)
        CHECK(avatar_m_position_scale_default(&scales[i], 50.0),
              "Create valid joint position scale");

    FakeDrive fake;
    reset_fake(&fake);
    JointDrivePort port = make_port(&fake);
    const TeachingConfig config = teaching_config();
    TeachingRuntimeInputs runtime = teaching_runtime();
    TeachingState state;
    TeachingOutputs output = {0};
    state_teaching_enter(&state, &config, 12U);

    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              100U, TEACH_EVENT_SELECT_LINE, &output) ==
              STATE_STEP_RUNNING &&
          state.last_event_accepted && state.phase == TEACH_PHASE_WAIT_POINT,
          "TEACHING accepts line selection through fake joint-drive port");

    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              101U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING && state.captured_point_count == 1U &&
          state.working_segment.points[0].point_valid &&
          state.last_event_accepted,
          "TEACHING records measured point with healthy drives");

    double expected_q = 0.0;
    CHECK(avatar_m_position_units_to_joint_rad(&scales[0],
              fake.actual_units[0], &expected_q) &&
          fabs((double)state.working_segment.points[0].joint_position_rad[0] -
               expected_q) < 1.0e-6,
          "Recorded joint position matches measured raw position conversion");
    CHECK(fake.sends == 0U,
          "TEACHING never sends motion commands itself");

    state_teaching_enter(&state, &config, 13U);
    (void)state_teaching_step(&state, &robot, &runtime, &port, scales,
                             100U, TEACH_EVENT_SELECT_LINE, &output);
    fake.drive_enabled[2] = false;
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              101U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          state.captured_point_count == 0U &&
          state.error == TEACH_ERR_DRIVES_NOT_READY,
          "TEACHING rejects recording with axis 3 disabled");
    fake.drive_enabled[2] = true;

    fake.axis_feedback_valid[4] = false;
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              102U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          state.captured_point_count == 0U &&
          state.error == TEACH_ERR_DRIVES_NOT_READY,
          "TEACHING rejects missing axis 5 feedback");
    fake.axis_feedback_valid[4] = true;

    fake.poll_ok = false;
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              103U, TEACH_EVENT_NONE, &output) ==
              STATE_STEP_FAILED,
          "TEACHING rejects failed drive polling");
    fake.poll_ok = true;

    reset_fake(&fake);
    runtime.robot_homed = false;
    state_teaching_enter(&state, &config, 14U);
    (void)state_teaching_step(&state, &robot, &runtime, &port, scales,
                             100U, TEACH_EVENT_SELECT_LINE, &output);
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              101U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          state.captured_point_count == 0U &&
          state.error == TEACH_ERR_ROBOT_NOT_HOMED,
          "TEACHING rejects capture until robot is homed");

    printf("\nTEACHING port tests failed: %d\n", failures);
    return failures != 0;
}
