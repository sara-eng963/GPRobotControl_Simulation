/*
 * Host-only TEACHING regression: fake JointDrivePort, no CANopen or SIL Kit.
 * Verifies measured position conversion and recording eligibility.
 * Does not test hardware safety or manual-guidance control.
 */
#include "../States/state_teaching.h"
#include "fake_joint_drive.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(condition, message) do { \
    if (condition) printf("[PASS] %s\n", message); \
    else { printf("[FAIL] %s\n", message); ++failures; } \
} while (0)

static void reset_fake(FakeDrive *f)
{
    fake_joint_drive_reset(f);
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i)
        f->axis[i].actual_position_units = (int32_t)(100 + 10 * i);
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
    JointDrivePort port = fake_joint_drive_make_port(&fake);
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
              fake.axis[0].actual_position_units, &expected_q) &&
          fabs((double)state.working_segment.points[0].joint_position_rad[0] -
               expected_q) < 1.0e-6,
          "Recorded joint position matches measured raw position conversion");
    CHECK(fake.commands == 0U,
          "TEACHING never sends motion commands itself");

    state_teaching_enter(&state, &config, 13U);
    (void)state_teaching_step(&state, &robot, &runtime, &port, scales,
                             100U, TEACH_EVENT_SELECT_LINE, &output);
    fake.axis[2].operation_enabled = false;
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              101U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          state.captured_point_count == 0U &&
          state.error == TEACH_ERR_DRIVES_NOT_READY,
          "TEACHING rejects recording with axis 3 disabled");
    fake.axis[2].operation_enabled = true;

    fake.axis[4].feedback_valid = false;
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              102U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          state.captured_point_count == 0U &&
          state.error == TEACH_ERR_DRIVES_NOT_READY,
          "TEACHING rejects missing axis 5 feedback");
    fake.axis[4].feedback_valid = true;

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
