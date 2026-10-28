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
        .default_speed_mps = TEACHING_SPEED_PRESET_MEDIUM_MPS,
        .minimum_speed_mps = 0.001F,
        .maximum_speed_mps = 0.1F,
        .speed_low_mps = TEACHING_SPEED_PRESET_LOW_MPS,
        .speed_medium_mps = TEACHING_SPEED_PRESET_MEDIUM_MPS,
        .speed_high_mps = TEACHING_SPEED_PRESET_HIGH_MPS,
        .duplicate_position_tolerance_m = 0.002F,
        .duplicate_orientation_tolerance_rad = 0.010F,
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


    const uint8_t first_count = state.captured_point_count;
    const TeachingPhase first_phase = state.phase;
    const TaughtProgram first_draft = state.draft;
    const TaughtPoint first_point = state.working_segment.points[0];
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              102U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          state.last_event_accepted &&
          state.error == TEACH_ERR_NONE &&
          state.status_message_id == TEACH_MSG_POINT_ALREADY_RECORDED &&
          state.captured_point_count == first_count &&
          state.phase == first_phase &&
          memcmp(&state.draft, &first_draft, sizeof(first_draft)) == 0 &&
          memcmp(&state.working_segment.points[0], &first_point,
                 sizeof(first_point)) == 0,
          "TEACHING treats repeated identical point as a harmless no-op");

    state.working_segment.points[0] = first_point;
    for (size_t i = 0; i < 4U; ++i)
        state.working_segment.points[0].orientation_quat[i] =
            -state.working_segment.points[0].orientation_quat[i];
    const TaughtPoint equivalent_point = state.working_segment.points[0];
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              103U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          state.last_event_accepted &&
          state.error == TEACH_ERR_NONE &&
          state.status_message_id == TEACH_MSG_POINT_ALREADY_RECORDED &&
          state.captured_point_count == first_count &&
          state.phase == first_phase &&
          memcmp(&state.working_segment.points[0], &equivalent_point,
                 sizeof(equivalent_point)) == 0,
          "TEACHING treats q and -q duplicate orientations as equivalent");

    state.working_segment.points[0] = first_point;
    state.working_segment.points[0].orientation_quat[0] =
        -first_point.orientation_quat[1];
    state.working_segment.points[0].orientation_quat[1] =
        first_point.orientation_quat[0];
    state.working_segment.points[0].orientation_quat[2] =
        -first_point.orientation_quat[3];
    state.working_segment.points[0].orientation_quat[3] =
        first_point.orientation_quat[2];
    const TaughtPoint orientation_only_point = state.working_segment.points[0];
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              104U, TEACH_EVENT_RECORD_POINT, &output) ==
              STATE_STEP_RUNNING &&
          !state.last_event_accepted &&
          state.error == TEACH_ERR_ORIENTATION_ONLY_POINT &&
          state.status_message_id == TEACH_MSG_MOVE_TO_DISTINCT_POINT &&
          state.captured_point_count == first_count &&
          state.phase == first_phase &&
          memcmp(&state.working_segment.points[0], &orientation_only_point,
                 sizeof(orientation_only_point)) == 0,
          "TEACHING rejects orientation-only duplicate without advancing stage");

    state_teaching_enter(&state, &config, 15U);
    (void)state_teaching_step(&state, &robot, &runtime, &port, scales,
                             100U, TEACH_EVENT_SELECT_LINE, &output);
    CHECK(fabsf(state.teaching_speed_mps - config.speed_medium_mps) < 1.0e-6F &&
          fabsf(state.working_segment.speed_mps - config.speed_medium_mps) < 1.0e-6F,
          "TEACHING starts on the Medium speed preset");
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              101U, TEACH_EVENT_SPEED_HIGH, &output) ==
              STATE_STEP_RUNNING &&
          state.last_event_accepted &&
          fabsf(state.teaching_speed_mps - config.speed_high_mps) < 1.0e-6F &&
          fabsf(state.working_segment.speed_mps - config.speed_high_mps) < 1.0e-6F &&
          fabsf(state.draft.global_speed_scale - 1.0F) < 1.0e-6F,
          "TEACHING applies the High preset to the active segment");

    state.draft.segment_count = 1U;
    state.draft.segments[0].speed_mps = config.speed_high_mps;
    state.draft.global_speed_scale = 0.5F;
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              102U, TEACH_EVENT_SPEED_LOW, &output) ==
              STATE_STEP_RUNNING &&
          state.last_event_accepted &&
          fabsf(state.draft.segments[0].speed_mps - config.speed_low_mps) < 1.0e-6F &&
          fabsf(state.working_segment.speed_mps - config.speed_low_mps) < 1.0e-6F &&
          fabsf(state.draft.global_speed_scale - 1.0F) < 1.0e-6F,
          "TEACHING applies the Low preset to completed and active segments");
    CHECK(state_teaching_step(&state, &robot, &runtime, &port, scales,
                              103U, TEACH_EVENT_SELECT_ARC, &output) ==
              STATE_STEP_RUNNING &&
          state.last_event_accepted &&
          fabsf(state.working_segment.speed_mps - config.speed_low_mps) < 1.0e-6F,
          "TEACHING carries the selected preset into future segments");

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
