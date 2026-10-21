#include "../supervisor_dry_run_adapter.h"
#include "../../StateMachine/Tests/fake_joint_drive.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool sample_input(void *context, FtSensorSample *sample)
{
    *sample = *(FtSensorSample *)context;
    return true;
}

static HandGuidingDryRunConfig test_config(FtSensorSample *sample)
{
    HandGuidingDryRunConfig cfg = {0};
    cfg.max_sensor_age_us = 1500U;
    cfg.max_feedback_age_us = 1500U;
    cfg.max_runtime_age_ms = 2U;
    cfg.max_safety_age_ms = 2U;
    cfg.require_new_feedback = true;
    cfg.sample_context = sample;
    cfg.read_sample = sample_input;
    cfg.pipeline_config.sensor.geometry_verified = true;
    cfg.pipeline_config.sensor.flange_T_sensor.rotation_wxyz[0] = 1.0;
    cfg.pipeline_config.sensor.sensor_T_grip.rotation_wxyz[0] = 1.0;
    cfg.pipeline_config.bias.valid = true;
    cfg.pipeline_config.mapper.model_verified = true; /* synthetic only */
    cfg.pipeline_config.mapper.damping_lambda = 0.02;
    cfg.pipeline_config.mapper.minimum_sigma = 1e-10;
    cfg.pipeline_config.mapper.maximum_step_s = 0.004;
    cfg.pipeline_config.admittance.maximum_step_s = 0.004;
    cfg.pipeline_config.require_new_sample = true;
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis) {
        cfg.pipeline_config.admittance.virtual_mass[axis] = 2.0;
        cfg.pipeline_config.admittance.damping[axis] = 4.0;
        cfg.pipeline_config.admittance.speed_limit[axis] = 0.5;
        cfg.pipeline_config.admittance.acceleration_limit[axis] = 10.0;
        cfg.pipeline_config.mapper.joint_acceleration_limit[axis] = 100.0;
        cfg.pipeline_config.mapper.tracking_error_limit[axis] = 0.2;
    }
    return cfg;
}

int main(void)
{
    RobotConfig robot;
    robot_config_init_ur5(&robot); /* host mathematical fixture, not hardware */
    const double q[ROBOT_DOF] = {0.3, -1.0, 1.2, -0.5, 0.7, 0.1};
    AvatarMPositionScale scales[ROBOT_DOF];
    FakeDrive fake;
    fake_joint_drive_reset(&fake);
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis) {
        assert(avatar_m_position_scale_default(&scales[axis], 50.0));
        assert(avatar_m_joint_rad_to_position_units(
            &scales[axis], q[axis], &fake.axis[axis].actual_position_units));
        fake.axis[axis].timestamp_us = 1000U;
    }
    JointDrivePort drive = fake_joint_drive_make_port(&fake);
    FtSensorSample sample = {
        .force_N = {0.5, 0.0, 0.0},
        .received_time_us = 1000U, .valid = true
    };
    HandGuidingDryRunConfig config = test_config(&sample);
    HandGuidingDryRunState state = {0};
    assert(hand_guiding_dry_run_init(&state, &config));
    SupervisorGuidanceDryRunPort port = hand_guiding_dry_run_port(&state);
    assert(port.tick != NULL && port.stop != NULL);
    SupervisorGuidanceDryRunCycle cycle = {0};
    cycle.feedback_healthy = true;
    cycle.scales = scales;
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis)
        assert(joint_drive_port_read_axis(&drive, axis, &cycle.feedback[axis]));
    cycle.robot = &robot;
    cycle.now_us = 1100U;
    cycle.now_ms = 1U;
    cycle.runtime_timestamp_ms = 1U;
    cycle.safety_timestamp_ms = 1U;
    cycle.period_s = 0.002;
    cycle.teaching_active = true;
    cycle.manual_guidance_requested = true;
    cycle.motion_permitted = true;
    cycle.robot_homed = true;
    cycle.drives_ready = true;
    cycle.communication_healthy = true;
    cycle.safety_valid = true;

    assert(port.tick(port.context, &cycle));
    assert(state.pipeline.active && !state.last_output.valid);
    assert(fake.commands == 0U);

    sample.received_time_us = 3000U;
    cycle.now_us = 3100U;
    cycle.now_ms = 3U;
    cycle.runtime_timestamp_ms = 3U;
    cycle.safety_timestamp_ms = 3U;
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis) {
        fake.axis[axis].timestamp_us = 3000U;
        ++fake.sequence[axis];
        cycle.feedback[axis] = fake.axis[axis];
        cycle.feedback[axis].sequence = fake.sequence[axis];
    }
    assert(port.tick(port.context, &cycle));
    assert(state.last_output.valid && state.pipeline.active);
    assert(fake.commands == 0U); /* NO CAN/drive write */

    /* Replay a just-used feedback sequence -> latch, even with new FT data. */
    sample.received_time_us = 5000U;
    cycle.now_us = 5100U;
    cycle.now_ms = 5U;
    cycle.runtime_timestamp_ms = 5U;
    cycle.safety_timestamp_ms = 5U;
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis)
        cycle.feedback[axis].timestamp_us = 5000U;
    assert(!port.tick(port.context, &cycle));
    assert(state.pipeline.fault_latched && !state.last_output.valid);
    assert(fake.commands == 0U);

    /* No implicit latch clear; only external explicit reset authorizes it. */
    assert(!port.tick(port.context, &cycle));
    hand_guiding_dry_run_reset(&state);
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis)
        ++cycle.feedback[axis].sequence;
    assert(port.tick(port.context, &cycle));
    port.stop(port.context);
    assert(!state.pipeline.active && !state.last_output.valid);
    assert(!state.pipeline.fault_latched);

    /* Unsafe supervisory conditions are reported, never turned into motion. */
    cycle.estop_active = true;
    assert(!port.tick(port.context, &cycle));
    assert(state.pipeline.fault_latched && fake.commands == 0U);
    hand_guiding_dry_run_reset(&state);
    cycle.estop_active = false;
    cycle.safety_timestamp_ms = 0U; /* stale safety update */
    assert(!port.tick(port.context, &cycle));
    assert(state.pipeline.fault_latched && fake.commands == 0U);
    /* Feedback can be individually fresh while describing significantly
     * different instants. The optional skew guard is inclusive at limit. */
    cycle.safety_timestamp_ms = cycle.now_ms;
    HandGuidingDryRunConfig skew_cfg = test_config(&sample);
    skew_cfg.max_feedback_skew_us = 500U;
    skew_cfg.require_new_feedback = false;
    skew_cfg.pipeline_config.require_new_sample = false;
    HandGuidingDryRunState skew_state = {0};
    assert(hand_guiding_dry_run_init(&skew_state, &skew_cfg));
    SupervisorGuidanceDryRunPort skew_port =
        hand_guiding_dry_run_port(&skew_state);
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis)
        cycle.feedback[axis].timestamp_us = 5000U;
    cycle.feedback[4].timestamp_us = 4500U; /* 500us skew: allowed */
    assert(skew_port.tick(skew_port.context, &cycle));
    assert(skew_state.pipeline.active && !skew_state.pipeline.fault_latched);

    cycle.feedback[4].timestamp_us = 4499U; /* 501us: reject and latch */
    assert(!skew_port.tick(skew_port.context, &cycle));
    assert(skew_state.pipeline.fault_latched && !skew_state.last_output.valid);
    assert(fake.commands == 0U);
    hand_guiding_dry_run_reset(&skew_state);
    assert(!skew_port.tick(skew_port.context, &cycle)); /* latch persists */
    /* Explicitly disabled guard retains legacy individual-age behavior. */
    skew_cfg.max_feedback_skew_us = 0U;
    assert(hand_guiding_dry_run_init(&skew_state, &skew_cfg));
    assert(skew_port.tick(skew_port.context, &cycle));
    assert(fake.commands == 0U);

    puts("PASS: 4E2A dry-run feedback/FT gate and zero motion dispatch");
    return 0;
}
