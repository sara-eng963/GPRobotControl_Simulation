#include "../hand_guiding_pipeline.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const double q0[ROBOT_DOF] = {0.3, -1.0, 1.2, -0.5, 0.7, 0.1};

static HandGuidingPipelineConfig fixture_config(void)
{
    HandGuidingPipelineConfig c = {0};
    c.sensor.geometry_verified = true; /* synthetic fixture ONLY */
    c.sensor.flange_T_sensor.rotation_wxyz[0] = 1.0;
    c.sensor.sensor_T_grip.rotation_wxyz[0] = 1.0;
    c.bias.valid = true; /* synthetic known zero; NOT a measured tare */
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        c.admittance.virtual_mass[i] = 2.0;
        c.admittance.damping[i] = 4.0;
        c.admittance.speed_limit[i] = 0.5;
        c.admittance.acceleration_limit[i] = 10.0;
        c.mapper.joint_acceleration_limit[i] = 100.0;
        c.mapper.tracking_error_limit[i] = 0.2;
    }
    c.admittance.maximum_step_s = 0.004;
    c.mapper.model_verified = true; /* test only, no real-arm approval */
    c.mapper.damping_lambda = 0.02;
    c.mapper.minimum_sigma = 1e-10;
    c.mapper.maximum_step_s = 0.004;
    c.require_new_sample = true;
    return c;
}

static HandGuidingPipelineInput fixture_input(const FtSensorSample *sample)
{
    HandGuidingPipelineInput i = {0};
    i.sample = sample;
    i.measured_joint_rad = q0;
    i.now_us = 1100U;
    i.max_sensor_age_us = 3000U;
    i.period_s = 0.002;
    i.gates.teaching_active = true;
    i.gates.motion_permitted = true;
    i.gates.robot_homed = true;
    i.gates.drives_ready = true;
    i.gates.feedback_fresh = true;
    return i;
}

static void cleared(const HandGuidingPipelineState *s,
                    const HandGuidingPipelineOutput *o)
{
    assert(!s->active && s->fault_latched);
    assert(!s->admittance.enabled && !s->mapper.enabled);
    assert(!o->valid && !o->joint_limited);
    for (size_t k = 0U; k < ROBOT_DOF; ++k) {
        assert(o->wrench_base_tcp[k] == 0.0);
        assert(o->twist_base_tcp[k] == 0.0);
        assert(o->joint_position_ref[k] == 0.0);
        assert(o->joint_velocity_ref[k] == 0.0);
    }
}

static void close_to(double a, double b)
{
    assert(isfinite(a) && fabs(a-b) < 1e-10);
}

int main(void)
{
    RobotConfig robot;
    robot_config_init_ur5(&robot);
    HandGuidingPipelineConfig c = fixture_config();
    FtSensorSample sample = {
        .force_N = {0.5, 0.0, 0.0},
        .torque_Nm = {0.0, 0.0, 0.0},
        .received_time_us = 1000U, .valid = true
    };
    HandGuidingPipelineInput i = fixture_input(&sample);
    HandGuidingPipelineState s = {0};
    HandGuidingPipelineOutput out = {0};

    /* Arm validates input and initializes the joint reference; no motion. */
    assert(hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    assert(s.active && !s.fault_latched);
    for (size_t k = 0U; k < ROBOT_DOF; ++k) {
        close_to(s.mapper.position_ref[k], q0[k]);
        close_to(s.admittance.velocity[k], 0.0);
    }

    /* Compare composed pipeline against independent calls to real modules. */
    sample.received_time_us = 3000U;
    i.now_us = 3100U;
    double w_expected[ROBOT_DOF] = {0};
    double v_expected[ROBOT_DOF] = {0};
    AdmittanceState adm = {0};
    CartesianJointMapperState map = {0};
    CartesianJointMapperOutput m_expected = {0};
    assert(wrench_frame_sensor_to_base_tcp(&sample, i.now_us,
        i.max_sensor_age_us, &c.sensor, &c.bias, &robot, q0,
        c.mapper.model_verified, w_expected));
    assert(admittance_enable(&adm, &c.admittance));
    assert(cartesian_joint_mapper_enable(&map, &c.mapper, &robot, q0));
    assert(admittance_step(&c.admittance, &adm, w_expected,
                           i.period_s, v_expected));
    assert(cartesian_joint_mapper_step(&map, &c.mapper, &robot, q0,
                                       v_expected, i.period_s, &m_expected));
    assert(hand_guiding_pipeline_step(&s, &robot, &i, &out));
    assert(out.valid && s.active && !s.fault_latched);
    bool moved = false;
    for (size_t k = 0U; k < ROBOT_DOF; ++k) {
        close_to(out.wrench_base_tcp[k], w_expected[k]);
        close_to(out.twist_base_tcp[k], v_expected[k]);
        close_to(out.joint_position_ref[k], m_expected.position_ref[k]);
        close_to(out.joint_velocity_ref[k], m_expected.velocity_ref[k]);
        if (fabs(out.joint_position_ref[k] - q0[k]) > 1e-12) moved = true;
    }
    assert(moved);

    /* Old sample cannot be reissued as a newly acquired reading. */
    assert(!hand_guiding_pipeline_step(&s, &robot, &i, &out));
    cleared(&s, &out);
    assert(!hand_guiding_pipeline_arm(&s, &c, &robot, &i));

    /* Fault recovery must be explicit; normal stop is not fault reset. */
    hand_guiding_pipeline_reset(&s);
    assert(!s.fault_latched && !s.active);
    assert(hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    hand_guiding_pipeline_stop(&s);
    assert(!s.active && !s.fault_latched && !s.mapper.enabled);
    assert(hand_guiding_pipeline_arm(&s, &c, &robot, &i));

    sample.received_time_us = 5000U;
    i.now_us = 5100U;
    i.gates.feedback_fresh = false;
    assert(!hand_guiding_pipeline_step(&s, &robot, &i, &out));
    cleared(&s, &out);
    i.gates.feedback_fresh = true;
    hand_guiding_pipeline_reset(&s);

    /* Failure at arm must prevent later motion and latch a fault. */
    i.now_us = 9000U; /* F/T is stale */
    assert(!hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    assert(s.fault_latched);
    hand_guiding_pipeline_reset(&s);
    i.now_us = 5100U;
    assert(hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    sample.received_time_us = 7000U;
    i.now_us = 7100U;
    i.gates.motion_permitted = false;
    assert(!hand_guiding_pipeline_step(&s, &robot, &i, &out));
    cleared(&s, &out);
    i.gates.motion_permitted = true;
    hand_guiding_pipeline_reset(&s);

    /* Downstream singularity rejection disables upstream admittance too. */
    c.mapper.minimum_sigma = 1e6;
    assert(hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    sample.received_time_us = 9000U;
    i.now_us = 9100U;
    assert(!hand_guiding_pipeline_step(&s, &robot, &i, &out));
    cleared(&s, &out);
    hand_guiding_pipeline_reset(&s);
    c = fixture_config();

    i.period_s = 0.005; /* synthetic timing fault */
    assert(hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    sample.received_time_us = 11000U;
    i.now_us = 11100U;
    assert(!hand_guiding_pipeline_step(&s, &robot, &i, &out));
    cleared(&s, &out);
    hand_guiding_pipeline_reset(&s);
    i.period_s = 0.002;
    sample.valid = false;
    assert(!hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    assert(s.fault_latched);
    hand_guiding_pipeline_reset(&s);
    sample.valid = true;
    i.gates.teaching_active = false;
    assert(!hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    assert(s.fault_latched);
    hand_guiding_pipeline_reset(&s);
    i.gates.teaching_active = true;

    /* The module freezes its configuration at arming time. */
    assert(hand_guiding_pipeline_arm(&s, &c, &robot, &i));
    c.admittance.virtual_mass[0] = NAN;
    sample.received_time_us = 13000U;
    i.now_us = 13100U;
    assert(hand_guiding_pipeline_step(&s, &robot, &i, &out));
    assert(out.valid);

    puts("PASS: Stage 4E1 composed pipeline, lifecycle, freshness and fault latch");
    return 0;
}
