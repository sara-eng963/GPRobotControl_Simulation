/* Host-only source integration: mock RS-485 -> FtSensorSample -> existing
 * Tustin admittance + Jacobian mapping. Absolutely no CAN/drive send. */
#include "../ft_serial_transport.h"
#include "../hand_guiding_pipeline.h"
#include "p05_mock_ft_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const double measured_rad[ROBOT_DOF] = {0.3, -1.0, 1.2, -0.5, 0.7, 0.1};

static HandGuidingPipelineConfig fixture(void)
{
    HandGuidingPipelineConfig c = {0};
    c.sensor.geometry_verified = true; /* synthetic, never a real approval */
    c.sensor.flange_T_sensor.rotation_wxyz[0] = 1.0;
    c.sensor.sensor_T_grip.rotation_wxyz[0] = 1.0;
    c.bias.valid = true; /* synthetic unloaded bias */
    c.mapper.model_verified = true; /* synthetic robot model */
    c.mapper.damping_lambda = 0.02;
    c.mapper.minimum_sigma = 1e-10;
    c.mapper.maximum_step_s = 0.004;
    c.admittance.maximum_step_s = 0.004;
    c.require_new_sample = true;
    for (size_t i = 0; i < ROBOT_DOF; ++i) {
        c.admittance.virtual_mass[i] = 2.0;
        c.admittance.damping[i] = 4.0;
        c.admittance.speed_limit[i] = 0.5;
        c.admittance.acceleration_limit[i] = 10.0;
        c.mapper.joint_acceleration_limit[i] = 100.0;
        c.mapper.tracking_error_limit[i] = 0.2;
    }
    return c;
}
static HandGuidingPipelineInput input(const FtSensorSample *sample, uint32_t t)
{
    HandGuidingPipelineInput in = {0};
    in.sample = sample;
    in.now_us = t;
    in.max_sensor_age_us = 3000U;
    in.measured_joint_rad = measured_rad;
    in.period_s = 0.002;
    in.gates.teaching_active = true;
    in.gates.motion_permitted = true; /* synthetic claim, no safety evidence */
    in.gates.robot_homed = true;
    in.gates.drives_ready = true;
    in.gates.feedback_fresh = true;
    return in;
}
static bool deliver(FtSerialPort *p, uint8_t *packet_bytes,
                    uint32_t seq, int32_t fmN, uint32_t t,
                    FtSensorSample *out)
{
    packet(packet_bytes, seq, fmN);
    assert(ft_serial_isr_receive(p, packet_bytes, PACKET_LEN, t));
    assert(ft_serial_poll(p, t + 100U, 128U) == PACKET_LEN);
    return ft_serial_latest(p, t + 100U, out);
}
int main(void)
{
    RobotConfig robot;
    robot_config_init_ur5(&robot); /* repository mathematical fixture */
    HandGuidingPipelineConfig cfg = fixture();
    HandGuidingPipelineState state = {0};
    HandGuidingPipelineOutput out = {0};
    FtSerialPort port;
    FtSerialProtocol decoder = mock_protocol();
    FtSensorSample sample = {0};
    FtSerialDryRunSampler sampler;
    uint8_t bytes[PACKET_LEN];
    assert(ft_serial_init(&port, &decoder, 5000U, 1500U, 3000U));
    ft_serial_sampler_init(&sampler, &port);
    assert(!ft_serial_sampler_read(&sampler, &sample));

    assert(deliver(&port, bytes, 1U, 500, 1000U, &sample));
    (void)ft_serial_sampler_update(&sampler, 1100U, 128U);
    assert(ft_serial_sampler_read(&sampler, &sample));
    HandGuidingPipelineInput in = input(&sample, 1100U);
    assert(hand_guiding_pipeline_arm(&state, &cfg, &robot, &in));
    assert(state.active && !state.fault_latched);
    puts("P05-GUIDANCE-01 synthetic serial sample arms reference-only pipeline");

    assert(deliver(&port, bytes, 2U, 500, 3000U, &sample));
    (void)ft_serial_sampler_update(&sampler, 3100U, 128U);
    assert(ft_serial_sampler_read(&sampler, &sample));
    in = input(&sample, 3100U);
    assert(hand_guiding_pipeline_step(&state, &robot, &in, &out));
    assert(out.valid);
    bool nonzero = false;
    for (size_t axis = 0; axis < ROBOT_DOF; ++axis) {
        assert(isfinite(out.joint_position_ref[axis]));
        assert(isfinite(out.joint_velocity_ref[axis]));
        nonzero |= fabs(out.joint_position_ref[axis] - measured_rad[axis]) > 1e-12;
    }
    assert(nonzero);
    puts("P05-GUIDANCE-02 existing control pipeline computes six joint references");

    /* A reused sequence cannot authorize the next teach sample. */
    assert(!deliver(&port, bytes, 2U, 500, 5000U, &sample));
    (void)ft_serial_sampler_update(&sampler, 5100U, 128U);
    assert(!ft_serial_sampler_read(&sampler, &sample));
    in = input(NULL, 5100U);
    assert(!hand_guiding_pipeline_step(&state, &robot, &in, &out));
    assert(state.fault_latched && !out.valid);
    assert(deliver(&port, bytes, 3U, 500, 6000U, &sample));
    in = input(&sample, 6100U);
    assert(!hand_guiding_pipeline_arm(&state, &cfg, &robot, &in));
    assert(!hand_guiding_pipeline_step(&state, &robot, &in, &out));
    puts("P05-GUIDANCE-03 replay-induced pipeline fault does not auto-reauthorize");

    hand_guiding_pipeline_reset(&state); /* only a host fixture reset */
    assert(hand_guiding_pipeline_arm(&state, &cfg, &robot, &in));
    assert(deliver(&port, bytes, 4U, 500, 8000U, &sample));
    in = input(&sample, 8100U);
    assert(hand_guiding_pipeline_step(&state, &robot, &in, &out));
    (void)ft_serial_poll(&port, 12000U, 128U);
    assert(!ft_serial_latest(&port, 12000U, &sample));
    in = input(NULL, 12000U);
    assert(!hand_guiding_pipeline_step(&state, &robot, &in, &out));
    assert(state.fault_latched);
    puts("P05-GUIDANCE-04 stale serial measurement revokes reference validity");
    puts("PASS: four P05 virtual F/T-to-control scenario groups (no motion I/O)");
    return 0;
}
