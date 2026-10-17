#include "hand_guiding_pipeline.h"

#include <stddef.h>
#include <string.h>

/* Only composition/lifecycle resides here. Reuse all numerical modules. */
static void clear_controllers(HandGuidingPipelineState *state)
{
    if (state == NULL) return;
    state->active = false;
    state->have_last_sample = false;
    admittance_disable(&state->admittance);
    cartesian_joint_mapper_disable(&state->mapper);
}

static bool trip(HandGuidingPipelineState *state)
{
    if (state != NULL) {
        clear_controllers(state);
        state->fault_latched = true;
    }
    return false;
}

void hand_guiding_pipeline_reset(HandGuidingPipelineState *state)
{
    if (state != NULL) memset(state, 0, sizeof(*state));
}

void hand_guiding_pipeline_stop(HandGuidingPipelineState *state)
{
    clear_controllers(state);
}

static bool guards_valid(const HandGuidingPipelineInput *input)
{
    return input != NULL && input->gates.teaching_active &&
           input->gates.motion_permitted && input->gates.robot_homed &&
           input->gates.drives_ready && input->gates.feedback_fresh &&
           input->sample != NULL && input->measured_joint_rad != NULL;
}

static bool config_valid(const HandGuidingPipelineConfig *config,
                         const RobotConfig *robot)
{
    if (config == NULL || !sensor_config_validate(&config->sensor) ||
        !config->bias.valid ||
        !admittance_params_valid(&config->admittance) ||
        !cartesian_joint_mapper_config_valid(&config->mapper, robot))
        return false;
    /* The sensor bias finiteness check is performed by wrench processing. */
    return true;
}

bool hand_guiding_pipeline_arm(
    HandGuidingPipelineState *state,
    const HandGuidingPipelineConfig *config,
    const RobotConfig *robot,
    const HandGuidingPipelineInput *input)
{
    if (state == NULL || state->fault_latched) return false;
    if (state->active || !guards_valid(input) ||
        !config_valid(config, robot)) return trip(state);

    /* Require a real valid sensor sample on entry, even though arm itself
     * will not produce a joint reference or move the robot. */
    double probe_wrench[ADMITTANCE_AXES];
    if (!wrench_frame_sensor_to_base_tcp(
            input->sample, input->now_us, input->max_sensor_age_us,
            &config->sensor, &config->bias, robot,
            input->measured_joint_rad, config->mapper.model_verified,
            probe_wrench)) return trip(state);

    state->config = *config;
    if (!admittance_enable(&state->admittance, &state->config.admittance) ||
        !cartesian_joint_mapper_enable(&state->mapper, &state->config.mapper,
            robot, input->measured_joint_rad)) return trip(state);

    state->last_sample_us = input->sample->received_time_us;
    state->have_last_sample = true;
    state->active = true;
    return true;
}

bool hand_guiding_pipeline_step(
    HandGuidingPipelineState *state,
    const RobotConfig *robot,
    const HandGuidingPipelineInput *input,
    HandGuidingPipelineOutput *output)
{
    if (output != NULL) memset(output, 0, sizeof(*output));
    if (state == NULL || !state->active || state->fault_latched ||
        output == NULL || !guards_valid(input)) return trip(state);

    const HandGuidingPipelineConfig *config = &state->config;
    if (!config_valid(config, robot)) return trip(state);

    if (config->require_new_sample && state->have_last_sample) {
        /* Strictly newer modulo-2^32 timestamps over a short interval. */
        const uint32_t elapsed =
            (uint32_t)(input->sample->received_time_us - state->last_sample_us);
        if (elapsed == 0U || elapsed >= UINT32_C(0x80000000))
            return trip(state);
    }

    double wrench[ADMITTANCE_AXES];
    double twist[ADMITTANCE_AXES];
    CartesianJointMapperOutput mapped = {0};

    if (!wrench_frame_sensor_to_base_tcp(
            input->sample, input->now_us, input->max_sensor_age_us,
            &config->sensor, &config->bias, robot,
            input->measured_joint_rad, config->mapper.model_verified,
            wrench)) return trip(state);

    if (!admittance_step(&config->admittance, &state->admittance,
                         wrench, input->period_s, twist)) return trip(state);

    if (!cartesian_joint_mapper_step(&state->mapper, &config->mapper, robot,
            input->measured_joint_rad, twist, input->period_s, &mapped) ||
        !mapped.valid) return trip(state);

    /* Publish only after all components succeeded. */
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        output->wrench_base_tcp[i] = wrench[i];
        output->twist_base_tcp[i] = twist[i];
        output->joint_position_ref[i] = mapped.position_ref[i];
        output->joint_velocity_ref[i] = mapped.velocity_ref[i];
    }
    output->sigma_min = mapped.sigma_min;
    output->joint_limited = mapped.limited;
    output->valid = true;
    state->last_sample_us = input->sample->received_time_us;
    state->have_last_sample = true;
    return true;
}
