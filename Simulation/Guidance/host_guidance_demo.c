#include "host_guidance_demo.h"
#include "virtual_ft_sensor.h"

#include "../../ControlCore/Kinematics/control_jacobian.h"
#include "../../ControlCore/Analysis/singularity.h"

#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

const char *gp_host_guidance_failure_name(GpHostGuidanceFailure reason)
{
    switch (reason) {
    case GP_GUIDANCE_FAIL_NONE: return "none";
    case GP_GUIDANCE_FAIL_PRE_SAMPLE: return "pre_sample_safety_or_feedback_gate";
    case GP_GUIDANCE_FAIL_INPUT: return "virtual_ft_input_unavailable";
    case GP_GUIDANCE_FAIL_SENSOR_FRAME: return "virtual_ft_sensor_frame_conversion";
    case GP_GUIDANCE_FAIL_FEEDBACK_CONVERSION: return "virtual_ft_feedback_conversion";
    case GP_GUIDANCE_FAIL_PIPELINE_AFTER_SAMPLE: return "pipeline_failure_unclassified";
    case GP_GUIDANCE_FAIL_WRENCH_FRAME: return "sensor_to_base_tcp_wrench_rejected";
    case GP_GUIDANCE_FAIL_ADMITTANCE: return "admittance_step_rejected";
    case GP_GUIDANCE_FAIL_MAPPER_SINGULARITY: return "mapper_singularity_threshold";
    case GP_GUIDANCE_FAIL_MAPPER_TRACKING: return "mapper_tracking_error_limit";
    case GP_GUIDANCE_FAIL_MAPPER_JOINT_LIMIT: return "mapper_joint_position_limit";
    case GP_GUIDANCE_FAIL_MAPPER_OTHER: return "mapper_dls_or_other_constraint";
    default: return "unclassified";
    }
}

static bool synthetic_ft(void *context, FtSensorSample *out)
{
    GpHostGuidanceDemo *d = (GpHostGuidanceDemo *)context;
    if (d == NULL || out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (d->virtual_ft_receiver != NULL) {
        double requested[6], measured[ROBOT_DOF];
        const SupervisorGuidanceDryRunCycle *cycle = d->sample_cycle;
        const uint32_t now_us = (uint32_t)((uint64_t)xTaskGetTickCount() *
            UINT64_C(1000000) / (uint64_t)configTICK_RATE_HZ);
        if (cycle == NULL || cycle->scales == NULL) {
            d->last_failure = GP_GUIDANCE_FAIL_PRE_SAMPLE;
            return false;
        }
        if (!gp_virtual_ft_receiver_read(d->virtual_ft_receiver,
                                         now_us, requested)) {
            d->last_failure = GP_GUIDANCE_FAIL_INPUT;
            return false;
        }
        for (size_t i=0; i<ROBOT_DOF; ++i)
            if (!cycle->feedback[i].feedback_valid ||
                !avatar_m_position_units_to_joint_rad(&cycle->scales[i],
                    cycle->feedback[i].actual_position_units, &measured[i]))
            {
                d->last_failure = GP_GUIDANCE_FAIL_FEEDBACK_CONVERSION;
                return false;
            }
        if (!gp_virtual_ft_sensor_sample(requested,
                &d->adapter.config.pipeline_config.sensor,
                &d->synthetic_observation_robot, measured, now_us, out)) {
            d->last_failure = GP_GUIDANCE_FAIL_SENSOR_FRAME;
            return false;
        }
        if (d->inject_ft_active)
            out->received_time_us -= 40000U;
        d->last_failure = GP_GUIDANCE_FAIL_NONE; /* sample accepted */
        d->diagnostic_sample = *out;
        memcpy(d->diagnostic_measured_q, measured, sizeof(measured));
        d->diagnostic_sample_ready = true;
        return true;
    }
    out->valid = true;
    const uint64_t now = (uint64_t)xTaskGetTickCount() *
        UINT64_C(1000000) / (uint64_t)configTICK_RATE_HZ;
    out->received_time_us = (uint32_t)(
        d->inject_ft_active ? now - UINT64_C(40000) : now);
    out->force_N[0] = 0.5; /* fake handle load, N */
    d->last_failure = GP_GUIDANCE_FAIL_NONE;
    return true;
}

/* Host-only diagnostic replay is invoked only AFTER a real rejected tick.
 * It uses COPIES of the pre-tick states and the original input sample.
 * It cannot clear a fault, change the live state or dispatch a target.
 * Reuse the same numerical functions; never duplicate control equations. */
static GpHostGuidanceFailure classify_failed_pipeline(
    GpHostGuidanceDemo *d, const HandGuidingPipelineState *before,
    const SupervisorGuidanceDryRunCycle *cycle)
{
    if (!d->diagnostic_sample_ready || !before->active)
        return GP_GUIDANCE_FAIL_PIPELINE_AFTER_SAMPLE;

    const HandGuidingPipelineConfig *cfg = &before->config;
    const RobotConfig *robot = &d->synthetic_observation_robot;
    const double *q = d->diagnostic_measured_q;
    double wrench[ADMITTANCE_AXES] = {0};
    if (!wrench_frame_sensor_to_base_tcp(
            &d->diagnostic_sample, (uint32_t)cycle->now_us,
            d->adapter.config.max_sensor_age_us, &cfg->sensor, &cfg->bias,
            robot, q, cfg->mapper.model_verified, wrench))
        return GP_GUIDANCE_FAIL_WRENCH_FRAME;

    AdmittanceState admittance_copy = before->admittance;
    double twist[ADMITTANCE_AXES] = {0};
    if (!admittance_step(&cfg->admittance, &admittance_copy,
                        wrench, cycle->period_s, twist))
        return GP_GUIDANCE_FAIL_ADMITTANCE;

    CartesianJointMapperState mapper_copy = before->mapper;
    CartesianJointMapperOutput output = {0};
    if (cartesian_joint_mapper_step(
            &mapper_copy, &cfg->mapper, robot, q, twist, cycle->period_s,
            &output) && output.valid)
        return GP_GUIDANCE_FAIL_PIPELINE_AFTER_SAMPLE;

    /* Discriminate common mapper rejections without weakening any gate. */
    double max_track = 0.0;
    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis) {
        const double distance = fabs(before->mapper.position_ref[axis] - q[axis]);
        if (distance > max_track) max_track = distance;
        if (q[axis] < robot->limits.qMin[axis] ||
            q[axis] > robot->limits.qMax[axis] ||
            before->mapper.position_ref[axis] < robot->limits.qMin[axis] ||
            before->mapper.position_ref[axis] > robot->limits.qMax[axis])
            return GP_GUIDANCE_FAIL_MAPPER_JOINT_LIMIT;
    }
    d->diagnostic_max_tracking_error_rad = max_track;
    if (max_track > cfg->mapper.tracking_error_limit[0])
        return GP_GUIDANCE_FAIL_MAPPER_TRACKING;

    double J[ROBOT_DOF][ROBOT_DOF];
    double sigma = 0.0;
    control_jacobian(robot, q, J);
    if (!singularity_sigma_min((const double (*)[ROBOT_DOF])J, &sigma) ||
        !isfinite(sigma) || sigma < cfg->mapper.minimum_sigma) {
        d->diagnostic_sigma_min = sigma;
        return GP_GUIDANCE_FAIL_MAPPER_SINGULARITY;
    }
    d->diagnostic_sigma_min = sigma;
    return GP_GUIDANCE_FAIL_MAPPER_OTHER;
}

static void record(GpHostGuidanceDemo *d, const GpGuidanceTraceRow *r)
{
    /* Short bounded copy under the same FreeRTOS POSIX scheduler. */
    taskENTER_CRITICAL();
    unsigned next = (d->head + 1U) % GP_GUIDANCE_RING_SIZE;
    if (next != d->tail) {
        d->rows[d->head] = *r;
        d->head = next;
    } else if (d->dropped < UINT32_MAX) {
        ++d->dropped;
    }
    taskEXIT_CRITICAL();
}

static bool tick(void *context, const SupervisorGuidanceDryRunCycle *cycle)
{
    GpHostGuidanceDemo *d = (GpHostGuidanceDemo *)context;
    if (d == NULL || cycle == NULL) return false;
    SupervisorGuidanceDryRunCycle observation = *cycle;
    observation.robot = &d->synthetic_observation_robot;
    if (d->cycles < UINT32_MAX) ++d->cycles;
    const bool inject = d->inject != GP_GUIDANCE_INJECT_NONE &&
        d->cycles >= d->inject_after;
    d->inject_ft_active = inject &&
        d->inject == GP_GUIDANCE_INJECT_STALE_FT;
    if (inject && d->inject == GP_GUIDANCE_INJECT_BAD_FEEDBACK)
        observation.feedback_healthy = false;

    d->candidate_pending = false;
    memset(&d->candidate, 0, sizeof(d->candidate));
    d->last_failure = GP_GUIDANCE_FAIL_PRE_SAMPLE;
    d->diagnostic_sample_ready = false;
    const HandGuidingPipelineState before = d->adapter.pipeline;
    d->sample_cycle = &observation;
    const bool accepted = hand_guiding_dry_run_tick(&d->adapter, &observation);
    d->sample_cycle = NULL;
    if (accepted) {
        d->last_failure = GP_GUIDANCE_FAIL_NONE;
    } else if (d->last_failure == GP_GUIDANCE_FAIL_NONE) {
        d->last_failure = classify_failed_pipeline(d, &before, &observation);
    }
    if (accepted && d->adapter.last_output.valid) {
        d->candidate.valid = true;
        d->candidate.produced_us = cycle->now_us;
        for (size_t i = 0U; i < ROBOT_DOF; ++i) {
            d->candidate.position_rad[i] =
                d->adapter.last_output.joint_position_ref[i];
            d->candidate.velocity_rad_s[i] =
                d->adapter.last_output.joint_velocity_ref[i];
        }
        d->candidate_pending = true;
    }
    GpGuidanceTraceRow r = {0};
    r.time_us = cycle->now_us;
    r.cycle = d->cycles;
    r.accepted = accepted;
    r.valid = d->adapter.last_output.valid;
    r.fault = d->adapter.pipeline.fault_latched;
    r.feedback_measurement_valid = true;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        const JointDriveAxisFeedback *feedback = &cycle->feedback[i];
        uint64_t age_us = 0;
        if (feedback->timestamp_us <= cycle->now_us)
            age_us = cycle->now_us - feedback->timestamp_us;
        else
            r.feedback_measurement_valid = false;
        if (age_us > r.feedback_max_age_us)
            r.feedback_max_age_us = age_us;
        if (!feedback->feedback_valid ||
            !avatar_m_position_units_to_joint_rad(
                &cycle->scales[i], feedback->actual_position_units,
                &r.q_actual[i]))
            r.feedback_measurement_valid = false;
    }
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        r.wrench[i] = d->adapter.last_output.wrench_base_tcp[i];
        r.twist[i] = d->adapter.last_output.twist_base_tcp[i];
        r.qref[i] = d->adapter.last_output.joint_position_ref[i];
    }
    record(d, &r);
    return accepted;
}
static void stop(void *context)
{
    GpHostGuidanceDemo *d = (GpHostGuidanceDemo *)context;
    if (d != NULL) {
        d->candidate_pending = false;
        memset(&d->candidate, 0, sizeof(d->candidate));
        hand_guiding_dry_run_stop(&d->adapter);
    }
}

static bool take_candidate(void *context, SupervisorGuidanceCandidate *out)
{
    GpHostGuidanceDemo *d = (GpHostGuidanceDemo *)context;
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (d == NULL || !d->candidate_pending) return false;
    *out = d->candidate;
    d->candidate_pending = false;
    memset(&d->candidate, 0, sizeof(d->candidate));
    return true;
}

bool gp_host_guidance_init(GpHostGuidanceDemo *d,
                           const RobotConfig *robot,
                           GpGuidanceFaultKind kind, unsigned after)
{
    if (d == NULL || robot == NULL ||
        (kind != GP_GUIDANCE_INJECT_NONE && after < 5U)) return false;
    memset(d, 0, sizeof(*d));
    d->synthetic_observation_robot = *robot;
    /* Only this observation fixture is perturbed: actual SIL Kit motors,
     * FK and HOMING continue using the UNMODIFIED reference model.
     * The reference simulated home has a singular wrist (q5=0). */
    d->synthetic_observation_robot.kinematics.thetaOffset[4] += 0.15;
    d->inject = kind;
    d->inject_after = after;
    HandGuidingDryRunConfig cfg = {0};
    cfg.read_sample = synthetic_ft;
    cfg.sample_context = d;
    cfg.max_sensor_age_us = 30000U;
    cfg.max_feedback_age_us = 100000U; /* generous POSIX-host fixture */
    cfg.max_runtime_age_ms = 100U;
    cfg.max_safety_age_ms = 100U;
    cfg.require_new_feedback = false; /* host RTOS and TPDO are not phase-locked */
    cfg.pipeline_config.require_new_sample = false;
    cfg.pipeline_config.sensor.geometry_verified = true;
    cfg.pipeline_config.sensor.flange_T_sensor.rotation_wxyz[0] = 1.0;
    cfg.pipeline_config.sensor.sensor_T_grip.rotation_wxyz[0] = 1.0;
    cfg.pipeline_config.bias.valid = true;
    cfg.pipeline_config.mapper.model_verified = true; /* SYNTHETIC ONLY */
    cfg.pipeline_config.mapper.damping_lambda = 0.02;
    cfg.pipeline_config.mapper.minimum_sigma = 1e-6;
    cfg.pipeline_config.mapper.maximum_step_s = 0.004;
    cfg.pipeline_config.admittance.maximum_step_s = 0.004;
    for (size_t i = 0U; i < ROBOT_DOF; ++i) {
        cfg.pipeline_config.admittance.virtual_mass[i] = 2.0;
        cfg.pipeline_config.admittance.damping[i] = 4.0;
        cfg.pipeline_config.admittance.speed_limit[i] = 0.5;
        cfg.pipeline_config.admittance.acceleration_limit[i] = 10.0;
        cfg.pipeline_config.mapper.joint_acceleration_limit[i] = 100.0;
        cfg.pipeline_config.mapper.tracking_error_limit[i] = 0.2;
    }
    return hand_guiding_dry_run_init(&d->adapter, &cfg);
}

void gp_host_guidance_use_virtual_ft(GpHostGuidanceDemo *d,
                                     GpVirtualFtReceiver *receiver)
{
    if (d != NULL) d->virtual_ft_receiver = receiver;
}

SupervisorGuidanceDryRunPort gp_host_guidance_port(GpHostGuidanceDemo *d)
{
    SupervisorGuidanceDryRunPort port = {0};
    if (d != NULL && d->adapter.configured) {
        port.context = d;
        port.tick = tick;
        port.take_candidate = take_candidate;
        port.stop = stop;
    }
    return port;
}

void gp_host_guidance_flush(GpHostGuidanceDemo *d, const char *name)
{
    if (d == NULL || name == NULL || name[0] == 0) return;
    if (!d->open_attempted) {
        d->open_attempted = true;
        d->csv = fopen(name, "w");
        if (d->csv == NULL) {
            perror("4E2B dry-run trace");
            return;
        }
        fputs("time_us,cycle,accepted,valid,fault,"
              "fx,fy,fz,tx,ty,tz,vx,vy,vz,wx,wy,wz,"
              "q1,q2,q3,q4,q5,q6,"
              "feedback_measurement_valid,feedback_max_age_us,"
              "actual_q1,actual_q2,actual_q3,actual_q4,actual_q5,actual_q6\n",
              d->csv);
    }
    if (d->csv == NULL) return;
    unsigned dropped;
    taskENTER_CRITICAL();
    dropped = d->dropped;
    taskEXIT_CRITICAL();
    if (dropped != d->drops_reported) {
        fprintf(stderr, "[4E2B] WARNING: %u total CSV trace rows lost; "
                        "host scheduler/HMI consumer lagged\\n", dropped);
        d->drops_reported = dropped;
    }
    bool any = false;
    for (;;) {
        GpGuidanceTraceRow row;
        bool have = false;
        taskENTER_CRITICAL();
        if (d->tail != d->head) {
            row = d->rows[d->tail];
            d->tail = (d->tail + 1U) % GP_GUIDANCE_RING_SIZE;
            have = true;
        }
        taskEXIT_CRITICAL();
        if (!have) break;
        any = true;
        fprintf(d->csv, "%llu,%u,%u,%u,%u",
                (unsigned long long)row.time_us, row.cycle,
                (unsigned)row.accepted, (unsigned)row.valid,
                (unsigned)row.fault);
        for (size_t i = 0U; i < ROBOT_DOF; ++i)
            fprintf(d->csv, ",%.9g", row.wrench[i]);
        for (size_t i = 0U; i < ROBOT_DOF; ++i)
            fprintf(d->csv, ",%.9g", row.twist[i]);
        for (size_t i = 0U; i < ROBOT_DOF; ++i)
            fprintf(d->csv, ",%.12g", row.qref[i]);
        fprintf(d->csv, ",%u,%llu",
                (unsigned)row.feedback_measurement_valid,
                (unsigned long long)row.feedback_max_age_us);
        for (size_t i = 0U; i < ROBOT_DOF; ++i)
            fprintf(d->csv, ",%.12g", row.q_actual[i]);
        fputc('\n', d->csv);
    }
    if (any) (void)fflush(d->csv);
}
