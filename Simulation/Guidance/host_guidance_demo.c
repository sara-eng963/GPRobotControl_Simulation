#include "host_guidance_demo.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

static bool synthetic_ft(void *context, FtSensorSample *out)
{
    GpHostGuidanceDemo *d = (GpHostGuidanceDemo *)context;
    if (d == NULL || out == NULL) return false;
    memset(out, 0, sizeof(*out));
    out->valid = true;
    const uint64_t now = (uint64_t)xTaskGetTickCount() *
        UINT64_C(1000000) / (uint64_t)configTICK_RATE_HZ;
    out->received_time_us = (uint32_t)(
        d->inject_ft_active ? now - UINT64_C(40000) : now);
    out->force_N[0] = 0.5; /* fake handle load, N */
    return true;
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

    const bool accepted = hand_guiding_dry_run_tick(&d->adapter, &observation);
    GpGuidanceTraceRow r = {0};
    r.time_us = cycle->now_us;
    r.cycle = d->cycles;
    r.accepted = accepted;
    r.valid = d->adapter.last_output.valid;
    r.fault = d->adapter.pipeline.fault_latched;
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
    if (d != NULL) hand_guiding_dry_run_stop(&d->adapter);
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

SupervisorGuidanceDryRunPort gp_host_guidance_port(GpHostGuidanceDemo *d)
{
    SupervisorGuidanceDryRunPort port = {0};
    if (d != NULL && d->adapter.configured) {
        port.context = d;
        port.tick = tick;
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
              "q1,q2,q3,q4,q5,q6\n", d->csv);
    }
    if (d->csv == NULL) return;
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
        fputc('\n', d->csv);
    }
    if (any) (void)fflush(d->csv);
}
