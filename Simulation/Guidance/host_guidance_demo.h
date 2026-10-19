#ifndef GP_HOST_GUIDANCE_DEMO_H
#define GP_HOST_GUIDANCE_DEMO_H

/* This host-only SIL Kit fixture cannot command drives. Its modified robot
 * geometry and every tuning value are deliberately synthetic. */
#include "../../HandGuiding/supervisor_dry_run_adapter.h"
#include <stdio.h>
#include "virtual_ft_receiver.h"

typedef enum {
    GP_GUIDANCE_FAIL_NONE = 0,
    GP_GUIDANCE_FAIL_PRE_SAMPLE,
    GP_GUIDANCE_FAIL_INPUT,
    GP_GUIDANCE_FAIL_SENSOR_FRAME,
    GP_GUIDANCE_FAIL_FEEDBACK_CONVERSION,
    GP_GUIDANCE_FAIL_PIPELINE_AFTER_SAMPLE
} GpHostGuidanceFailure;

const char *gp_host_guidance_failure_name(GpHostGuidanceFailure reason);

#define GP_GUIDANCE_RING_SIZE 256U
typedef enum {
    GP_GUIDANCE_INJECT_NONE = 0,
    GP_GUIDANCE_INJECT_STALE_FT,
    GP_GUIDANCE_INJECT_BAD_FEEDBACK
} GpGuidanceFaultKind;
typedef struct {
    uint64_t time_us;
    unsigned cycle;
    bool accepted, valid, fault;
    double wrench[6], twist[6], qref[6];
    /* Pre-dispatch joint feedback, not post-command achieved state. */
    double q_actual[6];
    uint64_t feedback_max_age_us;
    bool feedback_measurement_valid;
} GpGuidanceTraceRow;
typedef struct {
    HandGuidingDryRunState adapter;
    RobotConfig synthetic_observation_robot;
    GpGuidanceFaultKind inject;
    unsigned inject_after;
    unsigned cycles;
    bool inject_ft_active;
    GpHostGuidanceFailure last_failure;
    bool motion_requested;
    bool candidate_pending;
    SupervisorGuidanceCandidate candidate;
    GpGuidanceTraceRow rows[GP_GUIDANCE_RING_SIZE];
    unsigned head, tail, dropped;
    unsigned drops_reported;
    FILE *csv;
    bool open_attempted;
    GpVirtualFtReceiver *virtual_ft_receiver; /* NULL => original 0.5 N */
    const SupervisorGuidanceDryRunCycle *sample_cycle; /* only during tick */
} GpHostGuidanceDemo;

bool gp_host_guidance_init(GpHostGuidanceDemo *demo,
                           const RobotConfig *simulation_robot,
                           GpGuidanceFaultKind kind, unsigned inject_after);
void gp_host_guidance_use_virtual_ft(GpHostGuidanceDemo *demo,
                                     GpVirtualFtReceiver *receiver);
SupervisorGuidanceDryRunPort gp_host_guidance_port(GpHostGuidanceDemo *demo);
/* Call only from the lower-priority HMI task; no CSV I/O on Supervisor. */
void gp_host_guidance_flush(GpHostGuidanceDemo *demo, const char *filename);
#endif
