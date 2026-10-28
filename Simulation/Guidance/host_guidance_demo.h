#ifndef HOST_GUIDANCE_DEMO_H
#define HOST_GUIDANCE_DEMO_H

/* Host-only candidate producer; only Supervisor can dispatch to drives.
 * The old observation fixture perturbs geometry. Interactive F/T uses the
 * shared simulation model. All tuning remains synthetic. */
#include "../../HandGuiding/supervisor_dry_run_adapter.h"
#include <stdio.h>
#include "virtual_ft_receiver.h"

typedef enum {
    HOST_GUIDANCE_FAIL_NONE = 0,
    HOST_GUIDANCE_FAIL_PRE_SAMPLE,
    HOST_GUIDANCE_FAIL_INPUT,
    HOST_GUIDANCE_FAIL_SENSOR_FRAME,
    HOST_GUIDANCE_FAIL_FEEDBACK_CONVERSION,
    HOST_GUIDANCE_FAIL_PIPELINE_AFTER_SAMPLE,
    HOST_GUIDANCE_FAIL_WRENCH_FRAME,
    HOST_GUIDANCE_FAIL_ADMITTANCE,
    HOST_GUIDANCE_FAIL_MAPPER_SINGULARITY,
    HOST_GUIDANCE_FAIL_MAPPER_TRACKING,
    HOST_GUIDANCE_FAIL_MAPPER_JOINT_LIMIT,
    HOST_GUIDANCE_FAIL_MAPPER_OTHER
} HostGuidanceFailure;

const char *host_guidance_failure_name(HostGuidanceFailure reason);

#define HOST_GUIDANCE_RING_SIZE 256U
typedef enum {
    HOST_GUIDANCE_INJECT_NONE = 0,
    HOST_GUIDANCE_INJECT_STALE_FT,
    HOST_GUIDANCE_INJECT_BAD_FEEDBACK
} HostGuidanceFaultKind;
typedef struct {
    uint64_t time_us;
    unsigned cycle;
    bool accepted, valid, fault;
    double wrench[6], twist[6], qref[6];
    /* Pre-dispatch joint feedback, not post-command achieved state. */
    double q_actual[6];
    uint64_t feedback_max_age_us;
    bool feedback_measurement_valid;
} HostGuidanceTraceRow;
/* Bounded, read-only snapshot of the latest two candidates; no drive API. */
typedef struct {
    bool has_last;
    bool has_previous;
    SupervisorGuidanceDryRunCycle last_cycle;
    SupervisorGuidanceCandidate last_candidate;
    SupervisorGuidanceCandidate previous_candidate;
} HostGuidanceGateAudit;

/* Simulation observation, published by the existing Supervisor callback. */
typedef struct {
    uint64_t time_us;
    bool enabled, force_applied, settled;
} HostGuidanceMotionStatus;

typedef struct {
    HandGuidingDryRunState adapter;
    const RobotConfig *simulation_robot; /* immutable, outlives this adapter */
    RobotConfig synthetic_observation_robot;
    HostGuidanceFaultKind inject;
    unsigned inject_after;
    unsigned cycles;
    bool inject_ft_active;
    HostGuidanceFailure last_failure;
    /* Host-only data for a non-committing replay AFTER a failed tick.
     * Never read by the physical STM32 or a motor command path. */
    FtSensorSample diagnostic_sample;
    double diagnostic_measured_q[ROBOT_DOF];
    double diagnostic_sigma_min;
    double diagnostic_max_tracking_error_rad;
    bool diagnostic_sample_ready;
    bool motion_requested;
    bool candidate_pending;
    SupervisorGuidanceCandidate candidate;
    HostGuidanceGateAudit gate_audit;
    HostGuidanceMotionStatus motion_status;
    unsigned stationary_feedback_sets;
    uint32_t stationary_sequence[ROBOT_DOF];
    int32_t stationary_position[ROBOT_DOF];
    HostGuidanceTraceRow rows[HOST_GUIDANCE_RING_SIZE];
    unsigned head, tail, dropped;
    unsigned drops_reported;
    FILE *csv;
    bool open_attempted;
    VirtualFtReceiver *virtual_ft_receiver; /* NULL => original 0.5 N */
    const SupervisorGuidanceDryRunCycle *sample_cycle; /* only during tick */
} HostGuidanceDemo;

bool host_guidance_init(HostGuidanceDemo *demo,
                           const RobotConfig *simulation_robot,
                           HostGuidanceFaultKind kind, unsigned inject_after);
void host_guidance_use_virtual_ft(HostGuidanceDemo *demo,
                                     VirtualFtReceiver *receiver);
SupervisorGuidanceDryRunPort host_guidance_port(HostGuidanceDemo *demo);
/* Call only from the lower-priority HMI task; no CSV I/O on Supervisor. */
void host_guidance_flush(HostGuidanceDemo *demo, const char *filename);
/* Captured in the Supervisor cycle and copied for diagnostics only. */
bool host_guidance_gate_audit_snapshot(
    HostGuidanceDemo *demo, HostGuidanceGateAudit *out);
/* False on stale observation; never treats repeated feedback as settled. */
bool host_guidance_motion_status(
    HostGuidanceDemo *demo, HostGuidanceMotionStatus *out);
#endif
